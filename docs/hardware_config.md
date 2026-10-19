# hardware_config.yaml 配置规范

`hardware_config.yaml` 描述仿真对外暴露的驱动器、六维力传感器和 IO 模块，并定义共享内存 PDO 的变量名称及关节计数换算参数。

本文依据 [硬件配置解析、PDO 布局及控制实现](../src/mujoco_simulator.cpp) 和 [共享内存结构](../include/rocos_mujoco/shared_memory_config.hpp)。完整模型示例见 [Talon 配置](../models/talon/hardware_config.yaml) 与 [charge_arm 配置](../models/charge_arm/hardware_config.yaml)。入口文件的定位规则见 [config.yaml 配置规范](config.md)。

## 1. 顶层结构与硬件类型

文件采用 YAML，顶层使用映射，必须包含 `hardware` 数组。每个数组元素描述一个设备。

| 字段 | 类型 | 规范与当前行为 |
| --- | --- | --- |
| `hardware` | 数组 | 必须存在且为数组；缺失或类型不符会导致加载失败。 |
| `hardware[].type` | 字符串 | 使用 `driver`、`ft_sensor` 或 `io`。缺少 `type` 的条目及未知类型目前会被跳过。 |
| `hardware[].id` | 整数 | 应显式填写非负、唯一的从站标识，并与控制端一致。解析器缺省值为 `-1`，目前不检查非负性或唯一性。 |
| `hardware[].inputs` | 映射 | 设备反馈到控制端的 PDO 定义。键是固定逻辑字段，值是对外发布的变量名称字符串。 |
| `hardware[].outputs` | 映射 | 控制端发送到设备的 PDO 定义；`ft_sensor` 不生成输出区。 |

`id` 不等于关节数组索引，也不等于启动配置中的 `ecat_id`。当前实现按 `driver`、`ft_sensor`、`io` 分类生成从站元数据；同类设备保持 YAML 中的出现顺序，不按 `id` 排序。

硬件配置没有像启动配置那样的完整未知字段检查。拼错字段可能被忽略或回退到默认值；类型转换错误则会报告 YAML 解析失败。

## 2. driver：关节驱动器

### 2.1 字段与关节映射

| 字段 | 类型 | 缺省值 | 含义 |
| --- | --- | --- | --- |
| `joint_name` | 字符串 | `joint_<id>` | MuJoCo 模型中的关节名称，区分大小写；建议始终显式填写。 |
| `transform` | 映射 | 各数值使用下表默认值 | PDO 原始计数与模型物理量之间的换算参数。 |
| `inputs` | 映射 | 整段缺失时不生成输入 PDO | 支持字段见 3.2。 |
| `outputs` | 映射 | 整段缺失时不生成输出 PDO | 支持字段见 3.3。 |

程序按 `joint_name` 查询关节，并从模型中取得自由度索引。名称不存在时，当前实现仍保留该驱动条目，启动日志中的 `mj_id` 为 `-1`；该条目不施加控制，物理反馈按零值进行计数换算。应在使用前确认所有驱动的 `mj_id` 都有效。

当前共享内存驱动控制路径面向单自由度 `hinge` / `slide` 关节，并使用自由度索引访问 `qpos` 和 `ctrl`。模型需要满足位置索引与自由度索引对应，且每个被控自由度有同索引、符合位置目标控制语义的执行器。仅填写 `joint_name` 不会自动查找或重排执行器。

包含 `free` / `ball` 关节或不同执行器排列的模型，不应直接假定该控制映射成立。侧栏能显示多自由度关节的数据，不代表共享内存驱动控制路径已支持相同映射。

### 2.2 transform：单位换算

| 字段 | 类型 | 默认值 | 含义与填写要求 |
| --- | --- | --- | --- |
| `cnt_per_unit` | 数值 | `1.0` | 每单位位置的计数，旋转关节按 rad，滑动关节按 m；必须配置为有限非零值。 |
| `ratio` | 数值 | `1.0` | 位置、速度换算倍率；必须为有限非零值，与控制端的倍率定义保持一致。 |
| `offset_pos_cnt` | 32 位整数 | `0` | 位置零点的计数偏移；当前速度反馈也会加上此偏移，详见下方公式。 |
| `torque_per_unit` | 数值 | `1.0` | 单位关节力矩或力对应的原始数值倍率；旋转关节按 N·m，滑动关节按 N；必须为有限非零值。 |
| `user_unit_name` | 字符串 | 不参与读取 | 示例中的说明性字段，通常填写 `rad` 或 `m`；不会触发单位转换。 |

非零、有限的倍率是正确配置要求，当前解析器没有主动校验这些条件。不要依赖错误配置在加载阶段一定被拒绝。

令 `C = cnt_per_unit`、`R = ratio`、`O = offset_pos_cnt`、`T = torque_per_unit`。当前换算行为如下，`trunc` 表示向零截断，所有整数转换都要求结果处于目标类型可表示范围内：

```text
目标位置：q_target   = (target_position - O) / C / R
目标速度：v_target   = target_velocity / C / R
目标力矩：tau_target = target_torque / T

位置反馈：position_actual_value = trunc(q * C * R) + O
速度反馈：velocity_actual_value = trunc(v * C * R) + O
力矩反馈：torque_actual_value   = trunc(tau * T)
```

其中 `tau = qfrc_actuator + qfrc_applied`，与仿真器当前反馈口径一致；它不是单独的力传感器测量值。`load_torque_value` 与 `torque_actual_value` 相同；次级位置、速度分别复制对应的主反馈。

特别注意：当前**速度反馈带 `O`，目标速度不减 `O`**。这是现有协议实现行为，不应按常规速度无零偏的假设另行解码。原始位置/速度为 `int32`，力矩为 `int16`，代码没有饱和裁剪；配置倍率时需保证全工作范围可表示。

旋转关节使用 rad、rad/s、N·m；滑动关节使用 m、m/s、N。`user_unit_name: deg` 不会让程序自动把角度转换为弧度。

### 2.3 示例中存在但仿真器不读取的字段

| 字段 | 示例约定 | 当前仿真行为 |
| --- | --- | --- |
| `torque_source` | 例如 `load_torque` | 不读取；不会改变上述力矩反馈来源。 |
| `limit.lower` / `limit.upper` | 位置下限/上限，rad 或 m | 不用于限制仿真命令。 |
| `limit.vel` | 速度上限，rad/s 或 m/s | 不用于限制仿真命令。 |
| `limit.acc` | 加速度上限，rad/s² 或 m/s² | 不用于限制仿真命令。 |
| `limit.jerk` | 加加速度上限，rad/s³ 或 m/s³ | 不用于限制仿真命令。 |
| `limit.effort` | 力矩/力上限，N·m 或 N | 不用于限制仿真命令。 |
| `transform.user_unit_name` | `rad` 或 `m` | 说明性字段，不参与数值换算。 |

这些字段可随硬件描述保留，但不能据此认为本仿真器已执行限位、限速或力矩限幅。模型中的关节和执行器约束应在 MJCF/URDF 中按实际需要设置。

## 3. PDO 名称、启用规则与布局

### 3.1 省略、默认值和禁用

PDO 的键名由代码定义，右侧字符串才是发布给控制端的变量名称。例如：

```yaml
inputs:
  status_word: "Status word"
  secondary_velocity_value: ""
outputs:
  control_word: "Control word"
```

规则如下：

1. 整个 `inputs` 或 `outputs` 段缺失：该方向不生成 PDO 字段。
2. 段存在时，未列出的已知字段：使用代码中的默认名称，仍会生成。
3. 字段值为 `""`：禁用该字段，不占用该方向的字节空间。
4. 因此 `inputs: {}`、`outputs: {}` 分别启用该设备类型的全部默认输入、输出字段。
5. 字段值应是名称字符串，不是初始数值、字节偏移或数据类型；不要用 `null` 表示禁用。
6. 未知 PDO 键不会增加新变量。变量按下表中的模板顺序紧密排列，不按 YAML 书写顺序排列；禁用字段会改变后续偏移。

对外名称需与控制端查找名称一致，注意大小写和空格。例如默认的 `Statusword` 与 Talon 示例中的 `Status word` 是两个不同名称。

### 3.2 driver 输入字段

表格顺序就是默认排列顺序。全部启用时共 **30 字节**。

| 键名 | 默认 PDO 名称 | 类型 / 字节 | 当前反馈 |
| --- | --- | --- | --- |
| `status_word` | `Statusword` | `uint16` / 2 | 仿真 CiA 402 状态字。 |
| `position_actual_value` | `Position actual value` | `int32` / 4 | 位置计数。 |
| `velocity_actual_value` | `Velocity actual value` | `int32` / 4 | 速度计数，包含当前实现的偏移规则。 |
| `torque_actual_value` | `Torque actual value` | `int16` / 2 | 力矩或力的原始值。 |
| `load_torque_value` | `Analog Input 1` | `int16` / 2 | 同 `torque_actual_value`。 |
| `secondary_position_value` | `Auxiliary position actual value` | `int32` / 4 | 同主位置反馈。 |
| `secondary_velocity_value` | `Secondary velocity value` | `int32` / 4 | 同主速度反馈。 |
| `digital_inputs` | `Digital Inputs` | `int32` / 4 | 固定写零。 |
| `digital_outputs` | `Digital Outputs` | `int32` / 4 | 固定写零，不回显输出命令。 |

### 3.3 driver 输出字段

全部启用时共 **17 字节**。

| 键名 | 默认 PDO 名称 | 类型 / 字节 | 当前处理 |
| --- | --- | --- | --- |
| `control_word` | `Controlword` | `uint16` / 2 | 驱动仿真 CiA 402 状态机。 |
| `mode_of_operation` | `Modes of operation` | `int8` / 1 | 控制模式；见下表。 |
| `target_position` | `Target position` | `int32` / 4 | 目标位置计数。 |
| `target_velocity` | `Target velocity` | `int32` / 4 | 目标速度计数。 |
| `target_torque` | `Target torque` | `int16` / 2 | 目标力矩或力的原始值。 |
| `digital_outputs` | `Digital Outputs` | `int32` / 4 | 生成 PDO 布局，但目前不执行输出命令。 |

| `mode_of_operation` | 当前仿真行为 |
| --- | --- |
| `6` | 按位置模式处理，不实现真实回零搜索流程。 |
| `8` | CSP 位置控制。 |
| `9` | CSV 速度控制。 |
| `10` | CST 力矩/力控制。 |
| 其他值 | 保持位置。 |

驱动未进入使能状态时保持位置；仅写目标值不会自动使能。有效控制需要控制端提供状态机控制字、模式以及相应的目标字段。

## 4. ft_sensor：六维力传感器

支持 `id`、`type: ft_sensor` 和 `inputs`；不生成输出 PDO。输入字段按以下顺序排列，全部启用时共 **12 字节**。

| 键名 | 默认 PDO 名称 | 类型 / 字节 |
| --- | --- | --- |
| `fx` | `Fx` | `int16` / 2 |
| `fy` | `Fy` | `int16` / 2 |
| `fz` | `Fz` | `int16` / 2 |
| `tx` | `Tx` | `int16` / 2 |
| `ty` | `Ty` | `int16` / 2 |
| `tz` | `Tz` | `int16` / 2 |

当前各字段固定写零，未配置与 MuJoCo `sensor`、`site` 的绑定，也未实现传感器倍率换算。不能将此配置视为已接通六维力测量。

## 5. io：IO 模块

除公共字段外，支持以下整数配置项，默认均为 `0`：

| 字段 | 含义 |
| --- | --- |
| `digital_in_channels` | 数字输入通道数。 |
| `digital_out_channels` | 数字输出通道数。 |
| `analog_in_channels` | 模拟输入通道数。 |
| `analog_out_channels` | 模拟输出通道数。 |

通道数应为非负整数；目前仅保存和打印，不会据此生成每通道 PDO 数组，也没有相应范围校验。实际 PDO 仍由以下固定模板生成。

| 方向 | 键名 | 默认 PDO 名称 | 类型 / 字节 |
| --- | --- | --- | --- |
| 输入 | `digital_inputs` | `Digital Inputs` | `int32` / 4 |
| 输入 | `digital_outputs` | `Digital Outputs` | `int32` / 4 |
| 输入 | `analog_inputs` | `Analog Inputs` | `int16` / 2 |
| 输出 | `digital_outputs` | `Digital Outputs` | `int32` / 4 |
| 输出 | `analog_outputs` | `Analog Outputs` | `int16` / 2 |

全部启用时输入 **10 字节**、输出 **6 字节**。当前所有输入固定写零，输出命令尚未接入仿真行为。

## 6. 配置示例

以下示例仅映射 `joint_1`，并展示三类设备的写法；它不是完整机械臂配置。按实际模型补充驱动条目，并与控制端核对从站标识、变量名称和换算参数。

```yaml
hardware:
  - id: 0
    type: driver
    joint_name: joint_1
    transform:
      cnt_per_unit: 156455.678
      ratio: 1.0
      offset_pos_cnt: 0
      torque_per_unit: 1.0
      user_unit_name: rad
    inputs:
      status_word: "Status word"
      position_actual_value: "Position actual value"
      velocity_actual_value: "Velocity actual value"
      torque_actual_value: "Torque actual value"
      secondary_position_value: ""
      secondary_velocity_value: ""
      digital_inputs: ""
      digital_outputs: ""
      # load_torque_value 省略，仍使用默认名称 "Analog Input 1"
    outputs:
      control_word: "Control word"
      mode_of_operation: "Mode of operation"
      target_position: "Target Position"
      target_velocity: "Target Velocity"
      target_torque: "Target Torque"
      digital_outputs: ""

  - id: 10
    type: ft_sensor
    inputs: {}  # 六个默认字段；当前反馈为零

  - id: 11
    type: io
    digital_in_channels: 8
    digital_out_channels: 8
    analog_in_channels: 1
    analog_out_channels: 1
    inputs: {}
    outputs: {}
```

滑动关节的 `cnt_per_unit` 按每米计数配置，`torque_per_unit` 按每牛顿原始数值倍率配置，并将说明性字段 `user_unit_name` 填为 `m`。

## 7. 配置边界与排查

共享内存结构的默认上限为 50 个从站、每从站每方向 25 个 PDO 变量。PDO 名称最多保留 71 字节，从站名称最多保留 79 字节，超长名称会截断；这里的长度是字节数，不是中文字符数。可编译覆盖的常量见 [shared_memory_config.hpp](../include/rocos_mujoco/shared_memory_config.hpp)，两端结构必须一致。

当前加载器不是完整的 schema 校验器，不会检查重复 `id`、未知关节、倍率有效性或设备总数是否越界。正确配置应满足这些约束，并在启动日志中核对设备数量、每个驱动的 `mj_id`、`pdo_in` / `pdo_out` 字节数。

常见问题：

- **`YAML missing 'hardware'`**：缺少 `hardware`，或其值不是数组。
- **`YAML parse`**：文件无法读取、YAML 语法错误或字段类型无法转换。
- **驱动存在但没有动作**：检查关节名称、执行器索引对应关系、控制字使能状态与控制模式。
- **控制端找不到 PDO**：检查名称大小写和空格；确认没有将字段设为 `""`，也没有省略整个方向段。
- **位置、速度偏移异常**：两端核对 `cnt_per_unit`、`ratio`、`offset_pos_cnt`，特别是速度反馈的偏移约定。
- **力矩数值异常**：核对 `torque_per_unit` 和 `int16` 表示范围；旋转关节用 N·m，滑动关节用 N。
- **FT / IO 始终为零**：这是当前实现的占位行为，不代表通过增加通道数或修改 PDO 名称就能启用测量。
