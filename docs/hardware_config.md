# hardware.yaml 硬件配置规范

`hardware.yaml`（旧文件名 `hardware_config.yaml`）描述仿真对外暴露的驱动器、六维力传感器和 IO 模块，并定义共享内存 PDO 的变量名称及关节计数换算参数。

本文依据 [硬件配置解析、PDO 布局及控制实现](../src/mujoco_simulator.cpp) 和 [共享内存结构](../include/rocos_mujoco/shared_memory_config.hpp)。完整模型示例见 [Talon 配置](../config/talon/hardware.yaml)。入口文件的定位规则见 [mujoco.yaml 启动配置规范](config.md)。

## 与 rocos-app 共用配置

MuJoCo 按当前 rocos-app 的换算字段读取硬件配置：`pos_per_unit`、`vel_per_unit`、`torque_per_unit`、`load_torque_per_unit`、`ratio` 和 `offset_pos_cnt`。两端应引用同一份硬件文件，避免副本的倍率与 PDO 名称发生偏差；文件名和存放目录不影响解析。

[Talon 启动配置](../config/talon/mjcf/mujoco.yaml) 引用本仓库的 `config/talon/hardware.yaml`，已迁移到 `pos_per_unit`。双臂的 [App 配置](../config/dualarm/README.md#两个-app-进程) 与仿真配置分别共用 `hardware_left.yaml` 和 `hardware_right.yaml`。不能假设不同仓库的 Talon 文件始终逐字段一致。

`torque_source` 选择负载力矩反馈通道：`load_torque` 使用 `load_torque_value`（int16），`secondary_position` 使用 `secondary_position_value`（int32）。两种通道均按 `load_torque_per_unit` 换算，不使用位置倍率或位置零偏。MuJoCo 根据同一字段编码对应 PDO；驱动力矩则独立按 `torque_per_unit` 编码。当前两种力矩的物理来源均为总驱动力矩，详见 2.2。

两端对省略 PDO 字段的处理有所不同：rocos-app 只绑定配置了非空名称的字段；仿真器在 `inputs` / `outputs` 段存在时会为省略的已知字段补默认名称（见第 3 节）。这些额外 PDO 不妨碍 Talon 按名称绑定已配置字段。

## 1. 顶层结构与硬件类型

文件采用 YAML，顶层使用映射，必须包含 `hardware` 数组。每个数组元素描述一个设备。

| 字段 | 类型 | 规范与当前行为 |
| --- | --- | --- |
| `hardware` | 数组 | 必须存在且为数组；缺失或类型不符会导致加载失败。 |
| `hardware[].type` | 字符串 | 使用 `driver`、`ft_sensor` 或 `io`。缺少 `type` 的条目及未知类型目前会被跳过。 |
| `hardware[].id` | 整数 | 应显式填写非负、唯一的从站标识，并与控制端一致。解析器缺省值为 `-1`，负数或同一主站内重复值会导致初始化失败；不同主站可复用同一从站 ID。 |
| `hardware[].inputs` | 映射 | 设备反馈到控制端的 PDO 定义。键是固定逻辑字段，值是对外发布的变量名称字符串。 |
| `hardware[].outputs` | 映射 | 控制端发送到设备的 PDO 定义；`ft_sensor` 不生成输出区。 |

`id` 不等于关节数组索引，也不等于启动配置中的 `ecat_id`。当前实现按 `driver`、`ft_sensor`、`io` 分类生成从站元数据；同类设备保持 YAML 中的出现顺序，不按 `id` 排序。

硬件配置没有像启动配置那样的完整未知字段检查。拼错字段可能被忽略或回退到默认值；类型转换错误则会报告 YAML 解析失败。

## 2. driver：关节驱动器

### 2.1 字段与关节映射

| 字段 | 类型 | 缺省值 | 含义 |
| --- | --- | --- | --- |
| `joint_name` | 字符串 | `joint_<id>` | MuJoCo 模型中的关节名称，区分大小写；建议始终显式填写。 |
| `torque_source` | 字符串 | `load_torque` | `secondary_position` 将负载力矩编码到辅助位置 PDO，其他值使用普通负载力矩通道。 |
| `transform` | 映射 | 各数值使用下表默认值 | PDO 原始计数与模型物理量之间的换算参数。 |
| `inputs` | 映射 | 整段缺失时不生成输入 PDO | 支持字段见 3.2。 |
| `outputs` | 映射 | 整段缺失时不生成输出 PDO | 支持字段见 3.3。 |

程序按 `joint_name` 查询模型关节，分别获取 `jnt_qposadr`、`jnt_dofadr`，并按执行器的关节传动绑定查找执行器，不要求这三个索引相同，也不依赖执行器书写顺序。启动日志打印主站 ID、位置地址和执行器索引。

当前控制路径支持单自由度 `hinge` / `slide` 关节，每个被控关节必须绑定一个单位齿比、无激活动力学的位置执行器。未知关节、`free` / `ball` 被控关节、缺失或多个执行器、非位置执行器，以及跨主站重复映射同一关节都会导致初始化失败。侧栏显示多自由度关节的能力独立于驱动控制映射。

### 2.2 transform：单位换算

| 字段 | 类型 | 默认值 | 含义与填写要求 |
| --- | --- | --- | --- |
| `pos_per_unit` | 数值 | `1.0` | 每单位位置的计数，旋转关节按 rad，滑动关节按 m；必须配置为有限非零值。 |
| `vel_per_unit` | 数值 | `pos_per_unit` | 每单位速度的计数；速度不使用位置零偏。 |
| `load_torque_per_unit` | 数值 | `1.0` | 负载力矩独立倍率；与 `torque_per_unit` 分别配置。 |
| `ratio` | 数值 | `1.0` | 位置、速度换算倍率；必须为有限非零值，与控制端的倍率定义保持一致。 |
| `offset_pos_cnt` | 32 位整数 | `0` | 位置零点的计数偏移；仅用于位置。 |
| `torque_per_unit` | 数值 | `1.0` | 单位关节力矩或力对应的原始数值倍率；旋转关节按 N·m，滑动关节按 N；必须为有限非零值。 |
| `user_unit_name` | 字符串 | 不参与读取 | 示例中的说明性字段，通常填写 `rad` 或 `m`；不会触发单位转换。 |

加载器检查以上五个倍率必须有限且非零，否则初始化失败。字段及默认值与当前 rocos-app 一致；旧字段 `cnt_per_unit` 会明确报错，应迁移为 `pos_per_unit`，不能同时保留。

令 `P = pos_per_unit`、`V = vel_per_unit`、`R = ratio`、`O = offset_pos_cnt`、`T = torque_per_unit`、`L = load_torque_per_unit`。当前换算行为如下，`round` 表示四舍五入（半整数远离零），`trunc` 表示向零截断；位置/速度转换会拒绝非有限值或超出 int32 范围的结果：

```text
目标位置：q_target   = (target_position - O) / P / R
目标速度：v_target   = target_velocity / V / R
目标力矩：tau_target = target_torque / T

位置反馈：position_actual_value = round(q * P * R) + O
速度反馈：velocity_actual_value = trunc(v * V * R)
力矩反馈：torque_actual_value   = trunc(tau * T)
负载力矩：load_torque_value     = trunc(tau * L)
```

其中 `tau = qfrc_actuator + qfrc_applied`，与仿真器当前反馈口径一致；它不是单独的力传感器测量值。`torque_source: load_torque`（默认）时，负载力矩通过 int16 的 `load_torque_value` 返回；`torque_source: secondary_position` 时，通过 int32 的 `secondary_position_value` 返回 `trunc(tau * L)`，并将 `load_torque_value` 置零。其他情况下次级位置复制位置反馈，次级速度始终复制速度反馈。

位置/速度 PDO 为 `int32`，普通力矩 PDO 为 `int16`。代码拒绝非有限值和越界反馈，不做饱和裁剪；配置倍率时需保证全工作范围可表示。速度反馈已按 App 语义移除位置零偏，旧控制端如曾依赖该偏移需同步迁移。

旋转关节使用 rad、rad/s、N·m；滑动关节使用 m、m/s、N。`user_unit_name: deg` 不会让程序自动把角度转换为弧度。

### 2.3 示例中存在但仿真器不读取的字段

| 字段 | 示例约定 | 当前仿真行为 |
| --- | --- | --- |
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
      pos_per_unit: 156455.678
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

滑动关节的 `pos_per_unit` 按每米计数配置，`torque_per_unit` 按每牛顿原始数值倍率配置，并将说明性字段 `user_unit_name` 填为 `m`。

## 7. 配置边界与排查

共享内存 ABI v2 的固定上限为 50 个从站、每从站每方向 25 个 PDO 变量，数量超限会初始化失败，不截断。容量不能通过宏覆盖。PDO 名称最多保留 71 字节，从站名称最多保留 79 字节，超长名称会截断；这里的长度是字节数，不是中文字符数。统一定义见 [rocos-shm](../third_party/rocos-shm/include/rocos_shm/abi.hpp)，两端必须使用匹配的 v2 库重新编译。

初始化校验每个 PDO 的字节偏移、大小与映射边界，支持某一方向或两个方向均为空。现有 YAML 字段只生成整字节 PDO；非字节对齐映射不受支持。多字节数值使用 `memcpy` 访问，允许紧凑布局中的非自然对齐字节偏移。v2 不改变已有变量名称和计数换算方式。

当前加载器检查从站 ID 非负且在主站内唯一、关节/执行器映射和倍率有效性，但不是完整的 schema 校验器。应在启动日志中核对设备数量、每个驱动的主站 ID、`qpos`、`actuator`、`pdo_in` / `pdo_out` 字节数。v2 邮箱的 `target_slave` 是描述数组索引，不是这里配置的从站 `id`；仿真器目前不支持 SDO。

常见问题：

- **`YAML missing 'hardware'`**：缺少 `hardware`，或其值不是数组。
- **`YAML parse`**：文件无法读取、YAML 语法错误或字段类型无法转换。
- **驱动存在但没有动作**：检查关节名称、执行器索引对应关系、控制字使能状态与控制模式。
- **控制端找不到 PDO**：检查名称大小写和空格；确认没有将字段设为 `""`，也没有省略整个方向段。
- **位置、速度偏移异常**：两端核对 `pos_per_unit`、`vel_per_unit`、`ratio`、`offset_pos_cnt`；速度无位置零偏。
- **力矩数值异常**：核对 `torque_per_unit` 和 `int16` 表示范围；旋转关节用 N·m，滑动关节用 N。
- **FT / IO 始终为零**：这是当前实现的占位行为，不代表通过增加通道数或修改 PDO 名称就能启用测量。
