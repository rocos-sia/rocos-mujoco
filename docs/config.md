# config.yaml 配置规范

`config.yaml` 定义仿真程序的启动参数：模型文件、硬件配置、共享内存主站标识、控制周期、运行时长和可视化开关。

本文对应当前仓库的 [启动参数解析代码](../src/main.cpp)。仓库示例见 [config/config.yaml](../config/config.yaml)；硬件配置内容见 [hardware_config.yaml 配置规范](hardware_config.md)。

## 1. 文件格式与启动方式

文件采用 YAML，顶层必须是映射，不能是数组或单个标量。字段名区分大小写；不在下表中的顶层字段会被拒绝。布尔值使用 `true` / `false`，数值字段使用数值，不填写单位后缀。

从仓库根目录启动：

```bash
./build/bin/rocos_mujoco_sim
```

默认配置路径是**进程工作目录**下的 `config/config.yaml`，不是可执行文件所在目录。也可以显式指定：

```bash
/path/to/rocos_mujoco_sim --config /path/to/config.yaml
```

`--config` 后面的相对路径以进程工作目录为基准。程序还支持 `--help` / `-h`；模型等业务参数通过 YAML 设置，不支持同名命令行覆盖项。

## 2. 字段定义

| 字段 | 类型 | 是否必填 / 默认值 | 规则与含义 |
| --- | --- | --- | --- |
| `model` | 字符串 | 新配置必填，无默认模型 | 直接指定存在的 MJCF 或 URDF 文件；不能填目录名、空字符串或不存在的文件。 |
| `hardware_config` | 字符串 | 可选；默认取模型文件同目录的 `hardware_config.yaml` | 显式配置时必须是非空路径；文件在仿真初始化时加载。 |
| `ecat_id` | 整数 | 可选，`0` | 非负的共享内存主站标识；连接同一仿真器的控制端应使用相同标识，独立实例使用不同标识。它不是硬件配置中单个从站的 `id`。 |
| `cycle_time_us` | 数值 | 可选，`1000.0` | 控制周期，单位微秒；必须是有限数，范围为 `[1, 4294967295]`。建议使用整数微秒，例如 `1000.0` 表示 1 ms。 |
| `duration_sec` | 数值 | 可选，`-1.0` | 运行时长，单位秒；必须是有限数，允许 `-1` 或任意非负值。`-1` 持续运行，`0` 仅初始化，正值按程序累计的仿真时间运行。 |
| `visualize` | 布尔值 | 可选，`false` | 是否开启仿真窗口。`true` 要求构建时开启 GLFW/OpenGL 可视化支持，且运行环境可创建窗口。 |
| `urdf` | 字符串 | 旧配置兼容字段 | 仅在没有 `model` 时作为模型路径别名；同时配置 `model` 与 `urdf` 会报错。新配置统一使用 `model`。 |

代码默认值与示例文件的取值不同：省略 `visualize` 时默认关闭窗口，而仓库示例明确设置为 `true`。

`duration_sec: 0` 仍会加载模型、硬件配置并初始化共享内存；它不是只做语法检查的模式。

## 3. 模型和硬件配置的路径规则

`model` 和显式填写的 `hardware_config` 均支持绝对路径。它们的相对路径统一以**启动 YAML 文件所在目录**为基准。

例如，启动配置位于 `/project/config/config.yaml`：

| 配置内容 | 实际路径 |
| --- | --- |
| `model: ../models/talon/scene.xml` | `/project/models/talon/scene.xml` |
| `model: ../models/talon/talon.xml` | `/project/models/talon/talon.xml` |
| `model: /data/robots/test.xml` | `/data/robots/test.xml` |
| `hardware_config: custom_hw.yaml` | `/project/config/custom_hw.yaml` |

程序只加载明确指定的模型文件，不扫描同目录下的 XML，也不自动补上 `scene.xml`。如果一个目录同时有机器人本体、场景和测试模型，应填写实际需要加载的文件。

省略 `hardware_config` 时，以**模型文件所在目录**为基准补上该文件名。例如 `model: /data/robots/test.xml` 对应默认硬件配置 `/data/robots/hardware_config.yaml`。显式填写 `hardware_config: hardware_config.yaml` 则指向启动 YAML 同目录，二者含义不同。

这些规则只负责定位入口模型文件。模型内部的 `include`、网格和纹理资源由 MuJoCo 模型加载器解析，启动程序不会重写它们。

## 4. 配置示例

以下示例保存于仓库的 `config/` 目录：

```yaml
model: ../models/talon/scene.xml
hardware_config: ../models/talon/hardware_config.yaml

ecat_id: 0
cycle_time_us: 1000.0
duration_sec: -1.0
visualize: true
```

如果硬件配置与模型同目录，可省略 `hardware_config`。下面的最小配置使用其他字段的默认值，在无头模式下持续运行：

```yaml
model: ../models/charge_arm/scene.xml
```

## 5. 周期与模型时间步长

`cycle_time_us` 用于主循环节拍、共享内存中的周期值、部分控制计算和程序累计的仿真时间。当前实现每个循环调用一次 `mj_step`，**不会用该字段覆盖 MJCF 中的 `option.timestep`**。

需要控制时间与物理时间一致时，应让模型时间步长等于 `cycle_time_us / 1000000`。例如控制周期 1000 μs 对应模型时间步长 0.001 s。小数微秒在循环等待时间及共享内存周期字段中会转为整数，因此推荐整数微秒。

## 6. 校验与迁移

以下情况会在启动配置解析阶段报错：

- 顶层不是 YAML 映射，或存在未知字段。
- 缺少 `model` 且没有兼容字段 `urdf`，或同时提供两者。
- 模型路径为空、指向目录，或文件不存在。
- 字段无法转换为所需类型。
- `ecat_id` 为负数，周期越界或不是有限数，运行时长既不是 `-1` 也不是非负有限数。

模型是否为有效的 MJCF/URDF、模型引用的资源能否加载，以及硬件配置内容是否合法，在后续初始化阶段检查。路径存在并不意味着模型内容有效。

旧的目录名写法需要迁移：

```yaml
# 旧写法，不再按 models/<名称>/scene.xml 解释
model: talon
```

```yaml
# 新写法，明确选择模型文件
model: ../models/talon/scene.xml
```

模型没有成功加载时，先检查路径基准和模型内部资源；提示硬件配置加载失败时，再按 [硬件配置规范](hardware_config.md) 检查 YAML 内容与关节映射。
