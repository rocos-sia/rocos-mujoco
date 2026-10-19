<p align="center">
  <img src="rocos-mujoco.png" alt="ROCOS MUJOCO" width="160">
</p>

# ROCOS MuJoCo

基于 **MuJoCo 3.4.0** 的 ROCOS 机器人仿真环境。通过 POSIX 共享内存模拟 EtherCAT 主站的数据接口，让 ROCOS 控制端在仿真中发送关节命令、读取反馈，并观察机器人运动和外力响应。

项目采用 **C++17 / CMake**，面向 **Linux / WSL**，支持交互式可视化和无头运行。当前版本见 [VERSION](VERSION)，配置与实现仍在迭代中。

[快速开始](#快速开始) · [无头运行](#无头运行) · [界面操作](#界面操作) · [配置文档](#模型与配置文档)

## 主要功能

| 功能 | 说明 |
| --- | --- |
| 模型加载 | 通过 YAML 直接指定 MJCF / URDF 文件路径，支持相对路径和绝对路径。 |
| 控制端对接 | 发布从站信息、PDO 布局和关节反馈，通过共享内存接收控制字、模式及目标值。 |
| 关节控制 | 简化的 CiA 402 状态机，支持 CSP 位置、CSV 速度和 CST 力矩/力控制。 |
| 模型显示 | 可勾选关节轴、坐标系，并通过滑条调整模型透明度。 |
| 实时数据 | 显示关节位置、速度、力矩/力，统一使用 SI 单位；支持旋转和滑动关节的数据展示。 |
| 可调侧栏 | 支持隐藏/显示、调整侧栏宽度与数据列宽度，数据框可完整显示科学计数法。 |
| 外力交互 | 在模型上选取受力点，拖动改变力方向，调节力的大小并显示作用标记。 |
| 无头运行 | 关闭 GLFW/OpenGL 可视化依赖，使用相同的仿真和共享内存控制流程。 |

```mermaid
flowchart LR
    A[ROCOS 控制端] <--> B[POSIX 共享内存与 PDO]
    B <--> C[rocos_mujoco_sim]
    C <--> D[MuJoCo 物理仿真]
    C --> E[可选的可视化窗口]
```

这里模拟的是 EtherCAT 主站的**共享内存接口与驱动行为**，运行仿真不需要连接真实 EtherCAT 从站。

## 快速开始

以下命令均在仓库根目录执行。

### 1. 准备依赖

| 依赖 | 要求 |
| --- | --- |
| 编译器 | 支持 C++17。 |
| CMake | 基础构建要求 3.16+；下方使用的预设建议采用 3.21+。 |
| MuJoCo | 3.4.0，需有匹配运行平台的头文件和共享库。 |
| yaml-cpp | 必需，用于解析启动和硬件配置。 |
| GLFW / OpenGL / libpng | 可视化构建需要；libpng 版本要求 1.6+。 |

Ubuntu / Debian 环境可安装以下构建依赖：

```bash
sudo apt update
sudo apt install -y build-essential cmake pkg-config \
  libyaml-cpp-dev libglfw3-dev libgl1-mesa-dev libpng-dev
```

CMake 优先使用仓库中的 `third_party/mujoco-3.4.0/`；该目录缺少 MuJoCo 共享库时，会尝试 `$HOME/.mujoco/mujoco-3.4.0/`。对应目录需要同时包含 `include/` 和 `lib/`。

### 2. 编译可视化版本

```bash
cmake --preset linux-release
cmake --build --preset linux-release --parallel
```

主要构建产物：

```text
build/linux-release/bin/rocos_mujoco_sim
build/linux-release/lib/librocos_mujoco.so
```

如果不使用预设，可直接配置构建目录：

```bash
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release \
  -DROCOS_MUJOCO_ENABLE_VISUALIZATION=ON
cmake --build build --parallel
```

这种方式生成的程序位于 `build/bin/rocos_mujoco_sim`。

### 3. 检查启动配置

编辑 [config/config.yaml](config/config.yaml)：

```yaml
model: ../models/talon/scene.xml
hardware_config: ../models/talon/hardware_config.yaml

ecat_id: 0
cycle_time_us: 1000.0
duration_sec: -1.0
visualize: true
```

- `model` 直接指定要加载的文件，不填写模型目录名。
- 模型和显式硬件配置的相对路径，以这个 YAML 文件所在目录为基准。
- `hardware_config` 可省略，默认读取模型文件同目录的 `hardware_config.yaml`。
- `duration_sec: -1` 持续运行；正值限定运行时长；`0` 仅初始化，但仍会创建共享内存等运行资源。

`cycle_time_us` 不会覆盖 MJCF 的 `option.timestep`。需要控制时间与物理时间一致时，应自行对齐两者，例如 1000 μs 对应 0.001 s。详细规则见 [启动配置规范](docs/config.md)。

### 4. 启动仿真

```bash
./build/linux-release/bin/rocos_mujoco_sim --config config/config.yaml
```

程序会加载模型和硬件描述、创建共享内存，并打开仿真窗口。未连接控制端或驱动未使能时，机器人保持位置；可以先查看模型、使用侧栏和调整视角。

省略 `--config` 时，程序读取**当前工作目录**下的 `config/config.yaml`。从其他目录启动时，建议显式传入配置文件的绝对路径。

## 无头运行

无头构建不需要 GLFW、OpenGL 和 libpng，适合没有图形桌面的 Linux 或 WSL 环境。MuJoCo、yaml-cpp 和 C++ 构建工具仍然需要。

```bash
cmake --preset linux-headless
cmake --build --preset linux-headless --parallel
```

将以下配置保存为 `config/headless.yaml`，也可在现有配置中将 `visualize` 改为 `false`：

```yaml
model: ../models/talon/scene.xml
ecat_id: 0
cycle_time_us: 1000.0
duration_sec: 5.0
visualize: false
```

```bash
./build/headless/bin/rocos_mujoco_sim --config config/headless.yaml
```

无头程序不能使用 `visualize: true`。仓库还提供以下构建预设：

| 预设 | 用途 | 构建目录 |
| --- | --- | --- |
| `linux-release` | 带可视化的 Release 构建 | `build/linux-release/` |
| `linux-debug` | 带可视化的 Debug 构建 | `build/linux-debug/` |
| `linux-headless` | 无头 Release 构建 | `build/headless/` |
| `wsl-headless` | WSL 无头 Release 构建 | `build/wsl-headless/` |

## 界面操作

### 侧栏

| 操作 | 功能 |
| --- | --- |
| `Tab` 或右上角按钮 | 隐藏/显示菜单。 |
| 拖动侧栏左边缘 | 调整侧栏整体宽度。 |
| 拖动侧栏内部竖向分隔线 | 调整名称列与数据列宽度。 |
| 点击栏目标题 | 展开/折叠栏目。 |
| 在侧栏上滚动 | 查看超出窗口高度的菜单内容。 |
| `Display → Joint` | 显示关节轴。 |
| `Display → Axis` | 显示模型 body 坐标系。 |
| `Display → Transparency` | 调整模型透明度；`0` 不额外透明，`1` 完全透明。 |
| `Joint position / velocity / torque` | 查看只读的实时关节数据；关节较多时可通过 `Joint page` 切换页。 |

旋转关节使用 **rad、rad/s、N·m**，滑动关节使用 **m、m/s、N**。多自由度关节按分量分行显示，其中球关节及自由关节的旋转位置显示为旋转向量。

### 视角与外力

以下操作在仿真画面内使用；侧栏会接管其区域内的鼠标交互。

| 操作 | 功能 |
| --- | --- |
| 左键拖动 | 普通模式下旋转视角。 |
| 右键拖动 | 平移视角。 |
| 中键拖动或滚轮 | 普通模式下缩放视角。 |
| 按住 `Shift` 操作视角 | 减小视角调整幅度。 |
| `Backspace` | 重置相机。 |
| `Ctrl` + 左键点击机器人 | 选取受力点并进入外力模式。 |
| 外力模式下左键拖动 | 改变外力方向。 |
| 外力模式下 `+` / `-` 或滚轮 | 调整外力大小。 |
| `Esc` | 清除当前外力。 |

## 连接 ROCOS 控制端

1. 先启动仿真器，确认日志中模型、硬件配置和共享内存初始化成功。
2. 控制端使用与仿真配置一致的 `ecat_id`，连接同一组共享内存对象。
3. 核对双方的从站 `id`、PDO 变量名称、计数倍率及偏移，并保证共享内存结构一致。
4. 由控制端按仿真 CiA 402 状态机完成驱动使能，再发送控制模式及目标值。

支持模式值 `8`（CSP）、`9`（CSV）和 `10`（CST）。模式 `6` 当前按位置控制处理，不执行真实回零搜索流程。

完整的字段、PDO 布局和换算公式见 [硬件配置规范](docs/hardware_config.md)。共享内存定义见 [shared_memory_config.hpp](include/rocos_mujoco/shared_memory_config.hpp)。

## 模型与配置文档

| 模型 | 说明 | 场景入口 | 硬件描述 |
| --- | --- | --- | --- |
| Talon | 7 轴旋转关节机械臂 | [scene.xml](models/talon/scene.xml) | [hardware_config.yaml](models/talon/hardware_config.yaml) |
| charge_arm | 1 个滑动关节与 6 个旋转关节 | [scene.xml](models/charge_arm/scene.xml) | [hardware_config.yaml](models/charge_arm/hardware_config.yaml) |

切换模型时，在启动 YAML 中填写对应文件路径，并选择与该模型匹配的硬件配置。自定义模型也可以放在仓库外，通过绝对路径加载。

| 文档 | 内容 |
| --- | --- |
| [config.yaml 配置规范](docs/config.md) | 启动字段、默认值、路径规则、周期与校验限制。 |
| [hardware_config.yaml 配置规范](docs/hardware_config.md) | 硬件类型、关节映射、PDO 名称、单位换算和排查说明。 |

## 当前实现范围

- 共享内存驱动控制路径面向单自由度旋转/滑动关节，依赖关节位置索引、自由度索引及执行器索引的对应关系。可视化能够展示多自由度关节，不代表控制映射支持所有模型布局。
- `ft_sensor` 和 `io` 已支持配置与 PDO 布局，但反馈目前固定为零，尚未接入实际测量或输出行为。
- 硬件 YAML 中的 `limit`、`torque_source` 和 `user_unit_name` 目前不参与仿真控制或单位转换；具体行为以配置规范为准。
- 循环使用软件定时，控制周期与 MuJoCo 时间步长需分别配置；不提供硬实时周期保证。

## 仓库结构

```text
rocos-mujoco/
├── config/                 # 启动配置
├── docs/                   # 配置规范
├── include/rocos_mujoco/   # 仿真接口与共享内存定义
├── models/                 # 模型、场景、网格与硬件描述
├── src/                    # 入口、仿真控制、渲染与原生菜单
├── third_party/            # MuJoCo 依赖
├── CMakeLists.txt
├── CMakePresets.json
├── rocos-mujoco.png
└── VERSION
```

## 常见问题

| 现象 | 检查方式 |
| --- | --- |
| 找不到启动配置 | 从仓库根目录运行，或使用 `--config` 指定绝对路径。 |
| 模型文件找不到或加载失败 | `model` 应为文件路径；检查相对于 YAML 的路径，以及模型引用的网格、纹理和 include 文件。 |
| 找不到 GLFW、OpenGL 或 PNG | 安装可视化开发依赖，或选择无头构建并设置 `visualize: false`。 |
| GLFW 窗口无法创建 | 确认运行环境可访问图形显示服务；没有桌面的环境使用无头模式。 |
| MuJoCo 库找不到或无法加载 | 检查上述依赖目录内的头文件和共享库，并确认共享库与运行平台匹配。 |
| 机器人不响应控制命令 | 检查主站标识、PDO 名称、关节映射、驱动使能状态和控制模式。 |

## 许可

本项目源码标注为 `GPL-3.0-or-later`。第三方组件及模型资源的许可以各自声明为准；MuJoCo 随附的第三方声明见 [THIRD_PARTY_NOTICES](third_party/mujoco-3.4.0/THIRD_PARTY_NOTICES)。
