# 双臂 MuJoCo 仿真

一个固定基座、左右各 7 轴、一个物理世界，向两个 ROCOS 控制端提供独立的 ABI v2 主站接口。

| 主站 | 硬件配置 | 从站 ID | 模型关节 |
| --- | --- | --- | --- |
| 左臂 `ecat_id=0` | `hardware_left.yaml` | `0～6` | `Left_Joint1～7` |
| 右臂 `ecat_id=1` | `hardware_right.yaml` | `0～6` | `Right_Joint1～7` |

## 肩部基坐标系试验

可选的 `left_base` / `right_base` 版本见 [试验启动、转换与回退说明](SHOULDER_TRIAL.md)。原配置继续使用公共 `base_link`。

## 启动

仓库根目录执行：

```bash
cmake --preset linux-release
cmake --build --preset linux-release --parallel
./build/linux-release/bin/rocos_mujoco_sim --config config/dualarm/mjcf/mujoco.yaml
```

无头运行：

```bash
cmake --preset linux-headless
cmake --build --preset linux-headless --parallel
./build/headless/bin/rocos_mujoco_sim --config config/dualarm/mjcf/headless.yaml
```

两个配置默认持续运行。启动前停止占用主站 `0` 或 `1` 的其他仿真器/主站；任一个 ID 已被占用时，本进程初始化失败。测试程序使用独立的大整数 ID。

先启动仿真器，再分别启动两个控制端。左臂控制端选择 `ecat_id=0` 并读取 `hardware_left.yaml`；右臂选择 `ecat_id=1` 并读取 `hardware_right.yaml`。每个控制端只看到 7 个从站，先选择模式 `8`、将当前反馈同步为目标位置，再按控制字 `6 → 7 → 15` 完成使能，随后发送运动目标。发送新控制字后应等待相应状态字，再继续下一步。

### 两个 App 进程

`app/` 提供两个独立启动入口，直接使用现有 rocos-app 二进制，无需修改 App 代码或其仓库配置。假设两个仓库为同级目录 `rocos-mujoco/` 和 `rocos-app/`；若目录不同，调整 `app/left.yaml`、`app/right.yaml` 中 scripts/web_root/tool_library 的资源路径。

| 入口 | 运动学链 | 主站 | HTTP | 诊断 |
| --- | --- | --- | --- | --- |
| `app/left.yaml` | `base_link → Left_Link7` | 0 | 127.0.0.1:8080 | 127.0.0.1:8082 |
| `app/right.yaml` | `base_link → Right_Link7` | 1 | 127.0.0.1:8090 | 127.0.0.1:8092 |

两份 robot 配置读取同一份双臂 URDF、分别截取各自链；硬件配置直接引用上表的 `hardware_left.yaml` / `hardware_right.yaml`，使控制端与仿真端共用换算参数。两份 robot 配置均选择 `master_type: mujoco`，对应的 `master_*.yaml` 设置周期 1000 μs、主站 ID 与 `ecat_id` 对齐；设备 ID 分别为 `dualarm_sim_left` / `dualarm_sim_right`，避免运行快照覆盖。当前无末端工具，TCP 等于各自链末端；场景中的基座安装高度不改变 App 的基坐标定义。

先启动上面的仿真器，再在两个终端分别执行（工作目录均为本仓库根目录）：

```bash
../rocos-app/build/bin/rocosAppMain --config=config/dualarm/app/left.yaml
```

```bash
../rocos-app/build/bin/rocosAppMain --config=config/dualarm/app/right.yaml
```

各端分别获取 HTTP 控制权、使能，再向各自端口发送 MoveJ。退出前分别下使能并确认 IDLE，再停止 App，最后停止仿真。两个 App 各自规划与控制，本入口不提供严格同步启动。

## 模型与限位来源

- `urdf/dualarm.urdf` 保留 SolidWorks 导出的连杆层级、关节轴、质量和惯量；网格路径改为相对 `../meshes/`。
- `mjcf/dualarm.xml` 使用同一套结构和完整惯量张量，添加 14 个位置执行器。左臂为蓝色，右臂为橙色。
- `mjcf/scene.xml` 添加地面、灯光和相机。固定基座平移至 `z=0.5890511 m`，使底部距地面约 1 mm；这是场景安装偏移，URDF 的机器人基坐标不变。
- 每个关节的上下限、速度、力矩上限复制 Talon 对应轴，左右臂使用相同符号；加速度也采用 Talon 值，全部 `jerk=10000 rad/s³`。这些是用户指定的仿真参数，并非双臂硬件标定值。
- 仿真周期为 `1000 μs`，MJCF 物理步长为 `0.001 s`。

| 轴（左右相同） | 下限 rad | 上限 rad | 速度 rad/s | 加速度 rad/s² | jerk rad/s³ | effort N·m |
| --- | --- | --- | --- | --- | --- | --- |
| 1 | -2.84 | 2.84 | 3 | 10 | 10000 | 30 |
| 2 | -2.22 | 2.22 | 3 | 10 | 10000 | 30 |
| 3 | -2.84 | 2.84 | 3 | 10 | 10000 | 30 |
| 4 | -2.23 | 2.23 | 3 | 10 | 10000 | 30 |
| 5 | -2.71 | 2.71 | 3 | 10 | 10000 | 30 |
| 6 | -2.11 | 2.11 | 3 | 10 | 10000 | 30 |
| 7 | -2.84 | 3.14 | 3 | 10 | 10000 | 30 |

URDF 标准 `limit` 只写入 `lower/upper/velocity/effort`，`acc/jerk` 保存在硬件 YAML。模型关节范围和执行器目标范围与上述限位一致。执行器采用与 talon 一致的 `kp=20000`、`kv=500`，不设置执行器力矩限幅。速度、加速度和 jerk 应由控制端轨迹规划器落实，仿真器不会读取这些字段来生成轨迹。

## 换算与碰撞边界

两份硬件文件明确使用临时仿真倍率：`pos_per_unit=1000000`、`vel_per_unit=1000000`、`ratio=1`、`offset_pos_cnt=0`、`torque_per_unit=100`、`load_torque_per_unit=100`。位置 1 rad 对应 1000000 计数，力矩 1 N·m 对应原始值 100。控制端必须使用相同参数；以后接入真实硬件参数时应同步修改两端。

视觉网格位于 group 2，碰撞网格位于 group 3。碰撞使用 STL 的凸包近似，启用双臂之间、非相邻连杆和地面的接触。仅显式排除基座与左右第一连杆这两对装配重叠；MuJoCo 还会执行自身的相邻体过滤。初始姿态无接触穿透，但凸包不能精确表达所有凹槽，后续大范围运动需要结合具体任务检查碰撞几何。

未使能的关节采用理想抱闸：在下使能边沿锁存实际位置、清零速度，并在物理步进后投影回锁定位置。启动未使能、故障/Quick Stop 和未知控制模式也保持锁定，不因重力或外力下垂。这是运动学抱闸近似，不模拟抱闸柔性、制动距离或真实制动力矩；不能用它验证急停距离或接触反力。

双臂位置执行器与 talon 对齐为 `kp=20000`、`kv=500`，不设置 `forcerange`，包括生成的肩部版本。URDF / hardware YAML 的 `effort=30` 保留 talon 额定配置含义，不等于 MuJoCo 的执行器限幅；CSP 重力前馈仍独立施加。

App 位置模式在每次使能前重置控制器指令基线、校验当前反馈、写入当前位置并建立新的 MoveHold；准备失败则不使能。动作完成进入 STOPPED 时仍保留最终运动目标。仿真器不改写控制端拥有的输出 PDO；其他控制端也必须在使能前同步目标位置，不能依赖驱动忽略旧目标。

ABI v2 没有控制端在线心跳，客户端退出后仿真器不能自动识别断连并清除旧 PDO 命令；应先下使能，再停止控制端。异常退出按现有约定停止并重启双方，不支持热接管。

## 验证

```bash
cmake --preset linux-headless -DBUILD_TESTING=ON -DROCOS_SHM_BUILD_TESTS=ON
cmake --build --preset linux-headless --parallel
ctest --test-dir build/headless --output-on-failure
```

双臂测试覆盖 14 轴 CSP 跟踪、单臂先连接、另一臂未使能时保持、外力下抱闸、运动中停用锁存、重复使能目标衔接、独立停用、双邮箱隔离、独立统计复位、每周期只步进一次、执行器乱序映射、重复关节/主站拒绝和 IPC 释放。共享内存周期通知不提供整帧原子快照保证。


### 实际 App 联调

在没有其他主站 0/1 和上述端口占用时运行（脚本自行启动三个进程，无需手动启动）：

```bash
python3 scripts/test_dualarm_apps.py
# 可指定现有 App 二进制：
python3 scripts/test_dualarm_apps.py --app ../rocos-app/build/bin/rocosAppMain
```

脚本仅依赖 Python 标准库，按 ROCOS 在线 OpenAPI 的控制权和 MoveJ 接口操作。先检查两份配置，再同时启动两个 App，验证独立使能、左右各 7 轴小幅 MoveJ、实际位置与速度到位、另一臂保持、独立下使能。MoveJ 使用 `velocity=0.1`、`acceleration=0.3`、`jerk=5`；这些保守测试值不改变配置中的 `jerk=10000` 上限。

通过标准为到位误差 <0.003 rad、停止时各轴速度 <0.01 rad/s、另一臂相对基线最大漂移 <0.005 rad。报告及进程日志保存在 `build/dualarm-app/test/`。脚本收尾下使能、释放控制权、停止两个 App，并等待 45 秒仿真时长结束，验证 IPC 清理。该联调不加入默认 CTest，以免自动占用主站 0/1。

普通 Linux 调度下也可运行；若 App 日志提示无法取得 SCHED_RR 权限，本测试结果仅证明功能联通与小幅运动，不证明硬实时周期或双臂同步性能。

### 抱闸与反复使能回归

先构建本仓库 `build/headless` 和 App 的 `build/dualarm/bin/rocosDualArmMain`，再执行：

```bash
python3 scripts/test_dualarm_hold.py
```

测试自行分配独立大整数主站 ID 和临时 HTTP/诊断端口，生成独立配置，启动并回收测试进程，不访问部署中的主站 0/1。覆盖初始抱闸、三轮使能/并发独立 MoveJ/下使能，以及左臂抱闸时右臂运动。报告与日志保存在 `build/hold-regression/`。

### 左肩 J1 旋转轴修正

左臂 CAD 导出的 `Left_Joint1 axis="0.99861 -0.035359 0.039057"` 与肩部安装圆盘法线相差约 90°，会导致 J1 点动整体抬臂。根据 `Left_Link1.STL` 安装面的面积加权法线，修正为 `axis="-0.039081 0 0.99924"`。URDF、MJCF 及肩部派生模型同步使用此轴；零位装配、惯量、J2～J7、右臂与求解基坐标方向不变。左右 J1 正向旋转轴在公共基座中均近似 +Y。

页面“肩部基座”的蓝色 Z 轴仍表示求解参考系，不代表 J1 转轴。该修正改变左臂非零 J1 时的 FK、IK 和动力学结果，旧模型计算的左臂轨迹及示教位姿需重新核对；不能仅靠旧/新肩部原点的平移转换修复。重启 MuJoCo、App，并刷新上位机加载新模型后生效。
