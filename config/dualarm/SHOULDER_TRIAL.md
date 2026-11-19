# 肩部基坐标系试验版 v1

这是独立配置，不覆盖现有 `dualarm.urdf`、`scene.xml`、App `app.yaml`。
第一版只改变求解原点，不旋转求解基坐标轴。机器人实体、网格、惯量、关节顺序、零位、正方向、限位、硬件倍率和主站 ID 保持一致。

## 坐标定义

| 臂 | App 求解链 | 新原点在旧 `base_link` 中的位置，m | 新轴方向 |
| --- | --- | --- | --- |
| 左臂 | `left_base → Left_Link7` | `(0.011578, -0.063, 0.091501)` | 与旧 `base_link` 平行 |
| 右臂 | `right_base → Right_Link7` | `(0.011578, -0.267, 0.091501)` | 与旧 `base_link` 平行 |

原点取自现有 URDF 的 J1 安装位置，尚未进行实体安装标定。`left_base` / `right_base` 是 J1 **之前**的固定 link，不是会随 J1 转动的 `Left_Link1` / `Right_Link1`。两条链分别保留 J1…J7。

原有 J1 的平移移入固定 mount，J1 的 rpy 和 axis 保持原值。因此，新基座 Z 轴不代表 J1 转轴。重力向量仍为 `(0,0,-9.81)`，BASE 点动方向不变。

公共 `base_link` 仍是旧 CAD 原点，本版不把它移动到底板，也不将 MuJoCo 世界原点当作 App 基坐标。MuJoCo 原有 `z=0.5890511 m` 场景安装平移仍保留。

机器可读安装关系：[shoulder_frames.json](shoulder_frames.json)。

## 启动与回退

三个仓库保持同级。先退出原 App，再停止原仿真；试验版使用同样的主站 0/1 与端口 8080/8082，不能与原版同时控制这些主站。

终端 1，仿真（无窗口）：

```bash
cd /home/rocos/Documents/GitHub/rocos-mujoco
cmake --preset linux-headless
cmake --build --preset linux-headless --parallel 2
./build/headless/bin/rocos_mujoco_sim --config config/dualarm/mjcf/headless_shoulder.yaml
```

图形窗口替代命令（先按原启动文档构建 linux-release）：

```bash
./build/linux-release/bin/rocos_mujoco_sim --config config/dualarm/mjcf/mujoco_shoulder.yaml
```

终端 2，独立双臂 App：

```bash
cd /home/rocos/Documents/GitHub/rocos-app
cmake -S . -B build/dualarm -DCMAKE_BUILD_TYPE=Release \
  -DROCOS_BUILD_DUALARM=ON -DCMAKE_PREFIX_PATH="$PWD/build/deps/prefix"
cmake --build build/dualarm --target rocosDualArmMain --parallel 2
./build/dualarm/bin/rocosDualArmMain --config dualarm/config/app_shoulder.yaml --check_config
./build/dualarm/bin/rocosDualArmMain --config dualarm/config/app_shoulder.yaml
```

查询 `GET /api/arms` 或 `GET /api/workcell/model`，确认左右 `base_link` 分别是 `left_base` / `right_base`，`joint_count` 为 7。不要仅根据端口判断当前坐标方案。

终端 3，Viz：

```bash
cd /home/rocos/Documents/GitHub/rocos-viz
npx vite --config design/dualarm/v2/vite.config.mjs --host 127.0.0.1 --port 5186 --strictPort
```

- [肩部试验页面](http://127.0.0.1:5186/?base=shoulder)
- [原公共基座页面](http://127.0.0.1:5186/)

页面仍是运动交互预览，只有日志可接真实 App。试验页面加载试验 URDF，基坐标轴分别显示在左右肩部；位姿反馈、工件坐标和位姿示例目标使用对应肩部参考系。工具 TCP 仍相对法兰定义。

若运行的是 `vite preview`，代码更新后先执行 `npx vite build --config design/dualarm/v2/vite.config.mjs`，再刷新页面。

回退：退出试验 App，再退出仿真；改回原 `mjcf/headless.yaml` 或 `mjcf/mujoco.yaml`、App `dualarm/config/app.yaml`，并打开不含 `?base=shoulder` 的页面。

## 目标、反馈和旧数据

令 `t` 为上表中当前臂的偏移，因两套基轴方向相同：

```text
p_shoulder = p_legacy - t
R_shoulder = R_legacy
p_legacy = p_shoulder + t
```

例如，原公共基座下位置 `(0.4, 0, 0.35) m`：

- 左臂新位置 `(0.388422, 0.063, 0.258499) m`。
- 右臂新位置 `(0.388422, 0.267, 0.258499) m`。

方向四元数不变。即使位置数字相同，左右肩部坐标下也不是同一个物理目标。

接口路径及请求格式不变，`BASE` / `reference_frame: base` 指当前臂配置中的求解基坐标：

- `MoveJ` / `MoveL_FK` 的关节目标保持原值。
- `MoveJ_IK`：当前核心直接求解**法兰**目标，须输入当前臂基坐标中的法兰位姿；不能把非零工具 TCP 目标直接当法兰目标。
- `MoveL`：输入所选工具的目标位姿，仍表达在当前臂基坐标中；工具变换由现有运动接口处理。
- 法兰/TCP 反馈、工件坐标、基坐标中的笛卡尔示教点必须转换。
- 法兰相对工具 TCP、工具包安装位姿和法兰相对负载质心不做上述平移。
- 不复用旧标定采样会话或未注明参考系的位姿文件。试验配置使用独立设备目录和 `build/dualarm-shoulder/{tools,logs}`，不会自动迁移已有生产记录。
- API 不接受 `WORLD` 作为新增目标参考系，也不自动识别旧坐标数据。应用层必须先转换，再调用相应接口。

可转换单个 SI 位姿 JSON 文件（position.xyz，orientation.xyzw），不发运动命令、不覆盖已有输出：

```bash
cd /home/rocos/Documents/GitHub/rocos-mujoco
python3 scripts/convert_dualarm_pose.py --arm left --to shoulder \
  --input old-pose.json --output left-shoulder-pose.json
python3 scripts/convert_dualarm_pose.py --arm left --to legacy \
  --input left-shoulder-pose.json --output restored-pose.json
```

## 生成与验证

派生模型由原模型生成，勿手工分别维护两套几何：

```bash
python3 scripts/generate_dualarm_shoulders.py
python3 scripts/generate_dualarm_shoulders.py --check
ctest --test-dir build/headless --output-on-failure
```

App 离线模型回归（不连接主站）：

```bash
cd /home/rocos/Documents/GitHub/rocos-app
cmake --build build/dualarm --target dualarm_shoulder_model_test --parallel 2
ctest --test-dir build/dualarm -R '^dualarm_' --output-on-failure
```

实际仿真联调（脚本自行启动仿真和 App；需主站 0/1 与端口空闲）：

```bash
python3 dualarm/tests/integration_mujoco.py --shoulder \
  --output build/dualarm-shoulder/integration
```

验证包括：24 组仿真姿态的几何/碰撞/动力学一致性；新模型双主站运动和隔离；App 两臂各 12 组 FK 坐标转换、非相同初值 IK 回验、J1 有效性和动力学一致性。冗余臂逆解可能得到不同关节解，应比较最终位姿而不是强求关节解相同。

## 本次验证结果

- MuJoCo 原有测试及新增模型几何、双主站试验测试通过；位姿转换工具的往返、非法输入和不覆盖文件检查通过。
- App 双臂离线测试与原 `model_test` / `model_reference_test` 回归通过。
- 实际 App＋MuJoCo 扩展联调共 29 项记录通过，包含两臂 MoveJ_IK、MoveL、MoveL_FK、四类点动、独立停止、并发控制、日志与退出清理。
- 浏览器确认新旧位姿转换一致、J1 点动时肩部坐标轴固定、左右反馈隔离、试验模型下载和参考系标签正确。
- Viz 单臂构建、双臂构建、控制与脚本 17 项回归以及双臂坐标/日志 7 项测试通过。

完整仿真联调报告位于同级 App 的 `build/dualarm-shoulder/integration/report.json`。这些结果证明本次坐标改造的功能一致性，不代表实际机械臂安装标定或全部工作空间的逆解覆盖。
