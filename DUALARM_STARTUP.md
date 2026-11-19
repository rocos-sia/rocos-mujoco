# 双臂系统命令行启动说明

按 **MuJoCo → App → Viz** 的顺序启动，分别使用三个终端。以下使用双臂独立入口，保留原有单臂入口。

> 当前双臂 Viz 的运动操作仍是页面内预览，尚未接入真实 App 运动控制；日志已支持连接真实 App。因此，启动三个服务后，页面上的运动按钮还不能驱动 MuJoCo 中的机械臂。

## 目录约定

三个仓库应位于同一个父目录，配置及模型资源通过相对路径引用：

```text
/home/rocos/Documents/GitHub/
├── rocos-mujoco/
├── rocos-app/
└── rocos-viz/
```

以下命令使用这台机器的目录。首次启动或代码更新后先编译；没有代码或依赖变化时，可以直接执行启动命令。

## 1. rocos-mujoco：双臂仿真

### 编译图形版

```bash
cd /home/rocos/Documents/GitHub/rocos-mujoco

cmake --preset linux-release
cmake --build --preset linux-release --parallel 2
```

### 启动图形版

需要在可用的图形桌面环境中运行：

```bash
./build/linux-release/bin/rocos_mujoco_sim \
  --config config/dualarm/mjcf/mujoco.yaml
```

### 无窗口运行（替代方式）

```bash
cd /home/rocos/Documents/GitHub/rocos-mujoco

cmake --preset linux-headless
cmake --build --preset linux-headless --parallel 2

./build/headless/bin/rocos_mujoco_sim \
  --config config/dualarm/mjcf/headless.yaml
```

图形版和无窗口版选择一种运行。启动前先退出占用相同主站的旧仿真实例。

一个仿真场景包含两条七轴机械臂：左臂使用 `ecat_id: 0`，右臂使用 `ecat_id: 1`。MuJoCo 通过共享内存提供主站，不使用 HTTP 端口。

## 2. rocos-app：一个进程的双臂控制器

### 编译独立目标

```bash
cd /home/rocos/Documents/GitHub/rocos-app

cmake -S . -B build/dualarm \
  -DCMAKE_BUILD_TYPE=Release \
  -DROCOS_BUILD_DUALARM=ON \
  -DCMAKE_PREFIX_PATH="$PWD/build/deps/prefix"

cmake --build build/dualarm \
  --target rocosDualArmMain --parallel 2
```

`build/deps/prefix` 是当前机器已有依赖的安装目录；在其他机器上，需要先准备项目依赖，并按实际安装位置配置 `CMAKE_PREFIX_PATH`。

### 检查配置

此命令只检查配置，不创建 Robot 或连接主站：

```bash
./build/dualarm/bin/rocosDualArmMain \
  --config dualarm/config/app.yaml \
  --check_config
```

### 启动控制器

确认 MuJoCo 已运行，再执行：

```bash
./build/dualarm/bin/rocosDualArmMain \
  --config dualarm/config/app.yaml
```

启动成功后会输出类似：

```text
Two Robots ready; HTTP 127.0.0.1:8080; diagnostics 127.0.0.1:8082
```

一个进程创建两个 Robot，左右臂使用独立的控制路径。不要同时运行连接相同主站的旧单臂 App 进程。

### 读取两臂状态

另开终端执行，以下请求不会触发运动：

```bash
curl -s http://127.0.0.1:8080/api/arms

curl -s http://127.0.0.1:8080/api/arms/left/robot/state

curl -s http://127.0.0.1:8080/api/arms/right/robot/state
```

### 配置与日志位置

- 双臂入口配置：`dualarm/config/app.yaml`
- 左臂配置：`dualarm/config/left.yaml`
- 右臂配置：`dualarm/config/right.yaml`
- 日志目录：`build/dualarm/logs/`

配置中的相对路径以声明它的 YAML 文件所在目录为基准解析。

## 3. rocos-viz：双臂页面

Node.js 版本要求：`^20.19.0 || >=22.12.0`。

### 安装依赖

首次安装或依赖变化时执行：

```bash
cd /home/rocos/Documents/GitHub/rocos-viz
npm ci
```

### 启动双臂开发预览

```bash
cd /home/rocos/Documents/GitHub/rocos-viz

npx vite \
  --config design/dualarm/v2/vite.config.mjs \
  --host 127.0.0.1 \
  --port 5186 \
  --strictPort
```

浏览器打开：

- [双臂页面](http://127.0.0.1:5186)
- [日志页面](http://127.0.0.1:5186/#logs)

在日志页面填写诊断地址 `http://127.0.0.1:8082`，然后点击“连接日志”。日志连接读取真实 App 的诊断数据，不会获取运动控制权。

如果 `5186` 已有预览服务，直接打开页面即可；需要重启时先结束原服务。`--strictPort` 会在端口被占用时报错，避免自动切换到其他端口。

这里使用独立的双臂 Vite 配置；普通 `npm run dev` 是原有单臂入口。模型资源来自同级 `rocos-mujoco` 仓库。

### 构建后预览（替代方式）

如需验证构建产物，先结束开发预览服务，再执行：

```bash
cd /home/rocos/Documents/GitHub/rocos-viz

npx vite build --config design/dualarm/v2/vite.config.mjs

npx vite preview \
  --config design/dualarm/v2/vite.config.mjs \
  --host 127.0.0.1 \
  --port 5186 \
  --strictPort
```

构建产物位于 `design/dualarm/v2/dist/`。开发预览与构建后预览选择一种运行。

## 服务地址

| 服务 | 地址或标识 |
| --- | --- |
| MuJoCo 左臂主站 | `ecat_id: 0`，共享内存 |
| MuJoCo 右臂主站 | `ecat_id: 1`，共享内存 |
| App 双臂 HTTP API | `127.0.0.1:8080` |
| App 诊断与日志 | `127.0.0.1:8082` |
| Viz 双臂页面 | `127.0.0.1:5186` |

## 退出顺序

1. 先停止机械臂运动。
2. 在 App 终端按 `Ctrl+C`，等待控制器退出。
3. 再在 MuJoCo 终端按 `Ctrl+C`，关闭仿真。
4. Viz 服务可独立通过 `Ctrl+C` 关闭。

保持 App 先于 MuJoCo 退出，让控制器在主站仍可用时完成关闭流程。

## 各仓库详细说明

- [MuJoCo 双臂配置说明](config/dualarm/README.md)
- [App 双臂控制器说明](../rocos-app/dualarm/README.md)
- [Viz 双臂页面说明](../rocos-viz/design/dualarm/v2/README.md)

## 肩部基坐标系试验入口

另有独立的肩部求解基坐标系配置，详见 [试验启动、坐标转换与回退](config/dualarm/SHOULDER_TRIAL.md)。原命令保持不变。


## 双臂上位机实时连接（2026-11-18）

5186 页面现在默认连接真实 App；使用上述独立 Vite 配置构建并启动，`/api` 自动代理到本机 8080，日志默认连接 8082。先启动 MuJoCo 和双臂 App，再打开 <http://127.0.0.1:5186/>，分别申请左/右臂控制权、使能后按住点动，松开停止。

模型、左右求解基座、使能、状态机和位姿以 App 返回值为准；无需通过 URL 选择实际基座。请求失败和运行错误显示在对应臂区域，日志详情仍在诊断页。已申请成功的控制权会保存在当前标签页的 sessionStorage；刷新后验证原凭据并恢复。短暂断线保留凭据，释放或 App 明确判定凭据失效时清除。

原本页面演示入口改为 <http://127.0.0.1:5186/?mode=demo&base=shoulder>。精确运动、工具标定、坐标和负载的页面写操作尚未接入，在实时模式禁用；不能把演示成功当作 App 执行成功。详细边界见 rocos-viz/design/dualarm/v2/README.md。


### 使用 sudo 启动 App 并验证实时调度

先正常停止旧 App，确认其退出，再从 rocos-app 根目录运行（密码由 sudo 交互提示输入）：

```bash
sudo ./build/dualarm/bin/rocosDualArmMain --config dualarm/config/app_shoulder.yaml
```

使能后，控制线程应记录 `Control scheduling: requested=SCHED_RR/20 actual=SCHED_RR/20 set_error=0`；watchdog 对应优先级 21。可用 `ps -L -p <App_PID> -o tid,cls,rtprio,comm` 检查 `rocos_control` 与 `rocos_watchdog`。

本次运行已验证两臂线程为 RR/20 和 RR/21；输出日志为 `rocos-app/build/dualarm-shoulder/run/app-sudo.log`。重启 App 会清除服务端控制租约，页面收到凭据失效后可重新申请；不提供强制夺取其他活跃会话的接口。
