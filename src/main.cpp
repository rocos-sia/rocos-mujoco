// Copyright 2026, Yang Luo
// SPDX-License-Identifier: GPL-3.0-or-later
//
// main.cpp
// rocos-mujoco 仿真器入口点。
// 以独立进程运行 MuJoCo 仿真，通过 POSIX 共享内存模拟 EtherCAT 主站，
// 供 rocos-app 的 rocos::Hardware 连接 —— 与真实硬件完全一致的接口。

#include <cstdlib>
#include <cstring>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <string>

#include "rocos_mujoco/mujoco_simulator.hpp"

/// @brief 解析文件路径 —— 依次尝试多个目录前缀，适配不同工作目录
/// 覆盖以下运行场景：
///   - 从 rocos-mujoco/          运行 → model/xxx
///   - 从 rocos-mujoco/build/     运行 → ../model/xxx
///   - 从 rocos-app/             运行 → rocos-mujoco/model/xxx
///   - 从 rocos-app/build/       运行 → ../rocos-mujoco/model/xxx
static std::string resolveFile(const std::string &name) {
    if (name.front() == '/') return name;  // 已是绝对路径
    std::ifstream t;
    // 按优先级尝试每种前缀
    for (const char* prefix : {"", "../", "rocos-mujoco/", "../rocos-mujoco/"}) {
        std::string candidate = std::string(prefix) + name;
        t.open(candidate);
        if (t.good()) return candidate;
        t.close();
    }
    return name;  // 未能解析，返回原始名称让调用方处理
}

static void printUsage(const char *prog) {
    std::cout << "用法: " << prog << " [选项]\n"
              << "选项:\n"
              << "  --model <name>       预置模型选择: charge_arm | talon\n"
              << "                        (自动设置 --urdf 和 --config)\n"
              << "  --urdf <path>         MJCF/URDF 模型路径\n"
              << "                         (默认: model/charge_arm.xml)\n"
              << "  --config <path>       硬件描述 YAML 路径\n"
              << "                         (默认: config/charge_arm_config.yaml)\n"
              << "  --ecat-id <n>         EtherCAT 主站 ID (默认: 0)\n"
              << "  --cycle-time <us>     控制周期 [微秒] (默认: 1000)\n"
              << "  --duration <sec>      运行时长 [秒] (默认: -1 = 永久)\n"
              << "  --visualize, -v       启用 3D 可视化窗口\n"
              << "  --headless            显式禁用可视化（默认行为，无需指定）\n"
              << "  --help                显示此帮助信息\n"
              << "\n"
              << "共享内存命名规则 (ecat_id=0):\n"
              << "  EcatBus:  /ecm0\n"
              << "  PD 输入:  /pd_input0\n"
              << "  PD 输出:  /pd_output0\n"
              << "  信号量:   /sync0_0 ... /sync0_9\n"
              << "\n"
              << "典型用法:\n"
              << "  # 启动 charge_arm 仿真（可视化）\n"
              << "  " << prog << " --model charge_arm -v\n"
              << "\n"
              << "  # 启动 talon 仿真（无头模式，运行 30 秒 — 用于测试）\n"
              << "  " << prog << " --model talon --duration 30\n"
              << "\n"
              << "  # 然后在另一个终端运行测试:\n"
              << "  ./build/bin/hardware_enable_test 0 charge_arm_config.yaml\n"
              << std::endl;
}

int main(int argc, char *argv[]) {
    // 默认使用 charge_arm 模型（与 RoboChargeX 保持一致）
    std::string urdf_path   = "model/charge_arm.xml";
    std::string config_path = "config/charge_arm_config.yaml";
    int         ecat_id     = 0;
    double      cycle_time  = 1000.0;   // 微秒
    double      duration    = -1.0;     // 秒，-1 = 永久
    bool        visualize   = false;

    // ---- 解析命令行参数 --------------------------------------------------
    for (int i = 1; i < argc; ++i) {
        std::string arg = argv[i];
        if (arg == "--help" || arg == "-h") {
            printUsage(argv[0]);
            return 0;
        } else if (arg == "--model" && i + 1 < argc) {
            std::string model = argv[++i];
            if (model == "charge_arm") {
                urdf_path   = "model/charge_arm.xml";
                config_path = "config/charge_arm_config.yaml";
            } else if (model == "talon") {
                urdf_path   = "model/scene.xml";
                config_path = "config/hardware_talon_config.yaml";
            } else {
                std::cerr << "未知模型: " << model
                          << " (可选: charge_arm, talon)" << std::endl;
                return 1;
            }
        } else if (arg == "--visualize" || arg == "-v") {
            visualize = true;
        } else if (arg == "--headless") {
            visualize = false;
        } else if (arg == "--urdf" && i + 1 < argc) {
            urdf_path = argv[++i];
        } else if (arg == "--config" && i + 1 < argc) {
            config_path = argv[++i];
        } else if (arg == "--ecat-id" && i + 1 < argc) {
            ecat_id = std::atoi(argv[++i]);
        } else if (arg == "--cycle-time" && i + 1 < argc) {
            cycle_time = std::atof(argv[++i]);
        } else if (arg == "--duration" && i + 1 < argc) {
            duration = std::atof(argv[++i]);
        } else {
            std::cerr << "未知参数: " << arg << std::endl;
            printUsage(argv[0]);
            return 1;
        }
    }

    // ---- 路径解析（尝试多个目录前缀以适配不同工作目录）------------------
    // urdf_path   = resolveFile(urdf_path);
    // config_path = resolveFile(config_path);

    // ---- 打印启动信息 ----------------------------------------------------
    std::cout << "╔══════════════════════════════════════════════════════════════╗\n";
    std::cout << "║  ROCOS MuJoCo 仿真器 (EtherCAT 主站模拟)                    ║\n";
    std::cout << "╠══════════════════════════════════════════════════════════════╣\n";
    std::cout << "║  模型:      " << std::setw(49) << std::left << urdf_path   << "║\n";
    std::cout << "║  配置:      " << std::setw(49) << std::left << config_path << "║\n";
    std::cout << "║  ecat_id:   " << std::setw(49) << std::left << ecat_id     << "║\n";
    std::cout << "║  周期:      " << std::setw(46) << std::left
              << (std::to_string(static_cast<int>(cycle_time)) + " us") << "║\n";
    std::cout << "║  可视化:    " << std::setw(49) << std::left
              << (visualize ? "开启" : "关闭 (无头模式)") << "║\n";
    std::cout << "╚══════════════════════════════════════════════════════════════╝\n\n";

    // ---- 创建并运行仿真器 ------------------------------------------------
    rocos_mujoco::MujocoSimulator sim(urdf_path, config_path,
                                       ecat_id, cycle_time);
    if (visualize) {
        sim.enableVisualization();
    }

    if (!sim.initialize()) {
        std::cerr << "仿真器初始化失败。" << std::endl;
        return 1;
    }

    sim.run(duration);

    std::cout << "仿真已结束。" << std::endl;
    return 0;
}
