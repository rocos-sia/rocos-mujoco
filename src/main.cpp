// Copyright 2026, Yang Luo
// SPDX-License-Identifier: GPL-3.0-or-later
//
// main.cpp
// rocos-mujoco 仿真器入口点。
// 以独立进程运行 MuJoCo 仿真，通过 POSIX 共享内存模拟 EtherCAT 主站，
// 供 rocos-app 的 rocos::Hardware 连接 —— 与真实硬件完全一致的接口。

#include <cmath>
#include <cstdint>
#include <filesystem>
#include <iomanip>
#include <iostream>
#include <limits>
#include <stdexcept>
#include <string>

#include <yaml-cpp/yaml.h>

#include "rocos_mujoco/mujoco_simulator.hpp"

namespace fs = std::filesystem;

template <typename T>
static T readOption(const YAML::Node &settings, const char *key, T fallback) {
    return settings[key] ? settings[key].as<T>() : fallback;
}

static void printUsage(const char *prog) {
    std::cout << "用法: " << prog << " [--config <启动配置.yaml>]\n"
              << "  默认读取工作目录的 config/config.yaml。\n"
              << "  模型、硬件配置、主站 ID、周期、时长和可视化均在 YAML 中设置。\n"
              << "  model 直接指定 MJCF/URDF 文件路径；硬件配置默认为模型同目录的 hardware_config.yaml。\n"
              << "  YAML 内的相对路径以配置文件所在目录为基准。\n"
              << "  --help, -h  显示帮助\n";
}

int main(int argc, char *argv[]) {
    fs::path settings_path = "config/config.yaml";
    for (int i = 1; i < argc; ++i) {
        const std::string arg = argv[i];
        if (arg == "--help" || arg == "-h") {
            printUsage(argv[0]);
            return 0;
        }
        if (arg == "--config" && i + 1 < argc) {
            settings_path = argv[++i];
        } else {
            std::cerr << "未知参数或缺少参数值: " << arg << std::endl;
            printUsage(argv[0]);
            return 1;
        }
    }

    std::string model_path;
    std::string config_path;
    int ecat_id;
    double cycle_time;
    double duration;
    bool visualize;
    try {
        const YAML::Node settings = YAML::LoadFile(settings_path.string());
        if (!settings.IsMap()) {
            throw std::runtime_error("启动配置必须是 YAML 映射");
        }
        for (const auto &entry : settings) {
            const std::string key = entry.first.as<std::string>();
            if (key != "model" && key != "urdf" && key != "hardware_config" &&
                key != "ecat_id" && key != "cycle_time_us" &&
                key != "duration_sec" && key != "visualize") {
                throw std::runtime_error("未知启动配置项: " + key);
            }
        }
        if (settings["model"] && settings["urdf"]) {
            throw std::runtime_error("model 与旧字段 urdf 不能同时指定，请使用 model 文件路径");
        }
        if (!settings["model"] && !settings["urdf"]) {
            throw std::runtime_error("必须指定 model 文件路径（MJCF/URDF）");
        }
        const auto resolve_path = [&](const std::string &name) {
            if (name.empty()) throw std::runtime_error("模型/硬件配置路径不能为空");
            return (settings_path.parent_path() / name).lexically_normal().string();
        };
        // Keep urdf as a legacy alias when model is absent.
        model_path = resolve_path(settings[settings["model"] ? "model" : "urdf"]
                                      .as<std::string>());
        if (!fs::is_regular_file(model_path)) {
            throw std::runtime_error("model 必须指向存在的 MJCF/URDF 文件: " + model_path);
        }
        config_path = settings["hardware_config"]
            ? resolve_path(settings["hardware_config"].as<std::string>())
            : (fs::path(model_path).parent_path() / "hardware_config.yaml").string();
        ecat_id = readOption<int>(settings, "ecat_id", 0);
        cycle_time = readOption<double>(settings, "cycle_time_us", 1000.0);
        duration = readOption<double>(settings, "duration_sec", -1.0);
        visualize = readOption<bool>(settings, "visualize", false);
        if (ecat_id < 0) {
            throw std::runtime_error("ecat_id 必须为非负整数");
        }
        if (!std::isfinite(cycle_time) || cycle_time < 1.0 ||
            cycle_time > std::numeric_limits<uint32_t>::max()) {
            throw std::runtime_error("cycle_time_us 必须在 [1, 4294967295] 范围内");
        }
        if (!std::isfinite(duration) || (duration < 0.0 && duration != -1.0)) {
            throw std::runtime_error("duration_sec 必须为 -1 或非负有限数值");
        }
    } catch (const std::exception &error) {
        std::cerr << "启动配置加载失败 (" << settings_path.string()
                  << "): " << error.what() << std::endl;
        return 1;
    }

    // ---- 打印启动信息 ----------------------------------------------------
    std::cout << "╔══════════════════════════════════════════════════════════════╗\n";
    std::cout << "║  ROCOS MuJoCo 仿真器 (EtherCAT 主站模拟)                    ║\n";
    std::cout << "╠══════════════════════════════════════════════════════════════╣\n";
    std::cout << "║  模型:      " << std::setw(49) << std::left << model_path  << "║\n";
    std::cout << "║  配置:      " << std::setw(49) << std::left << config_path << "║\n";
    std::cout << "║  ecat_id:   " << std::setw(49) << std::left << ecat_id     << "║\n";
    std::cout << "║  周期:      " << std::setw(46) << std::left
              << (std::to_string(static_cast<uint32_t>(cycle_time)) + " us") << "║\n";
    std::cout << "║  可视化:    " << std::setw(49) << std::left
              << (visualize ? "开启" : "关闭 (无头模式)") << "║\n";
    std::cout << "╚══════════════════════════════════════════════════════════════╝\n\n";

    // ---- 创建并运行仿真器 ------------------------------------------------
    rocos_mujoco::MujocoSimulator sim(model_path, config_path,
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
