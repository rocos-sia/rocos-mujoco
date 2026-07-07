// Copyright 2026, Yang Luo
// SPDX-License-Identifier: GPL-3.0-or-later
//
// main.cpp
// Entry point for the rocos-mujoco simulator.

#include <cstdlib>
#include <cstring>
#include <iostream>
#include <string>

#include "rocos_mujoco/mujoco_simulator.hpp"

static void printUsage(const char *prog) {
    std::cout << "Usage: " << prog << " [options]\n"
              << "Options:\n"
              << "  --urdf <path>        Path to MJCF/URDF model (default: model/charge_arm.xml)\n"
              << "  --config <path>      Path to hardware_config.yaml (default: config/hardware_config.yaml)\n"
              << "  --ecat-id <n>        EtherCAT master ID (default: 0)\n"
              << "  --cycle-time <us>    Cycle time in microseconds (default: 1000)\n"
              << "  --duration <sec>     Run duration in seconds (default: -1 = forever)\n"
              << "  --visualize, -v      Enable 3D visualization window\n"
              << "  --help               Show this help\n"
              << std::endl;
}

int main(int argc, char *argv[]) {
    std::string urdf_path   = "model/charge_arm.xml";
    std::string config_path = "config/charge_arm_config.yaml";
    int         ecat_id     = 0;
    double      cycle_time  = 1000.0;   // microseconds
    double      duration    = -1.0;     // seconds, -1 = forever
    bool        visualize   = false;

    // ---- Parse command-line arguments ----------------------------------
    for (int i = 1; i < argc; ++i) {
        std::string arg = argv[i];
        if (arg == "--help" || arg == "-h") {
            printUsage(argv[0]);
            return 0;
        } else if (arg == "--visualize" || arg == "-v") {
            visualize = true;
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
            std::cerr << "Unknown argument: " << arg << std::endl;
            printUsage(argv[0]);
            return 1;
        }
    }

    // ---- Create and run the simulator ----------------------------------
    rocos_mujoco::MujocoSimulator sim(urdf_path, config_path,
                                       ecat_id, cycle_time);
    if (visualize) {
        sim.enableVisualization();
    }

    if (!sim.initialize()) {
        std::cerr << "Failed to initialize simulator." << std::endl;
        return 1;
    }

    sim.run(duration);

    std::cout << "Simulation finished." << std::endl;
    return 0;
}
