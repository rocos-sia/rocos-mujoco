// Copyright 2026, Yang Luo
// SPDX-License-Identifier: GPL-3.0-or-later
//
// mujoco_simulator.cpp
// MuJoCo simulation that acts as the EtherCAT master — creates shared
// memory, populates slave / PDO metadata (from hardware_config.yaml),
// and bridges controller commands with simulated physics.

#include "rocos_mujoco/mujoco_simulator.hpp"

#include <yaml-cpp/yaml.h>

#include <mujoco/mujoco.h>

#include <algorithm>
#include <cerrno>
#include <chrono>
#include <cmath>
#include <cstring>
#include <iostream>
#include <limits>
#include <mutex>
#include <set>
#include <stdexcept>
#include <string>
#include <thread>

#ifdef ROCOS_MUJOCO_ENABLE_VISUALIZATION
#include "rocos_mujoco/hardware_mj.hpp"
#endif
#include "rocos_mujoco/shared_memory_config.hpp"

// ==========================================================================
// CiA 402 仿真状态机实现
// ==========================================================================
namespace rocos_mujoco {

uint16_t Cia402Sim::process(uint16_t cw) {
    bool so  = (cw >> 0) & 1;  // Switch On
    bool ev  = (cw >> 1) & 1;  // Enable Voltage
    bool qs  = (cw >> 2) & 1;  // Quick Stop
    bool eo  = (cw >> 3) & 1;  // Enable Operation
    bool fr  = (cw >> 7) & 1;  // Fault Reset

    if (fr && !fault_reset_prev_ && state_ == 7) {
        state_ = 1;
        fault_reset_prev_ = fr;
        return statusWord();
    }
    fault_reset_prev_ = fr;

    switch (state_) {
    case 0: state_ = 1; break;  // NotReadyToSwitchOn → SwitchOnDisabled 自动
    case 1: if (ev && qs) state_ = 2; break;          // Shutdown
    case 2:
        if (!ev)      state_ = 1;                      // Disable Voltage
        else if (so)  state_ = 3;                      // Switch On
        break;
    case 3:
        if (!ev)      state_ = 1;                      // Disable Voltage
        else if (!so) state_ = 2;                      // Shutdown
        else if (eo)  state_ = 4;                      // Enable Operation
        break;
    case 4:
        if (!ev)      state_ = 1;                      // Disable Voltage
        else if (!so) state_ = 2;                      // Shutdown
        else if (!eo) state_ = 3;                      // Disable Operation
        else if (!qs) state_ = 5;                      // Quick Stop
        break;
    case 5: if (!ev)  state_ = 1; break;               // Disable Voltage
    case 6: state_ = 7; break;                           // FaultReactionActive → Fault 自动
    case 7: break;                                       // Fault, wait reset
    default: break;
    }
    return statusWord();
}

uint16_t Cia402Sim::statusWord() const {
    constexpr uint16_t kRemote = 0x0200;
    switch (state_) {
    case 0: return kRemote | 0x0000;  // NotReadyToSwitchOn
    case 1: return kRemote | 0x0040;  // SwitchOnDisabled (bit6)
    case 2: return kRemote | 0x0021;  // ReadyToSwitchOn (bits 0,5)
    case 3: return kRemote | 0x0023;  // SwitchedOn (bits 0,1,5)
    case 4: return kRemote | 0x0027;  // OperationEnabled (bits 0,1,2,5)
    case 5: return kRemote | 0x0007;  // QuickStopActive (bits 0,1,2)
    case 6: return kRemote | 0x000F;  // FaultReactionActive
    case 7: return kRemote | 0x0008;  // Fault (bit3)
    default:return kRemote | 0x0000;
    }
}

}  // namespace rocos_mujoco

namespace rocos_mujoco {
namespace {

// ==========================================================================
// PDO field templates — map YAML keys → (default name, size in bytes)
// ==========================================================================

struct PdoFieldTemplate {
    const char *yaml_key;       // key in hardware_config.yaml inputs: / outputs:
    const char *default_name;   // canonical PDO variable name
    int         size;           // bytes
};

// ---- Drive (CiA 402) input fields ----------------------------------------
const PdoFieldTemplate DRIVE_INPUT_FIELDS[] = {
    {"status_word",              "Statusword",                       2}, // uint16
    {"position_actual_value",    "Position actual value",            4}, // int32
    {"velocity_actual_value",    "Velocity actual value",            4}, // int32
    {"torque_actual_value",      "Torque actual value",              2}, // int16
    {"load_torque_value",        "Analog Input 1",                   2}, // int16
    {"secondary_position_value", "Auxiliary position actual value",  4}, // int32
    {"secondary_velocity_value", "Secondary velocity value",         4}, // int32
    {"digital_inputs",           "Digital Inputs",                   4}, // int32
    {"digital_outputs",          "Digital Outputs",                  4}, // int32 (readback)
};
constexpr int DRIVE_INPUT_COUNT =
    sizeof(DRIVE_INPUT_FIELDS) / sizeof(DRIVE_INPUT_FIELDS[0]);

// ---- Drive (CiA 402) output fields ---------------------------------------
const PdoFieldTemplate DRIVE_OUTPUT_FIELDS[] = {
    {"control_word",      "Controlword",         2}, // uint16
    {"mode_of_operation", "Modes of operation",  1}, // int8
    {"target_position",   "Target position",     4}, // int32
    {"target_velocity",   "Target velocity",     4}, // int32
    {"target_torque",     "Target torque",       2}, // int16
    {"digital_outputs",   "Digital Outputs",     4}, // int32
};
constexpr int DRIVE_OUTPUT_COUNT =
    sizeof(DRIVE_OUTPUT_FIELDS) / sizeof(DRIVE_OUTPUT_FIELDS[0]);

// ---- FT sensor input fields ----------------------------------------------
const PdoFieldTemplate FT_INPUT_FIELDS[] = {
    {"fx", "Fx", 2},  {"fy", "Fy", 2},  {"fz", "Fz", 2},
    {"tx", "Tx", 2},  {"ty", "Ty", 2},  {"tz", "Tz", 2},
};
constexpr int FT_INPUT_COUNT =
    sizeof(FT_INPUT_FIELDS) / sizeof(FT_INPUT_FIELDS[0]);

// ---- IO module input fields ----------------------------------------------
const PdoFieldTemplate IO_INPUT_FIELDS[] = {
    {"digital_inputs",  "Digital Inputs",  4},
    {"digital_outputs", "Digital Outputs", 4},  // readback
    {"analog_inputs",   "Analog Inputs",   2},
};
constexpr int IO_INPUT_COUNT =
    sizeof(IO_INPUT_FIELDS) / sizeof(IO_INPUT_FIELDS[0]);

// ---- IO module output fields ---------------------------------------------
const PdoFieldTemplate IO_OUTPUT_FIELDS[] = {
    {"digital_outputs", "Digital Outputs", 4},
    {"analog_outputs",  "Analog Outputs",  2},
};
constexpr int IO_OUTPUT_COUNT =
    sizeof(IO_OUTPUT_FIELDS) / sizeof(IO_OUTPUT_FIELDS[0]);

// ---- Helper: build layout from templates + YAML node ---------------------
SlavePdoLayout buildLayout(const PdoFieldTemplate *templates,
                           int                      count,
                           bool                     is_input,
                           const YAML::Node        &node) {
    SlavePdoLayout layout;
    const char *section = is_input ? "inputs" : "outputs";

    if (!node[section]) return layout;

    const auto &sec = node[section];
    int offset = 0;

    for (int i = 0; i < count; ++i) {
        const auto &t = templates[i];
        // Read PDO name from YAML, fall back to default
        std::string pdo_name = t.default_name;
        if (sec[t.yaml_key]) {
            pdo_name = sec[t.yaml_key].as<std::string>();
        }
        // Skip empty names (variable not configured)
        if (pdo_name.empty()) continue;

        PdoVarInfo info;
        info.yaml_key = t.yaml_key;
        info.name     = pdo_name;
        info.offset   = offset;
        info.size     = t.size;

        if (is_input) {
            layout.inputs.push_back(info);
        } else {
            layout.outputs.push_back(info);
        }
        offset += t.size;
    }

    if (is_input) {
        layout.input_size = offset;
    } else {
        layout.output_size = offset;
    }
    return layout;
}

}  // namespace (anonymous)
}  // namespace rocos_mujoco

namespace rocos_mujoco {

// ==========================================================================
// YAML helpers
// ==========================================================================

static std::string yamlGetStr(const YAML::Node &node,
                              const std::string  &key,
                              const std::string  &def = "") {
    if (node[key]) return node[key].as<std::string>();
    return def;
}

static double yamlGetDouble(const YAML::Node &node,
                            const std::string  &key,
                            double              def = 0.0) {
    if (node[key]) return node[key].as<double>();
    return def;
}

static int yamlGetInt(const YAML::Node &node,
                      const std::string  &key,
                      int                def = 0) {
    if (node[key]) return node[key].as<int>();
    return def;
}

// ==========================================================================
// PDO layout builder (public)
// ==========================================================================

SlavePdoLayout MujocoSimulator::buildPdoLayout(const std::string &type,
                                                const YAML::Node  &node) {
    SlavePdoLayout layout;

    if (type == "driver") {
        auto in  = buildLayout(DRIVE_INPUT_FIELDS,  DRIVE_INPUT_COUNT,
                               true,  node);
        auto out = buildLayout(DRIVE_OUTPUT_FIELDS, DRIVE_OUTPUT_COUNT,
                               false, node);
        layout.inputs      = std::move(in.inputs);
        layout.outputs     = std::move(out.outputs);
        layout.input_size  = in.input_size;
        layout.output_size = out.output_size;
    } else if (type == "ft_sensor") {
        auto in = buildLayout(FT_INPUT_FIELDS, FT_INPUT_COUNT, true, node);
        layout.inputs      = std::move(in.inputs);
        layout.input_size  = in.input_size;
        layout.output_size = 0;
    } else if (type == "io") {
        auto in  = buildLayout(IO_INPUT_FIELDS,  IO_INPUT_COUNT,
                               true,  node);
        auto out = buildLayout(IO_OUTPUT_FIELDS, IO_OUTPUT_COUNT,
                               false, node);
        layout.inputs      = std::move(in.inputs);
        layout.outputs     = std::move(out.outputs);
        layout.input_size  = in.input_size;
        layout.output_size = out.output_size;
    }
    return layout;
}

// ==========================================================================
// Construction / Destruction
// ==========================================================================

MujocoSimulator::MujocoSimulator(const std::string &urdf_path,
                                 const std::string &hw_config_path,
                                 int                ecat_id,
                                 double             cycle_time_us)
    : MujocoSimulator(urdf_path, std::vector<MasterConfig>{{ecat_id, hw_config_path}},
                      cycle_time_us) {}

MujocoSimulator::MasterState::MasterState(const MasterConfig& config)
    : config(config), shm(std::make_unique<rocos::SharedMemoryConfig>(config.ecat_id)) {}

MujocoSimulator::MasterState::~MasterState() = default;

MujocoSimulator::MujocoSimulator(const std::string& model_path,
                               const std::vector<MasterConfig>& masters,
                               double cycle_time_us)
    : urdf_path_(model_path), cycle_time_us_(cycle_time_us) {
    if (masters.empty()) throw std::invalid_argument("At least one master is required");
    std::set<int> ids;
    for (const auto& config : masters) {
        if (config.ecat_id < 0 || !ids.insert(config.ecat_id).second) {
            throw std::invalid_argument("Master IDs must be non-negative and unique");
        }
        masters_.push_back(std::make_unique<MasterState>(config));
    }
    hw_config_path_ = masters.front().hardware_config;
}

int MujocoSimulator::getJointCount() const {
    int count = 0;
    for (const auto& master : masters_) count += master->joints.size();
    return count;
}

MujocoSimulator::~MujocoSimulator() {
    stop_commands_.store(true);
    for (auto& master : masters_) {
        if (master->command_thread.joinable()) master->command_thread.join();
    }
    bool model_owned_by_visualizer = false;
#ifdef ROCOS_MUJOCO_ENABLE_VISUALIZATION
    if (visualizer_ != nullptr) {
        model_owned_by_visualizer = true;
        visualizer_->close();
        delete visualizer_;
        visualizer_ = nullptr;
    }
#endif
    if (!model_owned_by_visualizer) {
        if (d_ != nullptr) { mj_deleteData(d_); d_ = nullptr; }
        if (m_ != nullptr) { mj_deleteModel(m_); m_ = nullptr; }
    }
    masters_.clear();
}

// ==========================================================================
// Lifecycle
// ==========================================================================

bool MujocoSimulator::initialize() {
    std::cout << "\033[1;32m[INFO]\033[0m MujocoSimulator initializing..."
              << std::endl;

    // 1. Load model
    if (visualize_) {
#ifdef ROCOS_MUJOCO_ENABLE_VISUALIZATION
        visualizer_ = new HardwareMJ(urdf_path_, /*dyn=*/true);
        visualizer_->setPhysicsEnabled(false);
        if (!visualizer_->init()) {
            std::cerr << "\033[1;31m[ERROR]\033[0m HardwareMJ init failed\n";
            return false;
        }
        m_ = visualizer_->model();
        d_ = visualizer_->data();
        std::cout << "\033[1;32m[INFO]\033[0m Visualization window opened"
                  << " (MuJoCo Simulate UI)\n";
#else
        std::cerr << "\033[1;31m[ERROR]\033[0m Visualization support is not built. "
                  << "Reconfigure with -DROCOS_MUJOCO_ENABLE_VISUALIZATION=ON "
                  << "or set visualize: false in config.yaml.\n";
        return false;
#endif
    } else {
        if (!loadModel(urdf_path_)) {
            std::cerr << "\033[1;31m[ERROR]\033[0m Failed to load URDF: "
                      << urdf_path_ << std::endl;
            return false;
        }
        std::cout << "\033[1;32m[INFO]\033[0m MuJoCo model loaded ("
                  << m_->nq << " DOFs, " << m_->nu << " actuators)\n";
    }

    // Validate every mapping before creating or publishing any IPC objects.
    std::set<std::string> mapped_joints;
    for (auto& master : masters_) {
        if (!loadHardwareConfig(*master)) return false;
        for (const auto& joint : master->joints) {
            if (!mapped_joints.insert(joint.joint_name).second) {
                std::cerr << "[ERROR] Joint mapped more than once: " << joint.joint_name << '\n';
                return false;
            }
        }
    }
    for (auto& master : masters_) {
        if (!setupSharedMemory(*master)) return false;
        auto& bus = *master->shm->ecatBus;
        rocos::atomicStore(bus.dt, static_cast<uint32_t>(cycle_time_us_));
        rocos::atomicStore(bus.current_state, ECAT_STATE_OP);
        rocos::atomicStore(bus.is_authorized, true);
        static_assert(sizeof(ROCOS_MUJOCO_VERSION_FULL) <= sizeof(bus.ecm_version),
                      "Product version exceeds the v2 version field");
        std::memcpy(bus.ecm_version, ROCOS_MUJOCO_VERSION_FULL,
                    sizeof(ROCOS_MUJOCO_VERSION_FULL));
        bus.backend_type = 0;
        bus.supported_cmds = 1u << rocos::CMD_RESET_STATS;
        bus.state_control_supported = 0;
        rocos::atomicStore(bus.connected_slave_num, bus.slave_num);
        rocos::atomicStore(bus.status_valid, 1u);
    }
    // Initialize feedback before clients can connect, including visual mode.
#ifdef ROCOS_MUJOCO_ENABLE_VISUALIZATION
    std::unique_lock<std::mutex> lock;
    if (visualizer_) lock = std::unique_lock<std::mutex>(visualizer_->physicsMutex());
#endif
    mj_forward(m_, d_);
    for (auto& master : masters_) writeSensorData(*master);
    for (auto& master : masters_) {
        if (!master->shm->publishReady()) {
            std::cerr << "[ERROR] publishReady: " << master->shm->lastError() << '\n';
            return false;
        }
        master->command_thread = std::thread(&MujocoSimulator::commandLoop, this,
                                              std::ref(*master));
    }

    std::cout << "\033[1;32m[INFO]\033[0m MujocoSimulator initialized.\n";
    return true;
}

void MujocoSimulator::step() {
    for (auto& master : masters_) readCommands(*master);
    for (auto& master : masters_) applyControl(*master);
    stepPhysics();
    for (auto& master : masters_) {
        writeSensorData(*master);
        updateCycleStats(*master);
    }
    for (auto& master : masters_) (void)master->shm->notifyClients();
    sim_time_ += cycle_time_us_ / 1e6;
}

void MujocoSimulator::run(double duration_sec) {
    double end_time = (duration_sec < 0.0)
                          ? std::numeric_limits<double>::max()
                          : sim_time_ + duration_sec;

    std::cout << "\033[1;32m[INFO]\033[0m Simulation running"
              << " (cycle=" << cycle_time_us_ << " us"
              << ", visualize=" << (visualize_ ? "yes" : "no") << ")\n";

    if (visualize_) {
#ifdef ROCOS_MUJOCO_ENABLE_VISUALIZATION
        // 可视化路径：物理由 MujocoSimulator 同步执行（与无头模式一致），
        // HardwareMJ 渲染线程只负责显示 —— 不再独立步进 mj_step。
        // 这解决了之前 readState() 总读到渲染线程旧数据的问题。
        visualizer_->setPhysicsEnabled(false);

        auto cycle_dur = std::chrono::microseconds(
            static_cast<long long>(cycle_time_us_));

        while (visualizer_->isRunning() && sim_time_ < end_time) {
            auto t0 = std::chrono::steady_clock::now();

            // 与无头模式完全相同的 step：读命令 → 控制 → 物理 → 写传感器
            // 必须在锁内执行，防止渲染线程同时访问 d_（mj_forward）。
            {
                std::lock_guard<std::mutex> lk(visualizer_->physicsMutex());
                step();
            }

            auto elapsed = std::chrono::duration_cast<
                std::chrono::microseconds>(
                std::chrono::steady_clock::now() - t0);
            if (elapsed < cycle_dur)
                std::this_thread::sleep_for(cycle_dur - elapsed);
        }
        visualizer_->close();
        std::cout << "\033[1;32m[INFO]\033[0m Simulation finished (sim_time="
                  << sim_time_ << " s)\n";
#else
        std::cerr << "\033[1;31m[ERROR]\033[0m Visualization support is not built.\n";
        return;
#endif
    } else {
        auto cycle_dur = std::chrono::microseconds(
            static_cast<long long>(cycle_time_us_));
        while (sim_time_ < end_time) {
            auto t0 = std::chrono::steady_clock::now();
            step();
            auto elapsed = std::chrono::duration_cast<
                std::chrono::microseconds>(
                std::chrono::steady_clock::now() - t0);
            if (elapsed < cycle_dur)
                std::this_thread::sleep_for(cycle_dur - elapsed);
        }
        std::cout << "\033[1;32m[INFO]\033[0m Simulation finished (sim_time="
                  << sim_time_ << " s)\n";
    }
}

// ==========================================================================
// Model loading
// ==========================================================================

bool MujocoSimulator::loadModel(const std::string &urdf_path) {
    char error[1000] = "";
    m_ = mj_loadXML(urdf_path.c_str(), nullptr, error, sizeof(error));
    if (m_ == nullptr) {
        std::cerr << "\033[1;31m[ERROR]\033[0m mj_loadXML: " << error << "\n";
        return false;
    }
    d_ = mj_makeData(m_);
    if (d_ == nullptr) {
        mj_deleteModel(m_); m_ = nullptr;
        return false;
    }
    // 从模型的 qpos0 初始化（不依赖 keyframe，确保非零初始位姿）
    if (m_->qpos0 != nullptr) {
        std::memcpy(d_->qpos, m_->qpos0, m_->nq * sizeof(mjtNum));
    }
    mj_forward(m_, d_);
    std::cout << "\033[1;32m[INFO]\033[0m Model: "
              << m_->nq << " qpos, " << m_->nv << " dof, "
              << m_->nu << " actuators\n";
    return true;
}

// ==========================================================================
// Hardware config loading (reads PDO names + transforms from YAML)
// ==========================================================================

bool MujocoSimulator::loadHardwareConfig(MasterState& master) {
    try {
        YAML::Node root = YAML::LoadFile(master.config.hardware_config);
        YAML::Node hw_list = root["hardware"];
        if (!hw_list || !hw_list.IsSequence()) {
            std::cerr << "\033[1;31m[ERROR]\033[0m YAML missing 'hardware'\n";
            return false;
        }

        master.joints.clear();
        master.ft_sensors.clear();
        master.io_modules.clear();

        std::set<int> slave_ids;
        for (const auto &node : hw_list) {
            if (!node["type"]) continue;
            std::string type = node["type"].as<std::string>();
            int id = yamlGetInt(node, "id", -1);
            if (id < 0 || !slave_ids.insert(id).second) {
                std::cerr << "[ERROR] Slave IDs must be non-negative and unique per master\n";
                return false;
            }

            if (type == "driver") {
                JointEntry j;
                j.slave_id   = id;
                j.joint_name = yamlGetStr(node, "joint_name",
                                          "joint_" + std::to_string(id));
                // Transform params
                if (node["transform"]) {
                    auto &t = node["transform"];
                    if (t["cnt_per_unit"]) {
                        std::cerr << "[ERROR] transform.cnt_per_unit was renamed to pos_per_unit\n";
                        return false;
                    }
                    j.pos_per_unit = yamlGetDouble(t, "pos_per_unit", 1.0);
                    j.vel_per_unit = yamlGetDouble(t, "vel_per_unit", j.pos_per_unit);
                    j.load_torque_per_unit = yamlGetDouble(t, "load_torque_per_unit", 1.0);
                    j.torque_per_unit = yamlGetDouble(t, "torque_per_unit", 1.0);
                    j.ratio           = yamlGetDouble(t, "ratio", 1.0);
                    j.offset_pos_cnt  = yamlGetInt(t, "offset_pos_cnt", 0);
                }
                j.load_torque_in_secondary_position =
                    yamlGetStr(node, "torque_source", "load_torque") == "secondary_position";
                // Build PDO layout from YAML inputs/outputs sections
                j.pdo = buildPdoLayout("driver", node);

                const int joint_id = mj_name2id(m_, mjOBJ_JOINT, j.joint_name.c_str());
                if (joint_id < 0 || (m_->jnt_type[joint_id] != mjJNT_HINGE &&
                                     m_->jnt_type[joint_id] != mjJNT_SLIDE)) {
                    std::cerr << "[ERROR] Expected a hinge/slide joint: " << j.joint_name << '\n';
                    return false;
                }
                j.mj_joint_qpos = m_->jnt_qposadr[joint_id];
                j.mj_joint_dof = m_->jnt_dofadr[joint_id];
                for (int actuator = 0; actuator < m_->nu; ++actuator) {
                    if (m_->actuator_trntype[actuator] == mjTRN_JOINT &&
                        m_->actuator_trnid[2 * actuator] == joint_id) {
                        if (j.mj_actuator >= 0) {
                            std::cerr << "[ERROR] Multiple actuators for " << j.joint_name << '\n';
                            return false;
                        }
                        j.mj_actuator = actuator;
                    }
                }
                if (j.mj_actuator < 0) {
                    std::cerr << "[ERROR] Missing position actuator for " << j.joint_name << '\n';
                    return false;
                }
                const int actuator = j.mj_actuator;
                if (m_->actuator_dyntype[actuator] != mjDYN_NONE ||
                    m_->actuator_gaintype[actuator] != mjGAIN_FIXED ||
                    m_->actuator_biastype[actuator] != mjBIAS_AFFINE ||
                    m_->actuator_gear[6 * actuator] != 1.0 ||
                    m_->actuator_gainprm[mjNGAIN * actuator] <= 0.0 ||
                    m_->actuator_biasprm[mjNBIAS * actuator + 1] !=
                        -m_->actuator_gainprm[mjNGAIN * actuator]) {
                    std::cerr << "[ERROR] Expected a unit-gear position actuator for "
                              << j.joint_name << '\n';
                    return false;
                }
                if (!std::isfinite(j.pos_per_unit) || j.pos_per_unit == 0.0 ||
                    !std::isfinite(j.ratio) || j.ratio == 0.0 ||
                    !std::isfinite(j.vel_per_unit) || j.vel_per_unit == 0.0 ||
                    !std::isfinite(j.load_torque_per_unit) || j.load_torque_per_unit == 0.0 ||
                    !std::isfinite(j.torque_per_unit) || j.torque_per_unit == 0.0) {
                    std::cerr << "[ERROR] Invalid transform for " << j.joint_name << '\n';
                    return false;
                }

                master.joints.push_back(j);
                std::cout << "  Drive: slave=" << id
                          << " joint=" << j.joint_name
                          << " ecat_id=" << master.config.ecat_id
                          << " qpos=" << j.mj_joint_qpos
                          << " actuator=" << j.mj_actuator
                          << " pdo_in=" << j.pdo.input_size << "B"
                          << " pdo_out=" << j.pdo.output_size << "B\n";

            } else if (type == "ft_sensor") {
                FTEntry ft;
                ft.slave_id = id;
                ft.pdo = buildPdoLayout("ft_sensor", node);
                master.ft_sensors.push_back(ft);
                std::cout << "  FT: slave=" << id
                          << " pdo_in=" << ft.pdo.input_size << "B\n";

            } else if (type == "io") {
                IOEntry io;
                io.slave_id = id;
                io.di_channels = yamlGetInt(node, "digital_in_channels", 0);
                io.do_channels = yamlGetInt(node, "digital_out_channels", 0);
                io.ai_channels = yamlGetInt(node, "analog_in_channels", 0);
                io.ao_channels = yamlGetInt(node, "analog_out_channels", 0);
                io.pdo = buildPdoLayout("io", node);
                master.io_modules.push_back(io);
                std::cout << "  IO: slave=" << id
                          << " di=" << io.di_channels
                          << " do=" << io.do_channels
                          << " ai=" << io.ai_channels
                          << " ao=" << io.ao_channels << "\n";
            }
        }
    } catch (const YAML::Exception &e) {
        std::cerr << "\033[1;31m[ERROR]\033[0m YAML parse: " << e.what() << "\n";
        return false;
    }
    return true;
}

// ==========================================================================
// Shared memory setup (follows rocos-ecm pattern exactly)
// ==========================================================================

bool MujocoSimulator::setupSharedMemory(MasterState& master) {
    if (master.joints.size() + master.ft_sensors.size() + master.io_modules.size() >
        MAX_SLAVE_NUM) {
        std::cerr << "[ERROR] Shared memory v2 permits at most "
                  << MAX_SLAVE_NUM << " slaves\n";
        return false;
    }
    if (!master.shm->createSharedMemory()) {
        std::cerr << "[ERROR] createSharedMemory: " << master.shm->lastError() << '\n';
        return false;
    }

    // Compute global PD sizes and per-slave base offsets
    int total_in  = 0;
    int total_out = 0;

    for (auto &j : master.joints) {
        j.pd_input_base  = total_in;
        j.pd_output_base = total_out;
        total_in  += j.pdo.input_size;
        total_out += j.pdo.output_size;
    }
    for (auto &ft : master.ft_sensors) {
        ft.pd_input_base = total_in;
        total_in += ft.pdo.input_size;
    }
    for (auto &io : master.io_modules) {
        io.pd_input_base  = total_in;
        io.pd_output_base = total_out;
        total_in  += io.pdo.input_size;
        total_out += io.pdo.output_size;
    }

    std::cout << "  Total PD input:  " << total_in  << " bytes\n";
    std::cout << "  Total PD output: " << total_out << " bytes\n";

    if (!master.shm->createPdDataMemoryProvider(total_in, total_out)) {
        std::cerr << "\033[1;31m[ERROR]\033[0m createPdDataMemoryProvider failed\n";
        return false;
    }
    // The common library zeroes nonempty PDO mappings; empty ones are null.

    // Populate slave metadata (exactly as rocos-ecm EcDemoApp::myAppSetup)
    int slave_idx = 0;

    auto populate_slave = [&](int id, const std::string &name,
                              const SlavePdoLayout &pdo,
                              int input_base, int output_base) -> bool {
        if (pdo.inputs.size() > MAX_PDINPUT_NUM ||
            pdo.outputs.size() > MAX_PDOUTPUT_NUM) {
            std::cerr << "[ERROR] Too many PDO variables for slave " << id << '\n';
            return false;
        }
        rocos::Slave &sl = master.shm->ecatBus->slaves[slave_idx];
        sl.id = id;
        std::strncpy(sl.name, name.c_str(), MAX_SLAVE_NAME_LEN - 1);
        sl.name[MAX_SLAVE_NAME_LEN - 1] = '\0';
        rocos::atomicStore(sl.online, 1u);
        rocos::atomicStore(sl.al_state, ECAT_STATE_OP);
        rocos::atomicStore(sl.runtime_valid, 3u);  // Virtual online/AL state.

        // Input variables — offset 必须是全局偏移（pdInputPtr + offset）
        sl.input_var_num = static_cast<int>(pdo.inputs.size());
        for (size_t v = 0; v < pdo.inputs.size(); ++v) {
            rocos::PdVar &pv = sl.input_vars[v];
            std::strncpy(pv.name, pdo.inputs[v].name.c_str(),
                         MAX_PD_NAME_LEN - 1);
            pv.name[MAX_PD_NAME_LEN - 1] = '\0';
            if (!rocos::setPdoBits(pv, 8LL * (input_base + pdo.inputs[v].offset),
                                   8LL * pdo.inputs[v].size, total_in)) {
                return false;
            }
            pv.index     = static_cast<uint16_t>(0x6000 + v);
            pv.sub_index = 1;
        }

        // Output variables — 同理，全局偏移
        sl.output_var_num = static_cast<int>(pdo.outputs.size());
        for (size_t v = 0; v < pdo.outputs.size(); ++v) {
            rocos::PdVar &pv = sl.output_vars[v];
            std::strncpy(pv.name, pdo.outputs[v].name.c_str(),
                         MAX_PD_NAME_LEN - 1);
            pv.name[MAX_PD_NAME_LEN - 1] = '\0';
            if (!rocos::setPdoBits(pv, 8LL * (output_base + pdo.outputs[v].offset),
                                   8LL * pdo.outputs[v].size, total_out)) {
                return false;
            }
            pv.index     = static_cast<uint16_t>(0x7000 + v);
            pv.sub_index = 1;
        }
        ++slave_idx;
        return true;
    };

    for (const auto &j : master.joints)
        if (!populate_slave(j.slave_id, j.joint_name, j.pdo,
                            j.pd_input_base, j.pd_output_base)) return false;
    for (const auto &ft : master.ft_sensors)
        if (!populate_slave(ft.slave_id,
                       "ft_sensor_" + std::to_string(ft.slave_id), ft.pdo,
                       ft.pd_input_base, 0)) return false;
    for (const auto &io : master.io_modules)
        if (!populate_slave(io.slave_id,
                       "io_module_" + std::to_string(io.slave_id), io.pdo,
                       io.pd_input_base, io.pd_output_base)) return false;

    rocos::atomicStore(master.shm->ecatBus->slave_num, slave_idx);
    return true;
}

// ==========================================================================
// Per-step: read commands from shared memory output area
// ==========================================================================

void MujocoSimulator::readCommands(MasterState& master) {
    if (!master.shm || !master.shm->pdOutputPtr) return;
    auto *out = static_cast<char *>(master.shm->pdOutputPtr);

    for (auto &j : master.joints) {
        char *base = out + j.pd_output_base;

        auto off = [&](const char *key) {
            return j.pdo.findOffset(key, false);
        };
        int o;
        if ((o = off("control_word"))      >= 0) {
            std::memcpy(&j.control_word, base + o, sizeof(j.control_word));
            j.cia.process(j.control_word);
        }
        if ((o = off("mode_of_operation")) >= 0)
            std::memcpy(&j.mode_of_operation, base + o, sizeof(j.mode_of_operation));
        if ((o = off("target_position"))   >= 0)
            std::memcpy(&j.target_position, base + o, sizeof(j.target_position));
        if ((o = off("target_velocity"))   >= 0)
            std::memcpy(&j.target_velocity, base + o, sizeof(j.target_velocity));
        if ((o = off("target_torque"))     >= 0)
            std::memcpy(&j.target_torque, base + o, sizeof(j.target_torque));
    }
}

// ==========================================================================
// Per-step: apply control to MuJoCo joints
// ==========================================================================

void MujocoSimulator::applyControl(MasterState& master) {
    for (auto &j : master.joints) {
        int dof = j.mj_joint_dof;
        if (dof < 0 || dof >= m_->nv) continue;

        if (!j.cia.isEnabled()) {
            // ---- 未使能：锁死或待在原地 ----
            if (!j.hold_valid) {
                j.hold_position = d_->qpos[j.mj_joint_qpos];
                j.hold_valid    = true;
            }
            d_->ctrl[j.mj_actuator] = j.hold_position;
            if (dof < m_->nv) d_->qfrc_applied[dof] = 0.0;
            j.pos_integral = 0.0;  // 下使能时清零积分器
            continue;
        }

        // ---- 模式切换检测，清零积分器避免 windup --------------------------
        if (j.prev_mode_of_operation != j.mode_of_operation) {
            j.pos_integral          = 0.0;
            j.prev_mode_of_operation = j.mode_of_operation;
        }

        // ---- 使能状态：根据 mode_of_operation 分发 ----
        // CiA 402 标准模式值:
        //   6  = Homing        → 同 CSP
        //   8  = CSP（位置）
        //   9  = CSV（速度）
        //   10 = CST（力矩）

        bool update_hold_position = true;
        switch (j.mode_of_operation) {
        case 8:   // CSP — 位置控制
        case 6: { // Homing
            // ---- CSP: MuJoCo 执行器跟踪 + 重力前馈 + 积分修正 -----------
            //  1. ctrl = target → 模型中的位置执行器负责动态跟踪
            //  2. qfrc_applied = qfrc_bias → 前馈抵消重力（执行器静态出力→0）
            //  3. qfrc_applied += KI*∫error → 积分补偿前馈残差
            double tp = cntToUnit(j.target_position, j.pos_per_unit,
                                  j.ratio, j.offset_pos_cnt);
            d_->ctrl[j.mj_actuator] = tp;

            // ---- 重力前馈 + 积分修正 -------------------------------------
            //  qfrc_bias 直接抵消重力 → 执行器不需产生静态力 → 零残差。
            //  积分仅补偿前馈与真实重力之间的小差异（建模误差等）。
            double actual = d_->qpos[j.mj_joint_qpos];
            double error  = tp - actual;
            double dt     = cycle_time_us_ / 1e6;

            j.pos_integral += error * dt;
            d_->qfrc_applied[dof] = d_->qfrc_bias[dof]    // ← 重力前馈
                                  + KI_ * j.pos_integral;  // ← 积分残差修正
            break;
        }
        case 9: { // CSV — 速度控制
            // 目标速度通过 qfrc_applied 实现：用阻尼差跟踪目标速度
            double tv = cntToUnit(j.target_velocity, j.vel_per_unit,
                                  j.ratio, 0);
            double verr = tv - d_->qvel[dof];
            constexpr double KV = 100.0;  // 速度跟踪增益
            d_->qfrc_applied[dof] = KV * verr;
            // actuator 以当前位置为目标，不额外发力
            d_->ctrl[j.mj_actuator] = d_->qpos[j.mj_joint_qpos];
            break;
        }
        case 10: { // CST — 力矩控制
            double tt = torqueToUnit(j.target_torque, j.torque_per_unit);
            d_->qfrc_applied[dof] = tt;
            // actuator 以当前位置为目标，不额外发力
            d_->ctrl[j.mj_actuator] = d_->qpos[j.mj_joint_qpos];
            break;
        }
        default:
            // 未知模式：待在原地
            if (!j.hold_valid) {
                j.hold_position = d_->qpos[j.mj_joint_qpos];
                j.hold_valid    = true;
            }
            d_->ctrl[j.mj_actuator] = j.hold_position;
            if (dof < m_->nv) d_->qfrc_applied[dof] = 0.0;
            update_hold_position = false;
            break;
        }

        // 持续更新锁死位置
        if (update_hold_position) {
            j.hold_position = d_->qpos[j.mj_joint_qpos];
            j.hold_valid    = true;
        }
    }
}

// ==========================================================================
// Per-step: advance MuJoCo physics
// ==========================================================================

void MujocoSimulator::stepPhysics() {
    // ---- 外力施加（来自可视化窗口的交互式外力）--------------------------
#ifdef ROCOS_MUJOCO_ENABLE_VISUALIZATION
    if (visualizer_ != nullptr) {
        const auto &fs = visualizer_->getForceState();
        // 先清零所有 xfrc_applied
        for (int i = 0; i < m_->nbody; ++i) {
            for (int k = 0; k < 6; ++k)
                d_->xfrc_applied[6 * i + k] = 0.0;
        }
        // 施加选中 body 的外力
        if (fs.active && fs.body_id >= 0 && fs.body_id < m_->nbody) {
            int b = fs.body_id;
            double len =
                std::sqrt(fs.direction[0] * fs.direction[0] +
                          fs.direction[1] * fs.direction[1] +
                          fs.direction[2] * fs.direction[2]);
            if (len > 1e-8) {
                double s = fs.magnitude / len;
                // 力（世界坐标系，作用在 COM）
                d_->xfrc_applied[6 * b + 0] = fs.direction[0] * s;
                d_->xfrc_applied[6 * b + 1] = fs.direction[1] * s;
                d_->xfrc_applied[6 * b + 2] = fs.direction[2] * s;
                // 力矩 = r × f（选点不在 COM 时产生力矩）
                double rx = fs.world_pos[0] - d_->xpos[3 * b + 0];
                double ry = fs.world_pos[1] - d_->xpos[3 * b + 1];
                double rz = fs.world_pos[2] - d_->xpos[3 * b + 2];
                double fx = d_->xfrc_applied[6 * b + 0];
                double fy = d_->xfrc_applied[6 * b + 1];
                double fz = d_->xfrc_applied[6 * b + 2];
                d_->xfrc_applied[6 * b + 3] = ry * fz - rz * fy;
                d_->xfrc_applied[6 * b + 4] = rz * fx - rx * fz;
                d_->xfrc_applied[6 * b + 5] = rx * fy - ry * fx;
            }
        }
    }
#endif

    mj_step(m_, d_);
}

// ==========================================================================
// Per-step: write sensor data to shared memory input area
// Uses dynamic PDO layout offsets from YAML.
// ==========================================================================

void MujocoSimulator::writeSensorData(MasterState& master) {
    if (!master.shm || !master.shm->ecatBus) return;
    auto *in = static_cast<char *>(master.shm->pdInputPtr);

    auto now = std::chrono::system_clock::now().time_since_epoch();
    rocos::atomicStore(master.shm->ecatBus->timestamp,
        std::chrono::duration_cast<std::chrono::microseconds>(now).count());
    rocos::atomicStore(master.shm->ecatBus->heartbeat_mono_ns, rocos::monotonicNs());
    if (!in) return;

    for (const auto &j : master.joints) {
        char *base = in + j.pd_input_base;
        int   dof  = j.mj_joint_dof;

        double q    = (dof >= 0) ? d_->qpos[j.mj_joint_qpos] : 0.0;
        double qdot = (dof >= 0) ? d_->qvel[dof] : 0.0;
        // Total generalized actuator force plus the controller's applied force.
        // Mouse/body forces and contact reactions are not included in this PDO.
        double tau  = (dof >= 0) ? (d_->qfrc_actuator[dof] + d_->qfrc_applied[dof]) : 0.0;

        int32_t pc = unitToCnt(q,    j.pos_per_unit, j.ratio, j.offset_pos_cnt, true);
        int32_t vc = unitToCnt(qdot, j.vel_per_unit, j.ratio);
        auto torque_count = [&](double scale) {
            const int32_t raw = unitToCnt(tau, scale, 1.0);
            if (raw < std::numeric_limits<int16_t>::min() ||
                raw > std::numeric_limits<int16_t>::max()) {
                throw std::out_of_range("Torque feedback exceeds int16 range");
            }
            return static_cast<int16_t>(raw);
        };
        const int16_t tc = torque_count(j.torque_per_unit);
        // The alternate torque channel has an int32 PDO, unlike load_torque.
        const int32_t secondary = j.load_torque_in_secondary_position
            ? unitToCnt(tau, j.load_torque_per_unit, 1.0) : pc;
        const int16_t load = j.load_torque_in_secondary_position
            ? 0 : torque_count(j.load_torque_per_unit);

        auto off = [&](const char *key) {
            return j.pdo.findOffset(key, true);
        };
        int o;
        #define W(name, val, type) \
            if ((o = off(#name)) >= 0) { \
                const type value = static_cast<type>(val); \
                std::memcpy(base + o, &value, sizeof(value)); \
            }

        W(status_word,              j.cia.statusWord(), uint16_t);
        W(position_actual_value,    pc,                int32_t);
        W(velocity_actual_value,    vc,     int32_t);
        W(torque_actual_value,      tc,     int16_t);
        W(load_torque_value,        load,     int16_t);
        W(secondary_position_value, secondary,     int32_t);
        W(secondary_velocity_value, vc,     int32_t);
        W(digital_inputs,           0,      int32_t);
        W(digital_outputs,          0,      int32_t);
        #undef W
    }

    // FT sensors — write zeros for now
    for (const auto &ft : master.ft_sensors) {
        char *base = in + ft.pd_input_base;
        for (const auto &v : ft.pdo.inputs) {
            std::memset(base + v.offset, 0, v.size);
        }
    }

    // IO modules — write zeros
    for (const auto &io : master.io_modules) {
        char *base = in + io.pd_input_base;
        for (const auto &v : io.pdo.inputs) {
            std::memset(base + v.offset, 0, v.size);
        }
    }
}

// ==========================================================================
// Per-step: signal waiting clients
// ==========================================================================

// Mailbox polling and waiting stay off the simulation thread. Unsupported
// commands are rejected by rocos-shm using supported_cmds before this handler.
void MujocoSimulator::commandLoop(MasterState& master) {
    while (!stop_commands_.load()) {
        master.shm->processCommand([this, &master](const rocos::AsyncCommandBox &request,
                                    rocos::AsyncCommandBox &) -> int32_t {
            if (request.cmd_type != rocos::CMD_RESET_STATS) {
                return rocos::UNSUPPORTED;
            }
            master.reset_requested.store(request.cmd_id);
            while (master.reset_completed.load() != request.cmd_id) {
                if (stop_commands_.load() ||
                    rocos::monotonicNs() >= request.deadline_mono_ns) {
                    // A paused cycle may still consume the reset later.
                    return rocos::OUTCOME_UNKNOWN;
                }
                std::this_thread::sleep_for(std::chrono::milliseconds(1));
            }
            return rocos::RESULT_OK;
        });
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
}

void MujocoSimulator::updateCycleStats(MasterState& master) {
    const uint64_t now = rocos::monotonicNs();
    const uint64_t reset = master.reset_requested.load();
    auto &bus = *master.shm->ecatBus;
    if (reset != master.reset_completed.load()) {
        master.cycle_count = 0;
        master.min_cycle_us = master.max_cycle_us = master.avg_cycle_us = 0.0;
        master.last_cycle_ns = 0;
        rocos::atomicStore(bus.min_cycle_time, 0.0);
        rocos::atomicStore(bus.max_cycle_time, 0.0);
        rocos::atomicStore(bus.avg_cycle_time, 0.0);
        rocos::atomicStore(bus.current_cycle_time, 0.0);
        master.reset_completed.store(reset);
    } else if (master.last_cycle_ns != 0) {
        const double elapsed_us = (now - master.last_cycle_ns) / 1000.0;
        master.min_cycle_us = master.cycle_count == 0 ? elapsed_us
                                        : std::min(master.min_cycle_us, elapsed_us);
        master.max_cycle_us = std::max(master.max_cycle_us, elapsed_us);
        ++master.cycle_count;
        master.avg_cycle_us += (elapsed_us - master.avg_cycle_us) / master.cycle_count;
        rocos::atomicStore(bus.min_cycle_time, master.min_cycle_us);
        rocos::atomicStore(bus.max_cycle_time, master.max_cycle_us);
        rocos::atomicStore(bus.avg_cycle_time, master.avg_cycle_us);
        rocos::atomicStore(bus.current_cycle_time, elapsed_us);
    }
    master.last_cycle_ns = now;
}

// ==========================================================================
// Unit conversion helpers
// ==========================================================================

double MujocoSimulator::cntToUnit(double cnt, double cnt_per_unit,
                                   double ratio, int32_t offset_pos_cnt) const {
    return (static_cast<double>(cnt) - offset_pos_cnt) / cnt_per_unit / ratio;
}

int32_t MujocoSimulator::unitToCnt(double unit, double cnt_per_unit,
                                    double ratio, int32_t offset_pos_cnt, bool nearest) const {
    const double raw = unit * cnt_per_unit * ratio;
    const double count = (nearest ? std::round(raw) : std::trunc(raw)) + offset_pos_cnt;
    if (!std::isfinite(count) || count < std::numeric_limits<int32_t>::min() ||
        count > std::numeric_limits<int32_t>::max())
        throw std::out_of_range("Sensor value exceeds encoder count range");
    return static_cast<int32_t>(count);
}

double MujocoSimulator::torqueToUnit(int16_t raw,
                                      double torque_per_unit) const {
    return static_cast<double>(raw) / torque_per_unit;
}



}  // namespace rocos_mujoco
