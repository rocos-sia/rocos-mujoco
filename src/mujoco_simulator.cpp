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
    : urdf_path_(urdf_path),
      hw_config_path_(hw_config_path),
      cycle_time_us_(cycle_time_us) {
    shm_ = new rocos::SharedMemoryConfig(ecat_id);
}

MujocoSimulator::~MujocoSimulator() {
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
    delete shm_;
    shm_ = nullptr;
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
                  << "or run without --visualize.\n";
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

    // 2. Parse hardware_config.yaml (PDO names + transform params)
    if (!loadHardwareConfig(hw_config_path_)) {
        std::cerr << "\033[1;31m[ERROR]\033[0m Failed to load hardware config\n";
        return false;
    }
    std::cout << "\033[1;32m[INFO]\033[0m Hardware config loaded ("
              << joints_.size() << " drives, "
              << ft_sensors_.size() << " ft, "
              << io_modules_.size() << " io)\n";

    // 3. Create shared memory
    if (!setupSharedMemory()) {
        std::cerr << "\033[1;31m[ERROR]\033[0m Failed to setup shared memory\n";
        return false;
    }
    std::cout << "\033[1;32m[INFO]\033[0m Shared memory created\n";

    // 4. Initial bus state
    shm_->ecatBus->current_state = ECAT_STATE_OP;
    shm_->ecatBus->is_authorized = true;
    shm_->ecatBus->slave_num     = static_cast<int>(
        joints_.size() + ft_sensors_.size() + io_modules_.size());

    if (!visualize_) {
        writeSensorData();
    }

    std::cout << "\033[1;32m[INFO]\033[0m MujocoSimulator initialized.\n";
    return true;
}

void MujocoSimulator::step() {
    readCommands();
    applyControl();
    stepPhysics();
    writeSensorData();
    signalClients();
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
        std::vector<double> vis_pos;

        while (visualizer_->isRunning() && sim_time_ < end_time) {
            auto t0 = std::chrono::steady_clock::now();

            // 与无头模式完全相同的 step：读命令 → 控制 → 物理 → 写传感器
            // 必须在锁内执行，防止渲染线程同时访问 d_（mj_forward）。
            {
                std::lock_guard<std::mutex> lk(visualizer_->physicsMutex());
                step();
            }

            // 将当前关节位置发送给渲染线程用于显示
            vis_pos.resize(m_->nv);
            for (int i = 0; i < m_->nv; ++i)
                vis_pos[i] = d_->qpos[i];
            visualizer_->writeCommand(vis_pos, {}, {});

            auto elapsed = std::chrono::duration_cast<
                std::chrono::microseconds>(
                std::chrono::steady_clock::now() - t0);
            if (elapsed < cycle_dur)
                std::this_thread::sleep_for(cycle_dur - elapsed);
        }
        visualizer_->close();
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

bool MujocoSimulator::loadHardwareConfig(const std::string &yaml_path) {
    try {
        YAML::Node root = YAML::LoadFile(yaml_path);
        YAML::Node hw_list = root["hardware"];
        if (!hw_list || !hw_list.IsSequence()) {
            std::cerr << "\033[1;31m[ERROR]\033[0m YAML missing 'hardware'\n";
            return false;
        }

        joints_.clear();
        ft_sensors_.clear();
        io_modules_.clear();

        for (const auto &node : hw_list) {
            if (!node["type"]) continue;
            std::string type = node["type"].as<std::string>();
            int id = yamlGetInt(node, "id", -1);

            if (type == "driver") {
                JointEntry j;
                j.slave_id   = id;
                j.joint_name = yamlGetStr(node, "joint_name",
                                          "joint_" + std::to_string(id));
                // Transform params
                if (node["transform"]) {
                    auto &t = node["transform"];
                    j.cnt_per_unit    = yamlGetDouble(t, "cnt_per_unit", 1.0);
                    j.torque_per_unit = yamlGetDouble(t, "torque_per_unit", 1.0);
                    j.ratio           = yamlGetDouble(t, "ratio", 1.0);
                    j.offset_pos_cnt  = yamlGetInt(t, "offset_pos_cnt", 0);
                }
                // Build PDO layout from YAML inputs/outputs sections
                j.pdo = buildPdoLayout("driver", node);

                // Resolve MuJoCo joint indices
                j.mj_joint_qpos = mj_name2id(m_, mjOBJ_JOINT,
                                             j.joint_name.c_str());
                if (j.mj_joint_qpos >= 0)
                    j.mj_joint_dof = m_->jnt_dofadr[j.mj_joint_qpos];

                joints_.push_back(j);
                std::cout << "  Drive: slave=" << id
                          << " joint=" << j.joint_name
                          << " mj_id=" << j.mj_joint_qpos
                          << " pdo_in=" << j.pdo.input_size << "B"
                          << " pdo_out=" << j.pdo.output_size << "B\n";

            } else if (type == "ft_sensor") {
                FTEntry ft;
                ft.slave_id = id;
                ft.pdo = buildPdoLayout("ft_sensor", node);
                ft_sensors_.push_back(ft);
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
                io_modules_.push_back(io);
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

bool MujocoSimulator::setupSharedMemory() {
    if (!shm_->createSharedMemory()) {
        std::cerr << "\033[1;31m[ERROR]\033[0m createSharedMemory failed\n";
        return false;
    }

    // Compute global PD sizes and per-slave base offsets
    int total_in  = 0;
    int total_out = 0;

    for (auto &j : joints_) {
        j.pd_input_base  = total_in;
        j.pd_output_base = total_out;
        total_in  += j.pdo.input_size;
        total_out += j.pdo.output_size;
    }
    for (auto &ft : ft_sensors_) {
        ft.pd_input_base = total_in;
        total_in += ft.pdo.input_size;
    }
    for (auto &io : io_modules_) {
        io.pd_input_base  = total_in;
        io.pd_output_base = total_out;
        total_in  += io.pdo.input_size;
        total_out += io.pdo.output_size;
    }

    std::cout << "  Total PD input:  " << total_in  << " bytes\n";
    std::cout << "  Total PD output: " << total_out << " bytes\n";

    if (!shm_->createPdDataMemoryProvider(total_in, total_out)) {
        std::cerr << "\033[1;31m[ERROR]\033[0m createPdDataMemoryProvider failed\n";
        return false;
    }
    std::memset(shm_->pdInputPtr,  0, static_cast<size_t>(total_in));
    std::memset(shm_->pdOutputPtr, 0, static_cast<size_t>(total_out));

    // Populate slave metadata (exactly as rocos-ecm EcDemoApp::myAppSetup)
    int slave_idx = 0;

    auto populate_slave = [&](int id, const std::string &name,
                              const SlavePdoLayout &pdo,
                              int input_base, int output_base) {
        rocos::Slave &sl = shm_->ecatBus->slaves[slave_idx];
        sl.id = id;
        std::strncpy(sl.name, name.c_str(), MAX_SLAVE_NAME_LEN - 1);
        sl.name[MAX_SLAVE_NAME_LEN - 1] = '\0';

        // Input variables — offset 必须是全局偏移（pdInputPtr + offset）
        sl.input_var_num = static_cast<int>(pdo.inputs.size());
        for (size_t v = 0; v < pdo.inputs.size() && v < MAX_PDINPUT_NUM; ++v) {
            rocos::PdVar &pv = sl.input_vars[v];
            std::strncpy(pv.name, pdo.inputs[v].name.c_str(),
                         MAX_PD_NAME_LEN - 1);
            pv.name[MAX_PD_NAME_LEN - 1] = '\0';
            pv.offset    = input_base + pdo.inputs[v].offset;
            pv.size      = pdo.inputs[v].size;
            pv.index     = static_cast<uint16_t>(0x6000 + v);
            pv.sub_index = 1;
        }

        // Output variables — 同理，全局偏移
        sl.output_var_num = static_cast<int>(pdo.outputs.size());
        for (size_t v = 0; v < pdo.outputs.size() && v < MAX_PDOUTPUT_NUM; ++v) {
            rocos::PdVar &pv = sl.output_vars[v];
            std::strncpy(pv.name, pdo.outputs[v].name.c_str(),
                         MAX_PD_NAME_LEN - 1);
            pv.name[MAX_PD_NAME_LEN - 1] = '\0';
            pv.offset    = output_base + pdo.outputs[v].offset;
            pv.size      = pdo.outputs[v].size;
            pv.index     = static_cast<uint16_t>(0x7000 + v);
            pv.sub_index = 1;
        }
        ++slave_idx;
    };

    for (const auto &j : joints_)
        populate_slave(j.slave_id, j.joint_name, j.pdo,
                       j.pd_input_base, j.pd_output_base);
    for (const auto &ft : ft_sensors_)
        populate_slave(ft.slave_id,
                       "ft_sensor_" + std::to_string(ft.slave_id), ft.pdo,
                       ft.pd_input_base, 0);
    for (const auto &io : io_modules_)
        populate_slave(io.slave_id,
                       "io_module_" + std::to_string(io.slave_id), io.pdo,
                       io.pd_input_base, io.pd_output_base);

    shm_->ecatBus->slave_num = slave_idx;
    return true;
}

// ==========================================================================
// Per-step: read commands from shared memory output area
// ==========================================================================

void MujocoSimulator::readCommands() {
    if (!shm_ || !shm_->pdOutputPtr) return;
    auto *out = static_cast<char *>(shm_->pdOutputPtr);

    for (auto &j : joints_) {
        char *base = out + j.pd_output_base;

        auto off = [&](const char *key) {
            return j.pdo.findOffset(key, false);
        };
        int o;
        if ((o = off("control_word"))      >= 0) {
            j.control_word = *reinterpret_cast<uint16_t *>(base + o);
            j.cia.process(j.control_word);
        }
        if ((o = off("mode_of_operation")) >= 0)
            j.mode_of_operation = *reinterpret_cast<int8_t   *>(base + o);
        if ((o = off("target_position"))   >= 0)
            j.target_position   = *reinterpret_cast<int32_t  *>(base + o);
        if ((o = off("target_velocity"))   >= 0)
            j.target_velocity   = *reinterpret_cast<int32_t  *>(base + o);
        if ((o = off("target_torque"))     >= 0)
            j.target_torque     = *reinterpret_cast<int16_t  *>(base + o);
    }
}

// ==========================================================================
// Per-step: apply control to MuJoCo joints
// ==========================================================================

void MujocoSimulator::applyControl() {
    for (auto &j : joints_) {
        int dof = j.mj_joint_dof;
        if (dof < 0 || dof >= m_->nu) continue;

        if (!j.cia.isEnabled()) {
            // ---- 未使能：锁死或待在原地 ----
            if (!j.hold_valid) {
                j.hold_position = d_->qpos[dof];
                j.hold_valid    = true;
            }
            d_->ctrl[dof] = j.hold_position;
            if (dof < m_->nv) d_->qfrc_applied[dof] = 0.0;
            continue;
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
            double tp = cntToUnit(j.target_position, j.cnt_per_unit,
                                  j.ratio, j.offset_pos_cnt);
            if (dof < m_->nu) {
                mjtNum lo = m_->actuator_ctrlrange[2 * dof];
                mjtNum hi = m_->actuator_ctrlrange[2 * dof + 1];
                tp = std::max(lo, std::min(hi, tp));
            }
            d_->ctrl[dof] = tp;
            if (dof < m_->nv) d_->qfrc_applied[dof] = 0.0;
            break;
        }
        case 9: { // CSV — 速度控制
            // 目标速度通过 qfrc_applied 实现：用阻尼差跟踪目标速度
            double tv = cntToUnit(j.target_velocity, j.cnt_per_unit,
                                  j.ratio, 0);
            double verr = tv - d_->qvel[dof];
            constexpr double KV = 100.0;  // 速度跟踪增益
            d_->qfrc_applied[dof] = KV * verr;
            // actuator 以当前位置为目标，不额外发力
            d_->ctrl[dof] = d_->qpos[dof];
            break;
        }
        case 10: { // CST — 力矩控制
            double tt = torqueToUnit(j.target_torque, j.torque_per_unit);
            d_->qfrc_applied[dof] = tt;
            // actuator 以当前位置为目标，不额外发力
            d_->ctrl[dof] = d_->qpos[dof];
            break;
        }
        default:
            // 未知模式：待在原地
            if (!j.hold_valid) {
                j.hold_position = d_->qpos[dof];
                j.hold_valid    = true;
            }
            d_->ctrl[dof] = j.hold_position;
            if (dof < m_->nv) d_->qfrc_applied[dof] = 0.0;
            update_hold_position = false;
            break;
        }

        // 持续更新锁死位置
        if (update_hold_position) {
            j.hold_position = d_->qpos[dof];
            j.hold_valid    = true;
        }
    }
}

// ==========================================================================
// Per-step: advance MuJoCo physics
// ==========================================================================

void MujocoSimulator::stepPhysics() {
    mj_step(m_, d_);
}

// ==========================================================================
// Per-step: write sensor data to shared memory input area
// Uses dynamic PDO layout offsets from YAML.
// ==========================================================================

void MujocoSimulator::writeSensorData() {
    if (!shm_ || !shm_->pdInputPtr || !shm_->ecatBus) return;
    auto *in = static_cast<char *>(shm_->pdInputPtr);

    auto now = std::chrono::steady_clock::now().time_since_epoch();
    shm_->ecatBus->timestamp = std::chrono::duration_cast<
        std::chrono::microseconds>(now).count();

    for (const auto &j : joints_) {
        char *base = in + j.pd_input_base;
        int   dof  = j.mj_joint_dof;

        double q    = (dof >= 0) ? d_->qpos[dof] : 0.0;
        double qdot = (dof >= 0) ? d_->qvel[dof] : 0.0;
        double tau  = (dof >= 0) ? d_->qfrc_applied[dof] : 0.0;

        int32_t pc = unitToCnt(q,    j.cnt_per_unit, j.ratio, j.offset_pos_cnt);
        int32_t vc = unitToCnt(qdot, j.cnt_per_unit, j.ratio, 0);
        int16_t tc = static_cast<int16_t>(tau * j.torque_per_unit);

        auto off = [&](const char *key) {
            return j.pdo.findOffset(key, true);
        };
        int o;
        #define W(name, val, type) \
            if ((o = off(#name)) >= 0) \
                *reinterpret_cast<type*>(base+o)=static_cast<type>(val)

        W(status_word,              j.cia.statusWord(), uint16_t);
        W(position_actual_value,    pc,                int32_t);
        W(velocity_actual_value,    vc,     int32_t);
        W(torque_actual_value,      tc,     int16_t);
        W(load_torque_value,        tc,     int16_t);
        W(secondary_position_value, pc,     int32_t);
        W(secondary_velocity_value, vc,     int32_t);
        W(digital_inputs,           0,      int32_t);
        W(digital_outputs,          0,      int32_t);
        #undef W
    }

    // FT sensors — write zeros for now
    for (const auto &ft : ft_sensors_) {
        char *base = in + ft.pd_input_base;
        for (const auto &v : ft.pdo.inputs) {
            if (v.size == 2)
                *reinterpret_cast<int16_t *>(base + v.offset) = 0;
            else if (v.size == 4)
                *reinterpret_cast<int32_t *>(base + v.offset) = 0;
        }
    }

    // IO modules — write zeros
    for (const auto &io : io_modules_) {
        char *base = in + io.pd_input_base;
        for (const auto &v : io.pdo.inputs) {
            if (v.size == 2)
                *reinterpret_cast<int16_t *>(base + v.offset) = 0;
            else if (v.size == 4)
                *reinterpret_cast<int32_t *>(base + v.offset) = 0;
        }
    }
}

// ==========================================================================
// Per-step: signal waiting clients
// ==========================================================================

void MujocoSimulator::signalClients() {
    shm_->updateSempahore();
}

// ==========================================================================
// Unit conversion helpers
// ==========================================================================

double MujocoSimulator::cntToUnit(double cnt, double cnt_per_unit,
                                   double ratio, int32_t offset_pos_cnt) const {
    return (static_cast<double>(cnt) - offset_pos_cnt) / cnt_per_unit / ratio;
}

int32_t MujocoSimulator::unitToCnt(double unit, double cnt_per_unit,
                                    double ratio, int32_t offset_pos_cnt) const {
    return static_cast<int32_t>(unit * cnt_per_unit * ratio) + offset_pos_cnt;
}

double MujocoSimulator::torqueToUnit(int16_t raw,
                                      double torque_per_unit) const {
    return static_cast<double>(raw) / torque_per_unit;
}



}  // namespace rocos_mujoco
