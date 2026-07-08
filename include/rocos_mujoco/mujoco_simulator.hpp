// Copyright 2026, Yang Luo
// SPDX-License-Identifier: GPL-3.0-or-later
//
// mujoco_simulator.hpp
// MuJoCo-based robot simulation that exposes hardware state via POSIX
// shared memory, following the rocos-ecm protocol.  PDO variable names
// and transform parameters are read from hardware_config.yaml so the
// controller (rocos-app) can connect unchanged.

#pragma once

#include <cstdint>
#include <string>
#include <unordered_map>
#include <vector>

#include <mujoco/mujoco.h>

#include "rocos_mujoco/hardware_mj.hpp"

// Forward declarations
namespace YAML { class Node; }
namespace rocos { class SharedMemoryConfig; }

namespace rocos_mujoco {

// ==========================================================================
// PDO variable descriptor — built from hardware_config.yaml at runtime
// ==========================================================================
struct PdoVarInfo {
    std::string yaml_key; ///< YAML key name (e.g. "status_word", "control_word")
    std::string name;     ///< PDO variable name (from YAML, e.g. "Status word")
    int         offset;   ///< byte offset within this slave's PD area
    int         size;     ///< size in bytes (sizeof(T))
};

/// Per-slave PDO layout, built dynamically from YAML config.
struct SlavePdoLayout {
    std::vector<PdoVarInfo> inputs;
    std::vector<PdoVarInfo> outputs;
    int input_size  = 0;   ///< total bytes for this slave's input area
    int output_size = 0;   ///< total bytes for this slave's output area

    /// @brief 按 yaml_key 查找 PDO 变量偏移，未找到返回 -1
    int findOffset(const std::string& yaml_key, bool is_input) const {
        const auto& vec = is_input ? inputs : outputs;
        for (const auto& v : vec) {
            if (v.yaml_key == yaml_key) return v.offset;
        }
        return -1;
    }
};

/// @brief 精简版 CiA 402 驱动状态机（仿真用）
/// 根据 control_word 位自动切换状态，返回对应的 status_word
class Cia402Sim {
public:
    Cia402Sim() = default;

    /// @brief 处理一个周期的 control_word，更新内部状态
    /// @return 当前的 status_word（uint16）
    uint16_t process(uint16_t control_word);

    /// @brief 当前是否处于 OperationEnabled 状态
    bool isEnabled() const { return state_ == 4; }  // OperationEnabled

    /// @brief 当前 CiA 402 状态值
    uint8_t state() const { return state_; }

    /// @brief 获取当前 status_word
    uint16_t statusWord() const;

private:
    uint8_t  state_{1};         // 初始 = SwitchOnDisabled
    bool     fault_reset_prev_{false};  // bit7 上升沿检测
};

// ==========================================================================
// MujocoSimulator
// ==========================================================================

class MujocoSimulator {
public:
    MujocoSimulator(const std::string &urdf_path,
                    const std::string &hw_config_path,
                    int                ecat_id       = 0,
                    double             cycle_time_us = 1000.0);

    ~MujocoSimulator();

    MujocoSimulator(const MujocoSimulator &)            = delete;
    MujocoSimulator &operator=(const MujocoSimulator &) = delete;

    // ---- Lifecycle ----------------------------------------------------
    bool initialize();
    void step();
    void run(double duration_sec = -1.0);

    void enableVisualization(bool enable = true) { visualize_ = enable; }

    // ---- Accessors ----------------------------------------------------
    double getSimTime()    const { return sim_time_; }
    int    getJointCount() const { return static_cast<int>(joints_.size()); }
    double getCycleTimeUs() const { return cycle_time_us_; }
    const std::string& getModelPath()  const { return urdf_path_; }
    const std::string& getConfigPath() const { return hw_config_path_; }

    /// @brief 获取 HardwareMJ 可视化后端指针（仅当 visualize_ 启用时非空）
    HardwareMJ* getVisualizer() const { return visualizer_; }

private:
    // ---- Initialization helpers ---------------------------------------
    bool loadModel(const std::string &urdf_path);
    bool loadHardwareConfig(const std::string &yaml_path);
    bool setupSharedMemory();

    /// Build PDO layout for one slave from its YAML config node.
    SlavePdoLayout buildPdoLayout(const std::string &type,
                                  const YAML::Node  &node);

    // ---- Per-step helpers ---------------------------------------------
    void readCommands();
    void applyControl();
    void stepPhysics();
    void writeSensorData();
    void signalClients();

    // ---- Unit conversions ---------------------------------------------
    double  cntToUnit(double cnt, double cnt_per_unit, double ratio,
                       int32_t offset_pos_cnt = 0) const;
    int32_t unitToCnt(double unit, double cnt_per_unit, double ratio,
                      int32_t offset_pos_cnt = 0) const;
    double  torqueToUnit(int16_t raw, double torque_per_unit) const;

    // ---- MuJoCo -------------------------------------------------------
    mjModel *m_ = nullptr;
    mjData  *d_ = nullptr;

    // ---- Shared memory ------------------------------------------------
    rocos::SharedMemoryConfig *shm_ = nullptr;

    // ---- Joint mapping ------------------------------------------------
    struct JointEntry {
        int         slave_id       = -1;
        int         mj_joint_qpos  = -1;
        int         mj_joint_dof   = -1;
        int         mj_actuator    = -1;
        std::string joint_name;
        int         pd_input_base  = 0;
        int         pd_output_base = 0;
        double      cnt_per_unit    = 1.0;
        double      torque_per_unit = 1.0;
        double      ratio           = 1.0;
        int32_t     offset_pos_cnt  = 0;
        SlavePdoLayout pdo;            ///< dynamic PDO layout from YAML
        // Cached command values (read from shm each cycle)
        uint16_t control_word      = 0;
        int8_t   mode_of_operation = 0;
        int32_t  target_position   = 0;
        int32_t  target_velocity   = 0;
        int16_t  target_torque     = 0;

        // CiA 402 state machine (仿真驱动状态迁移)
        Cia402Sim cia;

        // 下使能时锁死的目标位置（记录下使能瞬间的 qpos）
        double hold_position{0.0};
        bool   hold_valid{false};
    };
    std::vector<JointEntry> joints_;

    // ---- FT sensor mapping --------------------------------------------
    struct FTEntry {
        int            slave_id      = -1;
        int            pd_input_base = 0;
        SlavePdoLayout pdo;            ///< dynamic PDO layout from YAML
    };
    std::vector<FTEntry> ft_sensors_;

    // ---- IO module mapping --------------------------------------------
    struct IOEntry {
        int            slave_id       = -1;
        int            pd_input_base  = 0;
        int            pd_output_base = 0;
        SlavePdoLayout pdo;
        int di_channels = 0, do_channels = 0;
        int ai_channels = 0, ao_channels = 0;
    };
    std::vector<IOEntry> io_modules_;

    // ---- Paths & parameters -------------------------------------------
    std::string urdf_path_;
    std::string hw_config_path_;
    double      sim_time_      = 0.0;
    double      cycle_time_us_ = 1000.0;

    // ---- Visualization ------------------------------------------------
    HardwareMJ *visualizer_ = nullptr;
    bool        visualize_  = false;

    // ---- PD gains -----------------------------------------------------
    static constexpr double KP_ = 1000.0;
    static constexpr double KD_ = 50.0;
    static constexpr double KV_ = 50.0;
};

}  // namespace rocos_mujoco
