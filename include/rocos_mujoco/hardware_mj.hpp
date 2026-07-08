// Copyright 2026, Yang Luo
// SPDX-License-Identifier: GPL-3.0-or-later
//
// hardware_mj.hpp
// MuJoCo 硬件接口 —— 遵循 RoboChargeX HardwareMJ 模式。
// 渲染线程持有 GLFW 窗口、模型加载、物理步进和渲染；
// 读取线程持续上报 RobotData 给回调（与 RoboChargeX 一致）。

#pragma once

#include <mujoco/mujoco.h>

#include <atomic>
#include <functional>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

struct GLFWwindow;

namespace rocos_mujoco {

/// @brief 仿真侧机器人数据 —— 与 RoboChargeX HardwareInterface::RobotData 对齐
struct RobotData {
    int joint_count{0};
    std::vector<double> pos;
    std::vector<double> vel;
    std::vector<double> torque;
    std::vector<double> load_torque;
    std::vector<double> wrench;
    std::vector<double> cmd_pos;
    std::vector<double> cmd_vel;
    std::vector<double> cmd_torque;
    std::vector<int> joint_state;
    std::vector<bool> joint_enabled;
};

using Callback = std::function<void(RobotData& robot_data)>;

class HardwareMJ {
public:
    /// @param model_path   MJCF/URDF 模型文件路径（与 RoboChargeX 不同：RoboChargeX 用 model_id，
    ///                     我们用路径，更灵活 —— 符合 rocos-mujoco 独立子模块的角色）
    /// @param dyn          true = mj_step（完整动力学），false = mj_forward（仅运动学）
    explicit HardwareMJ(const std::string &model_path, bool dyn = true);
    ~HardwareMJ();

    // ---- 生命周期 ------------------------------------------------
    bool init();              // 启动渲染线程，阻塞直至就绪
    void close();             // 停止渲染线程，清理 GLFW + MuJoCo
    bool isRunning() const { return running_.load(); }

    /// @brief 开启/关闭渲染线程内的物理步进。
    ///        关闭时仅做 mj_forward + 渲染，物理由外部驱动。
    void setPhysicsEnabled(bool enable);

    // ---- 数据交换（线程安全）--------------------------------------
    void readState(std::vector<double> &pos,
                   std::vector<double> &vel,
                   std::vector<double> &torque);
    void writeCommand(const std::vector<double> &cmd_pos,
                      const std::vector<double> &cmd_vel,
                      const std::vector<double> &cmd_trq);

    // ---- 直接访问 -------------------------------------------------
    mjModel *model() const { return m_; }
    mjData  *data()  const { return d_; }
    int      jointNum() const { return nu_; }

    // ---- RoboChargeX 风格单关节接口 --------------------------------
    int  GetJointNum(int *num);
    int  GetJointPosition(int id, double *pos);
    int  GetJointVelocity(int id, double *vel);
    int  GetJointTorque(int id, double *torque);
    int  GetJointLoadTorque(int id, double *load_torque);

    // ---- RoboChargeX 风格全关节接口 --------------------------------
    int  GetPosition(double *pos);
    int  GetVelocity(double *vel);
    int  GetTorque(double *torque);
    int  GetLoadTorque(double *load_torque);
    int  GetWrench(double *wrench);

    // ---- 使能/去使能（仿真侧为空操作，真实控制由 MujocoSimulator 处理）---
    int  SetEnabled();
    int  SetDisabled();
    int  SetJointEnabled(int id);
    int  SetJointDisabled(int id);

    // ---- RoboChargeX 风格指令接口 ----------------------------------
    int  SetJointPosition(int id, double pos);
    int  SetJointVelocity(int id, double vel);
    int  SetJointTorque(int id, double torque);
    int  SetPosition(double *pos);

    // ---- 回调（与 RoboChargeX 一致）--------------------------------
    int  SetCallback(Callback callback);

    // ---- 传感器 ---------------------------------------------------
    std::string name() const { return "HardwareMJ (MuJoCo)"; }

private:
    void renderLoop();
    void fillRobotData(RobotData &rd);

    // ---- GLFW 回调（静态，使用 glfwGetWindowUserPointer）-----------
    static void cbKeyboard(GLFWwindow *w, int key, int, int act, int);
    static void cbMouseButton(GLFWwindow *w, int btn, int act, int);
    static void cbMouseMove(GLFWwindow *w, double x, double y);
    static void cbScroll(GLFWwindow *w, double, double yoff);
    static void cbResize(GLFWwindow *w, int w2, int h2);

    // ---- 模型 -----------------------------------------------------
    std::string model_path_;
    bool        dyn_;
    mjModel    *m_  = nullptr;
    mjData     *d_  = nullptr;
    int         nu_ = 0;

    // ---- GLFW ----------------------------------------------------
    GLFWwindow *window_ = nullptr;
    int  width_  = 1400;
    int  height_ = 900;

    // ---- MuJoCo 可视化 --------------------------------------------
    mjvCamera  cam_;
    mjvOption  opt_;
    mjvScene   scn_;
    mjrContext con_;

    // ---- 鼠标状态 ------------------------------------------------
    bool   btn_left_   = false;
    bool   btn_right_  = false;
    bool   btn_middle_ = false;
    double last_x_ = 0.0;
    double last_y_ = 0.0;
    static constexpr double kCameraSpeed = 1.0;
    static constexpr double kZoomSpeed   = 0.1;
    static constexpr double kShiftScale  = 0.5;

    // ---- 传感器 --------------------------------------------------
    int force_adr_  = -1;
    int torque_adr_ = -1;

    // ---- 线程 ----------------------------------------------------
    std::thread        render_thread_;
    std::thread        read_thread_;
    std::atomic<bool>  isReading_{false};
    std::mutex         mtx_;
    std::atomic<bool>  running_{false};
    std::atomic<bool>  ready_{false};
    std::atomic<bool>  physics_enabled_{true};  // 关闭时仅渲染，物理由外部驱动

    // ---- 状态镜像（mtx_ 保护）-------------------------------------
    std::vector<double> cmd_pos_, cmd_vel_, cmd_trq_;
    std::vector<double> sim_pos_, sim_vel_, sim_trq_;

    // ---- 回调 ----------------------------------------------------
    Callback callback_;
};

}  // namespace rocos_mujoco
