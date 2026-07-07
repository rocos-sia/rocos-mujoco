// Copyright 2026, Yang Luo
// SPDX-License-Identifier: GPL-3.0-or-later
//
// hardware_mj.hpp
// Standalone MuJoCo hardware interface — follows the RoboChargeX
// HardwareMJ pattern.  The render thread owns the GLFW window,
// model loading, physics stepping, and rendering.

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

class HardwareMJ {
public:
    /// @param model_path   Path to MJCF/URDF model file.
    /// @param dyn          true = mj_step (dynamics), false = mj_forward
    ///                     (kinematics only).
    explicit HardwareMJ(const std::string &model_path, bool dyn = true);
    ~HardwareMJ();

    // ---- Lifecycle ----------------------------------------------------
    bool init();              // start render thread, block until ready
    void close();             // stop render thread, cleanup
    bool isRunning() const { return running_.load(); }

    // ---- Data exchange (thread-safe) ----------------------------------
    void readState(std::vector<double> &pos,
                   std::vector<double> &vel,
                   std::vector<double> &torque);
    void writeCommand(const std::vector<double> &cmd_pos,
                      const std::vector<double> &cmd_vel,
                      const std::vector<double> &cmd_trq);

    // ---- Direct access ------------------------------------------------
    mjModel *model() const { return m_; }
    mjData  *data()  const { return d_; }
    int      jointNum() const { return nu_; }

    int  getWrench(double *wrench);
    void getPosition(double *pos);
    void getVelocity(double *vel);
    void getTorque(double *torque);
    void setPosition(const double *pos);
    void setTorque(const double *torque);

private:
    void renderLoop();

    // ---- GLFW callbacks (static, use glfwGetWindowUserPointer) --------
    static void cbKeyboard(GLFWwindow *w, int key, int, int act, int);
    static void cbMouseButton(GLFWwindow *w, int btn, int act, int);
    static void cbMouseMove(GLFWwindow *w, double x, double y);
    static void cbScroll(GLFWwindow *w, double, double yoff);
    static void cbResize(GLFWwindow *w, int w2, int h2);

    // ---- Model --------------------------------------------------------
    std::string model_path_;
    bool        dyn_;
    mjModel    *m_  = nullptr;
    mjData     *d_  = nullptr;
    int         nu_ = 0;

    // ---- GLFW ---------------------------------------------------------
    GLFWwindow *window_ = nullptr;
    int  width_  = 1400;
    int  height_ = 900;

    // ---- MuJoCo visualization -----------------------------------------
    mjvCamera  cam_;
    mjvOption  opt_;
    mjvScene   scn_;
    mjrContext con_;

    // ---- Mouse state --------------------------------------------------
    bool   btn_left_   = false;
    bool   btn_right_  = false;
    bool   btn_middle_ = false;
    double last_x_ = 0.0;
    double last_y_ = 0.0;
    static constexpr double kCameraSpeed = 1.0;
    static constexpr double kZoomSpeed   = 0.1;
    static constexpr double kShiftScale  = 0.5;

    // ---- Sensors ------------------------------------------------------
    int force_adr_  = -1;
    int torque_adr_ = -1;

    // ---- Threading ----------------------------------------------------
    std::thread        render_thread_;
    std::mutex         mtx_;
    std::atomic<bool>  running_{false};
    std::atomic<bool>  ready_{false};

    // ---- State mirror (protected by mtx_) -----------------------------
    std::vector<double> cmd_pos_, cmd_vel_, cmd_trq_;
    std::vector<double> sim_pos_, sim_vel_, sim_trq_;
};

}  // namespace rocos_mujoco
