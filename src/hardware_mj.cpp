// Copyright 2026, Yang Luo
// SPDX-License-Identifier: GPL-3.0-or-later
//
// hardware_mj.cpp
// Follows the RoboChargeX HardwareMJ pattern exactly.
// Render thread owns GLFW window, model loading, physics, rendering.

#include "rocos_mujoco/hardware_mj.hpp"

#include <GLFW/glfw3.h>
#include <mujoco/mujoco.h>

#include <algorithm>
#include <chrono>
#include <cstdio>
#include <cstring>
#include <iostream>
#include <thread>

namespace rocos_mujoco {

// ==========================================================================
// Construction / Destruction
// ==========================================================================

HardwareMJ::HardwareMJ(const std::string &model_path, bool dyn)
    : model_path_(model_path), dyn_(dyn) {}

HardwareMJ::~HardwareMJ() { close(); }

// ==========================================================================
// Lifecycle
// ==========================================================================

bool HardwareMJ::init() {
    if (running_.load()) return true;

    running_.store(true);
    render_thread_ = std::thread(&HardwareMJ::renderLoop, this);

    // Busy-wait until the render thread signals readiness
    while (!ready_.load() && running_.load())
        std::this_thread::sleep_for(std::chrono::milliseconds(5));

    return ready_.load();
}

void HardwareMJ::close() {
    if (!running_.load()) return;
    running_.store(false);
    if (render_thread_.joinable()) render_thread_.join();
}

// ==========================================================================
// Render loop (runs in dedicated thread — owns GLFW + MuJoCo)
// ==========================================================================

void HardwareMJ::renderLoop() {
    // ---- 1. Init GLFW --------------------------------------------------
    if (!glfwInit()) {
        std::cerr << "[MJ] glfwInit failed\n";
        running_.store(false); return;
    }
    glfwWindowHint(GLFW_SAMPLES, 4);
    window_ = glfwCreateWindow(width_, height_,
                               "MuJoCo Model Display", nullptr, nullptr);
    if (!window_) {
        std::cerr << "[MJ] glfwCreateWindow failed\n";
        glfwTerminate(); running_.store(false); return;
    }
    glfwMakeContextCurrent(window_);
    glfwSwapInterval(1);

    glfwSetWindowUserPointer(window_, this);
    glfwSetKeyCallback(window_,        &HardwareMJ::cbKeyboard);
    glfwSetMouseButtonCallback(window_, &HardwareMJ::cbMouseButton);
    glfwSetCursorPosCallback(window_,   &HardwareMJ::cbMouseMove);
    glfwSetScrollCallback(window_,      &HardwareMJ::cbScroll);
    glfwSetWindowSizeCallback(window_,  &HardwareMJ::cbResize);

    // ---- 2. Load model -------------------------------------------------
    char err[1000] = "Could not load model";
    m_ = mj_loadXML(model_path_.c_str(), nullptr, err, sizeof(err));
    if (!m_) {
        std::cerr << "[MJ] " << err << "\n";
        glfwDestroyWindow(window_); glfwTerminate();
        running_.store(false); return;
    }
    d_  = mj_makeData(m_);
    nu_ = m_->nu;

    // Resize buffers
    {
        std::lock_guard<std::mutex> lk(mtx_);
        cmd_pos_.assign(nu_, 0.0);
        cmd_vel_.assign(nu_, 0.0);
        cmd_trq_.assign(nu_, 0.0);
        sim_pos_.assign(nu_, 0.0);
        sim_vel_.assign(nu_, 0.0);
        sim_trq_.assign(nu_, 0.0);
    }

    // Sensors
    int fid = mj_name2id(m_, mjOBJ_SENSOR, "force_sensor");
    int tid = mj_name2id(m_, mjOBJ_SENSOR, "torque_sensor");
    force_adr_  = (fid >= 0) ? m_->sensor_adr[fid] : -1;
    torque_adr_ = (tid >= 0) ? m_->sensor_adr[tid] : -1;

    // ---- 3. Init visualization -----------------------------------------
    // MJCF models (charge_arm.xml) define their own <visual> settings
    // (headlight, haze, skybox), so we keep defaults and just show all groups.
    mjv_defaultCamera(&cam_);
    mjv_defaultOption(&opt_);
    mjv_defaultScene(&scn_);
    mjr_defaultContext(&con_);

    for (int g = 0; g < mjNGROUP; ++g) {
        opt_.geomgroup[g]    = 1;
        opt_.sitegroup[g]    = 1;
        opt_.jointgroup[g]   = 1;
        opt_.tendongroup[g]  = 1;
        opt_.actuatorgroup[g] = 1;
    }
    cam_.distance  = 3.0;
    cam_.azimuth   = 130.0;
    cam_.elevation = -20.0;
    cam_.lookat[0] = 0.0;
    cam_.lookat[1] = 0.0;
    cam_.lookat[2] = 0.5;

    mjv_makeScene(m_, &scn_, 2000);
    mjr_makeContext(m_, &con_, 200);

    // ---- 4. Signal readiness + enter render loop -----------------------
    ready_.store(true);

    // Note: MJCF models (like charge_arm.xml) define their own
    // <visual> settings including skybox texture — no manual
    // fog override needed.

    while (!glfwWindowShouldClose(window_) && running_.load()) {
        // Apply commands from external controller
        {
            std::lock_guard<std::mutex> lk(mtx_);
            for (int i = 0; i < nu_; ++i) {
                if (i < static_cast<int>(cmd_pos_.size()))
                    d_->ctrl[i] = cmd_pos_[i];
            }
        }

        // Step physics
        if (dyn_) mj_step(m_, d_);
        else      mj_forward(m_, d_);

        // Mirror state for external readers
        {
            std::lock_guard<std::mutex> lk(mtx_);
            for (int i = 0; i < nu_; ++i) {
                sim_pos_[i] = d_->qpos[i];
                sim_vel_[i] = d_->qvel[i];
                sim_trq_[i] = d_->qfrc_applied[i];
            }
        }

        // Render
        mjv_updateScene(m_, d_, &opt_, nullptr, &cam_, mjCAT_ALL, &scn_);
        mjr_render(mjrRect{0, 0, width_, height_}, &scn_, &con_);
        glfwSwapBuffers(window_);
        glfwPollEvents();
    }

    // ---- 5. Cleanup ----------------------------------------------------
    ready_.store(false);
    mjv_freeScene(&scn_);
    mjr_freeContext(&con_);
    mj_deleteData(d_);   d_ = nullptr;
    mj_deleteModel(m_);  m_ = nullptr;
    glfwDestroyWindow(window_);
    glfwTerminate();
    window_ = nullptr;
    running_.store(false);
}

// ==========================================================================
// Data exchange (thread-safe)
// ==========================================================================

void HardwareMJ::readState(std::vector<double> &pos,
                           std::vector<double> &vel,
                           std::vector<double> &torque) {
    std::lock_guard<std::mutex> lk(mtx_);
    pos    = sim_pos_;
    vel    = sim_vel_;
    torque = sim_trq_;
}

void HardwareMJ::writeCommand(const std::vector<double> &cp,
                              const std::vector<double> &cv,
                              const std::vector<double> &ct) {
    std::lock_guard<std::mutex> lk(mtx_);
    if (!cp.empty()) cmd_pos_ = cp;
    if (!cv.empty()) cmd_vel_ = cv;
    if (!ct.empty()) cmd_trq_ = ct;
}

void HardwareMJ::getPosition(double *pos) {
    std::lock_guard<std::mutex> lk(mtx_);
    for (int i = 0; i < nu_; ++i) pos[i] = sim_pos_[i];
}
void HardwareMJ::getVelocity(double *vel) {
    std::lock_guard<std::mutex> lk(mtx_);
    for (int i = 0; i < nu_; ++i) vel[i] = sim_vel_[i];
}
void HardwareMJ::getTorque(double *torque) {
    std::lock_guard<std::mutex> lk(mtx_);
    for (int i = 0; i < nu_; ++i) torque[i] = sim_trq_[i];
}
void HardwareMJ::setPosition(const double *pos) {
    std::lock_guard<std::mutex> lk(mtx_);
    for (int i = 0; i < nu_; ++i) cmd_pos_[i] = pos[i];
}
void HardwareMJ::setTorque(const double *torque) {
    std::lock_guard<std::mutex> lk(mtx_);
    for (int i = 0; i < nu_; ++i) cmd_trq_[i] = torque[i];
}

int HardwareMJ::getWrench(double *w) {
    if (!m_ || !d_) return -1;
    std::lock_guard<std::mutex> lk(mtx_);
    if (force_adr_ >= 0 && torque_adr_ >= 0) {
        for (int k = 0; k < 3; ++k) {
            w[k]   = d_->sensordata[force_adr_  + k];
            w[k+3] = d_->sensordata[torque_adr_ + k];
        }
        return 0;
    }
    std::fill(w, w + 6, 0.0);
    return 0;
}

// ==========================================================================
// Static GLFW callbacks
// ==========================================================================

void HardwareMJ::cbKeyboard(GLFWwindow *w, int key, int, int act, int) {
    auto *p = static_cast<HardwareMJ *>(glfwGetWindowUserPointer(w));
    if (act == GLFW_PRESS && key == GLFW_KEY_BACKSPACE)
        mjv_defaultCamera(&p->cam_);
}

void HardwareMJ::cbMouseButton(GLFWwindow *w, int btn, int act, int) {
    auto *p = static_cast<HardwareMJ *>(glfwGetWindowUserPointer(w));
    if (act == GLFW_PRESS) {
        if (btn == GLFW_MOUSE_BUTTON_LEFT)   p->btn_left_   = true;
        if (btn == GLFW_MOUSE_BUTTON_RIGHT)  p->btn_right_  = true;
        if (btn == GLFW_MOUSE_BUTTON_MIDDLE) p->btn_middle_ = true;
        glfwGetCursorPos(w, &p->last_x_, &p->last_y_);
    } else {
        if (btn == GLFW_MOUSE_BUTTON_LEFT)   p->btn_left_   = false;
        if (btn == GLFW_MOUSE_BUTTON_RIGHT)  p->btn_right_  = false;
        if (btn == GLFW_MOUSE_BUTTON_MIDDLE) p->btn_middle_ = false;
    }
}

void HardwareMJ::cbMouseMove(GLFWwindow *w, double x, double y) {
    auto *p = static_cast<HardwareMJ *>(glfwGetWindowUserPointer(w));
    if (!p->m_) return;
    if (!p->btn_left_ && !p->btn_right_ && !p->btn_middle_) return;

    double dx = (x - p->last_x_) / p->width_;
    double dy = (y - p->last_y_) / p->height_;
    if (glfwGetKey(w, GLFW_KEY_LEFT_SHIFT) == GLFW_PRESS) {
        dx *= p->kShiftScale; dy *= p->kShiftScale;
    }

    if (p->btn_left_)
        mjv_moveCamera(p->m_, mjMOUSE_ROTATE_V,
                       dx * p->kCameraSpeed, dy * p->kCameraSpeed,
                       &p->scn_, &p->cam_);
    else if (p->btn_right_)
        mjv_moveCamera(p->m_, mjMOUSE_MOVE_V,
                       dx * p->kCameraSpeed, dy * p->kCameraSpeed,
                       &p->scn_, &p->cam_);
    else if (p->btn_middle_)
        mjv_moveCamera(p->m_, mjMOUSE_ZOOM, 0,
                       dy * p->kZoomSpeed * p->kCameraSpeed,
                       &p->scn_, &p->cam_);

    p->last_x_ = x; p->last_y_ = y;
}

void HardwareMJ::cbScroll(GLFWwindow *w, double, double yoff) {
    auto *p = static_cast<HardwareMJ *>(glfwGetWindowUserPointer(w));
    if (!p->m_) return;
    double dy = -yoff * p->kZoomSpeed;
    if (glfwGetKey(w, GLFW_KEY_LEFT_SHIFT) == GLFW_PRESS) dy *= p->kShiftScale;
    mjv_moveCamera(p->m_, mjMOUSE_ZOOM, 0, dy, &p->scn_, &p->cam_);
}

void HardwareMJ::cbResize(GLFWwindow *w, int w2, int h2) {
    auto *p = static_cast<HardwareMJ *>(glfwGetWindowUserPointer(w));
    p->width_  = w2;
    p->height_ = (h2 == 0) ? 1 : h2;
}

}  // namespace rocos_mujoco
