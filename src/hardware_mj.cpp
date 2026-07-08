// Copyright 2026, Yang Luo
// SPDX-License-Identifier: GPL-3.0-or-later
//
// hardware_mj.cpp
// 遵循 RoboChargeX HardwareMJ 模式。
// 渲染线程持有 GLFW 窗口、模型加载、物理步进、渲染；
// 读取线程持续上报 RobotData 给回调。

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
// 构造 / 析构
// ==========================================================================

HardwareMJ::HardwareMJ(const std::string &model_path, bool dyn)
    : model_path_(model_path), dyn_(dyn) {}

HardwareMJ::~HardwareMJ() {
    close();
    // 停止读取线程（如果启动过）
    isReading_.store(false);
    if (read_thread_.joinable()) {
        read_thread_.join();
    }
}

// ==========================================================================
// 生命周期
// ==========================================================================

bool HardwareMJ::init() {
    if (running_.load()) return true;

    running_.store(true);
    render_thread_ = std::thread(&HardwareMJ::renderLoop, this);

    // 忙等直到渲染线程发出就绪信号（与 RoboChargeX 一致）
    while (!ready_.load() && running_.load())
        std::this_thread::sleep_for(std::chrono::milliseconds(5));

    return ready_.load();
}

void HardwareMJ::close() {
    if (!running_.load()) return;
    running_.store(false);
    if (render_thread_.joinable()) render_thread_.join();
}

void HardwareMJ::setPhysicsEnabled(bool enable) {
    physics_enabled_.store(enable);
}

// ==========================================================================
// 渲染循环（运行在专用线程 —— 持有 GLFW + MuJoCo）
// ==========================================================================

void HardwareMJ::renderLoop() {
    // ---- 1. 初始化 GLFW --------------------------------------------------
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

    // ---- 2. 加载模型 -----------------------------------------------------
    char err[1000] = "Could not load model";
    m_ = mj_loadXML(model_path_.c_str(), nullptr, err, sizeof(err));
    if (!m_) {
        std::cerr << "[MJ] " << err << "\n";
        glfwDestroyWindow(window_); glfwTerminate();
        running_.store(false); return;
    }
    d_  = mj_makeData(m_);
    nu_ = m_->nv;  // DOF 数，而非 actuator 数

    // 从模型的 qpos0 初始化位置（确保不从零开始）
    if (m_->qpos0 != nullptr) {
        std::memcpy(d_->qpos, m_->qpos0, m_->nq * sizeof(mjtNum));
    }

    // 初始化缓冲区
    {
        std::lock_guard<std::mutex> lk(mtx_);
        cmd_pos_.assign(nu_, 0.0);
        cmd_vel_.assign(nu_, 0.0);
        cmd_trq_.assign(nu_, 0.0);
        sim_pos_.assign(nu_, 0.0);
        sim_vel_.assign(nu_, 0.0);
        sim_trq_.assign(nu_, 0.0);
    }

    // 传感器
    int fid = mj_name2id(m_, mjOBJ_SENSOR, "force_sensor");
    int tid = mj_name2id(m_, mjOBJ_SENSOR, "torque_sensor");
    force_adr_  = (fid >= 0) ? m_->sensor_adr[fid] : -1;
    torque_adr_ = (tid >= 0) ? m_->sensor_adr[tid] : -1;

    // ---- 3. 初始化可视化 -------------------------------------------------
    mjv_defaultCamera(&cam_);
    mjv_defaultOption(&opt_);
    mjv_defaultScene(&scn_);
    mjr_defaultContext(&con_);

    // 显示全部可视化组
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

    // ---- 4. 发出就绪信号，进入渲染循环 ----------------------------------
    ready_.store(true);

    // 先把初始状态同步到 sim_ 缓冲区
    {
        std::lock_guard<std::mutex> lk(mtx_);
        for (int i = 0; i < nu_ && i < m_->nv; ++i) {
            sim_pos_[i] = d_->qpos[i];
            sim_vel_[i] = d_->qvel[i];
            sim_trq_[i] = 0.0;
        }
    }

    while (!glfwWindowShouldClose(window_) && running_.load()) {
        if (physics_enabled_.load()) {
            // 独立模式：渲染线程同时负责物理和控制。
            // 使用 d_->ctrl 驱动内置 actuator（遵循 RoboChargeX 模式）。
            {
                std::lock_guard<std::mutex> lk(mtx_);
                for (int i = 0; i < nu_ && i < m_->nu && i < static_cast<int>(cmd_pos_.size()); ++i) {
                    d_->ctrl[i] = cmd_pos_[i];
                }
            }

            if (dyn_) mj_step(m_, d_);
            else      mj_forward(m_, d_);

            {
                std::lock_guard<std::mutex> lk(mtx_);
                for (int i = 0; i < nu_ && i < m_->nv; ++i) {
                    sim_pos_[i] = d_->qpos[i];
                    sim_vel_[i] = d_->qvel[i];
                    sim_trq_[i] = d_->qfrc_applied[i];
                }
            }
        } else {
            // 外部驱动模式：物理由 MujocoSimulator::step() 同步执行。
            // 渲染线程只从 cmd_pos_ 获取显示位姿，做 mj_forward 更新场景。
            {
                std::lock_guard<std::mutex> lk(mtx_);
                for (int i = 0; i < nu_ && i < m_->nv && i < static_cast<int>(cmd_pos_.size()); ++i) {
                    d_->qpos[i] = cmd_pos_[i];
                }
            }
            mj_forward(m_, d_);  // 仅前向运动学，不做动力学积分
        }

        mjv_updateScene(m_, d_, &opt_, nullptr, &cam_, mjCAT_ALL, &scn_);
        mjr_render(mjrRect{0, 0, width_, height_}, &scn_, &con_);
        glfwSwapBuffers(window_);
        glfwPollEvents();
    }

    // ---- 5. 清理 ---------------------------------------------------------
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
// 数据交换（线程安全）
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

// ==========================================================================
// RoboChargeX 风格单关节接口
// ==========================================================================

int HardwareMJ::GetJointNum(int *num) {
    if (!(m_ && d_)) return -1;
    *num = nu_;
    return 0;
}

int HardwareMJ::GetJointPosition(int id, double *pos) {
    if (!(m_ && d_)) return -1;
    if (id < 0 || id >= nu_) return -1;
    std::lock_guard<std::mutex> lk(mtx_);
    *pos = sim_pos_[id];
    return 0;
}

int HardwareMJ::GetJointVelocity(int id, double *vel) {
    if (!(m_ && d_)) return -1;
    if (id < 0 || id >= nu_) return -1;
    std::lock_guard<std::mutex> lk(mtx_);
    *vel = sim_vel_[id];
    return 0;
}

int HardwareMJ::GetJointTorque(int id, double *torque) {
    if (!(m_ && d_)) return -1;
    if (id < 0 || id >= nu_) return -1;
    std::lock_guard<std::mutex> lk(mtx_);
    *torque = sim_trq_[id];
    return 0;
}

int HardwareMJ::GetJointLoadTorque(int id, double *load_torque) {
    if (!(m_ && d_)) return -1;
    if (id < 0 || id >= nu_) return -1;
    std::lock_guard<std::mutex> lk(mtx_);
    // 仿真中 qfrc_actuator 即为负载力矩（与 RoboChargeX 一致）
    *load_torque = (id < m_->nu) ? d_->qfrc_actuator[id] : 0.0;
    return 0;
}

// ==========================================================================
// RoboChargeX 风格全关节接口
// ==========================================================================

int HardwareMJ::GetPosition(double *pos) {
    if (!(m_ && d_)) return -1;
    std::lock_guard<std::mutex> lk(mtx_);
    for (int i = 0; i < nu_; ++i) pos[i] = sim_pos_[i];
    return 0;
}

int HardwareMJ::GetVelocity(double *vel) {
    if (!(m_ && d_)) return -1;
    std::lock_guard<std::mutex> lk(mtx_);
    for (int i = 0; i < nu_; ++i) vel[i] = sim_vel_[i];
    return 0;
}

int HardwareMJ::GetTorque(double *torque) {
    if (!(m_ && d_)) return -1;
    std::lock_guard<std::mutex> lk(mtx_);
    for (int i = 0; i < nu_; ++i) torque[i] = sim_trq_[i];
    return 0;
}

int HardwareMJ::GetLoadTorque(double *load_torque) {
    if (!(m_ && d_)) return -1;
    std::lock_guard<std::mutex> lk(mtx_);
    for (int i = 0; i < nu_ && i < m_->nu; ++i)
        load_torque[i] = d_->qfrc_actuator[i];
    return 0;
}

int HardwareMJ::GetWrench(double *wrench) {
    if (!m_ || !d_) return -1;
    std::lock_guard<std::mutex> lk(mtx_);
    if (force_adr_ >= 0 && torque_adr_ >= 0) {
        for (int k = 0; k < 3; ++k) {
            wrench[k]     = d_->sensordata[force_adr_  + k];
            wrench[k + 3] = d_->sensordata[torque_adr_ + k];
        }
        return 0;
    }
    std::fill(wrench, wrench + 6, 0.0);
    return 0;
}

// ==========================================================================
// 使能 / 去使能（仿真侧为空操作 —— 控制由 MujocoSimulator 处理）
// ==========================================================================

int HardwareMJ::SetEnabled()        { return 0; }
int HardwareMJ::SetDisabled()       { return 0; }
int HardwareMJ::SetJointEnabled(int /*id*/)  { return 0; }
int HardwareMJ::SetJointDisabled(int /*id*/) { return 0; }

// ==========================================================================
// RoboChargeX 风格指令接口
// ==========================================================================

int HardwareMJ::SetJointPosition(int id, double pos) {
    if (!(m_ && d_)) return -1;
    if (id < 0 || id >= nu_) return -1;
    std::lock_guard<std::mutex> lk(mtx_);
    if (dyn_)
        d_->ctrl[id] = pos;
    else
        d_->qpos[id] = pos;
    cmd_pos_[id] = pos;
    return 0;
}

int HardwareMJ::SetJointVelocity(int /*id*/, double /*vel*/) {
    // 仿真中速度模式暂不支持，返回 -1（与 RoboChargeX 一致）
    return -1;
}

int HardwareMJ::SetJointTorque(int /*id*/, double /*torque*/) {
    // 仿真中力矩模式暂不支持，返回 -1（与 RoboChargeX 一致）
    return -1;
}

int HardwareMJ::SetPosition(double *pos) {
    if (!(m_ && d_)) return -1;
    std::lock_guard<std::mutex> lk(mtx_);
    for (int i = 0; i < nu_; ++i) {
        if (dyn_)
            d_->ctrl[i] = pos[i];
        else
            d_->qpos[i] = pos[i];
        cmd_pos_[i] = pos[i];
    }
    return 0;
}

// ==========================================================================
// 回调（与 RoboChargeX 一致）
// ==========================================================================

int HardwareMJ::SetCallback(Callback callback) {
    callback_ = std::move(callback);

    // 如果设置回调且还没有读取线程，则启动（与 RoboChargeX 模式一致）
    if (callback_ && !isReading_.load()) {
        isReading_.store(true);
        read_thread_ = std::thread([this]() {
            while (isReading_.load()) {
                if (callback_) {
                    RobotData rd;
                    fillRobotData(rd);
                    callback_(rd);
                }
                std::this_thread::sleep_for(std::chrono::milliseconds(4));
            }
        });
    }
    return 0;
}

void HardwareMJ::fillRobotData(RobotData &rd) {
    if (!(m_ && d_)) return;

    std::lock_guard<std::mutex> lk(mtx_);

    rd.joint_count = nu_;

    rd.pos.resize(nu_);
    rd.vel.resize(nu_);
    rd.torque.resize(nu_);
    rd.load_torque.resize(nu_);
    rd.cmd_pos.resize(nu_);
    rd.cmd_vel.resize(nu_);
    rd.cmd_torque.resize(nu_);
    rd.joint_enabled.assign(nu_, true);   // 仿真里默认全部使能

    for (int i = 0; i < nu_; ++i) {
        rd.pos[i]         = sim_pos_[i];
        rd.vel[i]         = sim_vel_[i];
        rd.torque[i]      = sim_trq_[i];
        rd.load_torque[i] = (i < m_->nu) ? d_->qfrc_actuator[i] : 0.0;
        rd.cmd_pos[i]     = cmd_pos_[i];
        rd.cmd_vel[i]     = 0;               // 仿真中无此数据
        rd.cmd_torque[i]  = 0;
    }

    rd.wrench.resize(6);
    if (force_adr_ >= 0 && torque_adr_ >= 0) {
        for (int k = 0; k < 3; ++k) {
            rd.wrench[k]     = d_->sensordata[force_adr_  + k];
            rd.wrench[k + 3] = d_->sensordata[torque_adr_ + k];
        }
    } else {
        std::fill(rd.wrench.begin(), rd.wrench.end(), 0);
    }
}

// ==========================================================================
// 静态 GLFW 回调
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
