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
namespace {

// ==========================================================================
// Camera ray helper — compute world-space ray from mouse pixel coordinates
// ==========================================================================
struct CameraRay {
    mjtNum origin[3];
    mjtNum direction[3];
};

CameraRay computeCameraRay(const mjvScene &scn,
                           double mouse_x, double mouse_y,
                           int viewport_w, int viewport_h) {
    CameraRay ray{};
    const auto &cam = scn.camera[0];

    // 归一化设备坐标 [-1, 1]，左上角为 (-1, 1)，右下角为 (1, -1)
    double nx = (2.0 * mouse_x) / static_cast<double>(viewport_w) - 1.0;
    double ny = 1.0 - (2.0 * mouse_y) / static_cast<double>(viewport_h);

    // 相机在世界坐标系中的基向量
    mjtNum forward[3] = {cam.forward[0],  cam.forward[1],  cam.forward[2]};
    mjtNum up[3]      = {cam.up[0],       cam.up[1],       cam.up[2]};
    mjtNum right[3];
    mju_cross(right, forward, up);
    mju_normalize3(right);

    // MuJoCo 的 frustum 定义在 frustum_near 处（通常 ~0.01），而非单位距离。
    // 必须缩放到 z=1，否则所有像素的射线方向几乎完全相同 —— 这就是
    // "始终点选同一个位置" 的根本原因。
    double znear = cam.frustum_near;
    if (znear < 1e-6) znear = 0.01;  // 安全兜底
    double inv_znear = 1.0 / znear;

    // 近平面处的半高 / 半宽
    double vert_half_near   = (cam.frustum_top - cam.frustum_bottom) * 0.5;
    double vert_center_near = (cam.frustum_top + cam.frustum_bottom) * 0.5;
    double aspect = static_cast<double>(viewport_w) /
                    static_cast<double>(viewport_h);

    // 缩放到单位距离 (z=1)
    double half_h_at_1   = vert_half_near * inv_znear;    // = tan(fovy/2)
    double center_at_1_h = cam.frustum_center * inv_znear;
    double center_at_1_v = vert_center_near * inv_znear;

    // 相机空间中的目标点（z=1 沿 forward）
    double cx = center_at_1_h + nx * half_h_at_1 * aspect;
    double cy = center_at_1_v + ny * half_h_at_1;

    // 世界空间中的射线方向
    ray.direction[0] = cx * right[0] + cy * up[0] + forward[0];
    ray.direction[1] = cx * right[1] + cy * up[1] + forward[1];
    ray.direction[2] = cx * right[2] + cy * up[2] + forward[2];
    mju_normalize3(ray.direction);

    ray.origin[0] = cam.pos[0];
    ray.origin[1] = cam.pos[1];
    ray.origin[2] = cam.pos[2];

    return ray;
}

}  // namespace (anonymous)

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
    nv_ = m_->nv;  // DOF 数，而非 actuator 数

    // 从模型的 qpos0 初始化位置（确保不从零开始）
    if (m_->qpos0 != nullptr) {
        std::memcpy(d_->qpos, m_->qpos0, m_->nq * sizeof(mjtNum));
    }

    // 初始化缓冲区（按 DOF 数量）
    {
        std::lock_guard<std::mutex> lk(mtx_);
        cmd_pos_.assign(nv_, 0.0);
        cmd_vel_.assign(nv_, 0.0);
        cmd_trq_.assign(nv_, 0.0);
        sim_pos_.assign(nv_, 0.0);
        sim_vel_.assign(nv_, 0.0);
        sim_trq_.assign(nv_, 0.0);
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
        for (int i = 0; i < nv_ && i < m_->nv; ++i) {
            sim_pos_[i] = d_->qpos[i];
            sim_vel_[i] = d_->qvel[i];
            sim_trq_[i] = 0.0;
        }
    }

    while (!glfwWindowShouldClose(window_) && running_.load()) {
        if (physics_enabled_.load()) {
            // 独立模式：渲染线程同时负责物理和控制。
            // 使用 d_->ctrl 驱动内置 actuator（遵循 RoboChargeX 模式）。
            // ctrl 按执行器索引，cmd_pos_ 按 DOF 索引 —— 取更小的那个。
            {
                std::lock_guard<std::mutex> lk(mtx_);
                int nact = std::min(m_->nu, static_cast<int>(cmd_pos_.size()));
                for (int i = 0; i < nact; ++i) {
                    d_->ctrl[i] = cmd_pos_[i];
                }
            }

            if (dyn_) mj_step(m_, d_);
            else      mj_forward(m_, d_);

            {
                std::lock_guard<std::mutex> lk(mtx_);
                for (int i = 0; i < nv_ && i < m_->nv; ++i) {
                    sim_pos_[i] = d_->qpos[i];
                    sim_vel_[i] = d_->qvel[i];
                    sim_trq_[i] = d_->qfrc_applied[i];
                }
            }
        } else {
            // 外部驱动模式：物理由 MujocoSimulator::step() 同步执行。
            // 渲染线程只从 cmd_pos_ 获取显示位姿，做 mj_forward 更新场景。
            // mj_forward 必须在锁内执行 —— 与 MujocoSimulator::step() 竞争 d_。
            {
                std::lock_guard<std::mutex> lk(mtx_);
                int nq = std::min(m_->nq, static_cast<int>(cmd_pos_.size()));
                for (int i = 0; i < nq; ++i) {
                    d_->qpos[i] = cmd_pos_[i];
                }
                mj_forward(m_, d_);  // 仅前向运动学，不做动力学积分
            }
        }

        mjv_updateScene(m_, d_, &opt_, nullptr, &cam_, mjCAT_ALL, &scn_);

        // ---- 外力可视化 ----------------------------------------------------
        if (force_state_.active && force_state_.body_id >= 0 &&
            force_state_.body_id < m_->nbody) {
            int b = force_state_.body_id;

            // 更新受力点的世界坐标（从 body-local 转换）
            mjtNum bw[3];
            mju_mulMatVec(bw, d_->xmat + 9 * b, force_state_.local_pos, 3, 3);
            force_state_.world_pos[0] = d_->xpos[3 * b + 0] + bw[0];
            force_state_.world_pos[1] = d_->xpos[3 * b + 1] + bw[1];
            force_state_.world_pos[2] = d_->xpos[3 * b + 2] + bw[2];

            // 受力点小球
            if (scn_.ngeom < scn_.maxgeom) {
                auto *g   = scn_.geoms + scn_.ngeom;
                float  red[4]       = {1.0f, 0.15f, 0.1f, 0.9f};
                mjtNum sphere_sz[3] = {0.015, 0, 0};
                mjv_initGeom(g, mjGEOM_SPHERE, sphere_sz,
                             force_state_.world_pos, nullptr, red);
                scn_.ngeom++;
            }

            // 力方向箭头
            if (scn_.ngeom < scn_.maxgeom) {
                auto *g   = scn_.geoms + scn_.ngeom;
                float  yellow[4]     = {1.0f, 0.75f, 0.1f, 0.9f};
                mjtNum arrow_sz[3]   = {0.004, 0.012, 0.025};
                mjtNum arrow_end[3];
                double arrow_len =
                    0.06 + force_state_.magnitude * 0.004;
                for (int k = 0; k < 3; ++k)
                    arrow_end[k] = force_state_.world_pos[k] +
                                   force_state_.direction[k] * arrow_len;
                mjv_initGeom(g, mjGEOM_ARROW, arrow_sz,
                             nullptr, nullptr, yellow);
                mjv_connector(g, mjGEOM_ARROW, 0.006,
                              force_state_.world_pos, arrow_end);
                scn_.ngeom++;
            }
        }

        mjr_render(mjrRect{0, 0, width_, height_}, &scn_, &con_);

        // ---- 外力信息文字叠层（必须在 mjr_render 之后，否则会被覆盖）--------
        if (force_state_.active && force_state_.body_id >= 0) {
            char title[128], body[256];
            std::snprintf(title, sizeof(title),
                          "EXTERNAL FORCE  |  Body: %s (id=%d)",
                          mj_id2name(m_, mjOBJ_BODY,
                                     force_state_.body_id),
                          force_state_.body_id);
            std::snprintf(body, sizeof(body),
                          "Magnitude: %.1f N\n"
                          "Direction: [%+.3f, %+.3f, %+.3f]\n"
                          "[+/-] or Scroll  adjust magnitude\n"
                          "[Esc]  clear force\n"
                          "[Drag] change direction",
                          force_state_.magnitude,
                          force_state_.direction[0],
                          force_state_.direction[1],
                          force_state_.direction[2]);
            mjr_overlay(mjFONT_SHADOW, mjGRID_TOPLEFT,
                        mjrRect{0, 0, width_, height_},
                        title, body, &con_);
        }
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
    *num = nv_;
    return 0;
}

int HardwareMJ::GetJointPosition(int id, double *pos) {
    if (!(m_ && d_)) return -1;
    if (id < 0 || id >= nv_) return -1;
    std::lock_guard<std::mutex> lk(mtx_);
    *pos = sim_pos_[id];
    return 0;
}

int HardwareMJ::GetJointVelocity(int id, double *vel) {
    if (!(m_ && d_)) return -1;
    if (id < 0 || id >= nv_) return -1;
    std::lock_guard<std::mutex> lk(mtx_);
    *vel = sim_vel_[id];
    return 0;
}

int HardwareMJ::GetJointTorque(int id, double *torque) {
    if (!(m_ && d_)) return -1;
    if (id < 0 || id >= nv_) return -1;
    std::lock_guard<std::mutex> lk(mtx_);
    *torque = sim_trq_[id];
    return 0;
}

int HardwareMJ::GetJointLoadTorque(int id, double *load_torque) {
    if (!(m_ && d_)) return -1;
    if (id < 0 || id >= nv_) return -1;
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
    for (int i = 0; i < nv_; ++i) pos[i] = sim_pos_[i];
    return 0;
}

int HardwareMJ::GetVelocity(double *vel) {
    if (!(m_ && d_)) return -1;
    std::lock_guard<std::mutex> lk(mtx_);
    for (int i = 0; i < nv_; ++i) vel[i] = sim_vel_[i];
    return 0;
}

int HardwareMJ::GetTorque(double *torque) {
    if (!(m_ && d_)) return -1;
    std::lock_guard<std::mutex> lk(mtx_);
    for (int i = 0; i < nv_; ++i) torque[i] = sim_trq_[i];
    return 0;
}

int HardwareMJ::GetLoadTorque(double *load_torque) {
    if (!(m_ && d_)) return -1;
    std::lock_guard<std::mutex> lk(mtx_);
    int nact = std::min(nv_, m_->nu);
    for (int i = 0; i < nact; ++i)
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
    if (id < 0 || id >= nv_) return -1;
    std::lock_guard<std::mutex> lk(mtx_);
    if (dyn_ && id < m_->nu)
        d_->ctrl[id] = pos;   // ctrl 按执行器索引
    else if (!dyn_ && id < m_->nq)
        d_->qpos[id] = pos;   // qpos 按广义坐标索引
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
    int nact = std::min(nv_, m_->nu);
    int nq   = std::min(nv_, m_->nq);
    for (int i = 0; i < nv_; ++i) {
        if (dyn_ && i < nact)
            d_->ctrl[i] = pos[i];   // ctrl 按执行器索引
        else if (!dyn_ && i < nq)
            d_->qpos[i] = pos[i];   // qpos 按广义坐标索引
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

    rd.joint_count = nv_;

    rd.pos.resize(nv_);
    rd.vel.resize(nv_);
    rd.torque.resize(nv_);
    rd.load_torque.resize(nv_);
    rd.cmd_pos.resize(nv_);
    rd.cmd_vel.resize(nv_);
    rd.cmd_torque.resize(nv_);
    rd.joint_enabled.assign(nv_, true);   // 仿真里默认全部使能

    for (int i = 0; i < nv_; ++i) {
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

void HardwareMJ::cbKeyboard(GLFWwindow *w, int key, int, int act, int /*mods*/) {
    auto *p = static_cast<HardwareMJ *>(glfwGetWindowUserPointer(w));
    if (act != GLFW_PRESS && act != GLFW_REPEAT) return;

    switch (key) {
    case GLFW_KEY_BACKSPACE:
        mjv_defaultCamera(&p->cam_);
        break;
    case GLFW_KEY_ESCAPE:
        if (p->force_state_.active)
            std::cout << "[MJ] Force cleared via ESC\n";
        p->force_state_ = ForceInteractionState{};
        break;
    case GLFW_KEY_EQUAL:
    case GLFW_KEY_KP_ADD:
        if (p->force_state_.active)
            p->force_state_.magnitude =
                std::max(0.1, p->force_state_.magnitude + 0.1);
        break;
    case GLFW_KEY_MINUS:
    case GLFW_KEY_KP_SUBTRACT:
        if (p->force_state_.active)
            p->force_state_.magnitude =
                std::max(0.1, p->force_state_.magnitude - 0.1);
        break;
    default:
        break;
    }
}

void HardwareMJ::cbMouseButton(GLFWwindow *w, int btn, int act, int) {
    auto *p = static_cast<HardwareMJ *>(glfwGetWindowUserPointer(w));

    if (btn == GLFW_MOUSE_BUTTON_LEFT) {
        bool ctrl = (glfwGetKey(w, GLFW_KEY_LEFT_CONTROL) == GLFW_PRESS ||
                     glfwGetKey(w, GLFW_KEY_RIGHT_CONTROL) == GLFW_PRESS);

        if (act == GLFW_PRESS && ctrl) {
            // ---- Ctrl+Click: 射线拾取 body，进入力模式 -----------------
            double mx = 0, my = 0;
            glfwGetCursorPos(w, &mx, &my);

            // 确保 scn_.camera[0] 与当前 cam_ 同步
            mjv_updateCamera(p->m_, p->d_, &p->cam_, &p->scn_);

            CameraRay ray =
                computeCameraRay(p->scn_, mx, my, p->width_, p->height_);
            int geomid[1] = {-1};
            // 使用实际的 geomgroup 配置（而非 NULL），确保符合可视化设置
            mjtNum dist = mj_ray(p->m_, p->d_, ray.origin, ray.direction,
                                 p->opt_.geomgroup, 1, -1, geomid);

            bool picked = false;
            if (dist >= 0 && geomid[0] >= 0 &&
                geomid[0] < p->m_->ngeom) {
                int body_id = p->m_->geom_bodyid[geomid[0]];
                if (body_id > 0) {  // 排除 world body (id=0) — 地板等
                    p->force_state_.active   = true;
                    p->force_state_.dragging = true;  // 立即进入拖拽
                    p->force_state_.body_id  = body_id;

                    // 计算击中点的世界坐标
                    mjtNum hit[3];
                    for (int k = 0; k < 3; ++k)
                        hit[k] = ray.origin[k] +
                                 ray.direction[k] * dist;

                    // 转换为 body-local 坐标（跟随 body 移动）
                    mjtNum delta[3];
                    for (int k = 0; k < 3; ++k)
                        delta[k] =
                            hit[k] - p->d_->xpos[3 * body_id + k];
                    mjtNum xmat_inv[9];
                    mju_transpose(xmat_inv,
                                  p->d_->xmat + 9 * body_id, 3, 3);
                    mju_mulMatVec(p->force_state_.local_pos,
                                  xmat_inv, delta, 3, 3);
                    for (int k = 0; k < 3; ++k)
                        p->force_state_.world_pos[k] = hit[k];

                    // 默认力方向：从 hit 点指向相机（模拟推向外）
                    mjtNum from_body[3];
                    for (int k = 0; k < 3; ++k)
                        from_body[k] = p->scn_.camera[0].pos[k] -
                                       hit[k];
                    double len = std::sqrt(
                        from_body[0] * from_body[0] +
                        from_body[1] * from_body[1] +
                        from_body[2] * from_body[2]);
                    if (len > 1e-6) {
                        for (int k = 0; k < 3; ++k)
                            p->force_state_.direction[k] =
                                from_body[k] / len;
                    } else {
                        p->force_state_.direction[0] = 0;
                        p->force_state_.direction[1] = 0;
                        p->force_state_.direction[2] = -1;
                    }

                    picked = true;
                    std::cout << "[MJ] Force pick: body="
                              << mj_id2name(
                                     p->m_, mjOBJ_BODY, body_id)
                              << " (id=" << body_id << ")"
                              << " geom=" << geomid[0]
                              << " dist=" << dist
                              << " hit=[" << hit[0] << "," << hit[1]
                              << "," << hit[2] << "]"
                              << " dir=[" << p->force_state_.direction[0]
                              << "," << p->force_state_.direction[1]
                              << "," << p->force_state_.direction[2]
                              << "]\n";
                }
            }

            if (!picked) {
                // 未命中任何 robot body —— 清除外力
                if (p->force_state_.active) {
                    std::cout << "[MJ] Force cleared (miss or hit "
                              << "background, geomid=" << geomid[0]
                              << " dist=" << dist << ")\n";
                }
                p->force_state_ = ForceInteractionState{};
            }
            // Ctrl+Click 不触发相机旋转
            p->last_x_ = mx;
            p->last_y_ = my;
            return;
        }

        if (act == GLFW_PRESS) {
            if (p->force_state_.active) {
                // 力模式下普通左键：开始拖拽调方向
                p->force_state_.dragging = true;
                glfwGetCursorPos(w, &p->last_x_, &p->last_y_);
            } else {
                // 普通模式：相机旋转
                p->btn_left_ = true;
                glfwGetCursorPos(w, &p->last_x_, &p->last_y_);
            }
        } else {  // RELEASE
            p->force_state_.dragging = false;
            p->btn_left_             = false;
        }
        return;
    }

    // 中键和右键：只在非力拖拽模式时处理相机
    if (act == GLFW_PRESS) {
        if (btn == GLFW_MOUSE_BUTTON_RIGHT)  p->btn_right_  = true;
        if (btn == GLFW_MOUSE_BUTTON_MIDDLE) p->btn_middle_ = true;
        glfwGetCursorPos(w, &p->last_x_, &p->last_y_);
    } else {
        if (btn == GLFW_MOUSE_BUTTON_RIGHT)  p->btn_right_  = false;
        if (btn == GLFW_MOUSE_BUTTON_MIDDLE) p->btn_middle_ = false;
    }
}

void HardwareMJ::cbMouseMove(GLFWwindow *w, double x, double y) {
    auto *p = static_cast<HardwareMJ *>(glfwGetWindowUserPointer(w));
    if (!p->m_) return;
    if (!p->btn_left_ && !p->btn_right_ && !p->btn_middle_ &&
        !p->force_state_.dragging)
        return;

    double dx = (x - p->last_x_) / p->width_;
    double dy = (y - p->last_y_) / p->height_;

    // ---- 力方向拖拽 --------------------------------------------------
    if (p->force_state_.dragging) {
        // 确保相机数据是最新的
        mjv_updateCamera(p->m_, p->d_, &p->cam_, &p->scn_);

        const auto &cam = p->scn_.camera[0];
        // 相机在世界系的方向向量
        mjtNum forward[3] = {cam.forward[0], cam.forward[1],
                             cam.forward[2]};
        mjtNum up[3]      = {cam.up[0], cam.up[1], cam.up[2]};
        mjtNum right[3];
        mju_cross(right, forward, up);
        mju_normalize3(right);

        // 屏幕 dx,dy → 世界系力方向增量
        double sensitivity = 2.5;
        auto &dir          = p->force_state_.direction;
        dir[0] += sensitivity * (dx * right[0] - dy * up[0]);
        dir[1] += sensitivity * (dx * right[1] - dy * up[1]);
        dir[2] += sensitivity * (dx * right[2] - dy * up[2]);

        // 归一化
        double len =
            std::sqrt(dir[0] * dir[0] + dir[1] * dir[1] + dir[2] * dir[2]);
        if (len > 1e-8) {
            dir[0] /= len;
            dir[1] /= len;
            dir[2] /= len;
        }

        p->last_x_ = x;
        p->last_y_ = y;
        return;
    }

    // ---- 相机操作（原有逻辑）------------------------------------------
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

    // 力模式下滚轮调节力量大小
    if (p->force_state_.active) {
        p->force_state_.magnitude += yoff * 0.1;
        p->force_state_.magnitude =
            std::max(0.1, std::min(p->force_state_.magnitude, 500.0));
        return;
    }

    // 原有缩放逻辑
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
