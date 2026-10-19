// Copyright 2026, Yang Luo
// SPDX-License-Identifier: GPL-3.0-or-later

#include "simulation_menu.hpp"

#include <GLFW/glfw3.h>

#include <algorithm>
#include <cmath>
#include <cstdio>

namespace rocos_mujoco {
namespace {

// Bound the native UI's auxiliary framebuffer height for larger models.
constexpr int kJointsPerPage = 16;
constexpr int kHeaderHeight = 30;
constexpr int kResizeGrip = 5;
constexpr int kMinPanelWidth = 300;
constexpr int kMinLabelWidth = 100;
// Enough room for a sign, mantissa and a three-digit exponent at font scale
// 100.
constexpr int kMinValueWidth = 140;

int JointSize(int type) {
  return type == mjJNT_FREE ? 6 : type == mjJNT_BALL ? 3 : 1;
}

}  // namespace

SimulationMenu::SimulationMenu(const mjModel* model)
    : ui_(std::make_unique<mjUI>()), values_(model->njnt) {
  // The menu has its own fonts/aux buffer, independent of the scene's overlays.
  mjr_defaultContext(&context_);
  mjr_makeContext(nullptr, &context_, mjFONTSCALE_100);
  resize_cursor_ = glfwCreateStandardCursor(GLFW_HRESIZE_CURSOR);
  ui_->spacing = mjui_themeSpacing(0);
  ui_->spacing.total = panel_width_;
  ui_->spacing.label = label_width_;
  ui_->color = mjui_themeColor(0);
  ui_->rectid = 1;
  ui_->auxid = 0;
  state_.nrect = 3;
  Build(model);
}

SimulationMenu::~SimulationMenu() {
  glfwDestroyCursor(resize_cursor_);
  mjr_freeContext(&context_);
}

void SimulationMenu::Build(const mjModel* model) {
  int section_state[4] = {mjSECT_OPEN, mjSECT_OPEN, mjSECT_CLOSED,
                          mjSECT_CLOSED};
  for (int i = 0; i < ui_->nsect; ++i) {
    section_state[i] = ui_->sect[i].state;
  }
  ui_->nsect = 0;
  const mjuiDef display[] = {
      {mjITEM_SECTION, "Display", section_state[0]},
      {mjITEM_CHECKINT, "Joint", 1, &show_joint_},
      {mjITEM_CHECKINT, "Axis", 1, &show_axis_},
      {mjITEM_SLIDERNUM, "Transparency", 1, &transparency_, "0 1"},
      {mjITEM_END}};
  mjui_add(ui_.get(), display);
  if (model->njnt > kJointsPerPage) {
    mjuiDef paging[] = {{mjITEM_SLIDERINT, "Joint page", 1, &page_, ""},
                        {mjITEM_END}};
    std::snprintf(paging[0].other, sizeof(paging[0].other), "1 %d",
                  (model->njnt + kJointsPerPage - 1) / kJointsPerPage);
    mjui_add(ui_.get(), paging);
  }

  const char* titles[] = {"Joint position", "Joint velocity", "Joint torque"};
  const int begin = (page_ - 1) * kJointsPerPage;
  const int end = std::min(model->njnt, begin + kJointsPerPage);
  for (int section = 1; section <= 3; ++section) {
    mjuiDef header[] = {{mjITEM_SECTION, "", section_state[section]},
                        {mjITEM_STATIC, "SI units", 1, nullptr, ""},
                        {mjITEM_END}};
    std::snprintf(header[0].name, sizeof(header[0].name), "%s",
                  titles[section - 1]);
    std::snprintf(header[1].other, sizeof(header[1].other), "%s",
                  section == 1   ? "rad / m"
                  : section == 2 ? "rad/s / m/s"
                                 : "N.m / N");
    mjui_add(ui_.get(), header);
    for (int joint = begin; joint < end; ++joint) {
      auto& value = values_[joint];
      const int type = model->jnt_type[joint];
      const char* name = mj_id2name(model, mjOBJ_JOINT, joint);
      mjtNum* data = section == 1   ? value.position.data()
                     : section == 2 ? value.velocity.data()
                                    : value.torque.data();
      // One value per row also keeps multi-DOF joints' exponents readable.
      for (int component = 0; component < JointSize(type); ++component) {
        const bool linear =
            type == mjJNT_SLIDE || (type == mjJNT_FREE && component < 3);
        const char* unit = section == 1   ? (linear ? "m" : "rad")
                           : section == 2 ? (linear ? "m/s" : "rad/s")
                                          : (linear ? "N" : "N.m");
        const char* axes[] = {".x", ".y", ".z", ".rx", ".ry", ".rz"};
        const char* axis = type == mjJNT_FREE   ? axes[component]
                           : type == mjJNT_BALL ? axes[component + 3]
                                                : "";
        mjuiDef row[] = {{mjITEM_EDITNUM, "", 0, data + component, "1"},
                         {mjITEM_END}};
        std::snprintf(row[0].name, sizeof(row[0].name), "[%d] %.16s%s (%s)",
                      joint, name ? name : "joint", axis, unit);
        mjui_add(ui_.get(), row);
      }
    }
  }
  built_page_ = page_;
  Resize();
  Layout(state_.rect[0].width, state_.rect[0].height);
}

void SimulationMenu::Resize() {
  mjui_resize(ui_.get(), &context_);
  if (context_.auxWidth[ui_->auxid] != ui_->width ||
      context_.auxHeight[ui_->auxid] != ui_->maxheight) {
    mjr_addAux(ui_->auxid, ui_->width, ui_->maxheight, ui_->spacing.samples,
               &context_);
  }
}

void SimulationMenu::Update(const mjModel* model, const mjData* data,
                            mjvOption* option) {
  if (page_ != built_page_) Build(model);
  option->flags[mjVIS_JOINT] = show_joint_;
  option->frame = show_axis_ ? mjFRAME_BODY : mjFRAME_NONE;
  for (int joint = 0; joint < model->njnt; ++joint) {
    auto& value = values_[joint];
    const int type = model->jnt_type[joint];
    const int qpos = model->jnt_qposadr[joint];
    const int dof = model->jnt_dofadr[joint];
    const int size = JointSize(type);
    if (type == mjJNT_FREE || type == mjJNT_BALL) {
      // Quaternion positions are shown as rotation vectors, not Euler angles.
      const int offset = type == mjJNT_FREE ? 3 : 0;
      std::copy_n(data->qpos + qpos, offset, value.position.begin());
      mju_quat2Vel(value.position.data() + offset, data->qpos + qpos + offset,
                   1);
    } else {
      value.position[0] = data->qpos[qpos];
    }
    for (int component = 0; component < size; ++component) {
      value.velocity[component] = data->qvel[dof + component];
      // Match MujocoSimulator::writeSensorData(), including direct effort
      // control.
      value.torque[component] = data->qfrc_actuator[dof + component] +
                                data->qfrc_applied[dof + component];
    }
  }
}

void SimulationMenu::ApplyTransparency(const mjModel* model,
                                       mjvScene* scene) const {
  for (int i = 0; i < scene->ngeom; ++i) {
    auto& geom = scene->geoms[i];
    // Only fade model bodies; leave the floor, axes and force markers visible.
    if (geom.objtype == mjOBJ_GEOM && geom.objid >= 0 &&
        model->geom_bodyid[geom.objid] != 0) {
      geom.rgba[3] *= 1 - transparency_;
    }
  }
}

void SimulationMenu::Layout(int width, int height) {
  if (width > 0) {
    // Keep native item widths valid even when the window is extremely narrow.
    const int panel = std::max(kMinPanelWidth, std::min(panel_width_, width));
    const int label =
        std::clamp(label_width_, kMinLabelWidth,
                   std::max(kMinLabelWidth, panel - kMinValueWidth - 40));
    if (ui_->spacing.total != panel || ui_->spacing.label != label) {
      ui_->spacing.total = panel;
      ui_->spacing.label = label;
      Resize();
    }
  }
  const int panel = visible_ ? std::min(width, ui_->width) : 0;
  state_.rect[0] = {0, 0, width, height};
  state_.rect[1] = {width - panel, 0, panel,
                    std::max(0, height - kHeaderHeight)};
  state_.rect[2] = {0, 0, width - panel, height};
  toggle_rect_ = {std::max(0, width - 130), std::max(0, height - kHeaderHeight),
                  std::min(130, width), std::min(kHeaderHeight, height)};
}

int SimulationMenu::ColumnDivider() const {
  // Native item rectangles give the exact start of the value column.
  const auto& section = ui_->sect[1];
  const int item = section.nitem > 1 ? 1 : 0;
  return state_.rect[1].left + section.item[item].rect.left -
         ui_->spacing.itemmid / 2;
}

void SimulationMenu::Render(int width, int height) {
  if (width <= 0 || height <= 0) return;
  Layout(width, height);
  mjr_setBuffer(mjFB_WINDOW, &context_);
  if (visible_) {
    mjui_update(-1, -1, ui_.get(), &state_, &context_);
    mjui_render(ui_.get(), &state_, &context_);
    const int left = state_.rect[1].left;
    mjr_rectangle({left, std::max(0, height - kHeaderHeight),
                   state_.rect[1].width, std::min(height, kHeaderHeight)},
                  0.16f, 0.16f, 0.16f, 1);
    mjr_rectangle({left, 0, 2, height}, 0.45f, 0.45f, 0.45f, 1);
    mjr_rectangle({ColumnDivider(), 0, 1, state_.rect[1].height}, 0.3f, 0.3f,
                  0.3f, 1);
  }
  mjr_label(toggle_rect_, mjFONT_NORMAL,
            visible_ ? "Hide menu [Tab]" : "Show menu [Tab]", 0.2f, 0.2f, 0.2f,
            1, 0.9f, 0.9f, 0.9f, &context_);
}

bool SimulationMenu::HandleEvent(GLFWwindow* window, int type, int button,
                                 double scroll_x, double scroll_y, int key) {
  int width, height, framebuffer_width, framebuffer_height;
  glfwGetWindowSize(window, &width, &height);
  glfwGetFramebufferSize(window, &framebuffer_width, &framebuffer_height);
  if (width <= 0 || height <= 0 || framebuffer_width <= 0 ||
      framebuffer_height <= 0) {
    return false;
  }
  Layout(framebuffer_width, framebuffer_height);
  double x, y;
  glfwGetCursorPos(window, &x, &y);
  x *= static_cast<double>(framebuffer_width) / width;
  y = framebuffer_height - y * framebuffer_height / height;
  if (type == mjEVENT_KEY && key == GLFW_KEY_TAB) {
    visible_ = !visible_;
    glfwSetCursor(window, nullptr);
    return true;
  }
  if (drag_ != Drag::kNone) {
    if (type == mjEVENT_MOVE && drag_ != Drag::kToggle) {
      if (drag_ == Drag::kWidth) {
        panel_width_ =
            std::clamp(framebuffer_width - mju_round(x), kMinPanelWidth,
                       std::max(kMinPanelWidth, framebuffer_width - 40));
      } else {
        label_width_ = std::clamp(
            ui_->spacing.label + mju_round(x - ColumnDivider()), kMinLabelWidth,
            std::max(kMinLabelWidth, ui_->width - kMinValueWidth - 40));
      }
      Layout(framebuffer_width, framebuffer_height);
    }
    if (type == mjEVENT_RELEASE) {
      drag_ = Drag::kNone;
      glfwSetCursor(window, nullptr);
    }
    return true;
  }
  const bool over_toggle =
      mjr_findRect(mju_round(x), mju_round(y), 1, &toggle_rect_) == 0;
  if (type == mjEVENT_PRESS && button == GLFW_MOUSE_BUTTON_LEFT &&
      state_.dragrect == 0 && over_toggle) {
    visible_ = !visible_;
    drag_ = Drag::kToggle;
    glfwSetCursor(window, nullptr);
    return true;
  }
  if (over_toggle && state_.dragrect == 0 && type != mjEVENT_KEY) return true;
  if (!visible_) return false;
  const bool edge = std::abs(x - state_.rect[1].left) <= kResizeGrip;
  const bool column =
      y < state_.rect[1].height && std::abs(x - ColumnDivider()) <= kResizeGrip;
  if (state_.dragrect == 0) {
    glfwSetCursor(window, edge || column ? resize_cursor_ : nullptr);
    if (type == mjEVENT_PRESS && button == GLFW_MOUSE_BUTTON_LEFT &&
        (edge || column)) {
      drag_ = edge ? Drag::kWidth : Drag::kColumn;
      return true;
    }
  }
  state_.dx = x - state_.x;
  state_.dy = y - state_.y;
  state_.x = x;
  state_.y = y;
  state_.mouserect =
      mjr_findRect(mju_round(x), mju_round(y), 2, state_.rect + 1) + 1;
  if (x >= state_.rect[1].left && y >= state_.rect[1].height) {
    state_.mouserect = 1;
  }
  state_.left =
      glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_LEFT) == GLFW_PRESS;
  state_.right =
      glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_RIGHT) == GLFW_PRESS;
  state_.middle =
      glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_MIDDLE) == GLFW_PRESS;
  state_.control = glfwGetKey(window, GLFW_KEY_LEFT_CONTROL) == GLFW_PRESS ||
                   glfwGetKey(window, GLFW_KEY_RIGHT_CONTROL) == GLFW_PRESS;
  state_.shift = glfwGetKey(window, GLFW_KEY_LEFT_SHIFT) == GLFW_PRESS ||
                 glfwGetKey(window, GLFW_KEY_RIGHT_SHIFT) == GLFW_PRESS;
  state_.alt = glfwGetKey(window, GLFW_KEY_LEFT_ALT) == GLFW_PRESS ||
               glfwGetKey(window, GLFW_KEY_RIGHT_ALT) == GLFW_PRESS;
  state_.type = type;
  state_.key = key;
  state_.keytime = glfwGetTime();
  state_.sx = scroll_x * framebuffer_width / width;
  state_.sy = scroll_y * framebuffer_height / height;
  if (type == mjEVENT_PRESS) {
    const int translated = button == GLFW_MOUSE_BUTTON_LEFT ? mjBUTTON_LEFT
                           : button == GLFW_MOUSE_BUTTON_RIGHT
                               ? mjBUTTON_RIGHT
                               : mjBUTTON_MIDDLE;
    const double now = glfwGetTime();
    state_.doubleclick =
        translated == state_.button && now - state_.buttontime < 0.25;
    state_.button = translated;
    state_.buttontime = now;
    state_.dragrect = state_.mouserect;
    state_.dragbutton = translated;
  }
  const bool consumed =
      state_.dragrect == 1 || (state_.dragrect == 0 && state_.mouserect == 1);
  if (consumed) {
    mjui_event(ui_.get(), &state_, &context_);
  }
  if (type == mjEVENT_RELEASE) {
    state_.dragrect = 0;
    state_.dragbutton = 0;
  }
  return consumed;
}

}  // namespace rocos_mujoco
