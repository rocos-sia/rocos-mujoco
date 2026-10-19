// Copyright 2026, Yang Luo
// SPDX-License-Identifier: GPL-3.0-or-later

#ifndef ROCOS_MUJOCO_SRC_SIMULATION_MENU_HPP_
#define ROCOS_MUJOCO_SRC_SIMULATION_MENU_HPP_

#include <mujoco/mujoco.h>

#include <array>
#include <memory>
#include <vector>

struct GLFWwindow;
struct GLFWcursor;

namespace rocos_mujoco {

// Render-thread-only, read-only joint telemetry and visualization controls.
class SimulationMenu {
 public:
  explicit SimulationMenu(const mjModel* model);
  ~SimulationMenu();
  void Update(const mjModel* model, const mjData* data, mjvOption* option);
  void ApplyTransparency(const mjModel* model, mjvScene* scene) const;
  void Render(int width, int height);
  bool HandleEvent(GLFWwindow* window, int type, int button = 0,
                   double scroll_x = 0, double scroll_y = 0, int key = 0);

 private:
  struct JointValues {
    std::array<mjtNum, 6> position{};
    std::array<mjtNum, 6> velocity{};
    std::array<mjtNum, 6> torque{};
  };

  void Build(const mjModel* model);
  void Layout(int width, int height);
  void Resize();
  int ColumnDivider() const;

  enum class Drag { kNone, kWidth, kColumn, kToggle };

  std::unique_ptr<mjUI> ui_;
  mjuiState state_{};
  mjrContext context_{};
  GLFWcursor* resize_cursor_ = nullptr;
  mjrRect toggle_rect_{};
  bool visible_ = true;
  int panel_width_ = 380;
  int label_width_ = 170;
  Drag drag_ = Drag::kNone;
  std::vector<JointValues> values_;
  int show_joint_ = 0;
  int show_axis_ = 0;
  mjtNum transparency_ = 0;
  int page_ = 1;
  int built_page_ = 1;
};

}  // namespace rocos_mujoco

#endif  // ROCOS_MUJOCO_SRC_SIMULATION_MENU_HPP_
