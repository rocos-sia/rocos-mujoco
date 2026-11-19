// Copyright 2026, ROCOS contributors
// SPDX-License-Identifier: GPL-3.0-or-later
#include <mujoco/mujoco.h>

#include <cmath>
#include <iostream>
#include <memory>
#include <stdexcept>
#include <string>

namespace {
void Check(bool condition, const std::string& message) {
  if (!condition) throw std::runtime_error(message);
}
void Near(const mjtNum* a, const mjtNum* b, int count,
          const std::string& message) {
  for (int i = 0; i < count; ++i) {
    Check(std::isfinite(a[i]) && std::isfinite(b[i]) &&
              std::abs(a[i] - b[i]) < 1e-9, message);
  }
}
using Model = std::unique_ptr<mjModel, decltype(&mj_deleteModel)>;
using Data = std::unique_ptr<mjData, decltype(&mj_deleteData)>;
Model Load(const std::string& name) {
  char error[2048]{};
  auto* model = mj_loadXML((std::string(ROCOS_SOURCE_DIR) +
      "/config/dualarm/mjcf/" + name).c_str(), nullptr, error, sizeof(error));
  Check(model != nullptr, error);
  return Model(model, mj_deleteModel);
}
}  // namespace

int main() {
  try {
    auto old = Load("scene.xml"), trial = Load("scene_shoulder.xml");
    Data a(mj_makeData(old.get()), mj_deleteData);
    Data b(mj_makeData(trial.get()), mj_deleteData);
    // The shoulder mounting disks are normal to base +Y. J1 must spin about
    // that axis; the CAD-exported left axis was almost 90 degrees off.
    mj_forward(trial.get(), b.get());
    for (const char* name : {"Left_Joint1", "Right_Joint1"}) {
      const int joint = mj_name2id(trial.get(), mjOBJ_JOINT, name);
      Check(joint >= 0, "J1 present");
      const mjtNum* axis = b->xaxis + 3 * joint;
      Check(std::abs(axis[0]) < 1e-4 && std::abs(axis[1] - 1) < 1e-4 &&
                std::abs(axis[2]) < 1e-4, "J1 follows mounting disk normal");
    }
    Check(old->nq == 14 && trial->nq == 14 && trial->nv == 14 &&
              trial->nu == 14, "14 joints and actuators retained");
    Check(old->ngeom == trial->ngeom && old->nM == trial->nM,
          "Geometry count and mass matrix topology unchanged");
    for (int j = 0; j < old->njnt; ++j) {
      Check(std::string(mj_id2name(old.get(), mjOBJ_JOINT, j)) ==
            mj_id2name(trial.get(), mjOBJ_JOINT, j), "Joint order unchanged");
      Near(old->jnt_axis + 3*j, trial->jnt_axis + 3*j, 3, "Joint axes");
      Near(old->jnt_range + 2*j, trial->jnt_range + 2*j, 2, "Joint limits");
    }
    for (int sample = 0; sample < 24; ++sample) {
      for (int j = 0; j < 14; ++j) {
        a->qpos[j] = b->qpos[j] = sample ? 0.7 * std::sin(sample*0.37+j*0.41) : 0;
        a->qvel[j] = b->qvel[j] = 0.1 * std::cos(sample*0.2+j);
      }
      mj_forward(old.get(), a.get());
      mj_forward(trial.get(), b.get());
      for (int body = 1; body < old->nbody; ++body) {
        const char* name = mj_id2name(old.get(), mjOBJ_BODY, body);
        const int other = mj_name2id(trial.get(), mjOBJ_BODY, name);
        Check(other >= 0, "Original body retained");
        Near(a->xpos + 3*body, b->xpos + 3*other, 3, name);
        Near(a->xmat + 9*body, b->xmat + 9*other, 9, name);
      }
      for (int geom = 0; geom < old->ngeom; ++geom) {
        const int other = mj_name2id(trial.get(), mjOBJ_GEOM,
                                     mj_id2name(old.get(), mjOBJ_GEOM, geom));
        Check(other >= 0, "Original geom retained");
        Near(a->geom_xpos + 3*geom, b->geom_xpos + 3*other, 3, "Geom position");
        Near(a->geom_xmat + 9*geom, b->geom_xmat + 9*other, 9, "Geom rotation");
      }
      Near(a->qM, b->qM, old->nM, "Mass matrix unchanged");
      Near(a->qfrc_bias, b->qfrc_bias, 14, "Gravity/Coriolis unchanged");
      Check(a->ncon == b->ncon, "Contact count unchanged");
      for (const auto& side : {std::string("left"), std::string("right")}) {
        const int base = mj_name2id(trial.get(), mjOBJ_BODY, (side+"_base").c_str());
        Check(base >= 0, "Fixed shoulder body present");
        const mjtNum expected[] = {0.011578, side == "left" ? -0.063 : -0.267,
                                   0.5890511 + 0.091501};
        Near(b->xpos + 3*base, expected, 3, "Shoulder stays fixed at mounting point");
      }
    }
    std::cout << "24 poses: geometry, joint order, contacts and dynamics equivalent\n";
  } catch (const std::exception& error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
