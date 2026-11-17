// Copyright 2026, Yang Luo
// SPDX-License-Identifier: GPL-3.0-or-later
#include <unistd.h>

#include <array>
#include <chrono>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

#include "rocos_mujoco/mujoco_simulator.hpp"
#include "rocos_mujoco/shared_memory_config.hpp"

namespace rocos_mujoco {
struct SimulatorTestAccess {
  static const mjModel* Model(const MujocoSimulator& sim) { return sim.m_; }
  static const mjData* Data(const MujocoSimulator& sim) { return sim.d_; }
};
}  // namespace rocos_mujoco

namespace {
using rocos::SharedMemoryConfig;
using rocos_mujoco::MasterConfig;
using rocos_mujoco::MujocoSimulator;
using rocos_mujoco::SimulatorTestAccess;
const std::string kRoot = ROCOS_SOURCE_DIR;
const std::string kModel = kRoot + "/config/dualarm/mjcf/scene.xml";
constexpr double kCounts = 1000000.0;

void Check(bool condition, const std::string& message) {
  if (!condition) throw std::runtime_error(message);
}

std::vector<MasterConfig> Masters(int id) {
  return {{id, kRoot + "/config/dualarm/hardware_left.yaml"},
          {id + 1, kRoot + "/config/dualarm/hardware_right.yaml"}};
}

void Control(SharedMemoryConfig& client, uint16_t word, int8_t mode = 8) {
  for (int i = 0; i < 7; ++i) {
    client.setSlaveOutputVarValueByName<uint16_t>(i, "Control word", word);
    client.setSlaveOutputVarValueByName<int8_t>(i, "Mode of operation", mode);
  }
}

void Target(SharedMemoryConfig& client, int joint, double radians) {
  client.setSlaveOutputVarValueByName<int32_t>(joint, "Target Position",
                                               std::lround(radians * kCounts));
}

double Position(SharedMemoryConfig& client, int joint) {
  return client.getSlaveInputVarValueByName<int32_t>(joint,
                                                     "Position actual value") /
         kCounts;
}

void Enable(MujocoSimulator& sim, SharedMemoryConfig& client) {
  for (uint16_t word : {6, 7, 15}) {
    Control(client, word);
    sim.step();
  }
  for (int i = 0; i < 7; ++i) {
    Check((client.getSlaveInputVarValueByName<uint16_t>(i, "Status word") &
           0x6f) == 0x27,
          "drive enabled");
  }
}

void CheckPhysics(const MujocoSimulator& sim, int steps) {
  const auto* m = SimulatorTestAccess::Model(sim);
  const auto* d = SimulatorTestAccess::Data(sim);
  Check(std::abs(d->time - steps * 0.001) < 1e-9,
        "one physics step per cycle, not one per master");
  Check(std::abs(sim.getSimTime() - d->time) < 1e-9,
        "configured cycle and physics timestep aligned");
  for (int i = 0; i < m->nv; ++i) {
    Check(std::isfinite(d->qvel[i]) && std::isfinite(d->qpos[i]),
          "finite model state");
  }
  for (int i = 0; i < mjNWARNING; ++i) {
    Check(d->warning[i].number == 0, "no MuJoCo warnings");
  }
}

void TestDualControl(int id) {
  MujocoSimulator sim(kModel, Masters(id));
  Check(sim.initialize(), "dual master initialize");
  Check(sim.getJointCount() == 14, "14 controlled joints");
  const auto* model = SimulatorTestAccess::Model(sim);
  Check(model->nq == 14 && model->nv == 14 && model->nu == 14,
        "fixed base with 14 actuators");
  Check(SimulatorTestAccess::Data(sim)->ncon == 0, "no initial penetrations");
  SharedMemoryConfig left(id), right(id + 1);
  Check(left.connect(), "left controller connects alone");
  Check(left.ecatBus->slave_num == 7, "left has 7 slaves");
  for (int i = 0; i < 7; ++i) {
    Check(left.ecatBus->slaves[i].id == i, "left local slave IDs");
    Check(std::string(left.ecatBus->slaves[i].name) ==
              "Left_Joint" + std::to_string(i + 1),
          "left joint names");
  }
  int steps = 0;
  Enable(sim, left);
  steps += 3;
  // Left moves while the right controller has never connected.
  for (int n = 0; n < 1000; ++n) {
    Target(left, 0, 0.08 * std::min(1.0, n / 500.0));
    sim.step();
    ++steps;
  }
  Check(std::abs(Position(left, 0) - 0.08) < 0.005, "left moves independently");
  Check(right.connect(), "right controller connects later");
  Check(right.ecatBus->slave_num == 7, "right has 7 slaves");
  for (int i = 0; i < 7; ++i) {
    Check(right.ecatBus->slaves[i].id == i, "right local slave IDs");
    Check(std::string(right.ecatBus->slaves[i].name) ==
              "Right_Joint" + std::to_string(i + 1),
          "right joint names");
    Check(std::abs(Position(right, i)) < 0.04, "unconnected right arm holds");
    Check((right.getSlaveInputVarValueByName<uint16_t>(i, "Status word") &
           0x6f) == 0x40,
          "left enable does not enable right");
  }
  Enable(sim, right);
  steps += 3;
  std::array<double, 7> left_start{}, right_start{};
  for (int i = 0; i < 7; ++i) {
    left_start[i] = Position(left, i);
    right_start[i] = Position(right, i);
  }
  // Distinct signed targets exercise every mapping and both PDO regions.
  for (int n = 0; n < 2500; ++n) {
    double fraction = std::min(1.0, n / 1500.0);
    for (int i = 0; i < 7; ++i) {
      Target(left, i,
             left_start[i] + fraction * (0.02 * (i + 1) - left_start[i]));
      Target(right, i,
             right_start[i] + fraction * (-0.015 * (i + 1) - right_start[i]));
    }
    sim.step();
    ++steps;
  }
  for (int i = 0; i < 7; ++i) {
    Check(std::abs(Position(left, i) - 0.02 * (i + 1)) < 0.005,
          "left tracking joint " + std::to_string(i));
    Check(std::abs(Position(right, i) + 0.015 * (i + 1)) < 0.005,
          "right tracking joint " + std::to_string(i));
  }
  Check(left.waitForSignal() && right.waitForSignal(),
        "both cycle notifications");
  CheckPhysics(sim, steps);

  // A pending reset on one bus must not block the other bus mailbox.
  rocos::AsyncCommandBox request, response;
  request.cmd_type = rocos::CMD_RESET_STATS;
  Check(left.executeCommand(request, response, 10) == rocos::OUTCOME_UNKNOWN,
        "left reset pending without cycles");
  rocos::AsyncCommandBox unsupported, rejected;
  unsupported.cmd_type = rocos::CMD_SDO_READ;
  unsupported.target_slave = 0;
  unsupported.data_size = 4;
  Check(right.executeCommand(unsupported, rejected) == rocos::UNSUPPORTED,
        "right mailbox independent of pending left reset");
  int32_t result = rocos::BUSY;
  bool reset_seen = false;
  for (int i = 0; i < 100 && result == rocos::BUSY; ++i) {
    sim.step();
    ++steps;
    reset_seen |= left.getBusMaxCycleTime() == 0.0;
    Check(right.getBusMaxCycleTime() > 0.0,
          "left reset does not clear right stats");
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
    result = left.pollCommand(response);
  }
  Check(result == rocos::RESULT_OK && reset_seen,
        "left reset completes on cycle");

  std::array<double, 7> held{};
  for (int i = 0; i < 7; ++i) held[i] = Position(left, i);
  Control(left, 0);
  for (int n = 0; n < 1000; ++n) {
    Target(right, 0, -0.015 - 0.05 * std::min(1.0, n / 500.0));
    sim.step();
    ++steps;
  }
  Check(std::abs(Position(right, 0) + 0.065) < 0.005,
        "right moves with left disabled");
  for (int i = 0; i < 7; ++i) {
    Check(std::abs(Position(left, i) - held[i]) < 0.04, "disabled left holds");
  }
  CheckPhysics(sim, steps);
}

void TestValidationAndCleanup(int id) {
  bool rejected = false;
  try {
    auto masters = Masters(id);
    masters[1].ecat_id = id;
    MujocoSimulator sim(kModel, masters);
  } catch (const std::invalid_argument&) {
    rejected = true;
  }
  Check(rejected, "duplicate master IDs rejected");
  {
    auto masters = Masters(id);
    masters[1].hardware_config = masters[0].hardware_config;
    MujocoSimulator sim(kModel, masters);
    Check(!sim.initialize(), "same joint cannot belong to two masters");
  }
  {
    SharedMemoryConfig occupied(id + 1);
    Check(occupied.createSharedMemory(), "occupy second bus");
    {
      MujocoSimulator sim(kModel, Masters(id));
      Check(!sim.initialize(), "second bus ownership failure");
      SharedMemoryConfig observer(id);
      Check(!observer.connect(true),
            "first bus not published on partial setup");
    }
    SharedMemoryConfig observer(id);
    Check(!observer.connect(true), "first bus removed after failed startup");
  }
  {
    MujocoSimulator sim(kModel, Masters(id));
    Check(sim.initialize(), "both buses reusable after cleanup");
  }
  SharedMemoryConfig left(id), right(id + 1);
  Check(!left.connect(true) && !right.connect(true),
        "both buses removed on shutdown");
}

// A deliberately reversed actuator order catches accidental use of DOF as ctrl
// index.
void TestActuatorOrder(int id) {
  const auto dir = std::filesystem::temp_directory_path() /
                   ("rocos-actuator-order-" + std::to_string(getpid()));
  std::filesystem::create_directory(dir);
  struct Cleanup {
    std::filesystem::path path;
    ~Cleanup() { std::filesystem::remove_all(path); }
  } cleanup{dir};
  std::ofstream(dir / "model.xml") << R"(<mujoco><option timestep="0.001"/>
    <worldbody><body pos="3 0 2"><freejoint/>
    <geom size="0.1" mass="1"/></body><body><joint name="a" axis="0 0 1"/><geom size="0.1" mass="1"/>
    </body><body pos="1 0 0"><joint name="b" axis="0 0 1"/>
    <geom size="0.1" mass="1"/></body></worldbody><actuator>
    <position joint="b" kp="20" kv="1"/>
    <position joint="a" kp="20" kv="1"/></actuator></mujoco>)";
  std::ofstream(dir / "hardware.yaml")
      << "hardware:\n  - id: 0\n    type: driver\n    joint_name: a\n"
         "    inputs: {}\n    outputs: {}\n    transform:\n      pos_per_unit: "
         "1000000\n";
  MujocoSimulator sim((dir / "model.xml").string(),
                      (dir / "hardware.yaml").string(), id);
  Check(sim.initialize(), "reversed actuator model loads");
  SharedMemoryConfig client(id);
  Check(client.connect(), "reversed actuator client");
  client.setSlaveOutputVarValueByName<int32_t>(0, "Target position", 100000);
  client.setSlaveOutputVarValueByName<int8_t>(0, "Modes of operation", 8);
  for (int word : {6, 7, 15}) {
    client.setSlaveOutputVarValueByName<uint16_t>(0, "Controlword", word);
    sim.step();
  }
  for (int i = 0; i < 1000; ++i) sim.step();
  const auto* data = SimulatorTestAccess::Data(sim);
  Check(std::abs(data->qpos[7] - 0.1) < 0.001 && std::abs(data->qpos[8]) < 1e-8,
        "correct actuator follows joint binding");
}
}  // namespace

int main() {
  try {
    const int id = 30000000 + getpid() * 10;
    TestDualControl(id);
    TestValidationAndCleanup(id + 2);
    TestActuatorOrder(id + 4);
    std::cout << "Dual-arm control, isolation, mapping and lifecycle passed\n";
  } catch (const std::exception& error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
