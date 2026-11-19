// Copyright 2026, Yang Luo
// SPDX-License-Identifier: GPL-3.0-or-later
#include "rocos_mujoco/mujoco_simulator.hpp"
#include "rocos_mujoco/shared_memory_config.hpp"

#include <sys/wait.h>
#include <unistd.h>

#include <chrono>
#include <cmath>
#include <limits>
#include <cstdlib>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <stdexcept>
#include <string>
#include <thread>

namespace rocos_mujoco {
struct SimulatorTestAccess {
    static mjData* data(MujocoSimulator& sim) { return sim.d_; }
    static void feedback(MujocoSimulator& sim) {
        sim.writeSensorData(*sim.masters_.front());
    }
    static void control(MujocoSimulator& sim) {
        sim.readCommands(*sim.masters_.front());
        sim.applyControl(*sim.masters_.front());
    }
    static int32_t position(const MujocoSimulator& sim, double q, double ratio,
                            int32_t offset) {
        return sim.unitToCnt(q, 156455.678, ratio, offset, true);
    }
};
}

namespace {

void Check(bool condition, const char* message) {
    if (!condition) throw std::runtime_error(message);
}

class Fixture {
 public:
    Fixture() {
        char path[] = "/tmp/rocos-mujoco-shm-XXXXXX";
        char* directory = mkdtemp(path);
        Check(directory != nullptr, "mkdtemp");
        directory_ = directory;
        model = directory_ + "/model.xml";
        hardware = directory_ + "/hardware.yaml";
        std::ofstream(model) << R"(<mujoco><worldbody><body>
          <joint name="joint" type="hinge" axis="0 0 1"/>
          <geom type="sphere" size="0.1" mass="1"/>
          </body></worldbody><actuator><position joint="joint"/>
          </actuator></mujoco>)";
    }
    ~Fixture() { std::filesystem::remove_all(directory_); }

    void WriteHardware(const std::string& content) {
        std::ofstream(hardware) << content;
    }

    std::string model;
    std::string hardware;

 private:
    std::string directory_;
};

// Fork before the simulator creates its worker. Pipes serialize PDO exchanges,
// so this test does not imply a whole-frame atomicity guarantee for production.
class SimulatorProcess {
 public:
    SimulatorProcess(const Fixture& fixture, int id) {
        int commands[2];
        int replies[2];
        Check(pipe(commands) == 0 && pipe(replies) == 0, "pipe");
        pid_ = fork();
        Check(pid_ >= 0, "fork");
        if (pid_ == 0) {
            close(commands[1]);
            close(replies[0]);
            int result = 0;
            try {
                rocos_mujoco::MujocoSimulator simulator(
                    fixture.model, fixture.hardware, id);
                Check(simulator.initialize(), "child initialize");
                char response = 'r';
                Check(write(replies[1], &response, 1) == 1, "child ready");
                char command;
                while (read(commands[0], &command, 1) == 1 && command == 's') {
                    simulator.step();
                    Check(write(replies[1], &response, 1) == 1, "child step");
                }
            } catch (const std::exception& error) {
                std::cerr << error.what() << '\n';
                result = 1;
            }
            // The child's simulator is destroyed, including its IPC owner.
            _exit(result);
        }
        close(commands[0]);
        close(replies[1]);
        command_fd_ = commands[1];
        reply_fd_ = replies[0];
    }

    ~SimulatorProcess() {
        // EOF requests a clean shutdown even when a parent assertion fails.
        if (command_fd_ >= 0) close(command_fd_);
        if (reply_fd_ >= 0) close(reply_fd_);
        if (pid_ > 0) waitpid(pid_, nullptr, 0);
    }

    void WaitReady() {
        char reply;
        Check(read(reply_fd_, &reply, 1) == 1, "simulator ready/step reply");
    }

    void Step() {
        const char command = 's';
        Check(write(command_fd_, &command, 1) == 1, "request step");
        WaitReady();
    }

    void Stop() {
        close(command_fd_);
        command_fd_ = -1;
        int status;
        Check(waitpid(pid_, &status, 0) == pid_, "waitpid");
        pid_ = -1;
        Check(WIFEXITED(status) && WEXITSTATUS(status) == 0, "child exit");
    }

 private:
    pid_t pid_ = -1;
    int command_fd_ = -1;
    int reply_fd_ = -1;
};

void TestInterop(Fixture& fixture, int id) {
    fixture.WriteHardware("hardware:\n  - type: driver\n    id: 17\n"
                          "    joint_name: joint\n    inputs: {}\n    outputs: {}\n");
    SimulatorProcess simulator(fixture, id);
    simulator.WaitReady();
    rocos::SharedMemoryConfig client(id);
    Check(client.connect(), "v2 client connect");
    Check(client.header->abi_version == 2 &&
          std::memcmp(client.header->magic, "ROCOSHM2", 8) == 0, "v2 header");
    Check(client.header->input_bytes == 30 && client.header->output_bytes == 17,
          "existing packed PDO layout");
    const auto& bus = *client.ecatBus;
    Check(std::string(bus.ecm_version) == ROCOS_MUJOCO_VERSION_FULL, "product version");
    Check(bus.backend_type == 0 && bus.state_control_supported == 0 &&
          bus.supported_cmds == (1u << rocos::CMD_RESET_STATS), "sim capabilities");
    Check(bus.slave_num == 1 && bus.slaves[0].id == 17, "slave index vs id");
    Check(bus.slaves[0].info_valid == 0 && bus.slaves[0].identity_source == 0 &&
          rocos::atomicLoad(bus.slaves[0].runtime_valid) == 3, "virtual slave validity");
    Check(rocos::atomicLoad(bus.status_valid) == 1 &&
          rocos::atomicLoad(bus.connected_slave_num) == 1 &&
          rocos::atomicLoad(bus.dc_metric_kind) == 0, "no physical diagnostics");

    rocos::SharedMemoryConfig duplicate(id);
    Check(!duplicate.createSharedMemory(), "reject second owner");
    rocos::SharedMemoryConfig monitor(id);
    Check(monitor.connect(true) && !monitor.waitForSignal(), "read-only monitor");

    // Target position is at byte 3, deliberately unaligned.
    client.setSlaveOutputVarValueByName<int32_t>(0, "Target position", 123);
    client.setSlaveOutputVarValueByName<int8_t>(0, "Modes of operation", 8);
    client.setSlaveOutputVarValueByName<uint16_t>(0, "Controlword", 6);
    simulator.Step();
    Check(client.waitForSignal(), "cycle notification");
    Check((client.getSlaveInputVarValueByName<uint16_t>(0, "Statusword") & 0x6f)
              == 0x21, "control word consumed");
    const auto wall_us = std::chrono::duration_cast<std::chrono::microseconds>(
        std::chrono::system_clock::now().time_since_epoch()).count();
    Check(std::abs(wall_us - client.getTimestamp()) < 5000000, "Unix timestamp");
    const uint64_t heartbeat = rocos::atomicLoad(bus.heartbeat_mono_ns);
    Check(heartbeat > 0 && rocos::monotonicNs() - heartbeat < 5000000000ULL,
          "monotonic heartbeat");
    simulator.Step();
    Check(client.getBusCurrentCycleTime() > 0, "cycle statistics");

    rocos::AsyncCommandBox request, response;
    request.cmd_type = rocos::CMD_SDO_READ;
    request.target_slave = 0;
    request.data_size = 4;
    Check(client.executeCommand(request, response) == rocos::UNSUPPORTED &&
          response.status == rocos::BOX_FAILED && response.data_size == 0,
          "unsupported SDO completes without a cycle");

    request = rocos::AsyncCommandBox{};
    request.cmd_type = rocos::CMD_RESET_STATS;
    // The client stops waiting locally, but the original reset stays in flight.
    Check(client.executeCommand(request, response, 10) == rocos::OUTCOME_UNKNOWN,
          "paused simulator cannot acknowledge reset");
    Check(!rocos::terminalStatus(response.status), "local timeout is not terminal");
    Check(client.executeCommand(request, response) == rocos::BUSY, "no overwrite");
    int32_t result = rocos::BUSY;
    bool saw_cleared_stats = false;
    const uint64_t end = rocos::monotonicNs() + 2000000000ULL;
    while (result == rocos::BUSY && rocos::monotonicNs() < end) {
        simulator.Step();
        saw_cleared_stats = saw_cleared_stats ||
            (client.getBusMinCycleTime() == 0 && client.getBusMaxCycleTime() == 0 &&
             client.getBusAvgCycleTime() == 0 && client.getBusCurrentCycleTime() == 0);
        std::this_thread::sleep_for(std::chrono::milliseconds(2));
        result = client.pollCommand(response);
    }
    Check(result == rocos::RESULT_OK && response.status == rocos::BOX_SUCCESS,
          "reset acknowledged after cycle boundary");
    Check(saw_cleared_stats, "statistics cleared before reset success");
    monitor.disconnect();
    client.disconnect();
    simulator.Stop();
    rocos::SharedMemoryConfig stopped(id);
    Check(!stopped.connect(), "owner removed IPC resources");
}

// Exercise the actual PDO path with deliberately different scales and a nonzero
// encoder offset, so sharing position/velocity/torque transforms cannot pass.
void TestTransforms(Fixture& fixture, int id) {
    using rocos_mujoco::MujocoSimulator;
    using rocos_mujoco::SimulatorTestAccess;
    const std::string base = "hardware:\n  - type: driver\n    id: 0\n"
        "    joint_name: joint\n    inputs: {}\n    outputs: {}\n";
    for (bool secondary : {false, true}) {
        fixture.WriteHardware(base + (secondary
            ? "    torque_source: secondary_position\n" : "") +
            "    transform:\n      pos_per_unit: 1000\n      vel_per_unit: 4000\n"
            "      ratio: -2\n      offset_pos_cnt: 321\n      torque_per_unit: 100\n"
            "      load_torque_per_unit: " + (secondary ? "40000\n" : "200\n"));
        MujocoSimulator sim(fixture.model, fixture.hardware, id);
        Check(sim.initialize(), "independent transform config");
        rocos::SharedMemoryConfig client(id);
        Check(client.connect(), "transform client");
        auto* data = SimulatorTestAccess::data(sim);
        data->qpos[0] = 0.125;
        data->qvel[0] = -0.375;
        data->qfrc_actuator[0] = -1.75;
        data->qfrc_applied[0] = 0.5;
        SimulatorTestAccess::feedback(sim);
        Check(client.getSlaveInputVarValueByName<int32_t>(0, "Position actual value") == 71,
              "position scale, negative ratio and offset");
        Check(client.getSlaveInputVarValueByName<int32_t>(0, "Velocity actual value") == 3000,
              "velocity uses own scale and no position offset");
        Check(client.getSlaveInputVarValueByName<int16_t>(0, "Torque actual value") == -125,
              "motor torque scale");
        Check(client.getSlaveInputVarValueByName<int16_t>(0, "Analog Input 1") ==
                  (secondary ? 0 : -250), "load torque independent scale");
        Check(client.getSlaveInputVarValueByName<int32_t>(0, "Auxiliary position actual value") ==
                  (secondary ? -50000 : 71), "secondary torque retains int32 range");
        client.setSlaveOutputVarValueByName<int8_t>(0, "Modes of operation", 8);
        client.setSlaveOutputVarValueByName<int32_t>(0, "Target position", 71);
        for (uint16_t word : {6, 7, 15}) {
            client.setSlaveOutputVarValueByName<uint16_t>(0, "Controlword", word);
            SimulatorTestAccess::control(sim);
        }
        Check(std::abs(data->ctrl[0] - 0.125) < 1e-12, "CSP decodes position scale");
        client.setSlaveOutputVarValueByName<int8_t>(0, "Modes of operation", 9);
        client.setSlaveOutputVarValueByName<int32_t>(0, "Target velocity", -2000);
        // The disabled brake above clears velocity; inject the CSV sample only
        // after activation so this remains a unit-conversion test.
        data->qvel[0] = -0.375;
        SimulatorTestAccess::control(sim);
        Check(std::abs(data->qfrc_applied[0] - 62.5) < 1e-12,
              "CSV decodes velocity scale without offset");
        client.setSlaveOutputVarValueByName<int8_t>(0, "Modes of operation", 10);
        client.setSlaveOutputVarValueByName<int16_t>(0, "Target torque", -150);
        SimulatorTestAccess::control(sim);
        Check(std::abs(data->qfrc_applied[0] + 1.5) < 1e-12, "CST torque scale");
    }
    fixture.WriteHardware(base + "    transform:\n      pos_per_unit: 1000\n");
    {
        MujocoSimulator sim(fixture.model, fixture.hardware, id);
        Check(sim.initialize(), "optional transform defaults");
        rocos::SharedMemoryConfig client(id);
        Check(client.connect(), "default transform client");
        auto* data = SimulatorTestAccess::data(sim);
        data->qvel[0] = 0.125;
        data->qfrc_applied[0] = 2.0;
        SimulatorTestAccess::feedback(sim);
        Check(client.getSlaveInputVarValueByName<int32_t>(0, "Velocity actual value") == 125,
              "velocity default equals position scale");
        Check(client.getSlaveInputVarValueByName<int16_t>(0, "Analog Input 1") == 2,
              "load torque defaults to one as in App");
    }
    for (const std::string field : {"cnt_per_unit: 1000", "pos_per_unit: 0",
            "vel_per_unit: 0", "load_torque_per_unit: .nan",
            "torque_per_unit: .inf", "ratio: 0"}) {
        fixture.WriteHardware(base + "    transform:\n      " + field + "\n");
        MujocoSimulator sim(fixture.model, fixture.hardware, id);
        Check(!sim.initialize(), "legacy key and invalid transforms rejected");
    }
}

void TestCapacityAndEmptyPdos(Fixture& fixture, int id) {
    for (int count : {0, 50, 51}) {
        std::string config = count == 0 ? "hardware: []\n" : "hardware:\n";
        for (int i = 0; i < count; ++i) {
            config += "  - type: io\n    id: " + std::to_string(i) + "\n";
        }
        fixture.WriteHardware(config);
        rocos_mujoco::MujocoSimulator simulator(fixture.model, fixture.hardware, id);
        Check(simulator.initialize() == (count <= 50), "50-slave limit");
        if (count > 50) continue;
        rocos::SharedMemoryConfig monitor(id);
        Check(monitor.connect(true), "zero PDO connect");
        Check(monitor.header->input_bytes == 0 && monitor.header->output_bytes == 0 &&
              !monitor.pdInputPtr && !monitor.pdOutputPtr, "zero PDO mappings");
        simulator.step();
        Check(monitor.getSlaveNum() == count &&
              rocos::atomicLoad(monitor.ecatBus->heartbeat_mono_ns) > 0,
              "empty PDO bus still publishes heartbeat");
    }
}

}  // namespace

int main() {
    try {
        rocos_mujoco::MujocoSimulator conversion("", "", 19000000 + getpid());
        for (double ratio : {-2.0, 0.5, 1.0}) {
            for (int count : {-122868, -1, 0, 1, 122868}) {
                const double q = count / (156455.678 * ratio);
                for (double direction : {-INFINITY, INFINITY}) {
                    Check(rocos_mujoco::SimulatorTestAccess::position(
                              conversion, std::nextafter(q, direction), ratio, 123) == count + 123,
                          "position feedback preserves nearest encoder count");
                }
            }
        }
        bool rejected = false;
        try { rocos_mujoco::SimulatorTestAccess::position(conversion, INFINITY, 1, 0); }
        catch (const std::out_of_range&) { rejected = true; }
        Check(rejected, "nonfinite encoder feedback rejected");
        Fixture fixture;
        const int id = 20000000 + getpid();
        TestInterop(fixture, id);
        TestCapacityAndEmptyPdos(fixture, id + 1000000);
        TestTransforms(fixture, id + 2000000);
        std::cout << "MuJoCo/v2 client interoperation passed\n";
    } catch (const std::exception& error) {
        std::cerr << error.what() << '\n';
        return 1;
    }
}
