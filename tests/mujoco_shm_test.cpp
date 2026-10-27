// Copyright 2026, Yang Luo
// SPDX-License-Identifier: GPL-3.0-or-later
#include "rocos_mujoco/mujoco_simulator.hpp"
#include "rocos_mujoco/shared_memory_config.hpp"

#include <sys/wait.h>
#include <unistd.h>

#include <chrono>
#include <cstdlib>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <stdexcept>
#include <string>
#include <thread>

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
        Fixture fixture;
        const int id = 20000000 + getpid();
        TestInterop(fixture, id);
        TestCapacityAndEmptyPdos(fixture, id + 1000000);
        std::cout << "MuJoCo/v2 client interoperation passed\n";
    } catch (const std::exception& error) {
        std::cerr << error.what() << '\n';
        return 1;
    }
}
