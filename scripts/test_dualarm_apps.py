#!/usr/bin/env python3
"""Exercise two unmodified App processes against the dual-arm simulator.

Uses only the Python standard library. Owns master IDs 0/1 for the duration of
this opt-in integration test; refuses existing IPC objects and occupied ports.
The finite simulation exits normally so its shared-memory owners clean up IPC.
"""
import argparse
import json
import math
from pathlib import Path
import socket
import subprocess
import time
import urllib.error
import urllib.request

ROOT = Path(__file__).resolve().parents[1]
OPENER = urllib.request.build_opener(urllib.request.ProxyHandler({}))


class App:
    def __init__(self, side, port, process):
        self.side, self.port, self.process = side, port, process
        self.headers = {"Content-Type": "application/json"}

    def request(self, path, body=None):
        if self.process.poll() is not None:
            raise RuntimeError(f"{self.side} App exited: {self.process.returncode}")
        data = None if body is None else json.dumps(body).encode()
        req = urllib.request.Request(f"http://127.0.0.1:{self.port}/api/{path}",
                                     data=data, headers=self.headers)
        with OPENER.open(req, timeout=3) as response:
            result = json.load(response)
        if not result.get("success") or result.get("code") != 0:
            raise RuntimeError(f"{self.side} {path}: {result}")
        return result.get("data")

    def state(self):
        state = self.request("robot/state")
        if state["robot_state"] == "ERROR_STATE":
            raise RuntimeError(f"{self.side} entered ERROR_STATE")
        joints = state["joint_states"]
        assert [j["name"] for j in joints] == [
            f"{self.side.title()}_Joint{i}" for i in range(1, 8)]
        assert all(math.isfinite(j[k]) for j in joints
                   for k in ("position", "velocity", "torque", "load_torque"))
        return state

    def acquire(self):
        result = self.request("control/acquire", {
            "client_name": "dualarm-simulation-test", "client_id": f"dualarm-{self.side}"})
        self.headers.update({"X-Rocos-Control-Token": result["token"],
                             "X-Rocos-Client-Id": result["client_id"]})


def positions(state):
    return [j["position"] for j in state["joint_states"]]


def max_error(actual, expected):
    return max(abs(a - b) for a, b in zip(actual, expected))


def wait_for(action, timeout=10):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        result = action()
        if result:
            return result
        time.sleep(0.05)
    raise TimeoutError(f"Condition not reached within {timeout}s")


def ipc_objects():
    paths = []
    for master in (0, 1):
        for name in (f"ecm_v2_{master}", f"ecm_v2_{master}_*",
                     f"sem.sync_v2_{master}_*", f"pd_input_v2_{master}",
                     f"pd_output_v2_{master}"):
            paths.extend(Path('/dev/shm').glob(name))
    return sorted(set(paths))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--app', type=Path,
                        default=ROOT.parent / 'rocos-app/build/bin/rocosAppMain')
    parser.add_argument('--sim', type=Path,
                        default=ROOT / 'build/headless/bin/rocos_mujoco_sim')
    parser.add_argument('--output', type=Path, default=ROOT / 'build/dualarm-app/test')
    args = parser.parse_args()
    args.app, args.sim, args.output = (p.resolve() for p in (args.app, args.sim, args.output))
    for executable in (args.app, args.sim):
        if not executable.is_file():
            parser.error(f"Missing binary: {executable}")
    if ipc_objects():
        raise RuntimeError(f"Master 0/1 IPC already exists: {ipc_objects()}")
    for port in (8080, 8082, 8090, 8092):
        with socket.socket() as sock:
            sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            sock.bind(('127.0.0.1', port))
    args.output.mkdir(parents=True, exist_ok=True)
    report = {"passed": False, "checks": [], "app_binary": str(args.app)}
    processes, logs, apps = [], [], []
    sim = None

    def start(name, command):
        log = (args.output / f'{name}.log').open('w')
        logs.append(log)
        proc = subprocess.Popen(command, cwd=ROOT, stdout=log, stderr=subprocess.STDOUT)
        processes.append(proc)
        return proc

    try:
        for side in ('left', 'right'):
            checked = subprocess.run([str(args.app), f'--config={ROOT}/config/dualarm/app/{side}.yaml',
                                      '--check_config'], capture_output=True, text=True, timeout=15)
            (args.output / f'check_{side}.log').write_text(checked.stdout + checked.stderr)
            checked.check_returncode()
        # JSON is valid YAML. Resolve resources explicitly in the temporary run config.
        sim_config = args.output / 'simulation.yaml'
        sim_config.write_text(json.dumps({
            "model": str(ROOT / 'config/dualarm/mjcf/scene.xml'),
            "masters": [{"ecat_id": i, "hardware_config": str(
                ROOT / f'config/dualarm/hardware_{side}.yaml')}
                for i, side in enumerate(('left', 'right'))],
            "cycle_time_us": 1000, "duration_sec": 45, "visualize": False}, indent=2))
        sim = start('simulator', [str(args.sim), '--config', str(sim_config)])

        def sim_ready():
            if sim.poll() is not None:
                raise RuntimeError(f"Simulator exited: {sim.returncode}")
            return all(Path(f'/dev/shm/ecm_v2_{i}').exists() for i in (0, 1))
        wait_for(sim_ready)
        time.sleep(0.2)
        for side, port in (('left', 8080), ('right', 8090)):
            proc = start(side, [str(args.app), f'--config={ROOT}/config/dualarm/app/{side}.yaml'])
            app = App(side, port, proc)
            apps.append(app)

            def http_ready():
                try:
                    return app.state()
                except urllib.error.URLError:
                    return False
            wait_for(http_ready)
            app.acquire()
        left, right = apps
        report['pids'] = {'simulator': sim.pid, 'left': left.process.pid, 'right': right.process.pid}
        assert not left.state()['is_enabled'] and not right.state()['is_enabled']
        left.request('robot/enable', {})
        wait_for(lambda: left.state()['is_enabled'])
        assert not right.state()['is_enabled'], 'left enable affected right'
        right.request('robot/enable', {})
        wait_for(lambda: right.state()['is_enabled'])
        time.sleep(0.5)
        report['checks'].append('independent enable and 7-axis bindings')
        targets = ([0.04, -0.03, 0.02, -0.04, 0.03, -0.02, 0.04],
                   [-0.03, 0.04, -0.04, 0.02, -0.02, 0.03, -0.04])
        for moving, holding, target in ((left, right, targets[0]), (right, left, targets[1])):
            baseline = positions(holding.state())
            start_pos = positions(moving.state())
            assert max_error(start_pos, target) > 0.02
            max_hold_drift = 0.0
            moving.request('robot/movej', {'joints': target, 'velocity': 0.1,
                                          'acceleration': 0.3, 'jerk': 5.0})

            def arrived():
                nonlocal max_hold_drift
                state = moving.state()
                held = holding.state()
                assert held['is_enabled']
                max_hold_drift = max(max_hold_drift, max_error(positions(held), baseline))
                assert max_hold_drift < 0.005, f'{holding.side} moved unexpectedly'
                settled = max(abs(j['velocity']) for j in state['joint_states']) < 0.01
                return (not state['motion_busy'] and settled and
                        max_error(positions(state), target) < 0.003) and state
            final = wait_for(arrived, timeout=12)
            report['checks'].append({
                'moving': moving.side, 'target_rad': target,
                'actual_rad': positions(final),
                'max_error_rad': max_error(positions(final), target),
                'other_arm_max_drift_rad': max_hold_drift,
                'state': final['robot_state']})
            print(json.dumps(report['checks'][-1]), flush=True)
        left.request('robot/disable', {})
        wait_for(lambda: not left.state()['is_enabled'] and left.state()['robot_state'] == 'IDLE')
        assert right.state()['is_enabled'], 'left disable affected right'
        report['checks'].append('independent disable')
        report['passed'] = True
    finally:
        cleanup_errors = []
        for app in reversed(apps):
            if 'X-Rocos-Control-Token' in app.headers and app.process.poll() is None:
                try:
                    state = app.state()
                    if state['is_enabled'] or state['robot_state'] != 'IDLE':
                        app.request('robot/disable', {})
                    wait_for(lambda: not app.state()['is_enabled'] and app.state()['robot_state'] == 'IDLE')
                    app.request('control/release', {})
                except Exception as error:
                    cleanup_errors.append(str(error))
        for proc in reversed(processes):
            if proc is sim:
                continue
            if proc.poll() is None:
                proc.terminate()
                try:
                    proc.wait(timeout=10)
                except subprocess.TimeoutExpired:
                    proc.kill()
                    proc.wait()
                    cleanup_errors.append('App shutdown timeout')
            if proc.returncode != 0:
                cleanup_errors.append(f'App exit code {proc.returncode}')
        if sim is not None:
            print('Apps stopped; waiting for finite simulation to release IPC.', flush=True)
            try:
                sim.wait(timeout=55)
            except subprocess.TimeoutExpired:
                sim.kill()
                sim.wait()
                cleanup_errors.append('Simulator shutdown timeout; IPC may remain')
            if sim.returncode != 0:
                cleanup_errors.append(f'Simulator exit code {sim.returncode}')
        for log in logs:
            log.close()
        if ipc_objects():
            cleanup_errors.append(f'IPC remains: {ipc_objects()}')
        report['cleanup_errors'] = cleanup_errors
        report['passed'] = report['passed'] and not cleanup_errors
        (args.output / 'report.json').write_text(json.dumps(report, indent=2) + '\n')
        if cleanup_errors:
            raise RuntimeError('; '.join(cleanup_errors))
    print(f"PASS: {args.output / 'report.json'}")


if __name__ == '__main__':
    main()
