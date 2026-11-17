# Copyright 2026, Yang Luo
# SPDX-License-Identifier: GPL-3.0-or-later
"""Exercise CLI validation and path resolution with isolated IPC IDs."""
import os
from pathlib import Path
import shutil
import subprocess
import sys
import tempfile

binary = Path(sys.argv[1]).resolve()
root = Path(sys.argv[2]).resolve()
model = root / 'config/dualarm/mjcf/scene.xml'
master_id = 40000000 + os.getpid() * 10

with tempfile.TemporaryDirectory(prefix='rocos-startup-') as directory:
    directory = Path(directory)
    for side in ('left', 'right'):
        shutil.copyfile(root / f'config/dualarm/hardware_{side}.yaml',
                        directory / f'{side}.yaml')
    common = f'model: {model}\nvisualize: false\nduration_sec: 0\n'
    masters = (f'masters:\n  - ecat_id: {master_id}\n'
               '    hardware_config: left.yaml\n'
               f'  - ecat_id: {master_id + 1}\n'
               '    hardware_config: right.yaml\n')
    cases = [
        ('two masters with relative hardware paths', common + masters, True),
        ('duplicate IDs', common + masters.replace(str(master_id + 1), str(master_id)), False),
        ('mixed schema', common + masters + 'ecat_id: 9\n', False),
        ('empty masters', common + 'masters: []\n', False),
        ('missing hardware', common + f'masters:\n  - ecat_id: {master_id}\n', False),
        ('unknown master field', common + masters.replace('    hardware_config: left.yaml',
                                                        '    typo: 1\n    hardware_config: left.yaml'), False),
        ('unknown top-level field', common + masters + 'typo: 1\n', False),
        ('negative ID', common + masters.replace(str(master_id), '-1'), False),
        ('missing hardware file', common + masters.replace('left.yaml', 'absent.yaml'), False),
        ('duplicate joint ownership', common + masters.replace('right.yaml', 'left.yaml'), False),
        ('Talon legacy schema', f'model: {root}/config/talon/mjcf/scene.xml\n'
         f'hardware_config: {root}/config/talon/hardware.yaml\necat_id: {master_id}\n'
         'visualize: false\nduration_sec: 0.01\n', True),
        ('two-master run loop', common.replace('duration_sec: 0', 'duration_sec: 0.01') + masters, True),
    ]
    for label, config, success in cases:
        path = directory / 'startup.yaml'
        path.write_text(config)
        result = subprocess.run([str(binary), '--config', str(path)], cwd='/',
                                capture_output=True, text=True, timeout=15)
        if (result.returncode == 0) != success:
            raise AssertionError(f'{label}: code={result.returncode}\n'
                                 f'{result.stdout}\n{result.stderr}')
        print(f'PASS {label}')
