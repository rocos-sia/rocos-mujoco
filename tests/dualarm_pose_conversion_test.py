#!/usr/bin/env python3
"""Offline checks for converting saved base-relative targets (never moves a robot)."""
import copy
import importlib.util
import json
import math
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest

ROOT = Path(__file__).resolve().parents[1]
SCRIPT = ROOT / 'scripts/convert_dualarm_pose.py'
spec = importlib.util.spec_from_file_location('converter', SCRIPT)
converter = importlib.util.module_from_spec(spec)
spec.loader.exec_module(converter)


class ConversionTest(unittest.TestCase):
    def setUp(self):
        self.pose = {'position': {'x': .4, 'y': 0, 'z': .35},
                     'orientation': {'x': 0, 'y': 0, 'z': math.sin(.2), 'w': math.cos(.2)}}

    def test_each_arm_roundtrip_and_original_unchanged(self):
        original = copy.deepcopy(self.pose)
        for arm, y in [('left', .063), ('right', .267)]:
            new = converter.convert(self.pose, arm, 'shoulder')
            for key, value in zip('xyz', [.388422, y, .258499]):
                self.assertAlmostEqual(new['position'][key], value)
            self.assertEqual(new['orientation'], self.pose['orientation'])
            back = converter.convert(new, arm, 'legacy')
            for key in 'xyz':
                self.assertAlmostEqual(back['position'][key], self.pose['position'][key])
        self.assertEqual(self.pose, original)

    def test_reject_invalid_pose(self):
        for value in [float('nan'), float('inf'), True, '0.4']:
            pose = copy.deepcopy(self.pose)
            pose['position']['x'] = value
            with self.assertRaises(ValueError):
                converter.convert(pose, 'left', 'shoulder')
        self.pose['orientation'] = dict.fromkeys('xyzw', 0)
        with self.assertRaises(ValueError):
            converter.convert(self.pose, 'right', 'shoulder')

    def test_cli_refuses_overwrite(self):
        with tempfile.TemporaryDirectory() as directory:
            src, dst = Path(directory) / 'old.json', Path(directory) / 'new.json'
            src.write_text(json.dumps(self.pose))
            command = [sys.executable, str(SCRIPT), '--arm', 'left', '--to',
                       'shoulder', '--input', str(src), '--output', str(dst)]
            subprocess.run(command, check=True, capture_output=True)
            before = dst.read_bytes()
            self.assertNotEqual(subprocess.run(command, capture_output=True).returncode, 0)
            self.assertEqual(dst.read_bytes(), before)
            self.assertEqual(json.loads(src.read_text()), self.pose)


if __name__ == '__main__':
    unittest.main()
