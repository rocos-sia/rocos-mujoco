#!/usr/bin/env python3
"""Convert one SI pose between legacy base_link and a shoulder-v1 base.

Input/output: {"position":{"x":...,"y":...,"z":...},
               "orientation":{"x":...,"y":...,"z":...,"w":...}}.
Only for poses expressed in the base (targets/object/flange feedback), not TCP
calibration transforms expressed in the flange. Does not send motion commands.
"""
import argparse
import json
import math
from pathlib import Path


def convert(pose, arm, destination):
    frames = json.loads((Path(__file__).resolve().parents[1] /
                        'config/dualarm/shoulder_frames.json').read_text())
    mount = frames['arms'][arm]
    if mount['rpy'] != [0, 0, 0]:
        raise ValueError('This converter only supports the translation-only v1 profile')
    for field, axes in [('position', 'xyz'), ('orientation', 'xyzw')]:
        for axis in axes:
            value = pose[field][axis]
            if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value):
                raise ValueError(f'{field}.{axis} must be a finite SI number')
    if abs(sum(pose['orientation'][axis] ** 2 for axis in 'xyzw') - 1) > 1e-6:
        raise ValueError('orientation must be a unit quaternion')
    result = dict(pose, position=dict(pose['position']))
    sign = -1 if destination == 'shoulder' else 1
    for axis, offset in zip('xyz', mount['translation']):
        result['position'][axis] += sign * offset
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--arm', choices=['left', 'right'], required=True)
    parser.add_argument('--to', choices=['shoulder', 'legacy'], required=True)
    parser.add_argument('--input', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    result = convert(json.loads(args.input.read_text()), args.arm, args.to)
    # Refuse overwriting source/previous output; originals remain available.
    with args.output.open('x') as file:
        file.write(json.dumps(result, indent=2, allow_nan=False) + '\n')


if __name__ == '__main__':
    main()
