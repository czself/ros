#!/usr/bin/env python3
"""Independent evidence acceptance; never trust controller completion counters."""
import argparse
import json
import math
from pathlib import Path

import numpy as np

from route_geometry import PaintGeometry, load_contract, segment_error


def audit(record, geometry, points):
    failures = []
    samples = record.get('samples', [])
    if record.get('result') != 'COMPLETE':
        failures.append('controller did not complete')
    if not samples:
        return {'accepted': False, 'failures': failures + ['no measured samples']}
    if not np.allclose(np.asarray(record.get('planned', [])), points, atol=1e-6):
        failures.append('recorded plan differs from approved contract')
    expected = 0
    maximum_cross = 0.
    contacts = 0
    previous = None
    for sample in samples:
        pose = np.asarray(sample['pose'], dtype=float)
        if pose.shape != (3,) or not np.isfinite(pose).all():
            failures.append('invalid measured pose'); break
        i = sample['segment']
        if i != expected:
            if i == expected + 1 and previous is not None and np.linalg.norm(
                    np.asarray(previous['pose'][:2]) - points[expected + 1]) < .04:
                expected = i
            else:
                failures.append('unordered or premature segment transition'); break
        if not 0 <= i < len(points) - 1:
            failures.append('invalid segment index'); break
        along, cross, length = segment_error(pose, *points[i:i + 2])
        maximum_cross = max(maximum_cross, cross)
        if cross > .17 or along < -.17 or along > length + .17:
            failures.append('measured corridor violation'); break
        ages = sample.get('ages', [])
        if len(ages) != 3 or not all(math.isfinite(a) and 0 <= a <= .8 for a in ages):
            failures.append('missing or stale pose/laser/depth evidence'); break
        if geometry.collision(*pose):
            contacts += 1
        if previous is not None:
            dt = sample['t'] - previous['t']
            p0 = np.asarray(previous['pose'])
            distance = np.linalg.norm(pose[:2] - p0[:2])
            angle = math.atan2(math.sin(pose[2] - p0[2]), math.cos(pose[2] - p0[2]))
            if dt <= 0 or dt > .3:
                failures.append('trajectory observation gap'); break
            if distance > .35 * dt + .015 or abs(angle) > .8 * dt + .03:
                failures.append('pose discontinuity or excessive motion'); break
            # Interpolate measured motion at <= 2 mm and <= 1 degree; inspect
            # the entire raster polygon, not just logged collision flags.
            steps = max(1, math.ceil(distance / .002), math.ceil(abs(angle) / .017))
            for j in range(1, steps):
                fraction = j / steps
                p = p0 + fraction * (pose - p0)
                p[2] = p0[2] + fraction * angle
                if geometry.collision(*p):
                    contacts += 1
        previous = sample
    if contacts:
        failures.append('ordinary paint contact in measured footprint sweep')
    if np.linalg.norm(np.asarray(samples[0]['pose'][:2]) - points[0]) > .08:
        failures.append('run did not begin at approved birth')
    home_error = float(np.linalg.norm(np.asarray(samples[-1]['pose'][:2]) - points[0]))
    if expected != len(points) - 2 or home_error >= .08:
        failures.append('ordered loop did not return to birth')
    end = samples[-1]['t']
    stopped = [s for s in samples if end - s['t'] <= .75]
    if (len(stopped) < 3 or end - stopped[0]['t'] < .65 or
            any(s['speed'] >= .01 or abs(s['yaw_rate']) >= .03 for s in stopped)):
        failures.append('no sustained measured stationary finish')
    if record.get('completed_segments') != list(range(len(points) - 1)):
        failures.append('completion manifest missing ordered segments')
    return {'accepted': not failures, 'failures': failures,
            'measured_samples': len(samples), 'paint_contacts': contacts,
            'max_cross_track': maximum_cross, 'home_error': home_error,
            'last_observed_segment': expected}


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('record'); parser.add_argument('contract'); parser.add_argument('texture')
    parser.add_argument('--output')
    args = parser.parse_args()
    _, points = load_contract(args.contract)
    report = audit(json.loads(Path(args.record).read_text()), PaintGeometry(args.texture), points)
    text = json.dumps(report, indent=2)
    if args.output:
        Path(args.output).write_text(text + '\n')
    print(text)
    return 0 if report['accepted'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
