#!/usr/bin/env python3
"""Read-only controller/traffic timing and goal-mode transition metrics."""
import argparse
from collections import defaultdict
import hashlib
import json
import math
from pathlib import Path
import rosbag


def measure(run, route_config):
    config = json.loads(route_config.read_text())
    targets = {point['name']: point['base_pose'] for point in config['points']}
    targets['HOME'] = config['home_pose']
    goal = None
    mode = 'UNKNOWN'
    gate = 'CLEAR'
    last = None
    states = defaultdict(float)
    by_goal = defaultdict(lambda: {'final_segments': 0, 'path_alignment_segments': 0,
                                   'reentries_after_final': 0})
    wait = 0.0
    with rosbag.Bag(str(run/'motion.bag')) as bag:
        for topic, message, stamp in bag.read_messages(topics=[
                '/route/progress', '/move_base/goal', '/move_base/result',
                '/move_base/ForwardPathFollower/status', '/traffic_light/gate_status']):
            t = stamp.to_sec()
            if topic == '/move_base/goal':
                pose = message.goal.target_pose.pose.position
                matches = [name for name, target in targets.items()
                           if math.hypot(pose.x-target['x'], pose.y-target['y']) < 1e-4]
                goal = matches[0] if len(matches) == 1 else None
                mode = 'UNKNOWN'
                continue
            if topic == '/move_base/result':
                goal = None
                continue
            if topic == '/route/progress':
                name = json.loads(message.data)['goal']
                if goal != name:
                    goal = name
                    mode = 'UNKNOWN'
                continue
            if last is not None:
                dt = min(.1, max(0, t-last))
                states['SIGNAL_WAIT' if 'WAIT_' in gate else mode] += dt
                if 'WAIT_' in gate:
                    wait += dt
            last = t
            if topic == '/traffic_light/gate_status':
                gate = message.data
                continue
            new = message.data
            if goal and new != mode:
                values = by_goal[goal]
                if new == 'FINAL_HEADING':
                    values['final_segments'] += 1
                if new == 'ALIGNING_TO_PATH':
                    values['path_alignment_segments'] += 1
                if mode == 'FINAL_HEADING' and new in (
                        'TRACKING', 'ALIGNMENT_BRAKING', 'ALIGNING_TO_PATH'):
                    values['reentries_after_final'] += 1
            mode = new
    return {'run': run.name, 'signal_wait_estimate_s': wait,
            'state_seconds': dict(states), 'by_goal': dict(by_goal),
            'route_config_sha256': hashlib.sha256(route_config.read_bytes()).hexdigest(),
            'definition': 'State integration on controller/gate events capped at 0.1 s; '
                          'goals matched by target coordinates and executor progress. '
                          'Reentry means FINAL_HEADING directly transitions to a '
                          'moving/braking path mode. Counts alone do not prove an action '
                          'is unnecessary; state and physical pose replay are required.'}


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('run', type=Path)
    parser.add_argument('--route-config', type=Path,
                        default=Path('/root/navigation/standee_photo_route.json'))
    parser.add_argument('--output', type=Path)
    args = parser.parse_args()
    report = measure(args.run, args.route_config)
    output = args.output or args.run/'navigation_phase_metrics.json'
    output.write_text(json.dumps(report, indent=2)+'\n')
    print(json.dumps(report, indent=2))
