#!/usr/bin/env python3
"""Offline supplementary measurements for the frozen twenty-mission batch."""
import argparse
import json
import math
from pathlib import Path

import rosbag
from diagnose_navigation_run import diagnose, yaw, angle


def point7_summary(goal):
    final = [r for r in goal['samples'] if r['mode'] in
             ('FINAL_HEADING', 'GOAL_SETTLE', 'GOAL_REACHED')]
    # Target yaw is in map; truth yaw is in world. Do not compare them without
    # registering the frames. Use the received map-frame heading error here.
    errors = [r['map_heading_error_rad'] for r in final]
    nonzero = [e for e in errors if abs(e) > .01]
    return {
        'goal_window_duration_s': goal['window']['end']-goal['window']['start'],
        'episodes': [{'mode': e['mode'], 'duration_s': e['end_s']-e['start_s']}
                     for e in goal['episodes']],
        'map_final_error_first_rad': errors[0] if errors else None,
        'map_final_error_last_rad': errors[-1] if errors else None,
        'map_final_error_sign_crossings_over_001rad': sum(
            a*b < 0 for a,b in zip(nonzero,nonzero[1:])),
        'scope': 'Map-frame estimated heading error sign crossings in final/settle '
                 'phase. World truth yaw is retained as a trajectory only; no '
                 'absolute map/world angular registration is assumed. Zero '
                 'post-final reentry cannot exclude earlier large path turns.'}


def supplementary(run, reuse=False):
    if reuse:
        previous = json.loads((run/'docs20_supplementary.json').read_text())
        goal = previous['point7_diagnostic']['goals']['POINT_7']
        previous['point7'] = point7_summary(goal)
        previous['measurement_schema_version'] = 2
        return previous
    first = last = None
    first_stamp = last_stamp = None
    with rosbag.Bag(str(run / 'motion.bag')) as bag:
        for _, message, stamp in bag.read_messages(topics=['/gazebo/link_states']):
            index = message.name.index('my_car::chassis')
            pose = message.pose[index]
            row = [pose.position.x, pose.position.y, yaw(pose.orientation)]
            if first is None:
                first, first_stamp = row, stamp.to_sec()
            last, last_stamp = row, stamp.to_sec()
    if first is None:
        raise ValueError('No physical chassis samples')
    def axle(p):
        return [p[0] + .0525 * math.cos(p[2]),
                p[1] + .0525 * math.sin(p[2])]
    a, b = axle(first), axle(last)
    physical = {
        'first_stamp_s': first_stamp, 'last_stamp_s': last_stamp,
        'first_chassis_pose': first, 'last_chassis_pose': last,
        'chassis_return_error_cm': 100 * math.hypot(last[0]-first[0], last[1]-first[1]),
        'axle_return_error_cm': 100 * math.hypot(b[0]-a[0], b[1]-a[1]),
        'yaw_return_error_rad': abs(angle(last[2]-first[2])),
        'scope': 'First/last physical poses in closed bag, same world frame. '
                 'Not an absolute map/world registration or runtime input.'}
    physical['chassis_within_3cm_004rad'] = (
        physical['chassis_return_error_cm'] <= 3 and
        physical['yaw_return_error_rad'] <= .04)
    diagnostic = diagnose(run, ['POINT_7'])
    goal = diagnostic['goals']['POINT_7']
    p7 = point7_summary(goal)
    return {'run': run.name, 'physical_home': physical, 'point7': p7,
            'point7_diagnostic': diagnostic, 'measurement_schema_version': 2}


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('run', type=Path)
    parser.add_argument('--reuse-physical', action='store_true',
                        help='Recalculate heading definition from retained samples')
    args = parser.parse_args()
    result = supplementary(args.run, args.reuse_physical)
    output = args.run / 'docs20_supplementary.json'
    output.write_text(json.dumps(result, ensure_ascii=False, indent=2)+'\n')
    print(json.dumps({k:v for k,v in result.items() if k != 'point7_diagnostic'},
                     ensure_ascii=False))
