#!/usr/bin/env python3
"""Read-only navigation phase diagnosis from a closed ROS bag.

Goal identities are resolved from target coordinates and executor progress,
never from the ordinal position of messages (a bag may miss its first goal).
Simulation truth is an offline diagnostic reference, never a runtime input.
"""
import argparse
import json
import math
from pathlib import Path

import rosbag
import rospy


def angle(value):
    return math.atan2(math.sin(value), math.cos(value))


def yaw(q):
    return math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))


def diagnose(run, wanted):
    targets = {}
    for directory in run.glob('POINT_*'):
        for path in directory.glob('*.json'):
            data = json.loads(path.read_text())
            if 'target_base_pose' in data:
                targets[directory.name] = data['target_base_pose']
    progress = {}
    windows = {}
    identities = {}
    active = None
    with rosbag.Bag(str(run/'motion.bag')) as bag:
        for topic, message, stamp in bag.read_messages(topics=[
                '/move_base/goal', '/move_base/result', '/route/progress']):
            t = stamp.to_sec()
            if topic == '/move_base/goal':
                p = message.goal.target_pose.pose.position
                matches = [name for name, pose in targets.items()
                           if math.hypot(p.x-pose['x'], p.y-pose['y']) < 1e-4]
                active = matches[0] if len(matches) == 1 else None
                identities[message.goal_id.id] = active
                if active in wanted:
                    windows.setdefault(active, {})['start'] = t
            elif topic == '/route/progress':
                record = json.loads(message.data)
                active = record['goal']
                progress.setdefault(active, []).append((t, record))
                if active in wanted:
                    windows.setdefault(active, {}).setdefault('start', t)
            else:
                name = identities.get(message.status.goal_id.id, active)
                if name in wanted:
                    windows.setdefault(name, {})['end'] = t
                active = None
    output = {'run': run.name, 'goals': {}, 'scope':
              'Closed bag only. Reported remaining distance comes from the executor. '
              'Map pose uses latest received TF without timestamp interpolation. '
              'Truth axle pose adds the configured 0.0525 m axle offset to chassis pose; '
              'absolute map/truth registration is not assumed.'}
    with rosbag.Bag(str(run/'motion.bag')) as bag:
        for name in wanted:
            window = windows[name]
            if 'end' not in window:
                raise ValueError('No result for '+name)
            mode = 'UNKNOWN'
            record = {}
            command = (0, 0)
            map_odom = odom_base = truth = None
            chassis_index = None
            episodes = []
            current = None
            samples = []
            rows = iter(progress[name])
            next_progress = next(rows, None)
            for topic, message, stamp in bag.read_messages(
                    topics=['/move_base/ForwardPathFollower/status', '/my_car/cmd_vel',
                            '/tf', '/gazebo/link_states'],
                    start_time=rospy.Time.from_sec(window['start']-1),
                    end_time=rospy.Time.from_sec(window['end'])):
                t = stamp.to_sec()
                while next_progress is not None and next_progress[0] <= t:
                    record = next_progress[1]
                    next_progress = next(rows, None)
                if topic == '/tf':
                    for transform in message.transforms:
                        pair = (transform.header.frame_id.lstrip('/'),
                                transform.child_frame_id.lstrip('/'))
                        tr = transform.transform.translation
                        pose = (tr.x, tr.y, yaw(transform.transform.rotation))
                        if pair == ('map', 'odom'):
                            map_odom = pose
                        elif pair == ('odom', 'base_footprint'):
                            odom_base = pose
                elif topic == '/gazebo/link_states':
                    if chassis_index is None:
                        chassis_index = message.name.index('my_car::chassis')
                    pose = message.pose[chassis_index]
                    truth = (pose.position.x, pose.position.y, yaw(pose.orientation))
                elif topic == '/my_car/cmd_vel':
                    command = (message.linear.x, message.angular.z)
                elif t >= window['start']:
                    row = {'t': t, 'mode': message.data,
                           'remaining_m': record.get('goal_remaining_m'),
                           'gate': record.get('gate_status'),
                           'command_v_mps': command[0], 'command_w_rps': command[1]}
                    if map_odom and odom_base:
                        ax, ay, aa = map_odom
                        bx, by, ba = odom_base
                        row['map_pose'] = [ax+math.cos(aa)*bx-math.sin(aa)*by,
                                           ay+math.sin(aa)*bx+math.cos(aa)*by, angle(aa+ba)]
                        target = targets[name]
                        row['map_heading_error_rad'] = angle(target['yaw']-row['map_pose'][2])
                    if truth:
                        x, y, a = truth
                        row['truth_chassis_pose'] = list(truth)
                        row['truth_axle_pose'] = [x+.0525*math.cos(a),
                                                 y+.0525*math.sin(a), a]
                    if current is None or message.data != mode:
                        if current is not None:
                            current['end_s'] = t
                            episodes.append(current)
                        current = {'mode': message.data, 'start_s': t, 'samples': []}
                    mode = message.data
                    current['samples'].append(row)
                    samples.append(row)
            if current:
                current['end_s'] = window['end']
                episodes.append(current)
            if not samples:
                raise ValueError('No controller samples for '+name)
            output['goals'][name] = {'target_pose': targets[name], 'window': window,
                                    'episodes': episodes, 'samples': samples}
    return output


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('run', type=Path)
    parser.add_argument('--goals', required=True, help='Comma-separated names')
    parser.add_argument('--output', required=True, type=Path)
    args = parser.parse_args()
    result = diagnose(args.run, args.goals.split(','))
    args.output.write_text(json.dumps(result, indent=2)+'\n')
    for name, goal in result['goals'].items():
        for episode in goal['episodes']:
            first, last = episode['samples'][0], episode['samples'][-1]
            print(name, episode['mode'], round(episode['end_s']-episode['start_s'], 3),
                  'remaining', first['remaining_m'], last['remaining_m'],
                  'heading', first.get('map_heading_error_rad'), last.get('map_heading_error_rad'))
