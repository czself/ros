#!/usr/bin/env python3
"""Capture message-level TF caller IDs for a short, bounded ROS interval."""
import argparse
import json
import time

import rospy
from tf2_msgs.msg import TFMessage


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--output', required=True)
    parser.add_argument('--duration', type=float, default=3.0)
    args = parser.parse_args()
    if args.duration <= 0:
        raise SystemExit('duration must be positive')

    rospy.init_node('capture_tf_evidence', anonymous=True, disable_signals=True)
    observations = {}

    def receive(message, topic):
        connection_header = getattr(message, '_connection_header', {}) or {}
        caller = connection_header.get('callerid', '')
        for transform in message.transforms:
            key = (transform.header.frame_id, transform.child_frame_id)
            item = observations.setdefault(key, {
                'parent': transform.header.frame_id,
                'child': transform.child_frame_id,
                'topics': set(),
                'caller_ids': set(),
                'samples': 0,
            })
            item['topics'].add(topic)
            item['caller_ids'].add(caller or 'unknown')
            item['samples'] += 1

    rospy.Subscriber('/tf', TFMessage, receive, callback_args='/tf', queue_size=100)
    rospy.Subscriber('/tf_static', TFMessage, receive, callback_args='/tf_static', queue_size=100)
    started = time.time()
    deadline = time.monotonic() + args.duration
    while time.monotonic() < deadline and not rospy.is_shutdown():
        time.sleep(0.05)
    ended = time.time()
    transforms = []
    for key in sorted(observations):
        item = observations[key]
        transforms.append({
            'parent': item['parent'],
            'child': item['child'],
            'topics': sorted(item['topics']),
            'caller_ids': sorted(item['caller_ids']),
            'samples': item['samples'],
        })
    result = {
        'started_wall_time': started,
        'ended_wall_time': ended,
        'duration_sec': ended - started,
        'transforms': transforms,
    }
    with open(args.output, 'w', encoding='utf-8') as stream:
        json.dump(result, stream, indent=2, sort_keys=True)
        stream.write('\n')
    print(json.dumps(result, indent=2, sort_keys=True))
    if not transforms:
        raise SystemExit('no TF messages observed')


if __name__ == '__main__':
    main()
