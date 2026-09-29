#!/usr/bin/env python3
"""Verify judge terminal/image correspondence by topic and source timestamp.

ROS assigns Header.seq per publisher; it is not the monitor's global display ID.
The display ID remains in the image overlay and terminal event payload.
"""
import argparse
from collections import Counter
import json
from pathlib import Path
import rosbag


def audit(run):
    topics = ['/inspection/judge_'+kind+'_image' for kind in ('yolo', 'person', 'ocr')]
    images, events = set(), []
    with rosbag.Bag(str(run/'motion.bag')) as bag:
        for topic, message, _ in bag.read_messages(topics=topics+['/inspection/judge_event']):
            if topic == '/inspection/judge_event':
                events.append(json.loads(message.data))
            else:
                images.add((topic, message.header.stamp.secs, message.header.stamp.nsecs))
    terminal = (run/'judge/recognition_terminal.txt').read_text()
    missing_images, missing_lines = [], []
    for event in events:
        stamp = event['source_stamp']
        key = (event['image_topic'], stamp['secs'], stamp['nsecs'])
        if key not in images:
            missing_images.append({'sequence': event['sequence'], 'kind': event['kind'],
                                   'source_stamp': stamp})
        for line in event['terminal_lines']:
            if line not in terminal:
                missing_lines.append({'sequence': event['sequence'], 'line': line})
    counts = Counter(event['kind'] for event in events)
    order = [event['sequence'] for event in events]
    monotonic = all(b > a for a, b in zip(order, order[1:]))
    complete = counts['yolo'] > 0 and counts['person'] == 5 and counts['ocr'] == 3
    return {'run': run.name, 'pair_key': ['image_topic', 'source_stamp.secs', 'source_stamp.nsecs'],
            'event_counts': dict(counts), 'sequence_increases': monotonic,
            'required_categories_complete': complete, 'missing_images': missing_images,
            'missing_terminal_lines': missing_lines,
            'pass': bool(events) and monotonic and complete and not missing_images and not missing_lines,
            'scope': 'Recorded source pairs, terminal lines, categories and event order. '
                     'Visual layout reviewed separately using screenshots.'}


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('run', type=Path)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    result = audit(args.run)
    args.output.write_text(json.dumps(result, ensure_ascii=False, indent=2)+'\n')
    print(json.dumps(result, ensure_ascii=False, indent=2))
    raise SystemExit(0 if result['pass'] else 1)
