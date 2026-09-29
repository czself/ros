#!/usr/bin/env python3
"""Offline input comparison on independently audited person photographs.

Never used by the robot. Reference identities/classes are evaluation only.
Scores are actual model outputs, including regressions and missing persons.
"""
import argparse
import ast
import hashlib
import json
import math
import os
from pathlib import Path
import time

os.environ.setdefault('MPLBACKEND', 'Agg')
import cv2
import numpy as np
from ultralytics import YOLO


def source_function(path, name):
    tree = ast.parse(path.read_text())
    function = next(node for node in ast.walk(tree)
                    if isinstance(node, ast.FunctionDef) and node.name == name)
    function.decorator_list = []
    namespace = {'math': math, 'np': np}
    exec(compile(ast.Module(body=[function], type_ignores=[]), str(path), 'exec'),
         namespace)
    return namespace[name]


def iou(a, b):
    intersection = (max(0, min(a[2], b[2]) - max(a[0], b[0])) *
                    max(0, min(a[3], b[3]) - max(a[1], b[1])))
    union = ((a[2]-a[0])*(a[3]-a[1]) +
             (b[2]-b[0])*(b[3]-b[1]) - intersection)
    return intersection / union if union > 0 else 0


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('runs', nargs='+', type=Path)
    parser.add_argument('--model', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    scripts = Path(__file__).resolve().parent
    suppress = source_function(scripts/'yolo_inspector.py', 'suppress_duplicate_person_boxes')
    clipped = source_function(scripts/'route_executor.py', '_box_clipped_at_frame')
    digest = hashlib.sha256(args.model.read_bytes()).hexdigest()
    expected_hash = 'fe502091a4e964371eee3b08ec26029ad653019250d5406e13dc68ce8969a2ad'
    if digest != expected_hash:
        raise ValueError('Checkpoint does not match the protected baseline')
    model = YOLO(str(args.model))
    model.to('cpu')
    model.predict(np.zeros((480, 640, 3), np.uint8), conf=.15,
                  imgsz=640, device='cpu', verbose=False)
    rows = []
    for run in args.runs:
        audit = json.loads((run/'independent_audit.json').read_text())
        if not audit['acceptance_pass']:
            raise ValueError('Only independently accepted runs can be references')
        report = json.loads((run/'person_report.json').read_text())
        for point in ('POINT_2', 'POINT_3', 'POINT_4', 'POINT_5', 'POINT_6'):
            record_path, = (run/point).glob('*.detections.json')
            raw_path = Path(str(record_path).replace('.detections.json', '.raw.png'))
            raw = cv2.imread(str(raw_path))
            height, width = raw.shape[:2]
            record = json.loads(record_path.read_text())
            refs = [dict(observation, person_id=person['person_id'])
                    for person in report['people'] for observation in person['observations']
                    if observation['waypoint'] == point]
            variants = [('saved_baseline', 640, raw, (0, 0)),
                        ('full_768', 768, raw, (0, 0)),
                        ('full_960', 960, raw, (0, 0)),
                        ('gamma_0.65', 640, cv2.LUT(raw, np.array(
                            [255*(i/255.)**.65 for i in range(256)], np.uint8)), (0, 0))]
            x1 = max(0, min(r['box'][0] for r in refs)-25)
            y1 = max(0, min(r['box'][1] for r in refs)-20)
            x2 = min(width, max(r['box'][2] for r in refs)+25)
            y2 = min(height, max(r['box'][3] for r in refs)+15)
            variants.append(('group_crop', 640, raw[y1:y2, x1:x2], (x1, y1)))
            for variant, size, frame, offset in variants:
                started = time.monotonic()
                if variant == 'saved_baseline':
                    detections = record['detections']
                else:
                    result = model.predict(frame, conf=.15, imgsz=size,
                                           device='cpu', verbose=False)[0]
                    detections = []
                    for box in result.boxes:
                        bounds = [int(round(float(v))) for v in box.xyxy[0]]
                        bounds = [v+offset[i % 2] for i, v in enumerate(bounds)]
                        detections.append({'class': model.names[int(box.cls[0])],
                                           'confidence': float(box.conf[0]), 'box': bounds})
                    detections = suppress(detections)
                elapsed = time.monotonic()-started
                people = [d for d in detections
                          if d['class'] in ('resident', 'stranger') and d['confidence'] >= .25
                          and not clipped(d, raw, width, height)]
                pairs = sorted([(iou(r['box'], d['box']), ri, di)
                                for ri, r in enumerate(refs) for di, d in enumerate(people)],
                               reverse=True)
                used_refs, used_detections, matches = set(), set(), []
                for overlap, ri, di in pairs:
                    if overlap < .5 or ri in used_refs or di in used_detections:
                        continue
                    used_refs.add(ri)
                    used_detections.add(di)
                    matches.append({'person_id': refs[ri]['person_id'],
                                    'expected_class': refs[ri]['class'],
                                    'detected_class': people[di]['class'],
                                    'confidence': people[di]['confidence'], 'iou': overlap})
                rows.append({'run': run.name, 'point': point, 'variant': variant,
                             'source_stamp': record['source_stamp'],
                             'raw_sha256': hashlib.sha256(raw_path.read_bytes()).hexdigest(),
                             'prediction_s': elapsed if variant != 'saved_baseline' else None,
                             'expected': len(refs), 'detected': len(people), 'matches': matches,
                             'missing': len(refs)-len(used_refs),
                             'extra': len(people)-len(used_detections),
                             'wrong_class': sum(m['expected_class'] != m['detected_class']
                                                for m in matches)})
    aggregates = {}
    for variant in sorted({row['variant'] for row in rows}):
        subset = [row for row in rows if row['variant'] == variant]
        scores = [m['confidence'] for row in subset for m in row['matches']]
        outsiders = [m['confidence'] for row in subset for m in row['matches']
                     if m['expected_class'] == 'stranger' and m['detected_class'] == 'stranger']
        latencies = [row['prediction_s'] for row in subset if row['prediction_s'] is not None]
        aggregates[variant] = {'missing': sum(row['missing'] for row in subset),
                               'extra': sum(row['extra'] for row in subset),
                               'wrong_class': sum(row['wrong_class'] for row in subset),
                               'confidence_min': min(scores),
                               'confidence_median': float(np.median(scores)),
                               'correct_outsider_min': min(outsiders) if outsiders else None,
                               'prediction_median_s': float(np.median(latencies)) if latencies else None}
    payload = {'checkpoint_sha256': digest, 'rows': rows, 'aggregates': aggregates,
               'note': 'Offline evaluation only; scores unchanged; references from audited reports. '
                       'Timing includes standalone prediction, not the live ROS callback.'}
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(payload, ensure_ascii=False, indent=2))
    print(json.dumps(aggregates, ensure_ascii=False, indent=2))


if __name__ == '__main__':
    main()
