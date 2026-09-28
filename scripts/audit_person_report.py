#!/usr/bin/env python3
"""Read-only person evidence audit; simulator truth is used only here."""
import argparse
import hashlib
import json
from pathlib import Path
import wave
import xml.etree.ElementTree as ET

import cv2
import numpy as np
import rosbag
from scipy.optimize import linear_sum_assignment


def audit(run_dir, world=Path('/root/competition_classic_adjusted_20260924.world')):
    report_path = run_dir / 'person_report.json'
    report = json.loads(report_path.read_text())
    config = json.loads((run_dir / 'person_reporting_config.json').read_text())
    categories = {}
    for model in ET.parse(world).getroot().findall('./world/model'):
        name = model.get('name', '')
        if name.startswith('person_standee_'):
            materials = [element.text or '' for element in model.findall('.//material/script/name')]
            categories[name] = ('stranger' if any('PersonF' in m for m in materials)
                                else 'resident')
    truth = {}
    snapshots, image_stamps = [], set()
    with rosbag.Bag(str(run_dir / 'motion.bag')) as bag:
        for topic, message, _ in bag.read_messages(topics=[
                '/gazebo/link_states', '/inspection/person_report', '/inspection/person_image']):
            if topic == '/gazebo/link_states' and not truth:
                for name, pose in zip(message.name, message.pose):
                    model = name.split('::')[0]
                    if model in categories:
                        truth[model] = {'world_xy': [pose.position.x, pose.position.y],
                                        'class': categories[model]}
            elif topic == '/inspection/person_report':
                payload = json.loads(message.data)
                if 'waypoint' in payload:
                    snapshots.append(payload)
            elif topic == '/inspection/person_image':
                image_stamps.add((message.header.stamp.secs, message.header.stamp.nsecs))
    people = report['people']
    truth_names = sorted(truth)
    assignments = []
    if people and truth_names:
        distances = np.linalg.norm(
            np.asarray([p['world_xy'] for p in people])[:, None, :]-
            np.asarray([truth[n]['world_xy'] for n in truth_names])[None, :, :], axis=2)
        rows, columns = linear_sum_assignment(distances)
        for row, column in zip(rows, columns):
            person, name = people[row], truth_names[column]
            assignments.append({'person_id': person['person_id'], 'truth_model': name,
                                'position_error_m': float(distances[row, column]),
                                'class_pass': person['class'] == truth[name]['class']})
    evidence_checks = []
    for snapshot in snapshots:
        name = snapshot['waypoint']
        stamp = snapshot['source_stamp']
        files = [p for p in (run_dir/name).glob('*.json')
                 if not p.name.endswith(('.detections.json', '.failure.json'))]
        matching = [json.loads(p.read_text()) for p in files]
        matching = [e for e in matching if e.get('source_stamp') == stamp]
        source_pass = False
        if len(matching) == 1:
            evidence = matching[0]
            detection = json.loads(Path(evidence['detection_record']).read_text())
            depth = np.load(evidence['depth_frame'], allow_pickle=False)
            raw = cv2.imread(evidence['raw_image'])
            source_pass = (detection['source_stamp'] == stamp and raw is not None and
                           depth.ndim == 2 and evidence['rgb_depth_stamp_delta_s'] <= .1 and
                           len(evidence.get('camera_intrinsics', [])) == 9 and
                           'camera_pose' in evidence)
        annotated = cv2.imread(snapshot['annotated_image'])
        evidence_checks.append({'waypoint': name, 'source_pass': source_pass,
                                'image_pass': annotated is not None,
                                'synchronized_topic_pass': (stamp['secs'], stamp['nsecs']) in image_stamps})
    crop_checks = []
    for person in report['foreign_people']:
        crops = [Path(o['foreign_crop_path']) for o in person['observations']
                 if o.get('foreign_crop_path')]
        crop_checks.append({'person_id': person['person_id'],
                            'pass': bool(crops) and all(cv2.imread(str(p)) is not None for p in crops)})
    audio = json.loads((run_dir/'person_report.audio.json').read_text())
    with wave.open(str(run_dir/'person_report.wav')) as stream:
        pcm = np.frombuffer(stream.readframes(stream.getnframes()), dtype='<i2')
        duration = stream.getnframes()/float(stream.getframerate())
    counts = {'total': len(truth),
              'resident': sum(p['class'] == 'resident' for p in truth.values()),
              'stranger': sum(p['class'] == 'stranger' for p in truth.values())}
    checks = {
        'reported_complete': report['complete'] and not report['issues'],
        'truth_inventory_available': len(truth) == len(categories) > 0,
        'counts_match_truth': report['counts'] == counts,
        'one_to_one_truth_coverage': len(assignments) == len(people) == len(truth),
        'all_classes_correct': bool(assignments) and all(a['class_pass'] for a in assignments),
        'all_positions_within_8cm': bool(assignments) and all(a['position_error_m'] <= .08 for a in assignments),
        'unique_person_ids': len({p['person_id'] for p in people}) == len(people),
        'required_views_synchronized': (set(s['waypoint'] for s in snapshots) == set(config['required_views'])
                                        and len(snapshots) == len(config['required_views'])
                                        and all(all(c[k] for k in ('source_pass','image_pass','synchronized_topic_pass'))
                                                for c in evidence_checks)),
        'foreign_crops_saved': len(crop_checks) == counts['stranger'] and all(c['pass'] for c in crop_checks),
        'terminal_report_saved': (run_dir/'people_terminal.txt').stat().st_size > 0 and
                                 report['speech_text'] in (run_dir/'person_report.txt').read_text(),
        'audio_report_hash_matches': audio['report_sha256'] == hashlib.sha256(report_path.read_bytes()).hexdigest(),
        'audio_text_matches': audio['text'] == report['speech_text'],
        'audio_synthesized_and_played': audio['synthesis_pass'] and audio['playback_requested'] and audio['playback_pass'],
        'audio_nonempty': duration > 1 and pcm.size > 0 and bool(np.any(pcm)),
    }
    return {'pass': all(checks.values()), 'checks': checks, 'counts': report['counts'],
            'street_counts': report['street_counts'], 'truth_assignments': assignments,
            'maximum_position_error_m': max((a['position_error_m'] for a in assignments), default=None),
            'view_evidence': evidence_checks, 'foreign_crop_evidence': crop_checks,
            'audio_duration_s': duration, 'world_sha256': hashlib.sha256(world.read_bytes()).hexdigest(),
            'scope': 'Static standees; playback exit status verifies delivery, not human audibility. Truth used only by audit.'}


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('run_dir', type=Path)
    parser.add_argument('--world', type=Path, default=Path('/root/competition_classic_adjusted_20260924.world'))
    args = parser.parse_args()
    result = audit(args.run_dir, args.world)
    text = json.dumps(result, ensure_ascii=False, indent=2)+'\n'
    (args.run_dir/'person_audit.json').write_text(text)
    print(text)
    raise SystemExit(0 if result['pass'] else 1)
