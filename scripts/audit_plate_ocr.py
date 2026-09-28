#!/usr/bin/env python3
"""Read-only plate audit against saved scene materials; no truth enters OCR."""
import hashlib
import json
from pathlib import Path
import re
import xml.etree.ElementTree as ET

import cv2
import numpy as np
import rosbag


def audit(run_dir):
    materials = (run_dir/'ocr_scene_materials.txt').read_text()
    truth = {}
    for name,block in re.findall(r'material\s+CarStandee/Plate(\d+)(.*?)(?=\nmaterial|\Z)',materials,re.S):
        texture = re.search(r'\btexture\s+(\S+)',block)
        truth['car_standee_plate_'+name] = Path(texture.group(1)).stem
    world = ET.parse('/root/competition_classic_adjusted_20260924.world')
    positions = {m.get('name'):[float(v) for v in m.findtext('pose').split()[:2]]
                 for m in world.getroot().findall('./world/model') if m.get('name') in truth}
    checks, rows = {}, []
    for point in ['POINT_8','POINT_9','POINT_10']:
        path = run_dir/'ocr'/(point+'.job.json')
        job = json.loads(path.read_text())
        result = json.loads((run_dir/'ocr'/(point+'.result.json')).read_text())
        published = json.loads((run_dir/'ocr'/(point+'.published.json')).read_text())
        position = job['frames'][0]['actual_base_pose']['position'][:2]
        model = min(positions,key=lambda name:float(np.linalg.norm(np.array(position)-positions[name])))
        evidence_pass = True
        for capture,recognized in zip(job['frames'],result['frames']):
            raw = cv2.imread(capture['raw_image']);crop = cv2.imread(capture['crop_image'])
            processed = cv2.imread(recognized['recognition_image'].replace('/home/sz/ros1_ws','/root/ros1_ws'))
            x1,y1,x2,y2 = capture['box']
            a,b,c,d = recognized['recognition_band']
            input_path = Path(recognized['recognition_image'].replace('/home/sz/ros1_ws','/root/ros1_ws'))
            evidence_pass = evidence_pass and (raw is not None and crop is not None and processed is not None
                and raw.shape[:2]==(1440,1920) and np.array_equal(raw[y1:y2,x1:x2],crop)
                and np.array_equal(crop[b:d,a:c],processed)
                and recognized['image_sha256']==hashlib.sha256(Path(capture['crop_image']).read_bytes()).hexdigest()
                and recognized['recognition_image_sha256']==hashlib.sha256(input_path.read_bytes()).hexdigest())
        unique = {(f['source_stamp']['secs'],f['source_stamp']['nsecs']) for f in result['frames']}
        row = {'waypoint':point,'text':result['text'],'expected':truth[model],
               'truth_model':model,'confidence':result['confidence'],
               'exact_match':result['text']==truth[model],
               'valid_consensus':result['valid'] and result['matching_frames']>=2 and
                                 result['confidence']>=.85 and len(unique)>=2,
               'source_evidence_pass':evidence_pass and len(result['frames'])==len(job['frames']),
               'job_sha_pass':result['job_sha256']==hashlib.sha256(path.read_bytes()).hexdigest(),
               'source_stamp':published['source_stamp'],
               'elapsed_ms':result['total_elapsed_ms']}
        rows.append(row)
    messages, image_stamps, reports = [], set(), {}
    with rosbag.Bag(str(run_dir/'motion.bag')) as bag:
        for topic,message,_ in bag.read_messages(topics=[
                '/inspection/plate_text','/inspection/plate_image','/inspection/plate_report']):
            if topic=='/inspection/plate_text':
                messages.append({'text':message.text,'valid':message.valid,
                                 'stamp':(message.header.stamp.secs,message.header.stamp.nsecs)})
            elif topic=='/inspection/plate_image':
                image_stamps.add((message.header.stamp.secs,message.header.stamp.nsecs))
            else:
                report=json.loads(message.data);reports[report['waypoint']]=report
    for row in rows:
        stamp = (row['source_stamp']['secs'],row['source_stamp']['nsecs'])
        row['topic_sync_pass'] = (any(m['stamp']==stamp and m['text']==row['text'] and m['valid']
                                     for m in messages) and stamp in image_stamps
                                  and row['waypoint'] in reports)
    checks['all_three_exact'] = all(r['exact_match'] for r in rows)
    checks['three_distinct_scene_plates'] = len({r['truth_model'] for r in rows})==3
    checks['all_consensus_valid'] = all(r['valid_consensus'] for r in rows)
    checks['all_camera_evidence_valid'] = all(r['source_evidence_pass'] and r['job_sha_pass'] for r in rows)
    checks['all_ros_messages_synchronized'] = all(r['topic_sync_pass'] for r in rows)
    checks['terminal_saved'] = (run_dir/'ocr/ocr_terminal.txt').stat().st_size>0
    return {'pass':all(checks.values()),'checks':checks,'plates':rows,
            'plate_exact_accuracy':sum(r['exact_match'] for r in rows)/3.0,
            'maximum_batch_elapsed_ms':max(r['elapsed_ms'] for r in rows),
            'truth_scope':'Scene filenames are read only here for evaluation, never by the OCR worker.'}
