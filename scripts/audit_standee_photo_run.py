#!/usr/bin/env python3
"""Independent evidence audit for one closed ten-photo ROS bag run.

Run this inside the ROS container so rosbag can read the recorded Gazebo
truth poses. It never sends a goal or publishes velocity.
"""

import argparse
import bisect
import collections
import json
import math
from pathlib import Path
import statistics

import cv2
import numpy as np
import rosbag


GROUND_M = 4.2
WHITE_THRESHOLD = 220
YAW_SAMPLE_STEP = math.radians(1.0)
POSE_SAMPLE_PERIOD_S = 0.01
FOOTPRINT = ((0.094, 0.085), (0.094, -0.085),
             (-0.094, -0.085), (-0.094, 0.085))
START_LINE = ('y', -1.400, 1.700, 0.32, 0.055)
STOP_LINES = (
    ('WESTBOUND', 'x', -0.515, 0.00, 0.38, 0.055),
    ('NORTHBOUND', 'y', -0.290, 1.70, 0.32, 0.055),
)
ZEBRA_BOXES = (
    (388, 1934, 6138, 6706, 'NORTHBOUND'),
    (7543, 8111, 4898, 6444, 'WESTBOUND'),
)
YOLO_SHA256 = 'fe502091a4e964371eee3b08ec26029ad653019250d5406e13dc68ce8969a2ad'
WAYPOINTS = [f'POINT_{i}' for i in range(1, 11)]
EXPECTED_PEOPLE = {'POINT_2': 3, 'POINT_3': 3, 'POINT_4': 3,
                   'POINT_5': 4, 'POINT_6': 5}
POINT_XY_TOLERANCES = {'POINT_3': 0.05, 'POINT_5': 0.025,
                       'POINT_7': 0.05, 'POINT_10': 0.05}
POINT_YAW_TOLERANCES = {'POINT_5': 0.025}
EXPECTED_CLASSES = {
    'resident', 'stranger', 'red_on', 'red_off', 'yellow_on',
    'yellow_off', 'green_on', 'green_off', 'license_plate',
}
TEXTURE_BOXES = (
    (388, 1934, 6138, 6706),
    (7543, 8111, 4898, 6444),
)


def wrap(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def sha256_matches(record, expected):
    return record.get('checkpoint_sha256') == expected


def stable_green(now, detections, sim_state, remaining, expected_hash):
    if len(detections) < 3 or sim_state != 'GREEN' or remaining < 2.0:
        return False
    last = detections[-3:]
    stamps = []
    for stamp, record in last:
        source = record.get('source_stamp', {}).get('seconds', 0.0)
        receive_age = stamp - source
        if (record.get('state') != 'GREEN' or
                float(record.get('confidence', 0.0)) < 0.50 or
                not sha256_matches(record, expected_hash) or
                not -0.05 <= receive_age <= 0.50):
            return False
        stamps.append(source)
    latest_age = now-stamps[-1]
    return (-0.05 <= latest_age <= 0.50 and
            all(0.0 < b-a <= 0.50 for a, b in zip(stamps, stamps[1:])))


def transition_audit(transitions, detectors, sim_states, remaining, expected_hash):
    results = []
    for stamp, gate in transitions:
        prior_sim = next((value for t, value in reversed(sim_states) if t <= stamp), '')
        prior_remaining = next((value for t, value in reversed(remaining) if t <= stamp), 0.0)
        prior_detections = [(t, d) for t, d in detectors if t <= stamp]
        checks = []
        last_three = prior_detections[-3:]
        for receive_stamp, d in last_three:
            source = d.get('source_stamp', {}).get('seconds', 0.0)
            receive_age = receive_stamp-source
            checks.append(
                d.get('state') == 'GREEN' and
                float(d.get('confidence', 0.0)) >= 0.50 and
                sha256_matches(d, expected_hash) and
                -0.05 <= receive_age <= 0.50)
        latest_source = (last_three[-1][1].get('source_stamp', {}).get('seconds', 0.0)
                         if last_three else 0.0)
        latest_green_fresh = -0.05 <= stamp-latest_source <= 0.50
        source_stamps = [d.get('source_stamp', {}).get('seconds', 0.0)
                         for _, d in last_three]
        consecutive = (len(source_stamps) == 3 and
                       all(0.0 < b-a <= 0.50
                           for a, b in zip(source_stamps, source_stamps[1:])))
        results.append({
            'stamp': round(stamp, 3), 'gate_status': gate,
            'sim_state': prior_sim, 'green_remaining_s': round(prior_remaining, 3),
            'last_three_yolo_green_fresh_at_receive': checks,
            'latest_yolo_source_fresh_at_authorization': latest_green_fresh,
            'source_stamps_consecutive': consecutive,
            'pass': (prior_sim == 'GREEN' and prior_remaining >= 2.0 and
                     len(checks) == 3 and all(checks) and
                     latest_green_fresh and consecutive),
        })
    return results


def box_clipped_at_frame(item, frame):
    height, width = frame.shape[:2]
    x1, y1, x2, y2 = (float(value) for value in item['box'])
    if x1 <= 0.0 or y1 <= 0.0 or x2 >= width:
        return True
    if y2 >= height-1:
        if item.get('class') not in ('resident', 'stranger'):
            return True
        left = max(0, min(width, int(math.floor(x1))))
        right = max(0, min(width, int(math.ceil(x2))))
        top = max(0, min(height, int(math.floor(y1))))
        roi = frame[top:height, left:right]
        if roi.size == 0:
            return True
        visible_rows = np.where(np.max(roi, axis=2) > 8)[0]
        if (visible_rows.size == 0 or
                height-1-(top+int(visible_rows.max())) < 4):
            return True
    return False


def photo_audit(run_dir, summary):
    results = []
    records = {item.get('waypoint'): item
               for item in summary.get('accepted_photo_records', [])}
    if list(records) != WAYPOINTS:
        return {'pass': False, 'reason': 'PHOTO_ORDER_OR_COUNT', 'points': results}
    for name in WAYPOINTS:
        record = records[name]
        point_dir = run_dir / name
        evidence = [path for path in point_dir.glob('*.json')
                    if not path.name.endswith('.detections.json')]
        detection_files = list(point_dir.glob('*.detections.json'))
        annotated = [path for path in point_dir.glob('*.png')
                     if not path.name.endswith('.raw.png')]
        raw = list(point_dir.glob('*.raw.png'))
        same_stamp = False
        frame_shape_ok = False
        detection_record = {}
        raw_frame = None
        if evidence and detection_files and annotated and raw:
            try:
                evidence_record = json.loads(evidence[0].read_text(encoding='utf-8'))
                detection_record = json.loads(
                    detection_files[0].read_text(encoding='utf-8'))
                summary_stamp = record.get('source_stamp', {}).get('seconds')
                detection_stamp = detection_record.get('source_stamp', {}).get('seconds')
                evidence_stamp = evidence_record.get('source_stamp', {}).get('seconds')
                same_stamp = (
                    summary_stamp is not None and detection_stamp is not None and
                    evidence_stamp is not None and
                    abs(summary_stamp-detection_stamp) <= 1e-6 and
                    abs(summary_stamp-evidence_stamp) <= 1e-6 and
                    detection_record.get('checkpoint_sha256') == YOLO_SHA256)
                raw_frame = cv2.imread(str(raw[0]), cv2.IMREAD_COLOR)
                annotated_rgb = cv2.imread(str(annotated[0]), cv2.IMREAD_COLOR)
                frame_shape_ok = (
                    raw_frame is not None and annotated_rgb is not None and
                    raw_frame.shape[:2] == (480, 640) and
                    annotated_rgb.shape[:2] == (480, 640))
            except (OSError, ValueError, TypeError):
                same_stamp = frame_shape_ok = False
        passed = (record.get('passed') is True and
                  record.get('validation') == 'PASS' and
                  record.get('depth_validation') == 'PASS' and
                  record.get('frame_age_ms', 1e9) <= 500 and
                  record.get('inference_latency_ms', 1e9) <= 250 and
                  same_stamp and frame_shape_ok)
        counts = record.get('detector_counts', {})
        detections = detection_record.get('detections', [])
        minimum_confidence = 0.15 if name in ('POINT_8', 'POINT_9', 'POINT_10') else 0.25
        usable = [item for item in detections
                  if float(item.get('confidence', 0.0)) >= minimum_confidence and
                  raw_frame is not None and not box_clipped_at_frame(item, raw_frame)]
        required_count = None
        computed_edge_excluded = 0
        if name in EXPECTED_PEOPLE:
            raw_people = [item for item in detections
                          if item.get('class') in ('resident', 'stranger') and
                          float(item.get('confidence', 0.0)) >= minimum_confidence]
            people = [item for item in usable
                      if item.get('class') in ('resident', 'stranger')]
            required_count = len(people)
            computed_edge_excluded = len(raw_people)-len(people)
            passed = passed and required_count == EXPECTED_PEOPLE[name]
            if name in ('POINT_2', 'POINT_5'):
                passed = passed and sum(item.get('class') == 'stranger'
                                        for item in people) >= 1
        elif name in ('POINT_1', 'POINT_7'):
            lamps = [item for item in usable
                     if item.get('class') in ('red_on', 'red_off', 'yellow_on',
                                              'yellow_off', 'green_on', 'green_off')]
            colors = {item.get('class', '').split('_', 1)[0] for item in lamps}
            required_count = len(lamps)
            passed = passed and colors == {'red', 'yellow', 'green'}
        else:
            plates = [item for item in usable if item.get('class') == 'license_plate']
            required_count = len(plates)
            passed = passed and required_count >= 1
        results.append({
            'waypoint': name, 'pass': bool(passed),
            'detector_counts': counts,
            'edge_clipped_person_boxes_excluded':
                record.get('edge_clipped_person_boxes_excluded', 0),
            'independent_required_box_count': required_count,
            'independent_edge_person_boxes_excluded': computed_edge_excluded,
            'summary_edge_person_boxes_excluded':
                record.get('edge_clipped_person_boxes_excluded', 0),
            'frame_age_ms': record.get('frame_age_ms'),
            'latency_ms': record.get('inference_latency_ms'),
            'depth': record.get('depth_validation'),
            'same_stamp_record_pass': same_stamp,
            'frame_shape_640x480': frame_shape_ok,
        })
    return {'pass': all(item['pass'] for item in results), 'points': results}


def make_paint_masks(texture):
    height, width = texture.shape
    if height != width:
        raise ValueError('ground texture must be square')
    white = texture >= WHITE_THRESHOLD
    ordinary = white.copy()

    def clear_line(line):
        axis, coordinate, lateral, half, thickness = line
        if axis == 'x':
            x0, x1 = coordinate-thickness, coordinate+thickness
            y0, y1 = lateral-half, lateral+half
        else:
            x0, x1 = lateral-half, lateral+half
            y0, y1 = coordinate-thickness, coordinate+thickness
        r0 = max(0, int(math.floor((0.5-x1/GROUND_M)*height)))
        r1 = min(height, int(math.ceil((0.5-x0/GROUND_M)*height)))
        c0 = max(0, int(math.floor((0.5-y1/GROUND_M)*width)))
        c1 = min(width, int(math.ceil((0.5-y0/GROUND_M)*width)))
        ordinary[r0:r1, c0:c1] = False

    clear_line(START_LINE)
    for _name, axis, line, center, half, thickness in STOP_LINES:
        clear_line((axis, line, center, half, thickness))
    for r0, r1, c0, c1 in TEXTURE_BOXES:
        ordinary[r0:r1+1, c0:c1+1] = False
    return white, ordinary


def footprint_contact(texture, ordinary, x, y, yaw):
    height, width = texture.shape
    c, s = math.cos(yaw), math.sin(yaw)
    vertices = []
    for lx, ly in FOOTPRINT:
        wx, wy = x + lx*c - ly*s, y + lx*s + ly*c
        row = (0.5-wx/GROUND_M)*height
        col = (0.5-wy/GROUND_M)*width
        if row < 0 or row >= height or col < 0 or col >= width:
            return {'out_of_bounds': True, 'ordinary': True}
        vertices.append((col, row))
    vertices = np.asarray(vertices)
    c0 = max(0, int(np.floor(vertices[:, 0].min()))-1)
    r0 = max(0, int(np.floor(vertices[:, 1].min()))-1)
    c1 = min(width, int(np.ceil(vertices[:, 0].max()))+2)
    r1 = min(height, int(np.ceil(vertices[:, 1].max()))+2)
    mask = np.zeros((r1-r0, c1-c0), dtype=np.uint8)
    polygon = np.rint(vertices - [c0, r0]).astype(np.int32)
    cv2.fillPoly(mask, [polygon], 1)
    mask = cv2.dilate(mask, np.ones((3, 3), dtype=np.uint8)) != 0
    white = texture[r0:r1, c0:c1] >= WHITE_THRESHOLD
    ordinary_hit = bool(np.any(mask & white & ordinary[r0:r1, c0:c1]))
    if ordinary_hit:
        return {'out_of_bounds': False, 'ordinary': True}

    rows, cols = np.nonzero(mask & white)
    conditional = {}
    if rows.size:
        rr, cc = rows + r0, cols + c0
        wx = (0.5-(rr+0.5)/height)*GROUND_M
        wy = (0.5-(cc+0.5)/width)*GROUND_M
        conditional['start_line'] = bool(np.any(
            (np.abs(wy-START_LINE[1]) <= START_LINE[4]) &
            (np.abs(wx-START_LINE[2]) <= START_LINE[3])))
        for name, axis, line, center, half, thickness in STOP_LINES:
            coordinate, lateral = (wx, wy) if axis == 'x' else (wy, wx)
            conditional[name] = bool(np.any(
                (np.abs(coordinate-line) <= thickness) &
                (np.abs(lateral-center) <= half)))
        for r0b, r1b, c0b, c1b, name in ZEBRA_BOXES:
            conditional[name + '_zebra'] = bool(np.any(
                (rr >= r0b) & (rr <= r1b) & (cc >= c0b) & (cc <= c1b)))
    return {'out_of_bounds': False, 'ordinary': False, 'conditional': conditional}


def run_bag_audit(bag_path, texture_path, summary):
    texture = cv2.imread(str(texture_path), cv2.IMREAD_GRAYSCALE)
    if texture is None:
        raise RuntimeError('cannot read ground texture: ' + str(texture_path))
    white, ordinary = make_paint_masks(texture)
    bag = rosbag.Bag(str(bag_path))
    detections, sim_states, remaining, gate_events, progress = [], [], [], [], []
    yolo_metrics = []
    authorization_history = {'NORTHBOUND': [(0.0, False)],
                             'WESTBOUND': [(0.0, False)]}
    gate_status = 'CLEAR'
    recent_detections = collections.deque(maxlen=3)
    last_pose = None
    next_pose_sample = None
    max_raw_pose_gap_s = 0.0
    max_position_gap = 0.0
    max_yaw_gap = 0.0
    sampled_poses = 0
    ordinary_contacts = 0
    out_of_bounds_contacts = 0
    conditional_contacts = collections.Counter()
    unauthorized_conditional_contacts = []
    first_ordinary_contact = None
    transitions = []
    last_go_green = None
    green_remaining = 0.0
    sim_state = ''
    expected_hash = YOLO_SHA256
    active_route_distance = active_route_time = 0.0
    progress_by_goal = collections.defaultdict(list)
    motor_commands = []

    def latest_at(series, stamp, default=None):
        times = [item[0] for item in series]
        index = bisect.bisect_right(times, stamp)-1
        return series[index][1] if index >= 0 else default

    def authorization_at(name, stamp):
        history = authorization_history[name]
        index = bisect.bisect_right([item[0] for item in history], stamp)-1
        return history[index][1] if index >= 0 else False

    def stable_green_at(stamp):
        index = bisect.bisect_right([item[0] for item in detections], stamp)-1
        if index < 2:
            return False
        local_detections = detections[index-2:index+1]
        return stable_green(
            stamp, local_detections,
            latest_at(sim_states, stamp, ''),
            latest_at(remaining, stamp, 0.0), expected_hash)

    def gate_status_at(stamp):
        return latest_at(gate_events, stamp, 'CLEAR')

    def check_pose(stamp, x, y, yaw):
        nonlocal sampled_poses, ordinary_contacts, out_of_bounds_contacts
        nonlocal first_ordinary_contact
        sampled_poses += 1
        contact = footprint_contact(texture, ordinary, x, y, yaw)
        if contact['out_of_bounds']:
            out_of_bounds_contacts += 1
        if contact['ordinary']:
            ordinary_contacts += 1
            if first_ordinary_contact is None:
                first_ordinary_contact = {
                    'stamp': round(stamp, 3),
                    'pose': [round(x, 4), round(y, 4), round(yaw, 4)],
                    'gate_status': gate_status_at(stamp),
                }
        for key, touches in contact.get('conditional', {}).items():
            if not touches:
                continue
            conditional_contacts[key] += 1
            if key == 'start_line':
                continue
            gate_name = key.replace('_zebra', '')
            permitted = authorization_at(gate_name, stamp)
            if key.endswith('_zebra'):
                permitted = permitted or stable_green_at(stamp)
            if not permitted and len(unauthorized_conditional_contacts) < 8:
                unauthorized_conditional_contacts.append({
                    'stamp': round(stamp, 3), 'marking': key,
                    'gate_status': gate_status_at(stamp),
                    'pose': [round(x, 4), round(y, 4), round(yaw, 4)],
                })

    topics = ['/gazebo/link_states', '/my_car/cmd_vel',
              '/traffic_light/gate_status',
              '/traffic_light/state', '/traffic_light/time_remaining',
              '/inspection/traffic_light', '/inspection/yolo/metrics',
              '/route/progress']
    chassis_index = None
    for topic, message, bag_stamp in bag.read_messages(topics=topics):
        t = bag_stamp.to_sec()
        if topic == '/traffic_light/state':
            sim_state = str(message.data).upper()
            sim_states.append((t, sim_state))
        elif topic == '/traffic_light/time_remaining':
            green_remaining = max(0.0, float(message.data))
            remaining.append((t, green_remaining))
        elif topic == '/inspection/traffic_light':
            try:
                record = json.loads(message.data)
            except (ValueError, TypeError):
                record = {'state': 'INVALID', 'confidence': 0.0}
            detections.append((t, record))
            recent_detections.append((t, record))
        elif topic == '/inspection/yolo/metrics':
            try:
                yolo_metrics.append((t, json.loads(message.data)))
            except (ValueError, TypeError):
                pass
        elif topic == '/traffic_light/gate_status':
            gate_status = str(message.data)
            gate_events.append((t, gate_status))
            if gate_status.endswith(':GO_GREEN') and gate_status != last_go_green:
                transitions.append((t, gate_status))
                last_go_green = gate_status
            elif not gate_status.endswith(':GO_GREEN'):
                last_go_green = None
            for name in authorization_history:
                authorization = None
                if gate_status.startswith(name + ':'):
                    if gate_status in (name + ':GO_GREEN', name + ':CLEARING',
                                       name + ':CLEARED'):
                        authorization = True
                    elif 'WAIT_' in gate_status:
                        authorization = False
                if authorization is not None:
                    history = authorization_history[name]
                    if history[-1][1] != authorization:
                        history.append((t, authorization))
        elif topic == '/route/progress':
            try:
                record = json.loads(message.data)
            except (ValueError, TypeError):
                continue
            goal = record.get('goal', '?')
            progress_by_goal[goal].append((t, record))
        elif topic == '/my_car/cmd_vel':
            motor_commands.append((t, float(message.linear.x),
                                   float(message.angular.z)))
        elif topic == '/gazebo/link_states':
            if chassis_index is None:
                try:
                    chassis_index = message.name.index('my_car::chassis')
                except ValueError:
                    raise RuntimeError('Gazebo truth bag has no my_car::chassis')
            if next_pose_sample is None:
                next_pose_sample = t
            if t + 1e-9 < next_pose_sample:
                continue
            pose = message.pose[chassis_index]
            x, y = pose.position.x, pose.position.y
            q = pose.orientation
            yaw = math.atan2(2.0*(q.w*q.z+q.x*q.y),
                             1.0-2.0*(q.y*q.y+q.z*q.z))
            if last_pose is not None:
                old_t, px, py, pyaw = last_pose
                raw_dt = max(0.0, t-old_t)
                max_raw_pose_gap_s = max(max_raw_pose_gap_s, raw_dt)
                dx, dy = x-px, y-py
                distance = math.hypot(dx, dy)
                delta_yaw = wrap(yaw-pyaw)
                steps = max(1, int(math.ceil(distance/0.01)),
                            int(math.ceil(abs(delta_yaw)/YAW_SAMPLE_STEP)))
                max_position_gap = max(max_position_gap, distance/steps)
                max_yaw_gap = max(max_yaw_gap, abs(delta_yaw)/steps)
                for step in range(1, steps+1):
                    fraction = step/float(steps)
                    check_pose(old_t+raw_dt*fraction,
                               px+dx*fraction, py+dy*fraction,
                               pyaw+delta_yaw*fraction)
            else:
                check_pose(t, x, y, yaw)
            last_pose = (t, x, y, yaw)
            next_pose_sample = t + POSE_SAMPLE_PERIOD_S

    mission_duration = bag.get_end_time()-bag.get_start_time()
    bag.close()
    transition_results = transition_audit(
        transitions, detections, sim_states, remaining, expected_hash)
    gate_counts = collections.Counter(value for _, value in gate_events)
    white_status_count = sum(value.startswith('WHITE_LINE:')
                             for _, value in gate_events)

    for goal, records in progress_by_goal.items():
        values = [float(record.get('route_progress_m', 0.0)) for _, record in records]
        distance = max(values, default=0.0)-min(values, default=0.0)
        active_route_distance += distance
        for (t0, a), (t1, b) in zip(records, records[1:]):
            if ('WAIT_' in str(a.get('gate_status', '')) or
                    'WAIT_' in str(b.get('gate_status', ''))):
                continue
            dt = max(0.0, t1-t0)
            active_route_time += dt

    speed = (active_route_distance/active_route_time
             if active_route_time > 0 else 0.0)
    command_times = [item[0] for item in motor_commands]
    gate_times_for_metrics = [item[0] for item in gate_events]

    def window_active(stamp, metric):
        window = float(metric.get('window_s', 1.0))
        begin = stamp-window
        c0 = bisect.bisect_left(command_times, begin)
        c1 = bisect.bisect_right(command_times, stamp)
        moving = any(abs(linear) > 0.01 or abs(angular) > 0.05
                     for _, linear, angular in motor_commands[c0:c1])
        g0 = bisect.bisect_left(gate_times_for_metrics, begin)
        g1 = bisect.bisect_right(gate_times_for_metrics, stamp)
        gate_active = any(value != 'CLEAR'
                          for _, value in gate_events[g0:g1])
        return moving or gate_active

    active_yolo_windows = [(t, m) for t, m in yolo_metrics
                           if window_active(t, m)]
    inactive_yolo_windows = len(yolo_metrics)-len(active_yolo_windows)
    hz_windows = [float(m['processed_hz']) for _, m in active_yolo_windows
                  if isinstance(m.get('processed_hz'), (int, float))]
    latency_windows = [float(m['p95_latency_ms']) for _, m in active_yolo_windows
                       if isinstance(m.get('p95_latency_ms'), (int, float))]
    age_windows = [float(m['max_frame_age_ms']) for _, m in active_yolo_windows
                   if isinstance(m.get('max_frame_age_ms'), (int, float))]
    source_stamps = [d.get('source_stamp', {}).get('seconds')
                     for _, d in detections
                     if isinstance(d.get('source_stamp', {}).get('seconds'), (int, float))]
    source_gaps = [b-a for a, b in zip(source_stamps, source_stamps[1:]) if b > a]
    p95 = lambda values: (float(np.percentile(values, 95)) if values else float('inf'))
    hz_fraction = (sum(value >= 5.0 for value in hz_windows)/len(hz_windows)
                   if hz_windows else 0.0)
    yolo_hashes_valid = all(
        item.get('checkpoint_sha256') == expected_hash and
        set(item.get('classes', [])) == EXPECTED_CLASSES
        for _, item in yolo_metrics)
    yolo_pass = bool(
        hz_windows and yolo_hashes_valid and
        statistics.mean(hz_windows) >= 4.95 and hz_fraction >= 0.95 and
        p95(latency_windows) <= 250.0 and
        age_windows and max(age_windows) <= 500.0 and
        source_gaps and max(source_gaps) <= 0.50)
    yolo_report = {
        'metric_windows': len(yolo_metrics),
        'active_metric_windows': len(active_yolo_windows),
        'inactive_metric_windows_excluded': inactive_yolo_windows,
        'mean_processed_hz': statistics.mean(hz_windows) if hz_windows else 0.0,
        'fraction_windows_at_least_5_hz': hz_fraction,
        'p95_of_window_p95_latency_ms': p95(latency_windows),
        'max_of_window_max_frame_age_ms': max(age_windows) if age_windows else None,
        'maximum_detection_source_gap_s': max(source_gaps) if source_gaps else None,
        'checkpoint_and_class_map_valid': yolo_hashes_valid,
        'pass': yolo_pass,
    }
    gate_times = [event[0] for event in gate_events]
    forward_command_seconds = rotation_command_seconds = 0.0
    maximum_forward_command = maximum_approach_command = 0.0
    for (t0, linear, angular), (t1, _, _) in zip(
            motor_commands, motor_commands[1:]):
        index = bisect.bisect_right(gate_times, t0)-1
        status = gate_events[index][1] if index >= 0 else 'CLEAR'
        dt = max(0.0, t1-t0)
        if 'WAIT_' not in status:
            if linear > 0.01:
                forward_command_seconds += dt
            elif abs(angular) > 0.05:
                rotation_command_seconds += dt
        maximum_forward_command = max(maximum_forward_command, linear)
        if status.endswith(':APPROACH'):
            maximum_approach_command = max(maximum_approach_command, linear)
    translation_speed = (active_route_distance/forward_command_seconds
                        if forward_command_seconds > 0 else 0.0)
    return {
        'bag_path': str(bag_path),
        'mission_duration_s': mission_duration,
        'sampled_pose_count': sampled_poses,
        'max_raw_pose_gap_s': max_raw_pose_gap_s,
        'max_translation_gap_m': max_position_gap,
        'max_yaw_gap_deg': math.degrees(max_yaw_gap),
        'ordinary_paint_contact_samples': ordinary_contacts,
        'out_of_bounds_pose_samples': out_of_bounds_contacts,
        'first_ordinary_contact': first_ordinary_contact,
        'conditional_marking_contact_samples': dict(conditional_contacts),
        'unauthorized_conditional_contacts': unauthorized_conditional_contacts,
        'gate_status_counts': dict(gate_counts),
        'white_line_gate_status_count': white_status_count,
        'green_authorization_transitions': transition_results,
        'all_green_authorizations_valid': bool(transition_results) and
            all(event['pass'] for event in transition_results),
        'active_route_progress_m': active_route_distance,
        'active_route_seconds_excluding_signal_waits': active_route_time,
        'active_route_speed_mps': speed,
        'yolo_live_metrics': yolo_report,
        'forward_command_seconds_excluding_signal_waits': forward_command_seconds,
        'translation_route_speed_mps': translation_speed,
        'rotation_command_seconds_excluding_signal_waits': rotation_command_seconds,
        'maximum_forward_command_mps': maximum_forward_command,
        'maximum_signal_approach_command_mps': maximum_approach_command,
    }


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('run_dir', type=Path)
    parser.add_argument('--texture', type=Path,
                        default=Path('/root/competition_ground_map.png'))
    parser.add_argument('--min-translation-speed', type=float, default=0.15)
    parser.add_argument('--max-mission-duration', type=float, default=180.0)
    parser.add_argument('--output', type=Path, default=None)
    args = parser.parse_args()
    summary_path = args.run_dir / 'run_summary.json'
    bag_path = args.run_dir / 'motion.bag'
    summary = json.loads(summary_path.read_text(encoding='utf-8'))
    expected_goals = WAYPOINTS + ['HOME']
    goals = summary.get('goals', [])
    sequence_pass = [item.get('name') for item in goals] == expected_goals
    goal_pose_checks = []
    for item in goals[:-1]:
        name = item.get('name')
        xy_limit = POINT_XY_TOLERANCES.get(name, 0.03)
        yaw_limit = POINT_YAW_TOLERANCES.get(name, 0.04)
        xy_error = item.get('position_error_m', float('inf'))
        yaw_error = item.get('heading_error_rad', float('inf'))
        goal_pose_checks.append({
            'goal': name, 'xy_error_m': xy_error, 'xy_limit_m': xy_limit,
            'yaw_error_rad': yaw_error, 'yaw_limit_rad': yaw_limit,
            'pass': (xy_error <= xy_limit and yaw_error <= yaw_limit),
        })
    goal_pass = (sequence_pass and
                 all(item.get('action_state') == 3 and
                     item.get('result') == 'ACTION_RESULT' and
                     item.get('attempts') == 1 for item in goals) and
                 all(item['pass'] for item in goal_pose_checks))
    parking = summary.get('parking') or {}
    yolo_hash_pass = summary.get('yolo_checkpoint_sha256') == YOLO_SHA256
    parking_pass = (summary.get('route_status') == 'COMPLETE_PARKED' and
                    parking.get('position_error_m', 1e9) <= 0.03 and
                    parking.get('heading_error_rad', 1e9) <= 0.04 and
                    parking.get('linear_speed_mps', 1e9) < 0.01 and
                    parking.get('angular_speed_rps', 1e9) < 0.01 and
                    parking.get('stationary_seconds', 0.0) >= 2.0)
    photos = photo_audit(args.run_dir, summary)
    bag_metrics = run_bag_audit(bag_path, args.texture, summary)
    bag_pass = (bag_metrics['ordinary_paint_contact_samples'] == 0 and
                bag_metrics['out_of_bounds_pose_samples'] == 0 and
                not bag_metrics['unauthorized_conditional_contacts'] and
                bag_metrics['white_line_gate_status_count'] == 0 and
                bag_metrics['all_green_authorizations_valid'] and
                bag_metrics['yolo_live_metrics']['pass'] and
                bag_metrics['max_translation_gap_m'] <= 0.01 and
                bag_metrics['max_yaw_gap_deg'] <= 1.0 and
                bag_metrics['max_raw_pose_gap_s'] <= 0.10)
    speed_pass = (bag_metrics['translation_route_speed_mps'] >=
                  args.min_translation_speed and
                  bag_metrics['mission_duration_s'] <= args.max_mission_duration and
                  bag_metrics['maximum_forward_command_mps'] <= 0.35+1e-6 and
                  bag_metrics['maximum_signal_approach_command_mps'] <= 0.12+1e-6)
    person_audit = None
    if summary.get('person_reporting_enabled'):
        from audit_person_report import audit
        person_audit = audit(args.run_dir)
        (args.run_dir/'person_audit.json').write_text(
            json.dumps(person_audit, ensure_ascii=False, indent=2)+'\n', encoding='utf-8')
    person_pass = person_audit is None or person_audit['pass']
    report = {
        'run_dir': str(args.run_dir),
        'route_status': summary.get('route_status'),
        'strict_acceptance': summary.get('strict_acceptance'),
        'yolo_checkpoint_hash_pass': yolo_hash_pass,
        'goal_pass': goal_pass,
        'goal_pose_checks': goal_pose_checks,
        'photo_pass': photos['pass'],
        'parking_pass': parking_pass,
        'speed_pass': speed_pass,
        'translation_speed_threshold_mps': args.min_translation_speed,
        'maximum_mission_duration_s': args.max_mission_duration,
        'bag_safety_pass': bag_pass,
        'person_report_pass': person_pass,
        'person_audit': person_audit,
        'photos': photos,
        'bag_audit': bag_metrics,
        'acceptance_pass': bool(summary.get('strict_acceptance') and yolo_hash_pass and goal_pass and
                                photos['pass'] and parking_pass and speed_pass and bag_pass and person_pass),
    }
    data = json.dumps(report, ensure_ascii=False, indent=2)
    if args.output:
        args.output.write_text(data+'\n', encoding='utf-8')
    print(data)
    raise SystemExit(0 if report['acceptance_pass'] else 1)


if __name__ == '__main__':
    main()
