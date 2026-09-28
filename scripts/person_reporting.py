#!/usr/bin/env python3
"""Static-person counting from camera observations, independent of Gazebo truth."""

import json
import math
from pathlib import Path

import numpy as np
import cv2


PERSON_CLASSES = ('resident', 'stranger')


def annotate_people(raw, people):
    """Keep labels inside each box's width so adjacent person labels stay readable."""
    canvas = raw.copy()
    for person in people:
        x1, y1, x2, y2 = map(int, person['box'])
        color = (0,165,255) if person['class'] == 'stranger' else (255,220,0)
        cv2.rectangle(canvas, (x1,y1), (x2,y2), color, 2)
        lines = [person['person_id'].replace('PERSON_', '#')+' '+person['street'],
                 person['class'], '%.3f' % person['confidence']]
        max_width = max(cv2.getTextSize(line, cv2.FONT_HERSHEY_SIMPLEX, 1, 1)[0][0]
                        for line in lines)
        scale = min(.34, max(.16, (x2-x1-4)/float(max_width)))
        top = max(0, y1-44)
        cv2.rectangle(canvas, (x1,top), (x2,y1-2), (20,20,20), -1)
        for index, line in enumerate(lines):
            cv2.putText(canvas, line, (x1+2, top+12+index*13),
                        cv2.FONT_HERSHEY_SIMPLEX, scale, color, 1, cv2.LINE_AA)
    return canvas


def appearance_descriptor(image, box):
    x1, y1, x2, y2 = map(int, box)
    height, width = image.shape[:2]
    crop = image[max(0,y1):min(height,y2), max(0,x1):min(width,x2)]
    if crop.size == 0:
        raise ValueError('EMPTY_PERSON_APPEARANCE')
    pixels = cv2.resize(crop, (16,32), interpolation=cv2.INTER_AREA).astype(float)
    pixels -= pixels.mean(axis=(0,1), keepdims=True)
    feature = pixels.ravel()
    length = float(np.linalg.norm(feature))
    if length < 1e-6:
        raise ValueError('UNINFORMATIVE_PERSON_APPEARANCE')
    return (feature/length).tolist()


def rotation_matrix(quaternion):
    q = np.asarray(quaternion, dtype=float)
    q /= np.linalg.norm(q)
    x, y, z, w = q
    return np.array([
        [1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],
        [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)],
        [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)],
    ])


def calibrated_intrinsics(camera):
    if 'K' in camera:
        return np.asarray(camera['K'], dtype=float).reshape(3, 3)
    width, height = int(camera['width']), int(camera['height'])
    focal = width / (2.0 * math.tan(float(camera['horizontal_fov_rad']) / 2.0))
    return np.array([[focal, 0, (width-1)/2.0],
                     [0, focal, (height-1)/2.0], [0, 0, 1]])


def foreground_position(item, depth, intrinsics, camera_pose, image_shape):
    """Use torso foreground depth, excluding the floor/box background."""
    x1, y1, x2, y2 = map(float, item['box'])
    dh, dw = depth.shape
    ih, iw = image_shape[:2]
    sx, sy = dw / float(iw), dh / float(ih)
    width, height = x2-x1, y2-y1
    left = max(0, int((x1+.20*width)*sx))
    right = min(dw, int(math.ceil((x1+.80*width)*sx)))
    top = max(0, int((y1+.20*height)*sy))
    bottom = min(dh, int(math.ceil((y1+.70*height)*sy)))
    roi = depth[top:bottom, left:right]
    valid = roi[np.isfinite(roi) & (roi >= .10) & (roi <= 4.0)]
    if valid.size < 30:
        raise ValueError('NO_TORSO_FOREGROUND_DEPTH')
    near = float(np.percentile(valid, 10))
    foreground = valid[np.abs(valid-near) <= .025]
    if foreground.size < 30:
        raise ValueError('INSUFFICIENT_FOREGROUND_DEPTH')
    z = float(np.median(foreground))
    k = np.asarray(intrinsics, dtype=float).reshape(3, 3)
    u, v = (x1+x2)/2.0*sx, (y1+y2)/2.0*sy
    optical = np.array([(u-k[0, 2])*z/k[0, 0],
                        (v-k[1, 2])*z/k[1, 1], z])
    position = rotation_matrix(camera_pose['quaternion']).dot(optical)
    position += np.array([camera_pose['x'], camera_pose['y'], camera_pose['z']])
    return position[:2].tolist(), {
        'foreground_depth_m': z, 'foreground_pixels': int(foreground.size),
        'torso_valid_pixels': int(valid.size),
        'depth_band_m': [round(float(foreground.min()), 4),
                         round(float(foreground.max()), 4)],
    }


def inside_polygon(point, polygon):
    x, y = point
    inside = False
    for a, b in zip(polygon, polygon[1:]+polygon[:1]):
        if ((a[1] > y) != (b[1] > y) and
                x < (b[0]-a[0])*(y-a[1])/(b[1]-a[1])+a[0]):
            inside = not inside
    return inside


def collect_observations(record, raw_image, depth, camera_pose, intrinsics,
                         config, waypoint, image_path):
    observations, errors = [], []
    for item in record.get('detections', []):
        if (item.get('class') not in PERSON_CLASSES or
                float(item.get('confidence', 0)) < config['minimum_detection_confidence']):
            continue
        try:
            position, measurement = foreground_position(
                item, depth, intrinsics, camera_pose, raw_image.shape)
            streets = [name for name, polygon in config['streets'].items()
                       if inside_polygon(position, polygon)]
            if len(streets) != 1:
                raise ValueError('PERSON_OUTSIDE_OR_AMBIGUOUS_STREET')
            observations.append({
                'class': item['class'], 'confidence': float(item['confidence']),
                'box': list(item['box']), 'world_xy': position, 'street': streets[0],
                'measurement': measurement, 'waypoint': waypoint,
                'source_stamp': record['source_stamp'], 'image_path': image_path,
                'appearance': appearance_descriptor(raw_image, item['box']),
            })
        except (ValueError, KeyError) as error:
            errors.append({'waypoint': waypoint, 'box': item.get('box'),
                           'class': item.get('class'), 'reason': str(error)})
    return observations, errors


class PersonCounter:
    """Associate static person positions once per accepted source frame.

    Assignment is one-to-one within each frame, so nearby different people
    cannot collapse into one track. Classes are evidence attached to a track,
    never an identity key or a source of the total count.
    """

    def __init__(self, config):
        self.config = config
        self.tracks = []
        self.errors = []
        self.frames = set()
        self.views = set()

    def add_view(self, waypoint, source_stamp, observations, errors=()):
        stamp = (int(source_stamp['secs']), int(source_stamp['nsecs']))
        key = (waypoint, stamp)
        if key in self.frames:
            return []
        self.frames.add(key)
        self.views.add(waypoint)
        self.errors.extend(errors)
        candidates = []
        for oi, observation in enumerate(observations):
            for ti, track in enumerate(self.tracks):
                distance = float(np.linalg.norm(np.array(observation['world_xy'])-
                                                np.array(track['world_xy'])))
                similarity = None
                if observation.get('appearance') is not None and track.get('appearance') is not None:
                    similarity = float(np.dot(observation['appearance'], track['appearance']))
                spatial_match = (distance <= self.config['association_radius_m'] and
                    (similarity is None or similarity >=
                     self.config.get('spatial_match_min_appearance_cosine',.90)))
                appearance_match = (similarity is not None and
                    distance <= self.config.get('appearance_radius_m',.12) and
                    similarity >= self.config.get('minimum_appearance_cosine',.94))
                if observation['street'] == track['street'] and (spatial_match or appearance_match):
                    candidates.append((distance, oi, ti))
        matches, used_tracks = {}, set()
        for _, oi, ti in sorted(candidates):
            if oi not in matches and ti not in used_tracks:
                matches[oi] = ti
                used_tracks.add(ti)
        resolved = []
        for oi, observation in enumerate(observations):
            if oi in matches:
                track = self.tracks[matches[oi]]
            else:
                track = {'person_id': 'PERSON_%03d' % (len(self.tracks)+1),
                         'world_xy': list(observation['world_xy']),
                         'appearance': observation.get('appearance'),
                         'street': observation['street'], 'observations': []}
                self.tracks.append(track)
            track['observations'].append(observation)
            resolved.append({**observation, 'person_id': track['person_id']})
        return resolved

    def report(self):
        people, issues = [], list(self.errors)
        counts = {'total': len(self.tracks), 'resident': 0, 'stranger': 0}
        streets = {name: {'total': 0, 'resident': 0, 'stranger': 0}
                   for name in self.config['streets']}
        for track in self.tracks:
            votes = {name: sum(o['confidence'] for o in track['observations']
                               if o['class'] == name) for name in PERSON_CLASSES}
            category = max(votes, key=votes.get)
            if all(votes[name] > 0 for name in PERSON_CLASSES):
                issues.append({'person_id': track['person_id'],
                               'reason': 'CLASS_CONFLICT_REQUIRES_REVIEW'})
            counts[category] += 1
            streets[track['street']]['total'] += 1
            streets[track['street']][category] += 1
            public_track = {key: value for key,value in track.items()
                            if key not in ('appearance','observations')}
            public_track['observations'] = [
                {key: value for key,value in o.items() if key != 'appearance'}
                for o in track['observations']]
            people.append({**public_track, 'class': category,
                           'confidence': max(o['confidence'] for o in track['observations']),
                           'class_votes': votes})
        expected = self.config['expected_inventory']
        checks = {name: counts[name] == expected[name]
                  for name in ('total', 'resident', 'stranger')}
        checks['streets'] = streets == expected['streets']
        checks['required_views'] = set(self.config['required_views']).issubset(self.views)
        checks['resolved_observations'] = not issues
        observations_total = sum(len(t['observations']) for t in self.tracks)
        text = '街区内共有%d人，其中社区人员%d人，外来人员%d人。' % (
            counts['total'], counts['resident'], counts['stranger'])
        for street, values in streets.items():
            text += '%s街区%d人，社区人员%d人，外来人员%d人。' % (
                street, values['total'], values['resident'], values['stranger'])
        foreigners = [person for person in people if person['class'] == 'stranger']
        text += '外来人员图片已保存。' if foreigners else '未检出外来人员。'
        complete = all(checks.values())
        if not complete:
            text = '人员统计待复核。' + text
        return {'counts': counts, 'street_counts': streets, 'people': people,
                'foreign_people': foreigners, 'observations_total': observations_total,
                'duplicate_observations': observations_total-len(self.tracks),
                'expected_inventory': expected, 'checks': checks,
                'issues': issues, 'complete': complete,
                'speech_text': text,
                'source': 'YOLO + foreground depth + map camera TF; no simulator truth'}


def save_report(counter, directory):
    report = counter.report()
    directory = Path(directory)
    directory.mkdir(parents=True, exist_ok=True)
    (directory/'person_report.json').write_text(
        json.dumps(report, ensure_ascii=False, indent=2)+'\n', encoding='utf-8')
    lines = [report['speech_text']]
    for person in report['foreign_people']:
        links = sorted({o['image_path'] for o in person['observations']})
        lines.append('%s街区发现外来人员%s，置信度%.3f，图片：%s' % (
            person['street'], person['person_id'], person['confidence'], '；'.join(links)))
    if report['issues']:
        lines.append('待复核事项：'+json.dumps(report['issues'], ensure_ascii=False))
    (directory/'person_report.txt').write_text('\n'.join(lines)+'\n', encoding='utf-8')
    return report
