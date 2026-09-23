#!/usr/bin/env python3
"""Reject copied truth maps and write a provenance manifest for a saved SLAM map."""
import argparse, hashlib, json, pathlib, sys
from PIL import Image

def sha(p): return hashlib.sha256(p.read_bytes()).hexdigest()

MANUAL_ARTIFACT = 'not-applicable-manual-teleop'


def publisher_set(value):
    """Normalize a comma/newline separated publisher observation."""
    return sorted({item.strip() for item in value.replace('\n', ',').split(',') if item.strip()})


def require_hash(path_value, expected, label):
    """Return a verified artifact record, failing closed on missing evidence."""
    if not path_value:
        raise SystemExit('rejected: missing %s file' % label)
    path = pathlib.Path(path_value)
    if not path.is_file():
        raise SystemExit('rejected: missing %s file: %s' % (label, path))
    actual = sha(path)
    if not expected:
        raise SystemExit('rejected: missing %s sha256' % label)
    if expected != actual:
        raise SystemExit('rejected: %s sha256 mismatch expected=%s actual=%s' %
                         (label, expected, actual))
    return {'file': str(path), 'sha256': actual, 'verified': True}


def read_trajectory(path):
    try:
        data = json.loads(pathlib.Path(path).read_text(encoding='utf-8'))
    except (OSError, ValueError) as exc:
        raise SystemExit('rejected: invalid trajectory JSON: %s' % exc)
    if not isinstance(data, dict):
        raise SystemExit('rejected: trajectory must be a JSON object')
    points = data.get('points', [])
    if not isinstance(points, list):
        raise SystemExit('rejected: trajectory points must be a list')
    return data, points


def parse_pose(value, label):
    try:
        parts = [float(item.strip()) for item in value.split(',')]
    except ValueError:
        raise SystemExit('rejected: invalid %s pose: %s' % (label, value))
    if len(parts) != 3:
        raise SystemExit('rejected: %s pose must be x,y,yaw' % label)
    return parts


def angle_error(a, b):
    return abs((a - b + 3.141592653589793) % (2.0 * 3.141592653589793) - 3.141592653589793)

def pixels(path):
    image = Image.open(path).convert('L')
    image.load()
    return image

def classes(image):
    # map_saver emits 0 occupied, 254 free, and 205 unknown by convention.
    data = list(image.getdata())
    return bytes(0 if v < 65 else 100 if v < 230 else 255 for v in data)

def yaml_meta(path):
    if not path or not pathlib.Path(path).is_file():
        return {}
    out = {}
    for line in pathlib.Path(path).read_text(encoding='utf-8').splitlines():
        if ':' not in line:
            continue
        key, value = line.split(':', 1)
        out[key.strip()] = value.strip()
    return out

def main():
    p=argparse.ArgumentParser()
    p.add_argument('pgm')
    p.add_argument('manifest')
    p.add_argument('--truth',default='maps/competition_ground_truth.pgm')
    p.add_argument('--yaml')
    p.add_argument('--publisher',required=True)
    p.add_argument('--publisher-before',default='')
    p.add_argument('--publisher-after',default='')
    p.add_argument('--topic-rates',default='')
    p.add_argument('--route',required=True)
    p.add_argument('--stop-reason',required=True)
    p.add_argument('--min-known-fraction',type=float,default=0.15)
    p.add_argument('--birth', default='4.0833,-3.9583,1.5708')
    p.add_argument('--session-id', default='unknown')
    p.add_argument('--tf-authorities', default='unknown')
    p.add_argument('--trajectory', default='')
    p.add_argument('--coverage-note', default='coverage not equivalent to full-world map')
    p.add_argument('--mapper-sha256', default='')
    p.add_argument('--log-sha256', default='')
    p.add_argument('--trajectory-sha256', default='')
    p.add_argument('--mapper-file', default='')
    p.add_argument('--log-file', default='')
    p.add_argument('--tf-evidence-file', default='')
    p.add_argument('--coverage-report', default='')
    p.add_argument('--session-start', default='')
    p.add_argument('--session-end', default='')
    p.add_argument('--trajectory-frame', default='chassis')
    p.add_argument('--birth-chassis', default='4.0833,-4.0833,1.5708')
    p.add_argument('--birth-axle', default='4.0833,-3.9583,1.5708')
    p.add_argument('--tf-evidence', default='node-name inventory; message authority not captured')
    a=p.parse_args()
    pgm=pathlib.Path(a.pgm); truth=pathlib.Path(a.truth)
    if not pgm.is_file(): raise SystemExit('missing map image')
    im=pixels(pgm); h=sha(pgm); th=sha(truth) if truth.exists() else None
    truth_im=pixels(truth) if truth.exists() else None
    pixel_equal=bool(truth_im and im.size == truth_im.size and list(im.getdata()) == list(truth_im.getdata()))
    # Also catch a truth image rewritten with a different PGM header or scale.
    normalized_equal=False
    if truth_im:
        normalized_equal = classes(im.resize(truth_im.size, Image.Resampling.NEAREST)) == classes(truth_im)
    if h==th or pixel_equal or normalized_equal:
        raise SystemExit('rejected: map pixels are identical to truth map')
    if a.publisher.strip() != '/slam_gmapping': raise SystemExit('rejected: /map publisher was '+a.publisher)
    if publisher_set(a.publisher_before) != ['/slam_gmapping']:
        raise SystemExit('rejected: /map publisher set before save was %s' % a.publisher_before)
    if publisher_set(a.publisher_after) != ['/slam_gmapping']:
        raise SystemExit('rejected: /map publisher set after save was %s' % a.publisher_after)
    if not a.session_start or not a.session_end:
        raise SystemExit('rejected: session start/end timestamps are required')
    raw=list(im.getdata())
    occupied=sum(v < 65 for v in raw)
    unknown=sum(65 <= v < 230 for v in raw)
    free=len(raw)-occupied-unknown
    known_fraction=(occupied+free)/len(raw)
    if known_fraction < a.min_known_fraction:
        raise SystemExit('rejected: known map fraction %.3f below %.3f' % (known_fraction, a.min_known_fraction))
    trajectory_path = pathlib.Path(a.trajectory)
    trajectory_record = require_hash(a.trajectory, a.trajectory_sha256, 'trajectory')
    trajectory, trajectory_points = read_trajectory(trajectory_path)
    artifactless_mode = (a.mapper_sha256 == MANUAL_ARTIFACT and a.log_sha256 == MANUAL_ARTIFACT)
    if artifactless_mode:
        mode = trajectory.get('mode')
        if mode not in ('manual_teleop', 'focused_autonomous'):
            raise SystemExit('rejected: artifactless hashes require manual_teleop or focused_autonomous trajectory')
        mapper_record = {'file': None, 'sha256': MANUAL_ARTIFACT, 'verified': False,
                         'verification': '%s_not_applicable' % mode}
        log_record = {'file': None, 'sha256': MANUAL_ARTIFACT, 'verified': False,
                      'verification': '%s_not_applicable' % mode}
    else:
        if not trajectory_points:
            raise SystemExit('rejected: automatic trajectory has no measured points')
        if 'stop_reason' not in trajectory or 'completed_waypoints' not in trajectory:
            raise SystemExit('rejected: automatic trajectory lacks completion/stop evidence')
        if trajectory.get('stop_reason') != a.stop_reason:
            raise SystemExit('rejected: stop reason mismatch manifest=%s trajectory=%s' %
                             (a.stop_reason, trajectory.get('stop_reason')))
        mapper_record = require_hash(a.mapper_file, a.mapper_sha256, 'mapper')
        log_record = require_hash(a.log_file, a.log_sha256, 'survey log')
    if not a.tf_evidence_file:
        raise SystemExit('rejected: missing message-level TF evidence file')
    tf_path = pathlib.Path(a.tf_evidence_file)
    if not tf_path.is_file():
        raise SystemExit('rejected: missing TF evidence file: %s' % tf_path)
    try:
        tf_evidence = json.loads(tf_path.read_text(encoding='utf-8'))
    except (OSError, ValueError) as exc:
        raise SystemExit('rejected: invalid TF evidence JSON: %s' % exc)
    if not isinstance(tf_evidence, dict) or not tf_evidence.get('transforms'):
        raise SystemExit('rejected: TF evidence has no sampled transforms')
    if not a.coverage_report:
        raise SystemExit('rejected: missing static coverage report')
    coverage_path = pathlib.Path(a.coverage_report)
    if not coverage_path.is_file():
        raise SystemExit('rejected: missing coverage report: %s' % coverage_path)
    try:
        coverage = json.loads(coverage_path.read_text(encoding='utf-8'))
    except (OSError, ValueError) as exc:
        raise SystemExit('rejected: invalid coverage report JSON: %s' % exc)
    if not isinstance(coverage, dict) or coverage.get('kind') != 'static_map_coverage_report':
        raise SystemExit('rejected: coverage report kind is invalid')
    if 'frontier_cells' not in coverage or 'gate_a_candidate' not in coverage:
        raise SystemExit('rejected: coverage report lacks frontier/gate fields')
    measured_birth = None
    if trajectory_points:
        first = trajectory_points[0]
        try:
            measured_birth = [float(first['x']), float(first['y']), float(first['yaw'])]
        except (KeyError, TypeError, ValueError):
            raise SystemExit('rejected: first trajectory point has no x/y/yaw')
    configured_chassis = parse_pose(a.birth_chassis, 'birth_chassis')
    configured_axle = parse_pose(a.birth_axle, 'birth_axle')
    if measured_birth and a.trajectory_frame == 'chassis':
        position_error = ((measured_birth[0] - configured_chassis[0]) ** 2 +
                          (measured_birth[1] - configured_chassis[1]) ** 2) ** 0.5
        yaw_error = angle_error(measured_birth[2], configured_chassis[2])
        if position_error > 0.25 or yaw_error > 0.35:
            raise SystemExit('rejected: measured chassis birth differs from configured birth '
                             'position=%.3f yaw=%.3f' % (position_error, yaw_error))
    data={'map_file':str(pgm),'sha256':h,'yaml_file':str(a.yaml) if a.yaml else None,
          'yaml_sha256':sha(pathlib.Path(a.yaml)) if a.yaml and pathlib.Path(a.yaml).is_file() else None,
          'truth_sha256':th,'truth_pixel_equal':pixel_equal,
          'truth_normalized_equal':normalized_equal,'size':list(im.size),'mode':im.mode,
          'resolution':yaml_meta(a.yaml).get('resolution'),'origin':yaml_meta(a.yaml).get('origin'),
          'occupied_cells':occupied,'free_cells':free,'unknown_cells':unknown,
          'known_fraction':known_fraction,'publisher':'/slam_gmapping','route':a.route,'stop_reason':a.stop_reason,
          'publisher_before':a.publisher_before,'publisher_after':a.publisher_after,'topic_rates':a.topic_rates,
          'session_id':a.session_id,'session_start':a.session_start,'session_end':a.session_end,
          'birth_pose':a.birth,'birth_pose_frame':'base_footprint',
          'birth_chassis_configured':configured_chassis,
          'birth_axle_configured':configured_axle,
          'birth_chassis_measured':measured_birth,
          'tf_authorities':a.tf_authorities.split(','),
          'trajectory_file':a.trajectory,'trajectory_frame':a.trajectory_frame,
          'trajectory_sha256':trajectory_record['sha256'],'mapper_sha256':mapper_record['sha256'],
          'survey_log_sha256':log_record['sha256'],'artifacts':{
              'trajectory':trajectory_record,'mapper':mapper_record,'survey_log':log_record},
          'birth_chassis':a.birth_chassis,'birth_axle':a.birth_axle,
          'tf_evidence':a.tf_evidence,'tf_evidence_file':str(tf_path),
          'tf_evidence_sha256':sha(tf_path),
          'coverage_report_file':str(coverage_path),
          'coverage_report_sha256':sha(coverage_path),
          'coverage_report_gate_a_candidate':bool(coverage['gate_a_candidate']),
          'coverage_report_frontier_cells':int(coverage['frontier_cells']),
          'coverage_note':a.coverage_note}
    pathlib.Path(a.manifest).write_text(json.dumps(data,indent=2)+'\n',encoding='utf-8')
    print(json.dumps(data,indent=2))
if __name__=='__main__': main()
