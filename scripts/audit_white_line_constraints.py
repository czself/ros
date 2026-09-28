#!/usr/bin/env python3
"""Read-only audit of white-line maps, ordered photo route, and live costmaps.

The optional ROS mode uses Navfn's make_plan service only; it never dispatches a
MoveBase goal or publishes velocity.
"""
import argparse
import hashlib
import json
import math
from pathlib import Path
import sys

import cv2
import numpy as np
import yaml

GROUND_M = 4.2
WHITE_THRESHOLD = 220
START_LINE = ('y', -1.400, 1.700, 0.32, 0.055)
STOP_LINES = (
    ('x', -0.515, 0.00, 0.38, 0.055),
    ('y', -0.290, 1.70, 0.32, 0.055),
)
ZEBRA_BOXES = ((388, 1934, 6138, 6706), (7543, 8111, 4898, 6444))
AXLE_OFFSET_M = 0.0525
CHASSIS_FOOTPRINT = ((0.094, 0.085), (0.094, -0.085),
                     (-0.094, -0.085), (-0.094, 0.085))
PATH_SAMPLE_STEP_M = 0.02
PATH_YAW_STEP_RAD = math.radians(2.0)
CORRIDOR_TOLERANCE_M = 0.18


def sha256(path):
    h = hashlib.sha256()
    with open(path, 'rb') as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b''):
            h.update(block)
    return h.hexdigest()


def load_map_yaml(path):
    path = Path(path)
    info = yaml.safe_load(path.read_text(encoding='utf-8'))
    image = Path(info['image'])
    if not image.is_absolute():
        image = path.parent / image
    return info, image


def texture_box(line, size):
    axis, coordinate, lateral_center, lateral_half, half_thickness = line
    if axis == 'x':
        min_x, max_x = coordinate-half_thickness, coordinate+half_thickness
        min_y, max_y = lateral_center-lateral_half, lateral_center+lateral_half
    else:
        min_x, max_x = lateral_center-lateral_half, lateral_center+lateral_half
        min_y, max_y = coordinate-half_thickness, coordinate+half_thickness
    r0 = max(0, int(np.floor((.5-max_x/GROUND_M)*size)))
    r1 = min(size, int(np.ceil((.5-min_x/GROUND_M)*size)))
    c0 = max(0, int(np.floor((.5-max_y/GROUND_M)*size)))
    c1 = min(size, int(np.ceil((.5-min_y/GROUND_M)*size)))
    return r0, r1, c0, c1


def texture_masks(texture):
    if texture.ndim != 2 or texture.shape[0] != texture.shape[1]:
        raise ValueError('ground texture must be a square grayscale image')
    all_white = texture >= WHITE_THRESHOLD
    conditional = np.zeros_like(all_white)
    size = texture.shape[0]
    for line in (START_LINE,) + STOP_LINES:
        r0, r1, c0, c1 = texture_box(line, size)
        conditional[r0:r1, c0:c1] = True
    for r0, r1, c0, c1 in ZEBRA_BOXES:
        conditional[r0:r1+1, c0:c1+1] = True
    conditional_white = all_white & conditional
    return all_white, all_white & ~conditional, conditional_white


def texture_policy_masks(texture):
    """Return actual white pixels by policy bucket, not whole exception ROIs."""
    all_white = texture >= WHITE_THRESHOLD
    named_rois = {name: np.zeros_like(all_white)
                  for name in ('start_line', 'stop_line', 'zebra')}
    for line in (START_LINE,):
        r0, r1, c0, c1 = texture_box(line, texture.shape[0])
        named_rois['start_line'][r0:r1, c0:c1] = True
    for line in STOP_LINES:
        r0, r1, c0, c1 = texture_box(line, texture.shape[0])
        named_rois['stop_line'][r0:r1, c0:c1] = True
    for r0, r1, c0, c1 in ZEBRA_BOXES:
        named_rois['zebra'][r0:r1+1, c0:c1+1] = True
    named_white = {name: all_white & roi for name, roi in named_rois.items()}
    conditional = np.logical_or.reduce(list(named_white.values()))
    return all_white & ~conditional, named_white


def expected_paint_cells(texture, width, height, resolution, origin):
    ordinary = texture_masks(texture)[1]
    size = texture.shape[0]
    ox, oy = origin[:2]
    mask = np.zeros((height, width), dtype=bool)
    for row in range(height):
        y0 = oy + (height-row-1)*resolution
        y1 = y0 + resolution
        for col in range(width):
            x0 = ox + col*resolution
            x1 = x0 + resolution
            if (x1 < -GROUND_M/2 or x0 > GROUND_M/2 or
                    y1 < -GROUND_M/2 or y0 > GROUND_M/2):
                continue
            r0 = max(0, int(np.floor((.5-x1/GROUND_M)*size)))
            r1 = min(size, int(np.ceil((.5-x0/GROUND_M)*size)))
            c0 = max(0, int(np.floor((.5-y1/GROUND_M)*size)))
            c1 = min(size, int(np.ceil((.5-y0/GROUND_M)*size)))
            mask[row, col] = r0 < r1 and c0 < c1 and bool(ordinary[r0:r1, c0:c1].any())
    return mask


def static_map_audit(source_yaml, overlay_yaml, texture_path):
    source_info, source_path = load_map_yaml(source_yaml)
    overlay_info, overlay_path = load_map_yaml(overlay_yaml)
    source = cv2.imread(str(source_path), cv2.IMREAD_GRAYSCALE)
    overlay = cv2.imread(str(overlay_path), cv2.IMREAD_GRAYSCALE)
    texture = cv2.imread(str(texture_path), cv2.IMREAD_GRAYSCALE)
    if source is None or overlay is None or texture is None:
        raise RuntimeError('cannot read source map, overlay, or ground texture')
    if source.shape != overlay.shape:
        raise ValueError('source and overlay map dimensions differ')
    if source_info['resolution'] != overlay_info['resolution'] or source_info['origin'] != overlay_info['origin']:
        raise ValueError('source and overlay map metadata differ')
    height, width = source.shape
    expected = expected_paint_cells(texture, width, height,
                                    float(source_info['resolution']), source_info['origin'])
    added = (source != 0) & (overlay == 0)
    cleared_source_occupied = (source == 0) & (overlay != 0)
    missing = expected & (overlay != 0)
    extra = added & ~expected
    already = expected & (source == 0)
    overlay_gray = overlay.astype(np.float64) / 255.0
    overlay_occ = (1.0-overlay_gray) * 100.0 if not int(overlay_info.get('negate', 0)) else overlay_gray*100.0
    decoded_occupied = overlay_occ > float(overlay_info['occupied_thresh'])*100.0
    expected_cells_lethal = bool(np.all(decoded_occupied[expected]))
    return {
        'texture_path': str(texture_path), 'texture_sha256': sha256(texture_path),
        'source_map_path': str(source_path), 'source_sha256': sha256(source_path),
        'overlay_map_path': str(overlay_path), 'overlay_sha256': sha256(overlay_path),
        'texture_size': list(texture.shape), 'map_size': [width, height],
        'resolution_m': float(source_info['resolution']), 'origin': source_info['origin'],
        'exception_policy': {'start_line': START_LINE, 'stop_lines': STOP_LINES,
                             'zebra_boxes': ZEBRA_BOXES},
        'expected_ordinary_paint_cells': int(expected.sum()),
        'manifest_permanent_paint_cells': 1711,
        'newly_added_overlay_cells': int(added.sum()),
        'paint_cells_already_occupied_in_source': int(already.sum()),
        'missing_expected_paint_cells': int(missing.sum()),
        'extra_added_cells_not_from_texture': int(extra.sum()),
        'cleared_source_occupied_cells': int(cleared_source_occupied.sum()),
        'all_expected_paint_cells_decode_lethal': expected_cells_lethal,
        'pass': bool(not missing.any() and not extra.any() and
                     not cleared_source_occupied.any() and expected_cells_lethal),
    }


def expected_occupancy_grid(overlay_yaml):
    """Derive map_server OccupancyGrid cells from the authored PGM."""
    info, image_path = load_map_yaml(overlay_yaml)
    image = cv2.imread(str(image_path), cv2.IMREAD_GRAYSCALE)
    if image is None:
        raise RuntimeError('cannot read overlay PGM for live-grid comparison')
    gray = image.astype(np.float64) / 255.0
    occupancy = gray if int(info.get('negate', 0)) else 1.0-gray
    expected = np.full(image.shape, -1, dtype=np.int8)
    expected[occupancy < float(info['free_thresh'])] = 0
    expected[occupancy > float(info['occupied_thresh'])] = 100
    # OccupancyGrid row 0 is the map bottom; PGM row 0 is the image top.
    return expected[::-1, :].reshape(-1), info, image.shape


def route_segments(points):
    result, total = [], 0.0
    for a, b in zip(points, points[1:]):
        length = math.hypot(b[0]-a[0], b[1]-a[1])
        result.append((a, b, total, length))
        total += length
    return result, total


def nearest_route(point, segments):
    x, y = point
    best = (float('inf'), 0.0)
    for a, b, start_s, length in segments:
        if length <= 1e-12:
            continue
        dx, dy = b[0]-a[0], b[1]-a[1]
        t = max(0.0, min(1.0, ((x-a[0])*dx+(y-a[1])*dy)/(length*length)))
        px, py = a[0]+t*dx, a[1]+t*dy
        distance = math.hypot(x-px, y-py)
        if distance < best[0]:
            best = (distance, start_s+t*length)
    return best


def static_route_audit(corridor_path, photo_path):
    contract = yaml.safe_load(Path(corridor_path).read_text(encoding='utf-8'))
    photo = json.loads(Path(photo_path).read_text(encoding='utf-8'))
    vertices = [tuple(map(float, p)) for p in contract['points']]
    segments, total = route_segments(vertices)
    points = [('HOME', float(photo['home_pose']['x']), float(photo['home_pose']['y']))]
    points += [(p['name'], float(p['base_pose']['x']), float(p['base_pose']['y']))
               for p in photo['points']]
    points.append(('HOME', float(photo['home_pose']['x']), float(photo['home_pose']['y'])))
    projections = []
    for i, (name, x, y) in enumerate(points):
        distance, progress = nearest_route((x, y), segments)
        if i == 0:
            progress = 0.0
        elif i == len(points)-1:
            progress = total
        projections.append({'name': name, 'cross_track_m': round(distance, 4),
                            'route_progress_m': round(progress, 4)})
    values = [p['route_progress_m'] for p in projections]
    monotonic = all(b+1e-3 >= a for a, b in zip(values, values[1:]))
    max_cross = max(p['cross_track_m'] for p in projections)
    return {'corridor_path': str(corridor_path), 'photo_path': str(photo_path),
            'corridor_length_m': round(total, 4), 'points': projections,
            'ordered_projection': bool(monotonic), 'max_point_cross_track_m': max_cross,
            'point_corridor_tolerance_m': CORRIDOR_TOLERANCE_M,
            'points_within_corridor': bool(max_cross <= CORRIDOR_TOLERANCE_M)}


def live_audit(corridor_path, photo_path, texture_path, overlay_yaml):
    """Read live maps and call make_plan only; this function never sends goals."""
    import rospy
    import tf
    from actionlib_msgs.msg import GoalStatusArray
    from geometry_msgs.msg import PoseStamped, Twist
    from nav_msgs.msg import OccupancyGrid
    from nav_msgs.srv import GetPlan
    from tf.transformations import euler_from_quaternion, quaternion_from_euler

    helper_dir = str(Path(__file__).resolve().parent)
    if helper_dir not in sys.path:
        sys.path.insert(0, helper_dir)
    if '/root' not in sys.path:
        sys.path.insert(0, '/root')
    from navigation_goal_safety import GridFootprintChecker, DEFAULT_FOOTPRINT, parse_footprint

    texture = cv2.imread(str(texture_path), cv2.IMREAD_GRAYSCALE)
    if texture is None:
        raise RuntimeError('cannot read texture for live footprint audit')
    ordinary_mask, exception_masks = texture_policy_masks(texture)
    tex_h, tex_w = texture.shape
    footprint = np.asarray(CHASSIS_FOOTPRINT, dtype=np.float64)

    rospy.init_node('audit_white_line_constraints', anonymous=True, disable_signals=True)
    topic_msgs = {}
    for name, topic in (
        ('map', '/map'),
        ('global_costmap', '/move_base/global_costmap/costmap'),
        ('local_costmap', '/move_base/local_costmap/costmap'),
    ):
        topic_msgs[name] = rospy.wait_for_message(topic, OccupancyGrid, timeout=8.0)

    def grid_meta(m):
        q = m.info.origin.orientation
        origin_yaw = euler_from_quaternion((q.x, q.y, q.z, q.w))[2]
        return {
            'frame': m.header.frame_id,
            'width': int(m.info.width), 'height': int(m.info.height),
            'resolution_m': float(m.info.resolution),
            'origin_pose': [float(m.info.origin.position.x),
                            float(m.info.origin.position.y),
                            float(m.info.origin.position.z), float(origin_yaw)],
            'unknown_cells': int(np.count_nonzero(np.asarray(m.data) < 0)),
            'free_cells': int(np.count_nonzero(np.asarray(m.data) == 0)),
            'inflated_cells': int(np.count_nonzero((np.asarray(m.data) > 0) & (np.asarray(m.data) < 100))),
            'lethal_cells': int(np.count_nonzero(np.asarray(m.data) >= 100)),
        }

    map_msg = topic_msgs['map']
    expected_grid, expected_info, expected_shape = expected_occupancy_grid(overlay_yaml)
    live_grid = np.asarray(map_msg.data, dtype=np.int8)
    q = map_msg.info.origin.orientation
    map_yaw = euler_from_quaternion((q.x, q.y, q.z, q.w))[2]
    overlay_matches_live = bool(
        live_grid.size == expected_grid.size and
        map_msg.info.width == expected_shape[1] and
        map_msg.info.height == expected_shape[0] and
        abs(map_msg.info.resolution-float(expected_info['resolution'])) < 1e-8 and
        abs(map_msg.info.origin.position.x-float(expected_info['origin'][0])) < 1e-8 and
        abs(map_msg.info.origin.position.y-float(expected_info['origin'][1])) < 1e-8 and
        abs(map_msg.info.origin.position.z) < 1e-8 and
        abs(map_yaw-float(expected_info['origin'][2])) < 1e-8 and
        map_msg.header.frame_id == 'map' and np.array_equal(live_grid, expected_grid)
    )
    same_map = overlay_matches_live
    local_topic = rospy.get_param('/move_base/local_costmap/static_layer/map_topic', '')
    global_topic = rospy.get_param('/move_base/global_costmap/static_layer/map_topic', '')
    white_gate = bool(rospy.get_param('/cmd_vel_watchdog/enforce_white_lines', False))
    traffic_gate = bool(rospy.get_param('/cmd_vel_watchdog/enforce_traffic', False))
    local_unknown = bool(rospy.get_param('/move_base/local_costmap/track_unknown_space', False))
    global_unknown = bool(rospy.get_param('/move_base/global_costmap/track_unknown_space', False))

    def footprint_matches(namespace):
        try:
            configured = parse_footprint(rospy.get_param(namespace + '/footprint'))
        except (KeyError, ValueError, TypeError):
            return False
        return (len(configured) == len(DEFAULT_FOOTPRINT) and
                all(math.hypot(a[0]-b[0], a[1]-b[1]) <= 1e-4
                    for a, b in zip(configured, DEFAULT_FOOTPRINT)))

    global_footprint_ok = footprint_matches('/move_base/global_costmap')
    local_footprint_ok = footprint_matches('/move_base/local_costmap')

    def global_source_cells_lethal():
        grid = topic_msgs['global_costmap']
        if grid.header.frame_id != 'map':
            return False
        values = np.asarray(grid.data, dtype=np.int16).reshape(grid.info.height, grid.info.width)
        source_occ = expected_grid.reshape(expected_shape[::-1])
        rows, cols = np.where(source_occ == 100)
        res = float(expected_info['resolution'])
        ox, oy, _ = (float(v) for v in expected_info['origin'])
        oq = grid.info.origin.orientation
        gyaw = euler_from_quaternion((oq.x, oq.y, oq.z, oq.w))[2]
        c, s = math.cos(gyaw), math.sin(gyaw)
        for row, col in zip(rows, cols):
            x = ox+(col+0.5)*res
            y = oy+(row+0.5)*res
            dx, dy = x-grid.info.origin.position.x, y-grid.info.origin.position.y
            gx, gy = c*dx+s*dy, -s*dx+c*dy
            gc = int(math.floor(gx/grid.info.resolution))
            gr = int(math.floor(gy/grid.info.resolution))
            if gr < 0 or gr >= grid.info.height or gc < 0 or gc >= grid.info.width:
                return False
            if values[gr, gc] < 100:
                return False
        return True

    global_source_cells_preserved = global_source_cells_lethal()

    # Refuse to call the plan service while another goal is active or the
    # base is moving.  The service itself is read-only, but an audit taken
    # while motion is underway cannot certify the live route state.
    status = rospy.wait_for_message('/move_base/status', GoalStatusArray, timeout=3.0)
    active = [{'id': s.goal_id.id, 'status': s.status}
              for s in status.status_list if s.status in (0, 1, 6, 7)]
    try:
        cmd = rospy.wait_for_message('/my_car/cmd_vel', Twist, timeout=2.0)
        cmd_zero = abs(cmd.linear.x) < 1e-6 and abs(cmd.angular.z) < 1e-6
        cmd_values = [float(cmd.linear.x), float(cmd.angular.z)]
    except rospy.ROSException:
        cmd_zero, cmd_values = False, None
    if active or not cmd_zero:
        raise RuntimeError('refusing make_plan audit: active goal or nonzero drive command')

    listener = tf.TransformListener()
    listener.waitForTransform('odom', 'map', rospy.Time(0), rospy.Duration(4.0))
    trans, quat = listener.lookupTransform('odom', 'map', rospy.Time(0))
    map_to_odom_yaw = euler_from_quaternion(quat)[2]

    checkers = {
        'map': GridFootprintChecker.from_message(map_msg, DEFAULT_FOOTPRINT, safety_margin=0.01),
        'global_costmap': GridFootprintChecker.from_message(
            topic_msgs['global_costmap'], DEFAULT_FOOTPRINT, safety_margin=0.01),
        'local_costmap': GridFootprintChecker.from_message(
            topic_msgs['local_costmap'], DEFAULT_FOOTPRINT, safety_margin=0.01),
    }
    route = json.loads(Path(photo_path).read_text(encoding='utf-8'))
    points = [('HOME', float(route['home_pose']['x']), float(route['home_pose']['y']),
               float(route['home_pose']['yaw']))]
    points += [(p['name'], float(p['base_pose']['x']), float(p['base_pose']['y']),
                float(p['base_pose']['yaw'])) for p in route['points']]
    points.append(('HOME', float(route['home_pose']['x']), float(route['home_pose']['y']),
                   float(route['home_pose']['yaw'])))
    corridor = yaml.safe_load(Path(corridor_path).read_text(encoding='utf-8'))
    vertices = [tuple(map(float, p)) for p in corridor['points']]
    segments, total = route_segments(vertices)
    expected_s = [nearest_route((p[1], p[2]), segments)[1] for p in points]
    expected_s[0], expected_s[-1] = 0.0, total

    service = rospy.ServiceProxy('/move_base/NavfnROS/make_plan', GetPlan)

    def pose_msg(x, y, yaw):
        p = PoseStamped()
        p.header.frame_id = 'map'
        p.header.stamp = rospy.Time(0)
        p.pose.position.x, p.pose.position.y = x, y
        q = quaternion_from_euler(0.0, 0.0, yaw)
        p.pose.orientation.x, p.pose.orientation.y, p.pose.orientation.z, p.pose.orientation.w = q
        return p

    def yaw_of(q):
        return euler_from_quaternion((q.x, q.y, q.z, q.w))[2]

    def route_project(x, y):
        return nearest_route((x, y), segments)

    def texture_footprint_contacts(x, y, yaw):
        """Rasterize the full buffered chassis exactly like the watchdog."""
        chassis_x = x-AXLE_OFFSET_M*math.cos(yaw)
        chassis_y = y-AXLE_OFFSET_M*math.sin(yaw)
        c, s = math.cos(yaw), math.sin(yaw)
        world = np.empty_like(footprint)
        world[:, 0] = chassis_x + footprint[:, 0]*c - footprint[:, 1]*s
        world[:, 1] = chassis_y + footprint[:, 0]*s + footprint[:, 1]*c
        rows = (0.5-world[:, 0]/GROUND_M)*tex_h
        cols = (0.5-world[:, 1]/GROUND_M)*tex_w
        if (np.any(rows < 0) or np.any(rows >= tex_h) or
                np.any(cols < 0) or np.any(cols >= tex_w)):
            return 'OUT_OF_BOUNDS', None, []
        c0 = max(0, int(np.floor(cols.min()))-1)
        r0 = max(0, int(np.floor(rows.min()))-1)
        c1 = min(tex_w, int(np.ceil(cols.max()))+2)
        r1 = min(tex_h, int(np.ceil(rows.max()))+2)
        polygon = np.rint(np.column_stack((cols-c0, rows-r0))).astype(np.int32)
        mask = np.zeros((r1-r0, c1-c0), dtype=np.uint8)
        cv2.fillPoly(mask, [polygon], 1)
        mask = cv2.dilate(mask, np.ones((3, 3), dtype=np.uint8)) != 0
        ordinary = ordinary_mask[r0:r1, c0:c1]
        if np.any(mask & ordinary):
            rr, cc = np.argwhere(mask & ordinary)[0]
            return 'ORDINARY_PAINT_CONTACT', [int(r0+rr), int(c0+cc)], []
        contact_types = [name for name, image_mask in exception_masks.items()
                         if np.any(mask & image_mask[r0:r1, c0:c1])]
        if contact_types:
            return 'CONDITIONAL_MARKING_CONTACT', None, contact_types
        return None, None, []

    legs = []
    for leg_index, (start, finish) in enumerate(zip(points, points[1:])):
        response = service(pose_msg(*start[1:]), pose_msg(*finish[1:]), 0.0)
        path = response.plan.poses
        record = {'from': start[0], 'to': finish[0], 'pose_count': len(path)}
        if not path:
            record.update({'status': 'NO_GLOBAL_PLAN'})
            legs.append(record)
            continue
        samples_checked = 0
        length = 0.0
        failures = []
        conditional_contacts = 0
        conditional_types = {'start_line': 0, 'stop_line': 0, 'zebra': 0}
        max_cross = 0.0
        previous_progress = expected_s[leg_index]
        target_progress = expected_s[leg_index+1]
        if target_progress < previous_progress:
            target_progress += total
        previous_pose = (start[1], start[2], start[3])
        for stamped in path:
            p = stamped.pose.position
            yaw = yaw_of(stamped.pose.orientation)
            x0, y0, yaw0 = previous_pose
            distance = math.hypot(p.x-x0, p.y-y0)
            length += distance
            delta_yaw = math.atan2(math.sin(yaw-yaw0), math.cos(yaw-yaw0))
            count = max(1, int(math.ceil(distance/PATH_SAMPLE_STEP_M)),
                        int(math.ceil(abs(delta_yaw)/PATH_YAW_STEP_RAD)))
            samples = [(x0+(p.x-x0)*j/count,
                        y0+(p.y-y0)*j/count,
                        yaw0+delta_yaw*j/count) for j in range(1, count+1)]
            for x, y, yaw in samples:
                samples_checked += 1
                cross_track, progress = route_project(x, y)
                max_cross = max(max_cross, cross_track)
                candidates = [progress+k*total for k in (-1, 0, 1)]
                lo, hi = expected_s[leg_index]-0.20, target_progress+0.20
                eligible = [s for s in candidates if lo <= s <= hi]
                unwrapped = min(eligible, key=lambda s: abs(s-previous_progress)) if eligible else progress
                if cross_track > CORRIDOR_TOLERANCE_M:
                    failures.append({'check': 'route_corridor', 'cross_track_m': round(cross_track, 4),
                                     'pose_map': [round(x, 3), round(y, 3)]})
                if unwrapped < previous_progress-0.05:
                    failures.append({'check': 'route_order_backtrack', 'progress_m': round(unwrapped, 3)})
                previous_progress = max(previous_progress, unwrapped)

                odom_x = trans[0]+math.cos(map_to_odom_yaw)*x-math.sin(map_to_odom_yaw)*y
                odom_y = trans[1]+math.sin(map_to_odom_yaw)*x+math.cos(map_to_odom_yaw)*y
                odom_yaw = yaw+map_to_odom_yaw
                for name, xx, yy, aa in (('map', x, y, yaw),
                                         ('global_costmap', x, y, yaw),
                                         ('local_costmap', odom_x, odom_y, odom_yaw)):
                    result = checkers[name].check_pose(xx, yy, aa)
                    if not result.safe:
                        failures.append({'check': name, 'reason': result.reason,
                                         'cell': result.cell,
                                         'pose_map': [round(x, 3), round(y, 3)]})
                        break
                texture_reason, texture_pixel, contact_types = texture_footprint_contacts(x, y, yaw)
                if texture_reason == 'CONDITIONAL_MARKING_CONTACT':
                    # Traffic markings are gated separately at runtime; this
                    # mask contains no ordinary lane boundaries.
                    conditional_contacts += 1
                    for kind in contact_types:
                        conditional_types[kind] += 1
                elif texture_reason:
                    failures.append({'check': 'watchdog_texture', 'reason': texture_reason,
                                     'texture_pixel': texture_pixel,
                                     'pose_map': [round(x, 3), round(y, 3)]})
                    break
                if failures:
                    break
            if failures:
                break
            previous_pose = (p.x, p.y, yaw)
        record.update({'status': 'CLEAR' if not failures else 'BLOCKED',
                       'path_length_m': round(length, 3),
                       'samples_at_2cm_or_finer': samples_checked,
                       'max_centerline_cross_track_m': round(max_cross, 4),
                       'conditional_texture_contacts': conditional_contacts,
                       'conditional_texture_contacts_by_type': conditional_types,
                       'first_failure': failures[0] if failures else None})
        print('plan %s -> %s: %s, %.3fm, %d samples' % (
            start[0], finish[0], record['status'], length, samples_checked), flush=True)
        legs.append(record)

    end_status = rospy.wait_for_message('/move_base/status', GoalStatusArray, timeout=3.0)
    end_active = [{'id': s.goal_id.id, 'status': s.status}
                  for s in end_status.status_list if s.status in (0, 1, 6, 7)]
    try:
        end_cmd = rospy.wait_for_message('/my_car/cmd_vel', Twist, timeout=2.0)
        end_cmd_zero = abs(end_cmd.linear.x) < 1e-6 and abs(end_cmd.angular.z) < 1e-6
    except rospy.ROSException:
        end_cmd_zero = False
    plans_clear = bool(legs and all(x['status'] == 'CLEAR' for x in legs))
    live_ready = bool(
        overlay_matches_live and same_map and local_topic == '/map' and global_topic == '/map' and
        global_source_cells_preserved and global_footprint_ok and local_footprint_ok and
        white_gate and traffic_gate and
        local_unknown and global_unknown and not active and cmd_zero and
        not end_active and end_cmd_zero and plans_clear)
    return {
        'map_topic_metadata': {k: grid_meta(v) for k, v in topic_msgs.items()},
        'live_map_matches_overlay_pgm': overlay_matches_live,
        'source_maps_cell_equal': bool(same_map),
        'live_local_static_layer_topic': local_topic,
        'live_global_static_layer_topic': global_topic,
        'global_static_cells_preserved': global_source_cells_preserved,
        'global_footprint_matches_measured': global_footprint_ok,
        'local_footprint_matches_measured': local_footprint_ok,
        'live_local_track_unknown_space': local_unknown,
        'live_global_track_unknown_space': global_unknown,
        'white_line_gate': white_gate,
        'traffic_gate': traffic_gate,
        'traffic_acceptance_allowed': traffic_gate,
        'texture_policy_acceptance_allowed': bool(white_gate and traffic_gate),
        'map_to_odom_xy_yaw': [round(trans[0], 4), round(trans[1], 4), round(map_to_odom_yaw, 4)],
        'active_goals': active,
        'drive_command_zero': cmd_zero,
        'drive_command_linear_angular': cmd_values,
        'active_goals_after_audit': end_active,
        'drive_command_zero_after_audit': end_cmd_zero,
        'route_legs': legs,
        'all_plans_clear': plans_clear,
        'live_ready_for_full_task': live_ready,
    }


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--source-yaml', default='current_slam_preview_local.yaml')
    parser.add_argument('--overlay-yaml', default='maps/current_slam_preview_white_lines.yaml')
    parser.add_argument('--texture', default='models/competition_ground/materials/textures/map.png')
    parser.add_argument('--corridor', default='navigation/inner_route.yaml')
    parser.add_argument('--photo-route', default='navigation/standee_photo_route.json')
    parser.add_argument('--output', default='')
    parser.add_argument('--live', action='store_true',
                        help='read ROS costmaps and call only Navfn make_plan (no goals)')
    args = parser.parse_args()
    result = {'static_map': static_map_audit(args.source_yaml, args.overlay_yaml, args.texture),
              'route_geometry': static_route_audit(args.corridor, args.photo_route)}
    if args.live:
        result['live'] = live_audit(args.corridor, args.photo_route, args.texture,
                                    args.overlay_yaml)
    result['overall_static_pass'] = (result['static_map']['pass'] and
                                     result['route_geometry']['ordered_projection'] and
                                     result['route_geometry']['points_within_corridor'])
    result['audit_scope'] = 'live_full' if args.live else 'static_only'
    result['overall_pass'] = bool(args.live and result['overall_static_pass'] and
                                  result.get('live', {}).get('live_ready_for_full_task', False))
    data = json.dumps(result, ensure_ascii=False, indent=2)
    if args.output:
        Path(args.output).write_text(data+'\n', encoding='utf-8')
    print(data)
    raise SystemExit(0 if (result['overall_pass'] if args.live else
                           result['overall_static_pass']) else 1)


if __name__ == '__main__':
    main()
