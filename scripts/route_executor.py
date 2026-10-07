#!/usr/bin/env python3
"""Execute the fixed 4.2 m route from the competition diagram.

The route is read from one contract rather than duplicated as ad-hoc Python
coordinates.  Its segments must join end-to-end, so a planner can never make
an unapproved diagonal jump from one diagram arrow to another.
"""

import math
import json
import time
import threading
import os
from collections import OrderedDict

import actionlib
import cv2
import numpy as np
import rospy
import yaml
import tf
from actionlib_msgs.msg import GoalStatus
from cv_bridge import CvBridge
from geometry_msgs.msg import PoseStamped, Twist
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from nav_msgs.msg import OccupancyGrid, Odometry
from navigation_goal_safety import GridFootprintChecker, DEFAULT_FOOTPRINT
from sensor_msgs.msg import Image, CameraInfo
from std_msgs.msg import String
from tf.transformations import quaternion_from_euler
from person_reporting import (PersonCounter, annotate_people, calibrated_intrinsics,
                              collect_observations, save_report)
from hd_plate_capture import HDPlateCapture
from home_alignment import (HOME_REFINEMENT_TIMEOUT_S,
                            HOME_XY_TOLERANCE_M, HOME_YAW_TOLERANCE_RAD,
                            home_navigation_tolerances,
                            should_retry_home_alignment)


CONTRACT_PATH = '/root/navigation/inner_route.yaml'
BEST_PT_SHA256 = 'fe502091a4e964371eee3b08ec26029ad653019250d5406e13dc68ce8969a2ad'


def load_route(path):
    """Load and validate the diagram's ordered, connected lane segments."""
    with open(path, encoding='utf-8') as stream:
        contract = yaml.safe_load(stream)
    if isinstance(contract, dict) and isinstance(contract.get('points'), list):
        points=[tuple(float(v) for v in p) for p in contract['points']]
        if len(points)<2 or points[0] != points[-1]:
            raise ValueError('inner route must return to its start')
        route=[]
        # The contract vertices are the only route goals. Between them,
        # move_base owns continuous local control and live obstacle avoidance.
        # Dense interpolated goals force a full stop and re-alignment every
        # few centimetres without improving route adherence.
        for i, point in enumerate(points[1:], 1):
            if i + 1 < len(points):
                nxt=points[i+1]
                heading=math.atan2(nxt[1]-point[1],nxt[0]-point[0])
            else:
                heading=float(contract.get('birth_yaw', 1.606236))
            route.append(('INNER_%02d'%i,point[0],point[1],heading))
        return route
    segments = contract.get('route_order', []) if isinstance(contract, dict) else []
    if not segments:
        raise ValueError('route contract has no route_order')

    route = []
    previous_end = None
    for index, segment in enumerate(segments, 1):
        name = segment.get('id')
        start, end = segment.get('from'), segment.get('to')
        heading = segment.get('heading')
        if (not name or not isinstance(start, list) or not isinstance(end, list)
                or len(start) != 2 or len(end) != 2 or heading is None):
            raise ValueError('invalid route segment %d' % index)
        start = tuple(float(value) for value in start)
        end = tuple(float(value) for value in end)
        heading = float(heading)
        if previous_end is not None and math.hypot(
                start[0] - previous_end[0], start[1] - previous_end[1]) > 0.03:
            raise ValueError('%s does not connect to the preceding segment' % name)
        if math.hypot(end[0] - start[0], end[1] - start[1]) < 0.03:
            raise ValueError('%s has no usable length' % name)
        if previous_end is None:
            route.append(('START', start[0], start[1], heading))
        distance = math.hypot(end[0] - start[0], end[1] - start[1])
        steps = max(1, int(math.ceil(distance / 0.20)))
        for step in range(1, steps + 1):
            fraction = step / float(steps)
            suffix = '_%02d' % step if steps > 1 else ''
            route.append((name + suffix,
                          start[0] + (end[0] - start[0]) * fraction,
                          start[1] + (end[1] - start[1]) * fraction,
                          heading))
        previous_end = end
    return route


def load_photo_points(path):
    """Load teleop-recorded inspection poses in their saved order."""
    with open(path, encoding='utf-8') as stream:
        entries = yaml.safe_load(stream) if path.endswith(('.yaml', '.yml')) else json.load(stream)
    if isinstance(entries, dict):
        entries = entries.get('points', [])
    entries = sorted(entries, key=lambda entry: (
        0, int(entry['name'].rsplit('_', 1)[1]))
        if entry.get('name', '').rsplit('_', 1)[-1].isdigit()
        else (1, entry.get('name', '')))
    route = []
    for entry in entries:
        pose = entry.get('base_pose', entry)
        name = entry.get('name')
        if not name or not all(key in pose for key in ('x', 'y', 'yaw')):
            raise ValueError('invalid photo point: %s' % entry)
        route.append((name, float(pose['x']), float(pose['y']), float(pose['yaw'])))
    if not route:
        raise ValueError('photo point file is empty')
    expected = ['POINT_%d' % index for index in range(1, 11)]
    names = [item[0] for item in route]
    if names != expected:
        raise ValueError('photo route must contain exactly POINT_1 through POINT_10 in order; got %s' % names)
    return route


def route_segments(points):
    segments = []
    progress = 0.0
    for a, b in zip(points, points[1:]):
        length = math.hypot(b[0]-a[0], b[1]-a[1])
        segments.append((a, b, progress, length))
        progress += length
    return segments, progress


def nearest_route_progress(point, segments):
    x, y = point
    best_distance, best_progress = float('inf'), 0.0
    for a, b, start_s, length in segments:
        if length <= 1e-9:
            continue
        dx, dy = b[0]-a[0], b[1]-a[1]
        t = max(0.0, min(1.0, ((x-a[0])*dx+(y-a[1])*dy)/(length*length)))
        px, py = a[0]+t*dx, a[1]+t*dy
        distance = math.hypot(x-px, y-py)
        if distance < best_distance:
            best_distance, best_progress = distance, start_s+t*length
    return best_distance, best_progress


def nearest_route_progress_in_interval(point, segments, route_length, low, high):
    """Project onto the loop only inside the active forward route interval."""
    x, y = point
    best_distance, best_progress = float('inf'), None
    for a, b, start_s, length in segments:
        if length <= 1e-9:
            continue
        dx, dy = b[0]-a[0], b[1]-a[1]
        t = max(0.0, min(1.0, ((x-a[0])*dx+(y-a[1])*dy)/(length*length)))
        px, py = a[0]+t*dx, a[1]+t*dy
        distance = math.hypot(x-px, y-py)
        base = start_s + t*length
        for lap in (-1, 0, 1, 2):
            progress = base + lap*route_length
            if low-1e-6 <= progress <= high+1e-6 and distance < best_distance:
                best_distance, best_progress = distance, progress
    return best_distance, best_progress


class RouteExecutor:
    def __init__(self):
        self.timeout = float(rospy.get_param('~waypoint_timeout', 75.0))
        contract = rospy.get_param('~route_contract', CONTRACT_PATH)
        photo_file = rospy.get_param('~photo_points_file', '')
        self.photo_route = bool(photo_file)
        self.route = load_photo_points(photo_file) if photo_file else load_route(contract)
        with open(contract, encoding='utf-8') as stream:
            raw_contract = yaml.safe_load(stream)
        # Parking always uses the designated birth slot from the route
        # contract, even when the active route is a separate photo-point list.
        self.birth_x, self.birth_y = (float(v) for v in raw_contract['points'][0])
        self.birth_yaw = float(raw_contract.get('birth_yaw', 1.606236))
        if photo_file:
            with open(photo_file) as stream:
                photo_config = json.load(stream)
            home = photo_config.get('home_pose', {})
            self.birth_x = float(home.get('x', self.birth_x))
            self.birth_y = float(home.get('y', self.birth_y))
            self.birth_yaw = float(home.get('yaw', self.birth_yaw))
            self.photo_acceptance = photo_config.get('photo_acceptance_criteria', {})
        else:
            self.photo_acceptance = {}
        self.goal_events = []
        self.latest_gate_status = 'CLEAR'
        self.gate_wall_time = 0.0
        self.static_map = None
        self.costmap = None
        self.local_costmap = None
        rospy.Subscriber('/map', OccupancyGrid,
                         lambda message: setattr(self, 'static_map', message), queue_size=1)
        rospy.Subscriber('/move_base/global_costmap/costmap', OccupancyGrid,
                         lambda message: setattr(self, 'costmap', message), queue_size=1)
        rospy.Subscriber('/move_base/local_costmap/costmap', OccupancyGrid,
                         lambda message: setattr(self, 'local_costmap', message), queue_size=1)
        rospy.Subscriber('/traffic_light/gate_status', String,
                         self._gate_cb, queue_size=1)
        self.route_vertices = bool(raw_contract.get('points')) and not photo_file
        self.corridor_vertices = [tuple(float(v) for v in point)
                                  for point in raw_contract.get('points', [])]
        self.corridor_segments, self.corridor_length = route_segments(self.corridor_vertices)
        self.route_cursor = 0.0
        self.planned_goal_progress = None
        self.status_pub = rospy.Publisher('/route/status', String, queue_size=1, latch=True)
        self.motion_progress_pub = rospy.Publisher('/route/progress', String, queue_size=10)
        self.client = actionlib.SimpleActionClient('/move_base', MoveBaseAction)
        self.cmd_pub = rospy.Publisher('/my_car/cmd_vel_nav', Twist, queue_size=1)
        self.capture_photos = bool(rospy.get_param('~capture_photos', False))
        self.strict_acceptance = bool(rospy.get_param('~strict_acceptance', False))
        self.no_progress_timeout = float(rospy.get_param('~no_progress_timeout', 8.0))
        # /root/ros1_ws is bind-mounted to the host as /home/sz/ros1_ws.
        self.photo_dir = rospy.get_param('~photo_dir', '/root/ros1_ws/photo_stops')
        self.person_counter = None
        if self.photo_route and self.capture_photos:
            with open(rospy.get_param('~person_config', '/root/navigation/person_reporting.json')) as stream:
                self.person_config = json.load(stream)
            self.person_counter = PersonCounter(self.person_config)
            self.person_report_pub = rospy.Publisher('/inspection/person_report', String,
                                                      queue_size=1, latch=True)
            self.person_image_pub = rospy.Publisher('/inspection/person_image', Image,
                                                    queue_size=1, latch=True)
        selected = rospy.get_param('~photo_waypoints', '')
        self.photo_waypoints = set(filter(None, (name.strip() for name in selected.split(','))))
        self.bridge = CvBridge()
        self.tf_listener = tf.TransformListener()
        self.hd_capture = (HDPlateCapture(self.bridge,self.tf_listener)
                           if self.capture_photos and rospy.get_param('~capture_hd_plates',True) else None)
        self.ocr_enabled = bool(rospy.get_param('~ocr_enabled',False))
        self.ocr_results = []
        self.latest_odom = None
        rospy.Subscriber('/odom', Odometry,
                         lambda message: setattr(self, 'latest_odom', message), queue_size=1)
        rospy.on_shutdown(self.stop_motion)
        self.latest_image = None
        self.latest_image_stamp = None
        self.raw_frames = OrderedDict()
        self.annotated_frames = OrderedDict()
        self.detection_records = OrderedDict()
        self.depth_frames = OrderedDict()
        self.photo_cache_lock = threading.Lock()
        self.accepted_photo_records = []
        self.latest_depth = None
        self.depth_info = None
        if self.capture_photos:
            rospy.Subscriber('/camera/image_raw', Image, self._image_cb, queue_size=1)
            rospy.Subscriber('/inspection/image', Image, self._annotation_cb, queue_size=1)
            rospy.Subscriber('/inspection/detections', String, self._detections_cb, queue_size=2)
            rospy.Subscriber('/camera/depth/image_raw', Image, self._depth_cb, queue_size=1)
            rospy.Subscriber('/camera/depth/camera_info', CameraInfo,
                             lambda message: setattr(self, 'depth_info', message), queue_size=1)
            rospy.Subscriber('/camera/camera_info', CameraInfo,
                             lambda message: setattr(self, 'depth_info', message), queue_size=1)

    def stop_motion(self):
        self.client.cancel_all_goals()
        self.cmd_pub.publish(Twist())

    def _image_cb(self, message):
        try:
            self.latest_image = self.bridge.imgmsg_to_cv2(message, 'bgr8')
            self.latest_image_stamp = message.header.stamp
            with self.photo_cache_lock:
                self._cache_frame(self.raw_frames, self._stamp_key(message.header.stamp),
                                  self.latest_image)
        except Exception as error:
            rospy.logwarn_throttle(5, 'photo capture image conversion failed: %s', error)

    @staticmethod
    def _stamp_key(stamp):
        return int(stamp.secs), int(stamp.nsecs)

    @staticmethod
    def _cache_frame(cache, key, value, limit=32):
        cache[key] = value
        cache.move_to_end(key)
        while len(cache) > limit:
            cache.popitem(last=False)

    def _annotation_cb(self, message):
        try:
            frame = self.bridge.imgmsg_to_cv2(message, 'bgr8')
            with self.photo_cache_lock:
                self._cache_frame(self.annotated_frames,
                                  self._stamp_key(message.header.stamp), frame)
        except Exception as error:
            rospy.logwarn_throttle(5, 'annotated image conversion failed: %s', error)

    def _detections_cb(self, message):
        try:
            record = json.loads(message.data)
            stamp = record['source_stamp']
            key = (int(stamp['secs']), int(stamp['nsecs']))
            with self.photo_cache_lock:
                self._cache_frame(self.detection_records, key, record)
        except (ValueError, TypeError, KeyError) as error:
            rospy.logwarn_throttle(5, 'YOLO detection record invalid: %s', error)

    def contextual_plate_boxes(self, name, record, annotated_frame):
        """Preserve a visible plate-shaped ROI when YOLO assigns a wrong class.

        The P10 reference and live frames show a single elongated plate ROI that
        this checkpoint labels green_off. At known plate views only, retain the
        raw class and add a marked contextual alias; no OCR is performed.
        """
        if name not in ('POINT_8', 'POINT_9', 'POINT_10'):
            return record, annotated_frame
        detections = list(record.get('detections', []))
        if any(item.get('class') == 'license_plate' and
               float(item.get('confidence', 0.0)) >= 0.15 for item in detections):
            return record, annotated_frame
        width, height = int(record['width']), int(record['height'])
        candidates = []
        for item in detections:
            label = item.get('class')
            if label not in ('red_on', 'red_off', 'yellow_on', 'yellow_off',
                             'green_on', 'green_off'):
                continue
            if float(item.get('confidence', 0.0)) < 0.15:
                continue
            x1, y1, x2, y2 = map(float, item['box'])
            box_width, box_height = x2 - x1, y2 - y1
            if (box_width < 40 or box_height < 14 or
                    box_width / max(1.0, box_height) < 1.8 or
                    x1 <= 0 or y1 <= 0 or x2 >= width or y2 >= height):
                continue
            candidates.append(item)
        if not candidates:
            return record, annotated_frame
        # If two class heads cover the same rectangle, retain the stronger one;
        # distinct rectangular candidates can represent multiple visible cars.
        selected = []
        for item in sorted(candidates,
                           key=lambda value: float(value.get('confidence', 0.0)),
                           reverse=True):
            x1, y1, x2, y2 = map(float, item['box'])
            area = max(0.0, x2-x1) * max(0.0, y2-y1)
            duplicate = False
            for prior in selected:
                px1, py1, px2, py2 = map(float, prior['box'])
                intersection = (max(0.0, min(x2, px2)-max(x1, px1)) *
                                max(0.0, min(y2, py2)-max(y1, py1)))
                prior_area = max(0.0, px2-px1) * max(0.0, py2-py1)
                if area + prior_area - intersection > 0 and \
                        intersection / (area + prior_area - intersection) >= 0.70:
                    duplicate = True
                    break
            if not duplicate:
                selected.append(item)
        record = dict(record)
        record['detections'] = list(detections)
        annotated_frame = annotated_frame.copy()
        for item in selected:
            alias = dict(item)
            alias['source_class'] = item['class']
            alias['label_source'] = 'contextual_plate_rectangle'
            alias['class'] = 'license_plate'
            record['detections'].append(alias)
            x1, y1, x2, y2 = map(int, item['box'])
            cv2.rectangle(annotated_frame, (x1, y1), (x2, y2), (255, 0, 255), 2)
            cv2.putText(annotated_frame, 'plate ROI %.2f*' % item['confidence'],
                        (x1, max(18, y1-6)), cv2.FONT_HERSHEY_SIMPLEX,
                        0.48, (255, 0, 255), 2, cv2.LINE_AA)
        rospy.loginfo('photo waypoint %s retained %d contextual plate ROI(s)',
                      name, len(selected))
        return record, annotated_frame

    @staticmethod
    def _box_clipped_at_frame(item, frame, width, height):
        x1, y1, x2, y2 = [float(v) for v in item['box']]
        if x1 <= 0.0 or y1 <= 0.0 or x2 >= width:
            return True
        if y2 >= height - 1:
            if frame is None or item.get('class') not in ('resident', 'stranger'):
                return True
            left = max(0, min(width, int(math.floor(x1))))
            right = max(0, min(width, int(math.ceil(x2))))
            top = max(0, min(height, int(math.floor(y1))))
            roi = frame[top:height, left:right]
            if roi.size == 0:
                return True
            visible_rows = np.where(np.max(roi, axis=2) > 8)[0]
            if (visible_rows.size == 0 or
                    height - 1 - (top + int(visible_rows.max())) < 4):
                return True
        return False

    def _validate_photo_detections(self, name, record, frame=None):
        if record is None:
            return False, 'NO_MATCHED_YOLO_RECORD'
        minimum_confidence = 0.15 if name in ('POINT_8', 'POINT_9', 'POINT_10') else 0.25
        detections = [item for item in record.get('detections', [])
                      if float(item.get('confidence', 0.0)) >= minimum_confidence]
        width, height = int(record['width']), int(record['height'])
        expected_people = {'POINT_2': 3, 'POINT_3': 3, 'POINT_4': 3,
                           'POINT_5': 4, 'POINT_6': 5}
        if name in expected_people:
            people = [item for item in detections
                      if item.get('class') in ('resident', 'stranger') and
                      not self._box_clipped_at_frame(item, frame, width, height)]
            if len(people) != expected_people[name]:
                return False, 'PERSON_COUNT:%d_EXPECTED_%d' % (len(people), expected_people[name])
            required = people
        elif name in ('POINT_1', 'POINT_7'):
            lamps = [item for item in detections
                     if item.get('class') in ('red_on', 'red_off', 'yellow_on',
                                              'yellow_off', 'green_on', 'green_off')]
            colors = {item['class'].split('_', 1)[0] for item in lamps}
            if colors != {'red', 'yellow', 'green'}:
                return False, 'INCOMPLETE_SIGNAL_LAMPS'
            required = lamps
        elif name in ('POINT_8', 'POINT_9', 'POINT_10'):
            plates = [item for item in detections
                      if item.get('class') == 'license_plate' and
                      not self._box_clipped_at_frame(item, frame, width, height)]
            if not plates:
                return False, 'NO_LICENSE_PLATE_BOX'
            required = plates
        else:
            required = detections
        for item in required:
            if self._box_clipped_at_frame(item, frame, width, height):
                return False, 'OBJECT_CLIPPED_AT_FRAME_EDGE:%s' % item.get('class')
        return True, 'PASS'

    def _depth_cb(self, message):
        try:
            depth = self.bridge.imgmsg_to_cv2(message, 'passthrough')
            depth = np.asarray(depth, dtype=np.float32)
            self.latest_depth = depth
            with self.photo_cache_lock:
                self._cache_frame(self.depth_frames,
                                  self._stamp_key(message.header.stamp), depth)
        except Exception as error:
            rospy.logwarn_throttle(5, 'depth capture conversion failed: %s', error)

    def _gate_cb(self, message):
        self.latest_gate_status = message.data
        self.gate_wall_time = time.monotonic()

    def depth_geometry(self, depth=None):
        """Return robust depth and horizontal coverage diagnostics in metres."""
        depth = self.latest_depth if depth is None else depth
        if depth is None:
            return None
        h, w = depth.shape[:2]
        crop = depth[int(h * .12):int(h * .90), int(w * .03):int(w * .97)]
        valid = crop[np.isfinite(crop) & (crop > .15) & (crop < 20.0)]
        if valid.size < 100:
            return None
        result = {'min_m': float(np.percentile(valid, 5)),
                  'median_m': float(np.percentile(valid, 50)),
                  'max_m': float(np.percentile(valid, 95))}
        if self.depth_info and self.depth_info.K[0] > 0:
            fx, cx = self.depth_info.K[0], self.depth_info.K[2]
            yy, xx = np.where(np.isfinite(depth) & (depth > .15) & (depth < 20.0))
            zz = depth[yy, xx]
            xx_m = (xx - cx) * zz / fx
            result['span_m'] = float(np.percentile(xx_m, 95) - np.percentile(xx_m, 5))
            result['required_distance_m'] = float(result['span_m'] / (2.0 * math.tan(0.785398)))
        return result

    def depth_target_metrics(self, name, record, depth):
        """Measure target range/coverage from the depth frame nearest the RGB stamp."""
        if depth is None or depth.ndim != 2:
            return False, 'DEPTH_FRAME_UNAVAILABLE', []
        height, width = depth.shape
        rgb_width, rgb_height = int(record['width']), int(record['height'])
        if rgb_width <= 0 or rgb_height <= 0:
            return False, 'RGB_DIMENSIONS_INVALID', []
        scale_x, scale_y = width / float(rgb_width), height / float(rgb_height)
        minimum_confidence = 0.15 if name in ('POINT_8', 'POINT_9', 'POINT_10') else 0.25
        detections = [item for item in record.get('detections', [])
                      if float(item.get('confidence', 0.0)) >= minimum_confidence]
        if name in ('POINT_2', 'POINT_3', 'POINT_4', 'POINT_5', 'POINT_6'):
            detections = [item for item in detections
                          if item.get('class') in ('resident', 'stranger')]
        elif name in ('POINT_1', 'POINT_7'):
            detections = [item for item in detections
                          if item.get('class') in ('red_on', 'red_off', 'yellow_on',
                                                   'yellow_off', 'green_on', 'green_off')]
        elif name in ('POINT_8', 'POINT_9', 'POINT_10'):
            detections = [item for item in detections
                          if item.get('class') == 'license_plate']
        if not detections:
            return False, 'NO_DEPTH_TARGETS', []
        measured = []
        for item in detections:
            x1, y1, x2, y2 = (float(v) for v in item['box'])
            # Rendered standee cutouts have sparse valid depth returns inside
            # YOLO boxes. Use the complete box and require a meaningful number
            # of valid samples instead of discarding its sparse visible edges.
            left = max(0, int(round(x1 * scale_x)))
            right = min(width, int(round(x2 * scale_x)))
            top = max(0, int(round(y1 * scale_y)))
            bottom = min(height, int(round(y2 * scale_y)))
            if right <= left or bottom <= top:
                return False, 'DEPTH_ROI_EMPTY:%s' % item.get('class'), measured
            roi = depth[top:bottom, left:right]
            valid = roi[np.isfinite(roi) & (roi > 0.20) & (roi < 5.0)]
            coverage = float(valid.size) / float(max(1, roi.size))
            metric = {
                'class': item.get('class'),
                'confidence': float(item.get('confidence', 0.0)),
                'valid_depth_pixels': int(valid.size),
                'valid_roi_fraction': coverage,
            }
            if valid.size:
                p10, median, p90 = np.percentile(valid, (10, 50, 90))
                metric.update({'median_depth_m': float(median),
                               'p10_depth_m': float(p10),
                               'p90_depth_m': float(p90)})
            measured.append(metric)
            if coverage < 0.05 or valid.size < 100:
                return False, 'DEPTH_COVERAGE_LOW:%s:%.3f' % (
                    item.get('class'), coverage), measured
            median = metric['median_depth_m']
            if median > 4.0:
                return False, 'PHOTO_TARGET_TOO_FAR:%s:%.2f' % (
                    item.get('class'), median), measured
        return True, 'PASS', measured

    def capture_photo(self, name, x, y, yaw):
        use_hd = self.hd_capture is not None and name in ('POINT_8','POINT_9','POINT_10')
        if use_hd:
            self.hd_capture.start()
        try:
            return self._capture_photo(name,x,y,yaw)
        finally:
            if use_hd:
                self.hd_capture.stop()

    def _capture_photo(self, name, x, y, yaw):
        if name.startswith('_TRANSITION_'):
            return True
        if not self.capture_photos or (self.photo_waypoints and name not in self.photo_waypoints):
            return True
        # Photo routes send the recorded camera framing yaw as the move_base
        # goal yaw.  Never publish a best-effort correction after success:
        # the watchdog may stop it at a painted line, leaving a wrong view.
        settle_seconds = max(0.15, float(
            rospy.get_param('~photo_settle_seconds', 0.3)))
        settle_started = time.monotonic()
        xy_tolerance = float(rospy.get_param('~photo_position_tolerance', 0.06))
        yaw_tolerance = float(rospy.get_param('~photo_heading_tolerance', 0.06))
        if name == 'POINT_3':
            xy_tolerance = max(
                xy_tolerance,
                float(rospy.get_param('~point_3_photo_position_tolerance', 0.05)))
        if name == 'POINT_7':
            xy_tolerance = max(
                xy_tolerance,
                float(rospy.get_param('~point_7_photo_position_tolerance', 0.05)))
        if name == 'POINT_10':
            xy_tolerance = max(
                xy_tolerance,
                float(rospy.get_param('~point_10_photo_position_tolerance', 0.05)))
        if name == 'POINT_5':
            xy_tolerance = min(
                xy_tolerance,
                float(rospy.get_param('~point_5_photo_position_tolerance', 0.025)))
            yaw_tolerance = min(
                yaw_tolerance,
                float(rospy.get_param('~point_5_photo_heading_tolerance', 0.025)))
        # Confirm velocity and pose together for one continuous interval.
        # Preserve both original timeout budgets and all acceptance thresholds.
        pose_deadline = (settle_started + max(2.0, settle_seconds * 4.0) +
                         max(2.0, settle_seconds * 8.0))
        pose_still_since = None
        previous_pose = None
        pose_errors = (float('inf'), float('inf'))
        pose_error = None
        actual_x = actual_y = actual_yaw = 0.0
        while not rospy.is_shutdown() and time.monotonic() < pose_deadline:
            self.cmd_pub.publish(Twist())
            odom = self.latest_odom
            motion_stable = False
            if odom is not None:
                odom_age = (rospy.Time.now() - odom.header.stamp).to_sec()
                motion_stable = (0.0 <= odom_age <= 0.5 and
                                 abs(odom.twist.twist.linear.x) < 0.01 and
                                 abs(odom.twist.twist.angular.z) < 0.01)
            try:
                stamp = self.tf_listener.getLatestCommonTime('map', 'base_footprint')
                (actual_x, actual_y, _), actual_q = self.tf_listener.lookupTransform(
                    'map', 'base_footprint', stamp)
                actual_yaw = math.atan2(
                    2.0 * (actual_q[3] * actual_q[2] + actual_q[0] * actual_q[1]),
                    1.0 - 2.0 * (actual_q[1] ** 2 + actual_q[2] ** 2))
                xy_error = math.hypot(actual_x - x, actual_y - y)
                yaw_error = math.atan2(math.sin(yaw - actual_yaw),
                                       math.cos(yaw - actual_yaw))
                pose_errors = (xy_error, yaw_error)
                now = time.monotonic()
                sample = (actual_x, actual_y, actual_yaw)
                pose_matches = (xy_error <= xy_tolerance and
                                abs(yaw_error) <= yaw_tolerance)
                pose_stable = (previous_pose is not None and
                               math.hypot(actual_x - previous_pose[0],
                                          actual_y - previous_pose[1]) <= 0.01 and
                               abs(self._angle_error(actual_yaw,
                                                     previous_pose[2])) <= 0.015)
                if motion_stable and pose_matches and pose_stable:
                    if pose_still_since is None:
                        pose_still_since = now
                    elif now - pose_still_since >= settle_seconds:
                        break
                else:
                    pose_still_since = None
                previous_pose = sample
                pose_error = None
            except (tf.Exception, tf.LookupException, tf.ConnectivityException) as error:
                pose_error = error
                pose_still_since = None
                previous_pose = None
            time.sleep(0.05)
        else:
            rospy.logwarn(
                'photo waypoint %s pose failed settle/tolerance: xy_error=%.3f m yaw_error=%.3f rad detail=%s',
                name, pose_errors[0], pose_errors[1], pose_error)
            return False
        settle_wait_s = time.monotonic() - settle_started
        rospy.loginfo('photo waypoint %s velocity/pose settled in %.3f s',
                      name, settle_wait_s)
        if self.latest_image is None:
            rospy.logwarn('photo waypoint %s reached but no camera frame is available', name)
            return False
        capture_start = rospy.Time.now().to_sec()
        for _ in range(max(1, int(rospy.get_param('~photo_burst_count', 8)))):
            rospy.sleep(float(rospy.get_param('~photo_burst_interval', 0.2)))
        candidates = []
        with self.photo_cache_lock:
            raw_items = list(self.raw_frames.items())
            detection_records = dict(self.detection_records)
            annotated_frames = dict(self.annotated_frames)
            depth_frames = dict(self.depth_frames)
        for key, raw in raw_items:
            record = detection_records.get(key)
            annotated = annotated_frames.get(key)
            if record is None or annotated is None or not depth_frames:
                continue
            source_seconds = float(record['source_stamp']['seconds'])
            if source_seconds + 1e-6 < capture_start:
                continue
            if record.get('checkpoint_sha256') != BEST_PT_SHA256:
                continue
            if float(record.get('frame_age_ms', float('inf'))) > 500.0:
                continue
            if float(record.get('latency_ms', float('inf'))) > 250.0:
                continue
            depth_key = min(depth_frames, key=lambda candidate:
                            abs((candidate[0] - key[0]) +
                                (candidate[1] - key[1]) * 1e-9))
            depth_delta = abs((depth_key[0] - key[0]) +
                              (depth_key[1] - key[1]) * 1e-9)
            if depth_delta > 0.10:
                continue
            candidates.append((key, raw, annotated, record, depth_frames[depth_key],
                               depth_key, depth_delta,
                               cv2.Laplacian(raw, cv2.CV_64F).var()))
        if not candidates:
            rospy.logwarn('photo waypoint %s has no matched RGB/YOLO/depth frame', name)
            return False
        # A transition frame can be sharp while one lamp is not detected.
        # Select among frames meeting the existing photo/depth requirements
        # before comparing sharpness; keep the best failed frame for diagnosis.
        valid_candidates = []
        for candidate in candidates:
            record, annotation = self.contextual_plate_boxes(name,candidate[3],candidate[2].copy())
            photo_valid, _ = self._validate_photo_detections(name,record,candidate[1])
            depth_valid, _, _ = self.depth_target_metrics(name,record,candidate[4])
            if photo_valid and depth_valid:
                valid_candidates.append(candidate)
        selected = max(valid_candidates or candidates, key=lambda item: item[7])
        (frame_key, raw_frame, annotated_frame, detection_record, depth_frame,
         depth_key, depth_delta, sharpness) = selected
        detection_record, annotated_frame = self.contextual_plate_boxes(
            name, detection_record, annotated_frame)
        photo_ok, photo_reason = self._validate_photo_detections(
            name, detection_record, raw_frame)
        depth_ok, depth_reason, target_depths = self.depth_target_metrics(
            name, detection_record, depth_frame)
        counted_detections = detection_record.get('detections', [])
        edge_clipped_person_boxes = 0
        if name in ('POINT_2', 'POINT_3', 'POINT_4', 'POINT_5', 'POINT_6'):
            frame_width, frame_height = (int(detection_record['width']),
                                         int(detection_record['height']))
            person_detections = [item for item in counted_detections
                                 if item.get('class') in ('resident', 'stranger')]
            counted_detections = [item for item in counted_detections
                                  if item.get('class') not in ('resident', 'stranger') or
                                  (float(item.get('confidence', 0.0)) >= 0.25 and
                                   not self._box_clipped_at_frame(
                                       item, raw_frame, frame_width, frame_height))]
            edge_clipped_person_boxes = len(person_detections) - sum(
                1 for item in counted_detections
                if item.get('class') in ('resident', 'stranger'))
        self.accepted_photo_records.append({
            'waypoint': name, 'source_stamp': detection_record['source_stamp'],
            'settle_wait_s': round(settle_wait_s, 4),
            'detector_counts': {label: sum(1 for item in counted_detections
                                            if item.get('class') == label)
                                for label in ('resident', 'stranger', 'license_plate',
                                              'red_on', 'red_off', 'yellow_on', 'yellow_off',
                                              'green_on', 'green_off')},
            'edge_clipped_person_boxes_excluded': edge_clipped_person_boxes,
            'validation': photo_reason, 'passed': photo_ok and depth_ok,
            'frame_age_ms': detection_record.get('frame_age_ms'),
            'inference_latency_ms': detection_record.get('latency_ms'),
            'depth_validation': depth_reason,
            'depth_target_count': sum(
                1 for metric in target_depths if 'median_depth_m' in metric),
        })
        import os
        stamp = time.strftime('%Y%m%d_%H%M%S')
        point_dir = os.path.join(self.photo_dir, name)
        stem = os.path.join(point_dir, '%s_%s' % (name, stamp))
        failure_reason = None
        if not photo_ok:
            failure_reason = 'YOLO/photo margin check: %s' % photo_reason
        elif not depth_ok:
            failure_reason = 'depth/range check: %s' % depth_reason
        if failure_reason:
            os.makedirs(point_dir, exist_ok=True)
            cv2.imwrite(stem + '.raw.png', raw_frame)
            cv2.imwrite(stem + '.png', annotated_frame)
            with open(stem + '.detections.json', 'w', encoding='utf-8') as stream:
                json.dump(detection_record, stream, ensure_ascii=False, indent=2)
            np.save(stem + '.depth.npy', depth_frame)
            with open(stem + '.failure.json', 'w', encoding='utf-8') as stream:
                json.dump({'waypoint': name, 'source_stamp': detection_record['source_stamp'],
                           'depth_source_stamp': {'secs': int(depth_key[0]),
                                                  'nsecs': int(depth_key[1])},
                           'rgb_depth_stamp_delta_s': depth_delta,
                           'photo_validation': photo_reason,
                           'depth_validation': depth_reason,
                           'depth_target_metrics': target_depths,
                           'raw_image': stem + '.raw.png',
                           'annotated_image': stem + '.png',
                           'depth_frame': stem + '.depth.npy'}, stream, indent=2)
            rospy.logerr('photo waypoint %s failed %s', name, failure_reason)
            return False
        os.makedirs(point_dir, exist_ok=True)
        if not cv2.imwrite(stem + '.raw.png', raw_frame):
            rospy.logwarn('photo waypoint %s could not write image %s', name, stem + '.png')
            return False
        if not cv2.imwrite(stem + '.png', annotated_frame):
            rospy.logwarn('photo waypoint %s could not write annotated image %s', name, stem + '.png')
            return False
        with open(stem + '.detections.json', 'w', encoding='utf-8') as stream:
            json.dump(detection_record, stream, ensure_ascii=False, indent=2)
        evidence = {'waypoint': name, 'target_base_pose': {'x': x, 'y': y, 'yaw': yaw},
                    'settle_wait_s': round(settle_wait_s, 4),
                    'image': stem + '.png', 'camera_topic': '/camera/image_raw',
                    'raw_image': stem + '.raw.png',
                    'annotated_image': stem + '.png',
                    'detection_record': stem + '.detections.json',
                    'source_stamp': detection_record['source_stamp'],
                    'depth_source_stamp': {'secs': int(depth_key[0]), 'nsecs': int(depth_key[1])},
                    'rgb_depth_stamp_delta_s': depth_delta,
                    'detector_counts': self.accepted_photo_records[-1]['detector_counts'],
                    'depth_target_metrics': target_depths,
                    'depth_frame': stem + '.depth.npy',
                    'photo_validation': photo_reason,
                    'sharpness': float(sharpness),
                    'depth_topic': '/camera/depth/image_raw',
                    'depth_geometry': self.depth_geometry(depth_frame),
                    'acceptance_criteria': self.photo_acceptance.get(name)}
        try:
            stamp = rospy.Time(int(frame_key[0]), int(frame_key[1]))
            self.tf_listener.waitForTransform('map', 'base_footprint', stamp, rospy.Duration(2))
            base_position, base_q = self.tf_listener.lookupTransform(
                'map', 'base_footprint', stamp)
            base_yaw = math.atan2(
                2 * (base_q[3] * base_q[2] + base_q[0] * base_q[1]),
                1 - 2 * (base_q[1] * base_q[1] + base_q[2] * base_q[2]))
            evidence['actual_base_pose'] = {
                'frame': 'base_footprint',
                'x': round(base_position[0], 4),
                'y': round(base_position[1], 4),
                'yaw': round(base_yaw, 4),
            }
            self.tf_listener.waitForTransform('map', 'camera_optical_frame', stamp, rospy.Duration(2))
            camera_position, camera_q = self.tf_listener.lookupTransform(
                'map', 'camera_optical_frame', stamp)
            camera_yaw = math.atan2(
                2 * (camera_q[3] * camera_q[2] + camera_q[0] * camera_q[1]),
                1 - 2 * (camera_q[1] * camera_q[1] + camera_q[2] * camera_q[2]))
            evidence['camera_pose'] = {
                'frame': 'camera_optical_frame',
                'x': round(camera_position[0], 4),
                'y': round(camera_position[1], 4),
                'z': round(camera_position[2], 4),
                'yaw': round(camera_yaw, 4),
                'quaternion': [round(float(v), 6) for v in camera_q],
                'view_direction_world': [
                    round(2 * (camera_q[0] * camera_q[2] + camera_q[3] * camera_q[1]), 6),
                    round(2 * (camera_q[1] * camera_q[2] - camera_q[3] * camera_q[0]), 6),
                    round(1 - 2 * (camera_q[0] ** 2 + camera_q[1] ** 2), 6),
                ],
            }
        except (tf.Exception, tf.LookupException, tf.ConnectivityException) as error:
            rospy.logwarn('camera pose unavailable at %s: %s', name, error)
        np.save(stem + '.depth.npy', depth_frame)
        if self.hd_capture is not None and name in ('POINT_8','POINT_9','POINT_10'):
            evidence['hd_plate'] = self.hd_capture.save(name,detection_record,self.photo_dir,capture_start)
        if self.person_counter is not None:
            camera_calibration = (dict(K=list(self.depth_info.K)) if self.depth_info is not None
                                  else self.person_config['camera'])
            evidence['camera_intrinsics'] = calibrated_intrinsics(camera_calibration).ravel().tolist()
            evidence['intrinsics_source'] = ('CameraInfo' if self.depth_info is not None
                                            else 'configured depth sensor FOV and dimensions')
            if name in self.person_config['required_views']:
                evidence['people'] = self.record_person_view(
                    name, dict(detection_record, detections=counted_detections),
                    raw_frame, depth_frame, evidence)
        with open(stem + '.json', 'w', encoding='utf-8') as stream:
            json.dump(evidence, stream, indent=2)
        rospy.loginfo('photo saved at %s', stem + '.png')
        return True

    def record_person_view(self, name, record, raw, depth, evidence):
        directory = os.path.join(self.photo_dir, 'persons')
        os.makedirs(directory, exist_ok=True)
        annotated_path = os.path.join(directory, name + '.png')
        if 'camera_pose' in evidence:
            observations, errors = collect_observations(
                record, raw, depth, evidence['camera_pose'], evidence['camera_intrinsics'],
                self.person_config, name, annotated_path)
        else:
            observations, errors = [], [{'waypoint': name, 'reason': 'CAMERA_TF_UNAVAILABLE'}]
        resolved = self.person_counter.add_view(name, record['source_stamp'], observations, errors)
        canvas = annotate_people(raw, resolved)
        for person in resolved:
            x1, y1, x2, y2 = map(int, person['box'])
            if person['class'] == 'stranger':
                crop = raw[max(0,y1):y2,max(0,x1):x2]
                crop_path = os.path.join(directory, person['person_id'] + '_' + name + '.png')
                cv2.imwrite(crop_path, crop)
                track = next(t for t in self.person_counter.tracks
                             if t['person_id'] == person['person_id'])
                track['observations'][-1]['foreign_crop_path'] = crop_path
        cv2.imwrite(annotated_path, canvas)
        report = self.person_counter.report()
        public_people = [{key: value for key,value in person.items() if key != 'appearance'}
                         for person in resolved]
        payload = {'waypoint': name, 'source_stamp': record['source_stamp'],
                   'people': public_people, 'cumulative_counts': report['counts'],
                   'annotated_image': annotated_path, 'issues': errors}
        self.person_report_pub.publish(String(data=json.dumps(payload,ensure_ascii=False)))
        image_msg = self.bridge.cv2_to_imgmsg(canvas,'bgr8')
        image_msg.header.stamp = rospy.Time(record['source_stamp']['secs'],
                                            record['source_stamp']['nsecs'])
        image_msg.header.frame_id = 'camera_optical_frame'
        self.person_image_pub.publish(image_msg)
        residents = sum(p['class']=='resident' for p in resolved)
        strangers = sum(p['class']=='stranger' for p in resolved)
        lines = ['[人物识别] %s 帧%.9f：本图%d人，社区%d人、外来%d人；累计去重%d人。图片：%s' % (
            name,record['source_stamp']['seconds'],len(resolved),residents,strangers,
            report['counts']['total'],annotated_path)]
        for person in resolved:
            lines.append('[人物识别] %s %s街区 %s 置信度%.3f，位置(%.3f, %.3f)' % (
                person['person_id'],person['street'],
                '外来人员' if person['class']=='stranger' else '社区人员',person['confidence'],
                person['world_xy'][0],person['world_xy'][1]))
        if errors:
            lines.append('[人物识别] 待复核：'+json.dumps(errors,ensure_ascii=False))
        with open(os.path.join(self.photo_dir,'people_terminal.txt'),'a',encoding='utf-8') as stream:
            stream.write('\n'.join(lines)+'\n')
        rospy.loginfo('%s',lines[0])
        return public_people

    @staticmethod
    def goal(x, y, yaw):
        goal = MoveBaseGoal()
        goal.target_pose = PoseStamped()
        goal.target_pose.header.frame_id = 'map'
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose.position.x = x
        goal.target_pose.pose.position.y = y
        q = quaternion_from_euler(0.0, 0.0, yaw)
        goal.target_pose.pose.orientation.x = q[0]
        goal.target_pose.pose.orientation.y = q[1]
        goal.target_pose.pose.orientation.z = q[2]
        goal.target_pose.pose.orientation.w = q[3]
        return goal

    def validate_goal(self, name, x, y, yaw, live=True):
        grids = [('map', self.static_map)]
        if live:
            if self.costmap is None or (rospy.Time.now() - self.costmap.header.stamp).to_sec() > 2.0:
                self.costmap = rospy.wait_for_message(
                    '/move_base/global_costmap/costmap', OccupancyGrid, timeout=5.0)
            grids.append(('global_costmap', self.costmap))
        for source, grid in grids:
            if grid is None or grid.header.frame_id.lstrip('/') != 'map':
                rospy.logerr('goal %s rejected: missing map-frame %s', name, source)
                return False
            result = GridFootprintChecker.from_message(
                grid, DEFAULT_FOOTPRINT, safety_margin=0.04).check_pose(x, y, yaw)
            if not result.safe:
                rospy.logerr('goal %s rejected by %s: %s cell=%s',
                             name, source, result.reason, result.cell)
                self.status_pub.publish('REJECTED:%s:%s' % (name, result.reason))
                return False
        return True

    def validate_ordered_corridor(self, name, goal_x, goal_y, goal_yaw):
        """Preflight the exact forward corridor that the local controller follows."""
        if not self.photo_route or not self.strict_acceptance:
            return True
        if not self.corridor_segments or self.corridor_length <= 0.0:
            rospy.logerr('%s rejected: ordered corridor is unavailable', name)
            return False
        try:
            current_x, current_y, current_yaw = self.motion_pose_map()
        except (tf.Exception, RuntimeError) as error:
            rospy.logerr('%s rejected: current pose unavailable: %s', name, error)
            return False
        if self.costmap is None or (rospy.Time.now() - self.costmap.header.stamp).to_sec() > 2.0:
            try:
                self.costmap = rospy.wait_for_message(
                    '/move_base/global_costmap/costmap', OccupancyGrid, timeout=5.0)
            except rospy.ROSException as error:
                rospy.logerr('%s rejected: global costmap unavailable: %s', name, error)
                return False
        if self.static_map is None or self.costmap is None:
            rospy.logerr('%s rejected: static/global map unavailable', name)
            return False
        goal_distance, goal_progress = nearest_route_progress(
            (goal_x, goal_y), self.corridor_segments)
        start_progress = self.route_cursor
        if goal_progress < start_progress - 0.01 and name == 'HOME':
            goal_progress += self.corridor_length
        if goal_progress < start_progress - 0.01:
            rospy.logerr('%s rejected: goal is behind the active route cursor', name)
            return False
        start_distance, projected_start = nearest_route_progress_in_interval(
            (current_x, current_y), self.corridor_segments, self.corridor_length,
            start_progress - 0.18, goal_progress + 0.18)
        if projected_start is not None:
            start_progress = max(start_progress, projected_start)
        if max(start_distance, goal_distance) > 0.18:
            rospy.logerr('%s rejected: endpoint outside corridor (%.3f/%.3f m)',
                         name, start_distance, goal_distance)
            return False
        if projected_start is None or goal_progress < start_progress - 0.02:
            rospy.logerr('%s rejected: goal is outside the active forward route interval', name)
            return False
        static_checker = GridFootprintChecker.from_message(
            self.static_map, DEFAULT_FOOTPRINT, safety_margin=0.02)
        global_checker = GridFootprintChecker.from_message(
            self.costmap, DEFAULT_FOOTPRINT, safety_margin=0.02)
        def corridor_pose(progress):
            wrapped = progress % self.corridor_length
            selected = None
            for a, b, segment_start, length in self.corridor_segments:
                if length > 1e-9 and segment_start - 1e-9 <= wrapped <= segment_start + length + 1e-9:
                    selected = (a, b, segment_start, length)
                    break
            if selected is None:
                raise ValueError('no corridor segment at progress %.3f' % progress)
            a, b, segment_start, length = selected
            fraction = max(0.0, min(1.0, (wrapped-segment_start)/length))
            px = a[0] + fraction*(b[0]-a[0])
            py = a[1] + fraction*(b[1]-a[1])
            tangent = math.atan2(b[1]-a[1], b[0]-a[0])
            return px, py, tangent

        samples = [(current_x, current_y, current_yaw)]
        try:
            start_on_route = corridor_pose(start_progress)
            goal_on_route = corridor_pose(goal_progress)
        except ValueError as error:
            rospy.logerr('%s rejected: %s', name, error)
            return False

        def append_connector(a, b, heading):
            distance = math.hypot(b[0]-a[0], b[1]-a[1])
            count = max(1, int(math.ceil(distance/0.02)))
            for index in range(1, count+1):
                fraction = index/float(count)
                samples.append((a[0]+fraction*(b[0]-a[0]),
                                a[1]+fraction*(b[1]-a[1]), heading))

        append_connector((current_x, current_y), start_on_route[:2], start_on_route[2])
        progress = start_progress + 0.02
        while progress < goal_progress - 0.02:
            samples.append(corridor_pose(progress))
            progress += 0.02
        append_connector(goal_on_route[:2], (goal_x, goal_y), goal_yaw)
        for sample_index, (px, py, heading) in enumerate(samples):
            for source, checker in (('map', static_checker),
                                    ('global_costmap', global_checker)):
                result = checker.check_pose(px, py, heading)
                if not result.safe:
                    rospy.logerr('%s rejected by ordered corridor %s: %s cell=%s at (%.3f, %.3f)',
                                 name, source, result.reason, result.cell, px, py)
                    return False
        rospy.loginfo('%s ordered corridor clear: %.3f m, endpoint cross-track %.3f m, %d samples',
                      name, goal_progress-start_progress,
                      max(start_distance, goal_distance), len(samples))
        self.planned_goal_progress = goal_progress
        return True

    @staticmethod
    def _angle_error(target, actual):
        return math.atan2(math.sin(target - actual), math.cos(target - actual))

    def navigate_goal(self, name, x, y, yaw, previous=None):
        """Send one goal; fail on an 8 s route-progress stall without retry."""
        terminal = {GoalStatus.SUCCEEDED, GoalStatus.ABORTED, GoalStatus.PREEMPTED,
                    GoalStatus.REJECTED, GoalStatus.RECALLED, GoalStatus.LOST}
        if not self.validate_goal(name, x, y, yaw):
            self.goal_events.append({'name': name, 'result': 'UNSAFE_GOAL'})
            return False
        if not self.validate_ordered_corridor(name, x, y, yaw):
            self.goal_events.append({'name': name, 'result': 'ORDERED_CORRIDOR_BLOCKED'})
            return False
        ordered_monitor = (self.photo_route and self.strict_acceptance and
                           self.corridor_length > 0.0)
        route_start = self.route_cursor
        route_goal = self.planned_goal_progress
        if ordered_monitor and route_goal is None:
            self.goal_events.append({'name': name, 'result': 'ORDERED_ROUTE_CURSOR_MISSING'})
            return False

        nav_xy_tolerance = float(rospy.get_param('~photo_nav_xy_tolerance', 0.04))
        nav_yaw_tolerance = float(rospy.get_param('~photo_nav_heading_tolerance', 0.05))
        if name == 'POINT_3':
            nav_xy_tolerance = max(nav_xy_tolerance, float(rospy.get_param(
                '~point_3_nav_xy_tolerance', 0.05)))
        if name == 'POINT_5':
            nav_xy_tolerance = min(nav_xy_tolerance, float(rospy.get_param(
                '~point_5_nav_xy_tolerance', 0.025)))
            nav_yaw_tolerance = min(nav_yaw_tolerance, float(rospy.get_param(
                '~point_5_nav_heading_tolerance', 0.025)))
        if name == 'POINT_7':
            nav_xy_tolerance = max(nav_xy_tolerance, float(rospy.get_param(
                '~point_7_nav_xy_tolerance', 0.05)))
        if name == 'POINT_10':
            nav_xy_tolerance = max(nav_xy_tolerance, float(rospy.get_param(
                '~point_10_nav_xy_tolerance', 0.05)))
        controller_xy_tolerance = min(0.05, nav_xy_tolerance)
        # Use the already-approved pose tolerance for the local controller.
        # Requiring 0.02 rad here made it keep turning after the task's 0.04
        # rad heading criterion was satisfied, which can stall short legs.
        controller_yaw_tolerance = min(0.04, nav_yaw_tolerance)

        # BaseLocalPlanner::setPlan sees Navfn's final path tangent, which can
        # differ from the saved camera heading. Set the explicit photo/home yaw
        # before dispatch so the single controller finishes at the requested pose.
        rospy.set_param('/move_base/forward_path_follower/target_x', float(x))
        rospy.set_param('/move_base/forward_path_follower/target_y', float(y))
        rospy.set_param('/move_base/forward_path_follower/target_yaw', float(yaw))
        rospy.set_param('/move_base/forward_path_follower/target_xy_tolerance',
                        float(controller_xy_tolerance))
        rospy.set_param('/move_base/forward_path_follower/target_yaw_tolerance',
                        float(controller_yaw_tolerance))
        if ordered_monitor:
            rospy.set_param('/move_base/forward_path_follower/route_start_s',
                            float(route_start))
            rospy.set_param('/move_base/forward_path_follower/route_goal_s',
                            float(route_goal))
        try:
            start_x, start_y, _ = self.motion_pose_map()
        except (tf.Exception, RuntimeError) as error:
            rospy.logerr('%s start pose unavailable: %s', name, error)
            return False
        started = time.monotonic()
        initial_goal_distance = math.hypot(x - start_x, y - start_y)
        progress_mark = 0.0
        last_progress_wall = started
        progress_quantum = 0.05
        if ordered_monitor:
            route_leg_length = max(0.0, route_goal - route_start)
            progress_quantum = min(0.05, max(0.01, route_leg_length * 0.25))
        final_alignment_started = None
        max_cross_track = 0.0
        self.client.send_goal(self.goal(x, y, yaw))
        reason = 'TIMEOUT'
        while not rospy.is_shutdown() and time.monotonic() - started < self.timeout:
            state = self.client.get_state()
            if state in terminal:
                reason = 'ACTION_RESULT'
                break
            try:
                current_x, current_y, current_yaw = self.motion_pose_map()
                remaining = math.hypot(x - current_x, y - current_y)
                if ordered_monitor:
                    cross_track, route_s = nearest_route_progress_in_interval(
                        (current_x, current_y), self.corridor_segments,
                        self.corridor_length, route_start - 0.10, route_goal + 0.05)
                    if route_s is None or cross_track > 0.18:
                        rospy.logerr(
                            '%s left ordered corridor: pose=(%.3f, %.3f), '
                            'projection=%s, interval=[%.3f, %.3f], cross_track=%.3f m',
                            name, current_x, current_y,
                            'none' if route_s is None else '%.3f' % route_s,
                            route_start - 0.10, route_goal + 0.05, cross_track)
                        reason = 'CORRIDOR_DEPARTURE'
                        break
                    if route_s > route_goal + 0.05:
                        reason = 'ROUTE_OVERSHOOT'
                        break
                    self.route_cursor = max(self.route_cursor, route_s)
                    route_delta = route_s - route_start
                    max_cross_track = max(max_cross_track, cross_track)
                else:
                    cross_track, route_delta = 0.0, 0.0
                progress = max(0.0, route_delta,
                               initial_goal_distance - remaining)
                now = time.monotonic()
                signal_wait = (now - self.gate_wall_time < 0.7 and
                               'WAIT_' in self.latest_gate_status)
                final_pose_alignment = (
                    remaining <= max(nav_xy_tolerance, 0.05) and
                    abs(self._angle_error(yaw, current_yaw)) > controller_yaw_tolerance)
                if final_pose_alignment:
                    if final_alignment_started is None:
                        final_alignment_started = now
                    elif now - final_alignment_started >= 10.0:
                        reason = 'FINAL_ALIGNMENT_STALL'
                        break
                else:
                    final_alignment_started = None
                if signal_wait:
                    # Red/yellow/unknown holds do not consume the motion
                    # stall budget. Start a fresh 8 s window when the gate
                    # releases the vehicle on a valid GREEN.
                    progress_mark = progress
                    last_progress_wall = now
                elif final_pose_alignment:
                    # Spatial progress naturally stops during the bounded
                    # final yaw settle; budget it separately from a route stall.
                    progress_mark = progress
                    last_progress_wall = now
                elif progress >= progress_mark + progress_quantum:
                    progress_mark = progress
                    last_progress_wall = now
                odom = self.latest_odom
                motion_record = {
                    'stamp': rospy.Time.now().to_sec(), 'goal': name,
                    'route_progress_m': round(progress, 4),
                    'cross_track_m': round(cross_track, 4),
                    'goal_remaining_m': round(remaining, 4),
                    'forward_speed_mps': (round(odom.twist.twist.linear.x, 4)
                                          if odom is not None else None),
                    'gate_status': self.latest_gate_status,
                }
                self.motion_progress_pub.publish(String(
                    data=json.dumps(motion_record, separators=(',', ':'))))
                if now - last_progress_wall >= self.no_progress_timeout and not signal_wait:
                    reason = 'NO_ROUTE_PROGRESS'
                    break
            except (tf.Exception, RuntimeError):
                pass
            time.sleep(0.05)

        state = self.client.get_state()
        duration = time.monotonic() - started
        if reason != 'ACTION_RESULT' and state not in terminal:
            self.client.cancel_goal()
            self.client.wait_for_result(rospy.Duration(2.0))
        self.cmd_pub.publish(Twist())
        event = {'name': name, 'duration_s': round(duration, 2),
                 'action_state': state, 'result': reason,
                 'route_progress_m': round(progress_mark, 4),
                 'max_cross_track_m': round(max_cross_track, 4),
                 'attempts': 1}
        if state == GoalStatus.SUCCEEDED and reason == 'ACTION_RESULT':
            try:
                actual_x, actual_y, actual_yaw = self.map_pose()
                xy_error = math.hypot(actual_x - x, actual_y - y)
                yaw_error = abs(self._angle_error(yaw, actual_yaw))
                event.update({'position_error_m': xy_error,
                              'heading_error_rad': yaw_error})
                if (xy_error <= nav_xy_tolerance and
                        yaw_error <= nav_yaw_tolerance):
                    if ordered_monitor:
                        self.route_cursor = max(self.route_cursor, route_goal)
                    self.goal_events.append(event)
                    return True
                event['result'] = 'ARRIVAL_MISMATCH'
            except (tf.Exception, RuntimeError) as error:
                event['result'] = 'ARRIVAL_POSE_UNAVAILABLE'
                event['detail'] = str(error)
        self.goal_events.append(event)
        rospy.logerr('goal %s failed once: state=%d reason=%s',
                     name, state, event['result'])
        return False

    def run(self):
        self.status_pub.publish('WAIT_MOVE_BASE')
        if not rospy.get_param('/cmd_vel_watchdog/enforce_white_lines', False):
            self.status_pub.publish('FAILED:WHITE_LINE_GUARD_DISABLED')
            rospy.logerr('route refused: white-line gate is disabled')
            return False
        if self.strict_acceptance and not rospy.get_param(
                '/cmd_vel_watchdog/enforce_traffic', False):
            self.status_pub.publish('FAILED:TRAFFIC_GATE_DISABLED')
            rospy.logerr('route refused: traffic-light gate is disabled')
            return False
        ready = self.client.wait_for_server(rospy.Duration(30.0))
        if not ready:
            self.status_pub.publish('FAILED:NO_MOVE_BASE')
            return False
        if self.static_map is None:
            self.static_map = rospy.wait_for_message('/map', OccupancyGrid, timeout=10.0)
        for name, x, y, yaw in self.route + [('HOME', self.birth_x, self.birth_y, self.birth_yaw)]:
            if not self.validate_goal(name, x, y, yaw, live=False):
                return False
        start_index = int(rospy.get_param('~start_waypoint', 0))
        previous = self.route[start_index - 1] if start_index > 0 else None
        photo_total = sum(1 for item in self.route if not item[0].startswith('_TRANSITION_'))
        photo_index = sum(1 for item in self.route[:start_index]
                          if not item[0].startswith('_TRANSITION_'))
        for index, (name, x, y, yaw) in enumerate(self.route[start_index:], start_index + 1):
            if rospy.is_shutdown():
                return False
            is_transition = name.startswith('_TRANSITION_')
            if is_transition:
                self.status_pub.publish('TRANSIT:%s' % name)
                rospy.loginfo('autonomous transit %s (%.2f, %.2f)', name, x, y)
            else:
                photo_index += 1
                self.status_pub.publish('GO:%02d/%02d:%s' % (photo_index, photo_total, name))
                rospy.loginfo('photo point %d/%d start: %s (%.2f, %.2f)',
                              photo_index, photo_total, name, x, y)
            if not self.navigate_goal(name, x, y, yaw, previous=previous):
                self.status_pub.publish('FAILED:NAV:%s' % name)
                rospy.logerr('route navigation failed: %s', name)
                return False
            if not self.capture_photo(name, x, y, yaw):
                self.status_pub.publish('FAILED:PHOTO_POSE:%02d:%s' % (photo_index, name))
                rospy.logerr('photo point %d/%d not captured: %s', photo_index, photo_total, name)
                return False
            rospy.loginfo('navigation target reached: %s (%.2f, %.2f)', name, x, y)
            previous = (name, x, y, yaw)
        if self.photo_route and self.capture_photos:
            captured_names = [item['waypoint'] for item in self.accepted_photo_records]
            if captured_names != ['POINT_%d' % i for i in range(1, 11)]:
                rospy.logerr('photo acceptance requires exactly 10 ordered records; got %s',
                             captured_names)
                return False
            people_records = [record for record in self.accepted_photo_records
                              if record['waypoint'] in
                              ('POINT_2', 'POINT_3', 'POINT_4', 'POINT_5', 'POINT_6')]
            person_boxes = sum(record['detector_counts'].get('resident', 0) +
                               record['detector_counts'].get('stranger', 0)
                               for record in people_records)
            outsider_boxes = sum(record['detector_counts'].get('stranger', 0)
                                 for record in people_records)
            self.person_boxes_by_view = person_boxes
            self.outsider_boxes_by_view = outsider_boxes
        if self.photo_route and not rospy.get_param('~park_only', False):
            if not self.return_home(
                    (previous[1], previous[2]) if previous is not None else None):
                return False
        parked = self.precise_park()
        if parked and self.person_counter is not None and not self.person_counter.report()['complete']:
            rospy.logerr('person report failed: %s',self.person_counter.report()['checks'])
            self.status_pub.publish('FAILED:PERSON_REPORT')
            return False
        if parked and self.ocr_enabled:
            deadline = time.monotonic()+5.0
            while time.monotonic()<deadline:
                results = []
                for name in ('POINT_8','POINT_9','POINT_10'):
                    path = os.path.join(self.photo_dir,'ocr',name+'.published.json')
                    if os.path.isfile(path):
                        with open(path,encoding='utf-8') as stream:
                            results.append(json.load(stream))
                self.ocr_results = results
                if len(results)==3:
                    break
                self.cmd_pub.publish(Twist())
                time.sleep(.05)
            if (len(self.ocr_results)!=3 or not all(
                    r['valid'] and r['matching_frames']>=2 and r['confidence']>=.85
                    for r in self.ocr_results)):
                self.status_pub.publish('FAILED:OCR_INCOMPLETE_OR_UNCONFIRMED')
                return False
        return parked

    def return_home(self, last_photo_position=None):
        """Return to HOME with one bounded retry for a successful pose mismatch."""
        self.status_pub.publish('RETURN_HOME')
        original_xy_tolerance = float(rospy.get_param('~photo_nav_xy_tolerance', 0.04))
        original_yaw_tolerance = float(
            rospy.get_param('~photo_nav_heading_tolerance', 0.05))
        home_xy_tolerance, home_yaw_tolerance = home_navigation_tolerances(
            original_xy_tolerance, original_yaw_tolerance)
        original_timeout = self.timeout
        try:
            rospy.set_param('~photo_nav_xy_tolerance', home_xy_tolerance)
            rospy.set_param('~photo_nav_heading_tolerance', home_yaw_tolerance)
            if self.navigate_goal('HOME', self.birth_x, self.birth_y, self.birth_yaw):
                return True

            event = self.goal_events[-1] if self.goal_events else {}
            action_succeeded = event.get('action_state') == GoalStatus.SUCCEEDED
            if not should_retry_home_alignment(
                    event.get('result'), action_succeeded, attempts=0):
                return False

            event['home_refinement_retry'] = {
                'attempt': 1,
                'reason': 'ARRIVAL_MISMATCH',
                'position_error_m': event.get('position_error_m'),
                'heading_error_rad': event.get('heading_error_rad'),
                'xy_tolerance_m': home_xy_tolerance,
                'yaw_tolerance_rad': home_yaw_tolerance,
            }
            self.status_pub.publish('HOME_FINAL_ALIGNMENT')
            self.timeout = min(original_timeout, HOME_REFINEMENT_TIMEOUT_S)
            corrected = self.navigate_goal(
                'HOME', self.birth_x, self.birth_y, self.birth_yaw)
            if not corrected:
                self.status_pub.publish('FAILED:HOME_ALIGNMENT')
            return corrected
        finally:
            self.timeout = original_timeout
            rospy.set_param('~photo_nav_xy_tolerance', original_xy_tolerance)
            rospy.set_param('~photo_nav_heading_tolerance', original_yaw_tolerance)

    def wheel_pose_map(self):
        """Transform fresh encoder odometry through the latest AMCL map->odom."""
        message = self.latest_odom
        if message is None or (rospy.Time.now() - message.header.stamp).to_sec() > 0.5:
            raise RuntimeError('wheel odometry is stale')
        (tx, ty, _), q = self.tf_listener.lookupTransform('map', 'odom', rospy.Time(0))
        map_yaw = math.atan2(2 * (q[3] * q[2] + q[0] * q[1]),
                             1 - 2 * (q[1] * q[1] + q[2] * q[2]))
        odom = message.pose.pose
        q = odom.orientation
        odom_yaw = math.atan2(2 * (q.w * q.z + q.x * q.y),
                              1 - 2 * (q.y * q.y + q.z * q.z))
        c, s = math.cos(map_yaw), math.sin(map_yaw)
        return (tx + c * odom.position.x - s * odom.position.y,
                ty + s * odom.position.x + c * odom.position.y,
                map_yaw + odom_yaw)

    def motion_pose_map(self):
        """Use encoders for motion progress and fall back to current map TF."""
        try:
            return self.wheel_pose_map()
        except (tf.Exception, RuntimeError):
            return self.map_pose()

    def map_pose(self):
        deadline = time.monotonic() + 0.5
        last_error = None
        while time.monotonic() < deadline:
            try:
                stamp = self.tf_listener.getLatestCommonTime('map', 'base_footprint')
                age = (rospy.Time.now() - stamp).to_sec()
                if age > 0.8:
                    raise RuntimeError('stale map pose (%.3f s)' % age)
                (x, y, _), q = self.tf_listener.lookupTransform(
                    'map', 'base_footprint', rospy.Time(0))
                yaw = math.atan2(2 * (q[3] * q[2] + q[0] * q[1]),
                                 1 - 2 * (q[1] ** 2 + q[2] ** 2))
                return x, y, yaw
            except (tf.Exception, RuntimeError) as error:
                last_error = error
                try:
                    return self.wheel_pose_map()
                except (tf.Exception, RuntimeError):
                    time.sleep(0.01)
        raise RuntimeError('map pose unavailable: %s' % last_error)

    def precise_park(self):
        """Require the HOME pose and zero velocity for two continuous seconds."""
        self.status_pub.publish('VERIFY_HOME')
        deadline = time.monotonic() + 5.0
        still_since = None
        last_pose = None
        while not rospy.is_shutdown() and time.monotonic() < deadline:
            self.cmd_pub.publish(Twist())
            try:
                x, y, yaw = self.map_pose()
            except (tf.Exception, RuntimeError) as error:
                self.status_pub.publish('FAILED:HOME_LOCALIZATION')
                rospy.logerr('HOME localization unavailable: %s', error)
                return False
            distance = math.hypot(self.birth_x - x, self.birth_y - y)
            yaw_error = abs(self._angle_error(self.birth_yaw, yaw))
            message = self.latest_odom
            fresh_odom = (message is not None and
                          (rospy.Time.now() - message.header.stamp).to_sec() <= 0.5)
            if not fresh_odom:
                self.status_pub.publish('FAILED:HOME_ODOM_STALE')
                return False
            linear = abs(message.twist.twist.linear.x)
            angular = abs(message.twist.twist.angular.z)
            last_pose = (x, y, yaw, distance, yaw_error, linear, angular)
            pose_ok = (distance <= HOME_XY_TOLERANCE_M and
                       yaw_error <= HOME_YAW_TOLERANCE_RAD)
            stopped = linear < 0.01 and angular < 0.01
            if not pose_ok:
                self.status_pub.publish('FAILED:HOME_POSE')
                break
            if stopped:
                if still_since is None:
                    still_since = time.monotonic()
                elif time.monotonic() - still_since >= 2.0:
                    self.parking = {
                        'frame': 'map', 'source': 'AMCL', 'x': x, 'y': y,
                        'yaw': yaw, 'position_error_m': distance,
                        'heading_error_rad': yaw_error,
                        'linear_speed_mps': linear, 'angular_speed_rps': angular,
                        'stationary_seconds': time.monotonic() - still_since,
                    }
                    self.status_pub.publish('HOME_PARKED')
                    return True
            else:
                still_since = None
            time.sleep(0.05)
        self.status_pub.publish('FAILED:HOME_NOT_STATIONARY')
        if last_pose:
            rospy.logerr('HOME failed: position=%.3f m yaw=%.3f rad v=%.3f w=%.3f',
                         last_pose[3], last_pose[4], last_pose[5], last_pose[6])
        return False

    def finish(self, success):
        import os
        self.client.cancel_all_goals()
        self.cmd_pub.publish(Twist())
        if self.capture_photos:
            os.makedirs(self.photo_dir, exist_ok=True)
            person_report = None
            if self.person_counter is not None:
                person_report = save_report(self.person_counter, self.photo_dir)
                with open(os.path.join(self.photo_dir,'person_reporting_config.json'),'w',encoding='utf-8') as stream:
                    json.dump(self.person_config,stream,ensure_ascii=False,indent=2)
                self.person_report_pub.publish(String(data=json.dumps(person_report,ensure_ascii=False)))
            report = {'route_status': 'COMPLETE_PARKED' if success else 'FAILED',
                      'localization': 'wheel_encoders+AMCL',
                      'map_file': rospy.get_param('/map_server/map_file', ''),
                      'goals': self.goal_events, 'parking': getattr(self, 'parking', None),
                      'strict_acceptance': self.strict_acceptance,
                      'yolo_checkpoint_sha256': BEST_PT_SHA256,
                      'accepted_photo_records': self.accepted_photo_records,
                      'person_boxes_by_view': getattr(self, 'person_boxes_by_view', None),
                      'outsider_boxes_by_view': getattr(self, 'outsider_boxes_by_view', None),
                      'person_reporting_enabled': self.person_counter is not None,
                      'hd_plate_capture_enabled': self.hd_capture is not None,
                      'ocr_enabled': self.ocr_enabled,
                      'ocr_results': self.ocr_results,
                      'person_report': person_report,
                      'photo_points': [name for name, *_ in self.route],
                      'completed_photos': sorted(os.listdir(self.photo_dir))}
            with open(os.path.join(self.photo_dir, 'run_summary.json'), 'w') as stream:
                json.dump(report, stream, indent=2)
        self.status_pub.publish('COMPLETE_PARKED' if success else 'FAILED:MISSION')


if __name__ == '__main__':
    # A stable name lets runtime_control cancel this mission before a new
    # navigation mode starts; anonymous names leave stale action clients.
    rospy.init_node('route_executor')
    executor = RouteExecutor()
    success = False
    try:
        success = executor.run()
    except (rospy.ROSException, RuntimeError, tf.Exception) as error:
        rospy.logerr("mission stopped: %s", error)
    finally:
        executor.finish(success)
    raise SystemExit(0 if success else 1)
