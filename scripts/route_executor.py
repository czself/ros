#!/usr/bin/env python3
"""Execute the fixed 4.2 m route from the competition diagram.

The route is read from one contract rather than duplicated as ad-hoc Python
coordinates.  Its segments must join end-to-end, so a planner can never make
an unapproved diagonal jump from one diagram arrow to another.
"""

import math
import json
import time

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
from std_srvs.srv import Empty
from std_msgs.msg import String
from tf.transformations import quaternion_from_euler


CONTRACT_PATH = '/root/navigation/inner_route.yaml'


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
        self.static_map = None
        self.costmap = None
        self.local_costmap = None
        rospy.Subscriber('/map', OccupancyGrid,
                         lambda message: setattr(self, 'static_map', message), queue_size=1)
        rospy.Subscriber('/move_base/global_costmap/costmap', OccupancyGrid,
                         lambda message: setattr(self, 'costmap', message), queue_size=1)
        rospy.Subscriber('/move_base/local_costmap/costmap', OccupancyGrid,
                         lambda message: setattr(self, 'local_costmap', message), queue_size=1)
        self.route_vertices = bool(raw_contract.get('points')) and not photo_file
        self.status_pub = rospy.Publisher('/route/status', String, queue_size=1, latch=True)
        self.client = actionlib.SimpleActionClient('/move_base', MoveBaseAction)
        self.cmd_pub = rospy.Publisher('/my_car/cmd_vel_nav', Twist, queue_size=1)
        self.capture_photos = bool(rospy.get_param('~capture_photos', False))
        # /root/ros1_ws is bind-mounted to the host as /home/sz/ros1_ws.
        self.photo_dir = rospy.get_param('~photo_dir', '/root/ros1_ws/photo_stops')
        selected = rospy.get_param('~photo_waypoints', '')
        self.photo_waypoints = set(filter(None, (name.strip() for name in selected.split(','))))
        self.bridge = CvBridge()
        self.tf_listener = tf.TransformListener()
        self.latest_odom = None
        rospy.Subscriber('/odom', Odometry,
                         lambda message: setattr(self, 'latest_odom', message), queue_size=1)
        rospy.on_shutdown(self.stop_motion)
        self.latest_image = None
        self.latest_depth = None
        self.depth_info = None
        if self.capture_photos:
            rospy.Subscriber('/camera/image_raw', Image, self._image_cb, queue_size=1)
            rospy.Subscriber('/camera/depth/image_raw', Image, self._depth_cb, queue_size=1)
            rospy.Subscriber('/camera/depth/camera_info', CameraInfo,
                             lambda message: setattr(self, 'depth_info', message), queue_size=1)

    def stop_motion(self):
        self.client.cancel_all_goals()
        self.cmd_pub.publish(Twist())

    def _image_cb(self, message):
        try:
            self.latest_image = self.bridge.imgmsg_to_cv2(message, 'bgr8')
        except Exception as error:
            rospy.logwarn_throttle(5, 'photo capture image conversion failed: %s', error)

    def _depth_cb(self, message):
        try:
            depth = self.bridge.imgmsg_to_cv2(message, 'passthrough')
            self.latest_depth = np.asarray(depth, dtype=np.float32)
        except Exception as error:
            rospy.logwarn_throttle(5, 'depth capture conversion failed: %s', error)

    def depth_geometry(self):
        """Return robust depth and horizontal coverage diagnostics in metres."""
        if self.latest_depth is None:
            return None
        depth = self.latest_depth
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

    def capture_photo(self, name, x, y, yaw):
        if name.startswith('_TRANSITION_'):
            return True
        if not self.capture_photos or (self.photo_waypoints and name not in self.photo_waypoints):
            return True
        # Photo routes send the recorded camera framing yaw as the move_base
        # goal yaw.  Never publish a best-effort correction after success:
        # the watchdog may stop it at a painted line, leaving a wrong view.
        self.cmd_pub.publish(Twist())
        rospy.sleep(0.25)
        try:
            stamp = self.tf_listener.getLatestCommonTime('map', 'base_footprint')
            (actual_x, actual_y, _), actual_q = self.tf_listener.lookupTransform(
                'map', 'base_footprint', stamp)
            actual_yaw = math.atan2(
                2.0 * (actual_q[3] * actual_q[2] + actual_q[0] * actual_q[1]),
                1.0 - 2.0 * (actual_q[1] ** 2 + actual_q[2] ** 2))
        except (tf.Exception, tf.LookupException, tf.ConnectivityException) as error:
            rospy.logwarn('photo waypoint %s pose unavailable: %s', name, error)
            return False
        xy_error = math.hypot(actual_x - x, actual_y - y)
        yaw_error = math.atan2(math.sin(yaw - actual_yaw), math.cos(yaw - actual_yaw))
        xy_tolerance = float(rospy.get_param('~photo_position_tolerance', 0.06))
        yaw_tolerance = float(rospy.get_param('~photo_heading_tolerance', 0.06))
        if name == 'POINT_3':
            xy_tolerance = max(
                xy_tolerance,
                float(rospy.get_param('~point_3_photo_position_tolerance', 0.05)))
        if name == 'POINT_5':
            xy_tolerance = min(
                xy_tolerance,
                float(rospy.get_param('~point_5_photo_position_tolerance', 0.025)))
            yaw_tolerance = min(
                yaw_tolerance,
                float(rospy.get_param('~point_5_photo_heading_tolerance', 0.025)))
        if xy_error > xy_tolerance or abs(yaw_error) > yaw_tolerance:
            rospy.logwarn(
                'photo waypoint %s pose mismatch: xy_error=%.3f m yaw_error=%.3f rad; skipping capture',
                name, xy_error, yaw_error)
            return False
        rospy.sleep(float(rospy.get_param('~photo_settle_seconds', 2.0)))
        if self.latest_image is None:
            rospy.logwarn('photo waypoint %s reached but no camera frame is available', name)
            return False
        frames = [self.latest_image.copy()]
        for _ in range(max(1, int(rospy.get_param('~photo_burst_count', 8))) - 1):
            rospy.sleep(float(rospy.get_param('~photo_burst_interval', 0.2)))
            if self.latest_image is not None:
                frames.append(self.latest_image.copy())
        frame = max(frames, key=lambda image: cv2.Laplacian(image, cv2.CV_64F).var())
        import os
        stamp = time.strftime('%Y%m%d_%H%M%S')
        point_dir = os.path.join(self.photo_dir, name)
        os.makedirs(point_dir, exist_ok=True)
        stem = os.path.join(point_dir, '%s_%s' % (name, stamp))
        if not cv2.imwrite(stem + '.png', frame):
            rospy.logwarn('photo waypoint %s could not write image %s', name, stem + '.png')
            return False
        evidence = {'waypoint': name, 'target_base_pose': {'x': x, 'y': y, 'yaw': yaw},
                    'image': stem + '.png', 'camera_topic': '/camera/image_raw',
                    'depth_topic': '/camera/depth/image_raw',
                    'depth_geometry': self.depth_geometry(),
                    'acceptance_criteria': self.photo_acceptance.get(name)}
        try:
            stamp = rospy.Time(0)
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
        with open(stem + '.json', 'w', encoding='utf-8') as stream:
            json.dump(evidence, stream, indent=2)
        rospy.loginfo('photo saved at %s', stem + '.png')
        return True

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

    @staticmethod
    def _angle_error(target, actual):
        return math.atan2(math.sin(target - actual), math.cos(target - actual))

    def rotation_sweep_clear(self, x, y, start_yaw, target_yaw, name, phase):
        """Require both static and rolling grids to clear a stationary turn."""
        try:
            if self.costmap is None or (rospy.Time.now() - self.costmap.header.stamp).to_sec() > 1.0:
                self.costmap = rospy.wait_for_message(
                    '/move_base/global_costmap/costmap', OccupancyGrid, timeout=3.0)
            if self.local_costmap is None or (rospy.Time.now() - self.local_costmap.header.stamp).to_sec() > 1.0:
                self.local_costmap = rospy.wait_for_message(
                    '/move_base/local_costmap/costmap', OccupancyGrid, timeout=3.0)
            global_grid = self.costmap
            local_grid = self.local_costmap
            if global_grid.header.frame_id.lstrip('/') != 'map':
                raise RuntimeError('global costmap is not in map frame')
            local_frame = local_grid.header.frame_id.lstrip('/')
            self.tf_listener.waitForTransform(local_frame, 'map', rospy.Time(0), rospy.Duration(1.0))
            local_t, local_q = self.tf_listener.lookupTransform(
                local_frame, 'map', rospy.Time(0))
            local_offset = math.atan2(2 * (local_q[3] * local_q[2] + local_q[0] * local_q[1]),
                                      1 - 2 * (local_q[1] * local_q[1] + local_q[2] * local_q[2]))
            c, s = math.cos(local_offset), math.sin(local_offset)
            local_x = local_t[0] + c * x - s * y
            local_y = local_t[1] + s * x + c * y
            local_start_yaw = start_yaw + local_offset
            delta = self._angle_error(target_yaw, start_yaw)
            sample_count = max(1, int(math.ceil(abs(delta) / 0.025)))
            global_checker = GridFootprintChecker.from_message(
                global_grid, DEFAULT_FOOTPRINT, safety_margin=0.01)
            local_checker = GridFootprintChecker.from_message(
                local_grid, DEFAULT_FOOTPRINT, safety_margin=0.01)
            for index in range(sample_count + 1):
                fraction = index / float(sample_count)
                global_yaw = start_yaw + delta * fraction
                local_yaw = local_start_yaw + delta * fraction
                global_result = global_checker.check_pose(x, y, global_yaw)
                if not global_result.safe:
                    rospy.logwarn('%s %s rotation blocked by global costmap: %s cell=%s at %.3f rad',
                                  name, phase, global_result.reason, global_result.cell, global_yaw)
                    return False
                local_result = local_checker.check_pose(local_x, local_y, local_yaw)
                if not local_result.safe:
                    rospy.logwarn('%s %s rotation blocked by local costmap: %s cell=%s at %.3f rad',
                                  name, phase, local_result.reason, local_result.cell, local_yaw)
                    return False
            rospy.loginfo('%s %s rotation sweep clear: %.1f degrees in %d footprint samples',
                          name, phase, math.degrees(delta), sample_count + 1)
            return True
        except (rospy.ROSException, tf.Exception, RuntimeError, ValueError) as error:
            rospy.logwarn('%s %s rotation sweep unavailable: %s', name, phase, error)
            return False

    def rotate_in_place_to_yaw(self, name, target_yaw, phase, position_tolerance=0.035):
        """Turn at the current physical point with one fixed, checked direction."""
        try:
            start_x, start_y, start_yaw = self.map_pose()
            start_motion_pose = self.progress_pose()
        except (tf.Exception, RuntimeError) as error:
            rospy.logerr('%s %s turn pose unavailable: %s', name, phase, error)
            return False
        initial_error = self._angle_error(target_yaw, start_yaw)
        if abs(initial_error) <= 0.025:
            return True
        if not self.rotation_sweep_clear(
                start_x, start_y, start_yaw, target_yaw, name, phase):
            return False

        direction = 1.0 if initial_error > 0.0 else -1.0
        deadline = time.monotonic() + abs(initial_error) / 0.12 + 8.0
        last_yaw = start_yaw
        last_yaw_progress = time.monotonic()
        rate = rospy.Rate(20)
        rospy.loginfo('%s %s turn starts at same point: %.1f degree error, direction=%+.0f',
                      name, phase, math.degrees(initial_error), direction)
        try:
            while not rospy.is_shutdown() and time.monotonic() < deadline:
                x, y, current_yaw = self.map_pose()
                motion_pose = self.progress_pose()
                error = self._angle_error(target_yaw, current_yaw)
                if abs(error) <= 0.025:
                    self.cmd_pub.publish(Twist())
                    rospy.sleep(0.2)
                    _, _, final_yaw = self.map_pose()
                    final_motion_pose = self.progress_pose()
                    position_drift = math.hypot(
                        final_motion_pose[0] - start_motion_pose[0],
                        final_motion_pose[1] - start_motion_pose[1])
                    final_error = self._angle_error(target_yaw, final_yaw)
                    if position_drift <= position_tolerance and abs(final_error) <= 0.04:
                        rospy.loginfo('%s %s turn complete at same point: drift=%.3f m yaw_error=%.3f rad',
                                      name, phase, position_drift, final_error)
                        return True
                    rospy.logwarn('%s %s turn settled outside tolerance: drift=%.3f m yaw_error=%.3f rad',
                                  name, phase, position_drift, final_error)
                    return False
                if direction * error < -0.015:
                    rospy.logwarn('%s %s turn overshot; stopping instead of reversing direction', name, phase)
                    return False
                position_drift = math.hypot(
                    motion_pose[0] - start_motion_pose[0],
                    motion_pose[1] - start_motion_pose[1])
                if position_drift > position_tolerance:
                    rospy.logwarn('%s %s turn drifted %.3f m; stopping', name, phase, position_drift)
                    return False
                if abs(self._angle_error(current_yaw, last_yaw)) > 0.008:
                    last_yaw, last_yaw_progress = current_yaw, time.monotonic()
                elif time.monotonic() - last_yaw_progress > 3.0:
                    rospy.logwarn('%s %s turn made no heading progress; stopping', name, phase)
                    return False
                command = Twist()
                command.angular.z = direction * min(0.15, max(0.06, 0.8 * abs(error)))
                self.cmd_pub.publish(command)
                rate.sleep()
        except (tf.Exception, RuntimeError) as error:
            rospy.logwarn('%s %s turn pose became unavailable: %s', name, phase, error)
        finally:
            self.cmd_pub.publish(Twist())
        rospy.logwarn('%s %s turn timed out at same-point heading control', name, phase)
        return False

    def navigate_goal(self, name, x, y, yaw, previous=None):
        """One pose goal, guarded before dispatch and bounded by actual progress."""
        position_first = self.photo_route and name == 'POINT_5' and previous is not None
        if self.photo_route and name.startswith('POINT_'):
            try:
                from dynamic_reconfigure.client import Client
                photo_xy_tolerance = float(
                    rospy.get_param('~photo_nav_xy_tolerance', 0.03))
                photo_yaw_tolerance = float(
                    rospy.get_param('~photo_nav_heading_tolerance', 0.04))
                # P5 first navigates to recorded XY with heading
                # unconstrained, then uses the checked same-point yaw servo.
                min_vel_x = -0.08
                if name == 'POINT_5':
                    photo_xy_tolerance = min(
                        photo_xy_tolerance,
                        float(rospy.get_param('~point_5_nav_xy_tolerance', 0.025)))
                    photo_yaw_tolerance = min(
                        photo_yaw_tolerance,
                        float(rospy.get_param('~point_5_nav_heading_tolerance', 0.025)))
                if name == 'POINT_3':
                    photo_xy_tolerance = max(
                        photo_xy_tolerance,
                        float(rospy.get_param('~point_3_nav_xy_tolerance', 0.05)))
                if name == 'POINT_7':
                    min_vel_x = 0.0
                if position_first:
                    # The incoming leg is pre-aligned to its path bearing,
                    # so forward motion can handle XY without DWA reversing
                    # or trying to satisfy the camera yaw at a distance.
                    min_vel_x = 0.0
                navigation_yaw_tolerance = math.pi if position_first else photo_yaw_tolerance
                Client('/move_base/DWAPlannerROS', timeout=3.0).update_configuration({
                    'xy_goal_tolerance': photo_xy_tolerance,
                    'yaw_goal_tolerance': navigation_yaw_tolerance,
                    # Keep reverse available except for the P6->P7 short leg.
                    'min_vel_x': min_vel_x,
                    # Keep the configured minimum translational threshold.
                    # This does not ban pure rotation, so twirling_scale
                    # separately scores unnecessary spin trajectories.
                    'min_vel_trans': 0.025,
                    'twirling_scale': float(
                        rospy.get_param('~photo_twirling_scale', 0.5)),
                })
            except Exception as error:
                rospy.logerr('could not set precise photo-point tolerances for %s: %s', name, error)
                self.goal_events.append({'name': name, 'result': 'PHOTO_TOLERANCE_CONFIG_FAILED'})
                return False
        terminal = {GoalStatus.SUCCEEDED, GoalStatus.ABORTED, GoalStatus.PREEMPTED,
                    GoalStatus.REJECTED, GoalStatus.RECALLED, GoalStatus.LOST}
        for attempt in range(1, 3):
            if not self.validate_goal(name, x, y, yaw):
                self.goal_events.append({'name': name, 'attempt': attempt, 'result': 'UNSAFE_GOAL'})
                return False
            if position_first:
                try:
                    current_x, current_y, _ = self.map_pose()
                except (tf.Exception, RuntimeError) as error:
                    rospy.logerr('%s path-bearing pose unavailable: %s', name, error)
                    return False
                path_bearing = math.atan2(y - current_y, x - current_x)
                if not self.rotate_in_place_to_yaw(name, path_bearing, 'approach-bearing'):
                    self.goal_events.append({'name': name, 'attempt': attempt,
                                             'result': 'APPROACH_BEARING_FAILED'})
                    return False
            start = time.monotonic()
            last_progress = start
            try:
                last_pose = self.progress_pose()
            except (tf.Exception, RuntimeError):
                last_pose = None
            self.client.send_goal(self.goal(x, y, yaw))
            reason = 'TIMEOUT'
            while not rospy.is_shutdown() and time.monotonic() - start < self.timeout:
                state = self.client.get_state()
                if state in terminal:
                    reason = 'ACTION_RESULT'
                    break
                try:
                    # AMCL's map->odom update can lag a scan while the base is
                    # visibly moving. Use encoder odometry for motion progress
                    # so a fresh, advancing wheel pose cannot be misreported
                    # as NO_PROGRESS merely because map TF is late.
                    pose = self.progress_pose()
                    if last_pose is None or math.hypot(pose[0]-last_pose[0], pose[1]-last_pose[1]) > 0.025 or abs(math.atan2(math.sin(pose[2]-last_pose[2]), math.cos(pose[2]-last_pose[2]))) > 0.06:
                        last_pose, last_progress = pose, time.monotonic()
                except (tf.Exception, RuntimeError):
                    pass
                if time.monotonic() - last_progress > float(
                        rospy.get_param('~no_progress_timeout', 20.0)):
                    reason = 'NO_PROGRESS'
                    break
                time.sleep(0.1)
            state = self.client.get_state()
            self.goal_events.append({'name': name, 'attempt': attempt,
                                     'duration_s': round(time.monotonic()-start, 2),
                                     'action_state': state, 'result': reason})
            if state == GoalStatus.SUCCEEDED:
                if position_first and not self.rotate_in_place_to_yaw(
                        name, yaw, 'camera-heading'):
                    self.goal_events.append({'name': name, 'result': 'CAMERA_HEADING_FAILED'})
                    return False
                return True
            self.client.cancel_goal()
            self.client.wait_for_result(rospy.Duration(2.0))
            self.cmd_pub.publish(Twist())
            rospy.logwarn('goal %s failed: attempt=%d state=%d reason=%s', name, attempt, state, reason)
            if attempt == 1 and not rospy.is_shutdown():
                try:
                    rospy.wait_for_service('/move_base/clear_costmaps', timeout=2.0)
                    rospy.ServiceProxy('/move_base/clear_costmaps', Empty)()
                except (rospy.ROSException, rospy.ServiceException):
                    pass
                time.sleep(1.0)
        return False

    def run(self):
        self.status_pub.publish('WAIT_MOVE_BASE')
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
        if self.photo_route and not rospy.get_param('~park_only', False):
            if not self.return_home(
                    (previous[1], previous[2]) if previous is not None else None):
                return False
        return self.precise_park()

    def return_home(self, last_photo_position=None):
        """Let the collision-checked planner return to the recorded start pose."""
        self.status_pub.publish('RETURN_HOME')
        # Keep the long return easy for DWA; precise_park tightens the goal only
        # after the robot is near HOME.
        try:
            from dynamic_reconfigure.client import Client
            Client('/move_base/DWAPlannerROS', timeout=3.0).update_configuration(
                {'xy_goal_tolerance': 0.05, 'yaw_goal_tolerance': 0.10,
                 'min_vel_x': -0.08, 'min_vel_trans': 0.025})
        except Exception as error:
            rospy.logerr('could not set return-home tolerances: %s', error)
            return False
        return self.navigate_goal('HOME', self.birth_x, self.birth_y, self.birth_yaw)

    def map_pose(self):
        stamp = self.tf_listener.getLatestCommonTime('map', 'base_footprint')
        if (rospy.Time.now() - stamp).to_sec() > 0.5:
            raise RuntimeError('stale map pose')
        (x, y, _), q = self.tf_listener.lookupTransform('map', 'base_footprint', stamp)
        yaw = math.atan2(2 * (q[3] * q[2] + q[0] * q[1]),
                         1 - 2 * (q[1] ** 2 + q[2] ** 2))
        return x, y, yaw

    def progress_pose(self):
        """Return fresh wheel odometry for motion progress, in its own frame."""
        message = self.latest_odom
        if message is None or (rospy.Time.now() - message.header.stamp).to_sec() > 1.0:
            return self.map_pose()
        q = message.pose.pose.orientation
        yaw = math.atan2(2 * (q.w * q.z + q.x * q.y),
                         1 - 2 * (q.y ** 2 + q.z ** 2))
        return (message.pose.pose.position.x, message.pose.pose.position.y, yaw)

    def precise_park(self):
        """Validate parking from AMCL after move_base's final pose controller."""
        for attempt in range(2):
            self.cmd_pub.publish(Twist())
            rospy.sleep(0.5)
            try:
                x, y, yaw = self.map_pose()
            except (tf.Exception, RuntimeError) as error:
                rospy.logerr('parking localization unavailable: %s', error)
                return False
            distance = math.hypot(self.birth_x - x, self.birth_y - y)
            yaw_error = math.atan2(math.sin(self.birth_yaw - yaw),
                                   math.cos(self.birth_yaw - yaw))
            self.parking = {'frame': 'map', 'source': 'AMCL', 'x': x, 'y': y,
                            'yaw': yaw, 'position_error_m': distance,
                            'heading_error_rad': abs(yaw_error)}
            if distance <= 0.06 and abs(yaw_error) <= 0.06:
                self.status_pub.publish('COMPLETE_PARKED')
                rospy.loginfo('parking verified by AMCL: %.3f m / %.3f rad',
                              distance, abs(yaw_error))
                return True
            if attempt == 0:
                try:
                    from dynamic_reconfigure.client import Client
                    Client('/move_base/DWAPlannerROS', timeout=3.0).update_configuration(
                        {'xy_goal_tolerance': 0.03, 'yaw_goal_tolerance': 0.04})
                except Exception as error:
                    rospy.logerr('could not set final parking tolerances: %s', error)
                    break
                rospy.loginfo('parking needs final alignment: %.3f m / %.3f rad',
                              distance, abs(yaw_error))
                if not self.navigate_goal('HOME', self.birth_x, self.birth_y,
                                          self.birth_yaw):
                    return False
        self.status_pub.publish('FAILED:PARK_POSE')
        rospy.logerr('parking pose mismatch: %.3f m / %.3f rad',
                     self.parking['position_error_m'],
                     self.parking['heading_error_rad'])
        return False

    def finish(self, success):
        import os
        self.client.cancel_all_goals()
        self.cmd_pub.publish(Twist())
        if self.capture_photos:
            os.makedirs(self.photo_dir, exist_ok=True)
            report = {'route_status': 'COMPLETE_PARKED' if success else 'FAILED',
                      'localization': 'wheel_encoders+AMCL',
                      'map_file': rospy.get_param('/map_server/map_file', ''),
                      'goals': self.goal_events, 'parking': getattr(self, 'parking', None),
                      'photo_points': [name for name, *_ in self.route],
                      'completed_photos': sorted(os.listdir(self.photo_dir))}
            with open(os.path.join(self.photo_dir, 'run_summary.json'), 'w') as stream:
                json.dump(report, stream, indent=2)


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
