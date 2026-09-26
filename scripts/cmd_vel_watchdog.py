#!/usr/bin/env python3
"""Final velocity safety gate for navigation and traffic-light compliance.

The node remains the sole navigation-mode publisher to Gazebo.  In addition
to the dead-man timeout it enforces both competition stop lines using the
AMCL map-to-chassis pose. RED, YELLOW, stale perception, or insufficient GREEN
time all stop the complete vehicle before its front edge reaches a line.
"""

import copy
import math
import threading
import time

import cv2
import numpy as np

import rospy
import tf
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool, Float32, String
from tf.transformations import euler_from_quaternion


# name, travel axis, travel direction, line coordinate, lateral center/half-width
# Coordinates were measured from the final 4.2 m competition-ground texture.
STOP_LINES = (
    ('WESTBOUND', 'x', -1.0, -0.515, 0.00, 0.38),
    ('NORTHBOUND', 'y', 1.0, -0.290, 1.70, 0.32),
)
FRONT_HALF_LENGTH = 0.090
REAR_HALF_LENGTH = 0.090
BRAKE_MARGIN = 0.100
SLOW_APPROACH_DISTANCE = 0.35
LINE_CLEARANCE = 0.04
MIN_GREEN_REMAINING = 2.0
SIGNAL_STALE_SECONDS = 0.75
POSE_STALE_SECONDS = 0.5
DRIVE_AXLE_OFFSET = 0.0525

# The competition-ground texture is the authoritative drawing for lane lines
# and zebra crossings.  A non-white sample means drivable pavement; every
# white sample is prohibited unless it falls in a stop-line exception that has
# been admitted on a stable GREEN.  Coordinates use the texture/world mapping
# recorded in task001/artifacts/stop_line_measurements.yaml.
GROUND_TEXTURE_PATH = '/root/competition_ground_map.png'
GROUND_SIZE_METERS = 4.2
WHITE_PIXEL_THRESHOLD = 220
# Chassis-origin footprint of the actual embedded competition-world car:
# 0.168 m body length and 0.1484 m wheel-to-wheel outside width, with a
# 1 cm buffer.  This intentionally differs from the base_footprint polygon
# in common_costmap.yaml because this guard checks the chassis frame.
FOOTPRINT = ((0.094, 0.085), (0.094, -0.085), (-0.094, -0.085), (-0.094, 0.085))
FOOTPRINT_SAMPLE_STEP = 0.012
WHITE_LINE_PREDICTION_SECONDS = 0.35
WHITE_LINE_PREDICTION_STEP = 0.025
STOP_LINE_HALF_THICKNESS = 0.055
# (row_min, row_max, col_min, col_max), measured from the authoritative
# 11,942 px ground texture.  The boxes bound the two disconnected zebra
# patterns only; ordinary lane boundaries and the outer white geometry are
# intentionally absent and therefore remain forbidden at every signal phase.
ZEBRA_PIXEL_BOXES = (
    (388, 1934, 6138, 6706),
    (7543, 8111, 4898, 6444),
)
# The painted line directly ahead of the declared birth pose is the start
# line, not a traffic-controlled stop line.  It is always traversable, but
# only inside its measured right-lane span.
START_LINE = ('y', -1.400, 1.700, 0.32, 0.055)


class CmdVelWatchdog:
    def __init__(self):
        self.enforce_traffic = rospy.get_param('~enforce_traffic', True)
        self.enforce_white_lines = rospy.get_param('~enforce_white_lines', True)
        self.timeout = max(0.15, float(rospy.get_param('~timeout', 0.45)))
        self.lock = threading.Lock()
        self.latest = Twist()
        self.last_wall = 0.0
        self.received = False
        self.pose = None
        self.pose_wall = 0.0
        self.tf_listener = tf.TransformListener()
        self.detected = 'NO_TRAFFIC_LIGHT'
        self.detected_count = 0
        self.detected_wall = 0.0
        self.sim_state = 'UNKNOWN'
        self.green_remaining = 0.0
        self.committed = {name: False for name, *_ in STOP_LINES}
        self.passed = {name: False for name, *_ in STOP_LINES}
        self.last_gate_status = None
        self.white_map = self.load_white_map(
            rospy.get_param('~ground_texture', GROUND_TEXTURE_PATH)
        )

        self.pub = rospy.Publisher('/my_car/cmd_vel', Twist, queue_size=1)
        self.gate_pub = rospy.Publisher(
            '/traffic_light/gate_status', String, queue_size=1, latch=True
        )
        self.braking_pub = rospy.Publisher(
            '/traffic_light/braking', Bool, queue_size=1, latch=True
        )
        rospy.Subscriber('/my_car/cmd_vel_nav', Twist, self.command_cb, queue_size=1)
        rospy.Subscriber(
            '/inspection/traffic_light', String, self.detection_cb, queue_size=1
        )
        # Simulation state is a fail-safe cross-check.  A visual GREEN is still
        # mandatory; this topic can veto an unsafe false-positive but cannot GO.
        rospy.Subscriber('/traffic_light/state', String, self.sim_state_cb, queue_size=1)
        rospy.Subscriber(
            '/traffic_light/time_remaining', Float32, self.remaining_cb, queue_size=1
        )
        self.timer = rospy.Timer(rospy.Duration(0.05), self.tick)
        self.publish_status('CLEAR')

    @staticmethod
    def zero():
        return Twist()

    @staticmethod
    def stop_translation(command):
        """Stop forward motion but keep a safe in-place alignment turn."""
        stopped = copy.deepcopy(command)
        if stopped.linear.x > 0.0:
            stopped.linear.x = 0.0
        return stopped

    @staticmethod
    def load_white_map(path):
        image = cv2.imread(path, cv2.IMREAD_GRAYSCALE)
        if image is None:
            rospy.logerr('white-line gate cannot load ground texture: %s', path)
            return None
        if image.shape[0] != image.shape[1]:
            rospy.logerr('white-line gate requires a square texture, got %s', image.shape)
            return None
        return image >= WHITE_PIXEL_THRESHOLD

    @staticmethod
    def body_samples():
        """Return perimeter and interior samples of the complete footprint."""
        min_x = min(point[0] for point in FOOTPRINT)
        max_x = max(point[0] for point in FOOTPRINT)
        min_y = min(point[1] for point in FOOTPRINT)
        max_y = max(point[1] for point in FOOTPRINT)
        samples = []
        x = min_x
        while x <= max_x + 1e-9:
            y = min_y
            while y <= max_y + 1e-9:
                samples.append((x, y))
                y += FOOTPRINT_SAMPLE_STEP
            x += FOOTPRINT_SAMPLE_STEP
        return samples

    @staticmethod
    def map_pixel(x, y, size):
        # Ground UV reverses texture rows along world x; texture columns map
        # to negative world y.
        row = int((0.5 - x / GROUND_SIZE_METERS) * size)
        col = int((0.5 - y / GROUND_SIZE_METERS) * size)
        return row, col

    def permitted_stop_line_pixel(self, x, y):
        """A stop-line pixel is legal only after this gate admitted GREEN."""
        for name, axis, _direction, line, lateral_center, lateral_half in STOP_LINES:
            coordinate = x if axis == 'x' else y
            lateral = y if axis == 'x' else x
            if ((not self.enforce_traffic or self.committed[name])
                    and abs(coordinate - line) <= STOP_LINE_HALF_THICKNESS
                    and abs(lateral - lateral_center) <= lateral_half):
                return True
        return False

    @staticmethod
    def zebra_pixel(row, col):
        return any(
            row_min <= row <= row_max and col_min <= col <= col_max
            for row_min, row_max, col_min, col_max in ZEBRA_PIXEL_BOXES
        )

    def permitted_white_pixel(self, x, y, row, col, now):
        # The traffic-light controller synchronizes both signals.  During its
        # stable GREEN phase a full vehicle may traverse either painted zebra
        # crossing.  At RED/YELLOW/stale detection, zebra paint remains a hard
        # boundary just like every other white marking.
        if self.zebra_pixel(row, col) and (not self.enforce_traffic or self.stable_green(now)):
            return True
        axis, line, lateral_center, lateral_half, half_thickness = START_LINE
        coordinate = x if axis == 'x' else y
        lateral = y if axis == 'x' else x
        if (abs(coordinate - line) <= half_thickness
                and abs(lateral - lateral_center) <= lateral_half):
            return True
        return self.permitted_stop_line_pixel(x, y)

    def footprint_white_status(self, x, y, yaw, now):
        """Return a violation for one complete chassis footprint pose."""
        assert self.white_map is not None
        cos_yaw, sin_yaw = math.cos(yaw), math.sin(yaw)
        height, width = self.white_map.shape
        vertices = []
        for local_x, local_y in FOOTPRINT:
            world_x = x + local_x * cos_yaw - local_y * sin_yaw
            world_y = y + local_x * sin_yaw + local_y * cos_yaw
            row = (0.5 - world_x / GROUND_SIZE_METERS) * height
            col = (0.5 - world_y / GROUND_SIZE_METERS) * width
            if row < 0 or row >= height or col < 0 or col >= width:
                return 'WHITE_LINE:OUT_OF_BOUNDS'
            vertices.append((col, row))
        # Fill every covered texture pixel, including all four exact edges.
        # One-pixel dilation conservatively covers subpixel polygon rounding.
        vertices = np.asarray(vertices)
        c0, r0 = np.maximum(0, np.floor(vertices.min(axis=0)).astype(int) - 1)
        c1, r1 = np.minimum([width, height], np.ceil(vertices.max(axis=0)).astype(int) + 2)
        paint = self.white_map[r0:r1, c0:c1]
        if not paint.any():
            return None
        mask = np.zeros(paint.shape, dtype=np.uint8)
        polygon = np.rint(vertices - [c0, r0]).astype(np.int32)
        cv2.fillPoly(mask, [polygon], 1)
        mask = cv2.dilate(mask, np.ones((3, 3), dtype=np.uint8))
        rows, cols = np.nonzero(paint & (mask != 0))
        if not self.enforce_traffic:
            # Crossing footprints can cover hundreds of thousands of white
            # pixels. Evaluate the same bounded exceptions in vector form.
            rows, cols = rows + r0, cols + c0
            xs = (0.5 - (rows + 0.5) / height) * GROUND_SIZE_METERS
            ys = (0.5 - (cols + 0.5) / width) * GROUND_SIZE_METERS
            permitted = np.zeros(rows.shape, dtype=bool)
            for z_r0, z_r1, z_c0, z_c1 in ZEBRA_PIXEL_BOXES:
                permitted |= ((rows >= z_r0) & (rows <= z_r1) &
                              (cols >= z_c0) & (cols <= z_c1))
            lines = [START_LINE] + [
                (axis, line, center, half, STOP_LINE_HALF_THICKNESS)
                for _name, axis, _direction, line, center, half in STOP_LINES]
            for axis, line, center, half, thickness in lines:
                coordinates, lateral = (xs, ys) if axis == 'x' else (ys, xs)
                permitted |= ((np.abs(coordinates - line) <= thickness) &
                              (np.abs(lateral - center) <= half))
            return 'WHITE_LINE:BLOCKED' if np.any(~permitted) else None
        for local_row, local_col in zip(rows, cols):
            row, col = int(local_row + r0), int(local_col + c0)
            world_x = (0.5 - (row + 0.5) / height) * GROUND_SIZE_METERS
            world_y = (0.5 - (col + 0.5) / width) * GROUND_SIZE_METERS
            if not self.permitted_white_pixel(world_x, world_y, row, col, now):
                return 'WHITE_LINE:BLOCKED'
        return None

    @staticmethod
    def predicted_pose(x, y, yaw, linear, angular, duration):
        """Integrate commanded axle twist and return the chassis origin."""
        axle_x = x + DRIVE_AXLE_OFFSET * math.cos(yaw)
        axle_y = y + DRIVE_AXLE_OFFSET * math.sin(yaw)
        future_yaw = yaw + angular * duration
        if abs(angular) < 1e-9:
            axle_x += linear * duration * math.cos(yaw)
            axle_y += linear * duration * math.sin(yaw)
        else:
            axle_x += linear / angular * (math.sin(future_yaw) - math.sin(yaw))
            axle_y -= linear / angular * (math.cos(future_yaw) - math.cos(yaw))
        return (axle_x - DRIVE_AXLE_OFFSET * math.cos(future_yaw),
                axle_y - DRIVE_AXLE_OFFSET * math.sin(future_yaw), future_yaw)

    def white_line_status(self, now, command):
        if self.white_map is None:
            return 'WHITE_LINE:MAP_UNAVAILABLE'
        if self.pose is None:
            return 'WHITE_LINE:POSE_UNAVAILABLE'
        if now - self.pose_wall > POSE_STALE_SECONDS:
            return 'WHITE_LINE:POSE_STALE'
        x, y, yaw = self.pose
        current = self.footprint_white_status(x, y, yaw, now)
        if current:
            return current

        # Test the swept footprint over a braking horizon before forwarding a
        # command.  This is deliberately independent of DWA: even a planner
        # that proposes a short corner-cut cannot make the body touch paint.
        if abs(command.linear.x) < 1e-4 and abs(command.angular.z) < 1e-4:
            return None
        t = WHITE_LINE_PREDICTION_STEP
        while t <= WHITE_LINE_PREDICTION_SECONDS + 1e-9:
            future_x, future_y, future_yaw = self.predicted_pose(
                x, y, yaw, command.linear.x, command.angular.z, t)
            if self.footprint_white_status(future_x, future_y, future_yaw, now):
                return 'WHITE_LINE:APPROACH'
            t += WHITE_LINE_PREDICTION_STEP
        return None

    def command_cb(self, message):
        with self.lock:
            self.latest = message
            self.last_wall = time.monotonic()
            self.received = True

    def update_pose(self):
        try:
            stamp = self.tf_listener.getLatestCommonTime('map', 'chassis')
            if (rospy.Time.now() - stamp).to_sec() > POSE_STALE_SECONDS:
                return
            position, quaternion = self.tf_listener.lookupTransform('map', 'chassis', stamp)
        except (tf.Exception, tf.LookupException, tf.ConnectivityException):
            return
        yaw = euler_from_quaternion(quaternion)[2]
        with self.lock:
            self.pose = (position[0], position[1], yaw)
            self.pose_wall = time.monotonic()

    def detection_cb(self, message):
        state = message.data.upper()
        with self.lock:
            if state == self.detected:
                self.detected_count += 1
            else:
                self.detected = state
                self.detected_count = 1
            self.detected_wall = time.monotonic()

    def sim_state_cb(self, message):
        with self.lock:
            self.sim_state = message.data.upper()

    def remaining_cb(self, message):
        with self.lock:
            self.green_remaining = max(0.0, float(message.data))

    def stable_green(self, now):
        perception_fresh = now - self.detected_wall <= SIGNAL_STALE_SECONDS
        return (
            perception_fresh
            and self.detected == 'GREEN'
            and self.detected_count >= 3
            and self.sim_state == 'GREEN'
            and self.green_remaining >= MIN_GREEN_REMAINING
        )

    @staticmethod
    def approach_heading(yaw, axis, direction):
        component = math.cos(yaw) if axis == 'x' else math.sin(yaw)
        return component * direction > 0.65

    def gate_command(self, command, now):
        if self.pose is None:
            return self.zero(), 'WHITE_LINE:POSE_UNAVAILABLE'
        if now - self.pose_wall > POSE_STALE_SECONDS:
            return self.zero(), 'WHITE_LINE:POSE_STALE'
        if not self.enforce_traffic:
            return command, 'CLEAR:TRAFFIC_BYPASSED_FOR_NAV_TEST'

        x, y, yaw = self.pose
        green = self.stable_green(now)
        for name, axis, direction, line, lateral_center, lateral_half in STOP_LINES:
            coordinate = x if axis == 'x' else y
            lateral = y if axis == 'x' else x
            if abs(lateral - lateral_center) > lateral_half:
                continue
            if not self.approach_heading(yaw, axis, direction):
                continue

            progress = direction * (coordinate - line)
            front_progress = progress + FRONT_HALF_LENGTH
            rear_progress = progress - REAR_HALF_LENGTH

            # Returning to the approach side begins a new encounter.
            if front_progress < -SLOW_APPROACH_DISTANCE:
                self.committed[name] = False
                self.passed[name] = False
                continue
            if self.passed[name]:
                continue

            # A vehicle admitted on a safe GREEN must clear the line rather
            # than stop with its body covering it when YELLOW begins.
            if self.committed[name]:
                # Authorization is revocable until the front edge actually
                # crosses.  This prevents an idle vehicle from carrying a
                # stale GREEN authorization into the next RED phase.
                if front_progress < 0.0 and not green:
                    self.committed[name] = False
                elif front_progress < 0.0:
                    return command, f'{name}:GO_GREEN'
                else:
                    if rear_progress > LINE_CLEARANCE:
                        self.committed[name] = False
                        self.passed[name] = True
                        return command, f'{name}:CLEARED'
                    return command, f'{name}:CLEARING'

            if green and front_progress >= -BRAKE_MARGIN:
                if command.linear.x > 0.01:
                    self.committed[name] = True
                    return command, f'{name}:GO_GREEN'
                return command, f'{name}:READY_GREEN'

            if front_progress >= -BRAKE_MARGIN:
                reason = self.detected
                if self.sim_state in ('RED', 'YELLOW'):
                    reason = self.sim_state
                elif self.detected == 'GREEN' and self.green_remaining < MIN_GREEN_REMAINING:
                    reason = 'GREEN_ENDING'
                return self.stop_translation(command), f'{name}:WAIT_{reason}'

            # Slow the last approach so the configured 10 cm margin exceeds
            # the physical stopping distance even at the next 20 Hz tick.
            if front_progress >= -SLOW_APPROACH_DISTANCE and command.linear.x > 0.12:
                slowed = copy.deepcopy(command)
                slowed.linear.x = 0.12
                return slowed, f'{name}:APPROACH'

        return command, 'CLEAR'

    def publish_status(self, status):
        braking = ':WAIT_' in status or status.startswith('WHITE_LINE:')
        self.gate_pub.publish(status)
        self.braking_pub.publish(braking)
        if status != self.last_gate_status:
            rospy.loginfo('traffic gate: %s', status)
            self.last_gate_status = status

    def tick(self, _event):
        self.update_pose()
        now = time.monotonic()
        with self.lock:
            fresh = self.received and now - self.last_wall <= self.timeout
            command = copy.deepcopy(self.latest) if fresh else self.zero()
            command, status = self.gate_command(command, now)
            white_line_status = (self.white_line_status(now, command)
                                 if self.enforce_white_lines else None)
            if white_line_status:
                # A predicted yaw sweep can hit paint too, so a white-line
                # stop is fully stationary.  Traffic-signal stops still keep
                # their safe in-place alignment behavior above.
                command = self.zero()
                status = white_line_status
        self.pub.publish(command)
        self.publish_status(status)


if __name__ == '__main__':
    rospy.init_node('cmd_vel_watchdog')
    CmdVelWatchdog()
    rospy.spin()
