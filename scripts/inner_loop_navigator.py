#!/usr/bin/env python3
"""Sensor-guided inner-loop navigator for the competition drawing.

The task route is a closed inner loop: begin at the fixed birth pose, follow
the inner corridor, take the next left branch at each of five intersections,
then stop back at the birth pose.  No world-coordinate turn points are used.
Laser and depth are used to decide whether an opening is a junction and to
fail safe if the path ahead is occupied.
"""
import json
import math
import time

import rospy
from gazebo_msgs.msg import ModelStates
from geometry_msgs.msg import Twist
from sensor_msgs import point_cloud2
from sensor_msgs.msg import LaserScan, PointCloud2
from std_msgs.msg import String

BIRTH = (1.714860, -1.599947, 1.606236)
TURN_COUNT = 5
CRUISE = 0.16
TURN_SPEED = 0.65
FRONT_STOP = 0.28
LEFT_OPEN = 0.72
MIN_BETWEEN_TURNS = 0.42
MAX_RUN_SECONDS = 240.0


def wrap(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


class InnerLoopNavigator:
    def __init__(self):
        self.pose = None
        self.scan = None
        self.depth_front = float('inf')
        self.turns = int(rospy.get_param('~initial_turns', 0))
        self.turning = False
        self.turn_yaw = 0.0
        self.last_turn_pose = None
        self.started = time.monotonic()
        self.history = []
        self.pub = rospy.Publisher('/my_car/cmd_vel_nav', Twist, queue_size=1)
        self.status = rospy.Publisher('/inner_loop/status', String, queue_size=1,
                                      latch=True)
        rospy.Subscriber('/gazebo/model_states', ModelStates, self.pose_cb, queue_size=1)
        rospy.Subscriber('/scan', LaserScan, self.scan_cb, queue_size=1)
        rospy.Subscriber('/camera/depth/points', PointCloud2, self.depth_cb, queue_size=1)

    def pose_cb(self, message):
        try:
            i = message.name.index('my_car')
        except ValueError:
            return
        pose = message.pose[i]
        q = pose.orientation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                         1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        self.pose = (pose.position.x, pose.position.y, yaw)

    def scan_cb(self, message):
        self.scan = message

    def depth_cb(self, message):
        # Sparse samples down the optical centre; depth is a second, independent
        # forward-obstacle veto rather than a source of fixed turn coordinates.
        samples = []
        try:
            for u in range(max(0, message.width // 2 - 40),
                           min(message.width, message.width // 2 + 41), 10):
                for v in range(max(0, message.height // 2 - 20),
                               min(message.height, message.height // 2 + 21), 10):
                    point = next(point_cloud2.read_points(
                        message, field_names=('x', 'y', 'z'), skip_nans=True,
                        uvs=[(u, v)]), None)
                    if point and 0.05 < point[2] < 4.0:
                        samples.append(point[2])
        except Exception:
            return
        if samples:
            samples.sort()
            self.depth_front = samples[len(samples) // 2]

    @staticmethod
    def sector(scan, centre, half_width):
        values = []
        for i, value in enumerate(scan.ranges):
            angle = scan.angle_min + i * scan.angle_increment
            if abs(wrap(angle - centre)) <= half_width and math.isfinite(value):
                if scan.range_min <= value <= scan.range_max:
                    values.append(value)
        if not values:
            return float('inf')
        values.sort()
        # A high percentile sees the opening at a junction even when one edge
        # of the sector still contains the adjacent wall.
        return values[int(0.75 * (len(values) - 1))]

    def enough_distance_since_turn(self):
        if self.last_turn_pose is None or self.pose is None:
            return True
        return math.hypot(self.pose[0] - self.last_turn_pose[0],
                          self.pose[1] - self.last_turn_pose[1]) >= MIN_BETWEEN_TURNS

    def publish(self, linear=0.0, angular=0.0, state=''):
        command = Twist()
        command.linear.x = linear
        command.angular.z = angular
        self.pub.publish(command)
        self.status.publish(state)

    def finish(self, result):
        self.publish(0.0, 0.0, result)
        record = {'result': result, 'turns': self.turns, 'history': self.history,
                  'final_pose': list(self.pose) if self.pose else None}
        with open('/root/inner_loop_evidence.json', 'w') as stream:
            json.dump(record, stream, indent=2)
        rospy.signal_shutdown(result)

    def tick(self, _event):
        if self.pose is None or self.scan is None:
            self.publish(0.0, 0.0, 'WAIT_SENSORS')
            return
        if time.monotonic() - self.started > MAX_RUN_SECONDS:
            self.finish('FAILED:TIMEOUT')
            return
        front = self.sector(self.scan, 0.0, math.radians(15))
        left = self.sector(self.scan, math.pi / 2.0, math.radians(22))
        if self.turning:
            turned = wrap(self.pose[2] - self.turn_yaw)
            if abs(turned) >= math.radians(82):
                self.turns += 1
                self.turning = False
                self.last_turn_pose = self.pose
                self.history.append({'turn': self.turns, 'pose': list(self.pose)})
                self.publish(0.0, 0.0, 'TURN_%d_DONE' % self.turns)
                return
            self.publish(0.0, TURN_SPEED, 'TURN_%d' % (self.turns + 1))
            return

        distance_home = math.hypot(self.pose[0] - BIRTH[0], self.pose[1] - BIRTH[1])
        if self.turns >= TURN_COUNT and distance_home < 0.20:
            self.finish('COMPLETE')
            return
        # The inside branch must be genuinely open in the laser scan.  Depth
        # only vetoes driving into a near object; it cannot create a junction.
        # A wide side opening announces an intersection in advance.  At the
        # compact inner corners the opening becomes visible only when the
        # forward wall is close; that is still a valid junction, not a reason
        # to remain stopped forever.
        left_branch = left > LEFT_OPEN or (front < FRONT_STOP and left > 0.38)
        if (self.turns < TURN_COUNT and left_branch
                and self.enough_distance_since_turn()):
            self.turning = True
            self.turn_yaw = self.pose[2]
            self.publish(0.0, 0.0, 'TURN_%d_ARMED' % (self.turns + 1))
            return
        if front < FRONT_STOP or self.depth_front < FRONT_STOP:
            self.publish(0.0, 0.0, 'WAIT_CLEAR:laser=%.2f:left=%.2f:depth=%.2f' %
                         (front, left, self.depth_front))
            return
        self.publish(CRUISE, 0.0, 'FOLLOW:%d:left=%.2f' % (self.turns, left))


if __name__ == '__main__':
    raise SystemExit('Retired: laser openings alone do not identify the required painted route. Use start_calibrated_navigation.sh.')
