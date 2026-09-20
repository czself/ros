#!/usr/bin/env python3
"""Low-speed, laser-protected waypoint survey for the accessible upper course."""
import math

import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan


ROUTES = {
    'upper_left': [
        (2.40, 2.40), (-2.40, 2.40), (-2.40, -1.62),
        (-2.24, -1.62), (-2.24, 2.35), (-1.72, 2.35),
        (-1.72, 2.40), (2.35, 2.40),
    ],
    'right_corridor': [
        (2.40, 2.40), (2.40, -1.62), (2.35, -1.62), (2.35, 2.40),
    ],
}
LINEAR_SPEED = 0.08
MAX_ANGULAR_SPEED = 0.30
GOAL_TOLERANCE = 0.10
FRONT_STOP_DISTANCE = 0.32
SETTLE_TIME = 1.0


def clamp(value, lower, upper):
    return max(lower, min(upper, value))


def normalize_angle(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


class WaypointMapper:
    def __init__(self):
        self.pose = None
        self.scan = None
        self.index = 0
        self.settle_until = None
        self.stopped = False
        route_name = rospy.get_param('~route', 'upper_left')
        self.waypoints = ROUTES.get(route_name)
        if self.waypoints is None:
            raise ValueError('Unknown survey route: {}'.format(route_name))
        self.pub = rospy.Publisher('/my_car/cmd_vel', Twist, queue_size=1)
        rospy.Subscriber('/my_car/odom', Odometry, self.on_odom, queue_size=1)
        rospy.Subscriber('/scan', LaserScan, self.on_scan, queue_size=1)

    def on_odom(self, odom):
        q = odom.pose.pose.orientation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        p = odom.pose.pose.position
        self.pose = (p.x, p.y, yaw)

    def on_scan(self, scan):
        self.scan = scan

    def front_distance(self):
        if self.scan is None:
            return float('inf')
        distances = []
        for index, distance in enumerate(self.scan.ranges):
            angle = self.scan.angle_min + index * self.scan.angle_increment
            if abs(angle) <= math.radians(20) and self.scan.range_min <= distance <= self.scan.range_max:
                distances.append(distance)
        return min(distances) if distances else float('inf')

    def stop(self, reason):
        if not self.stopped:
            rospy.logwarn(reason)
        self.stopped = True
        self.pub.publish(Twist())

    def run(self):
        rate = rospy.Rate(15)
        while not rospy.is_shutdown() and not self.stopped:
            if self.pose is None or self.scan is None:
                self.pub.publish(Twist())
                rate.sleep()
                continue

            if self.index >= len(self.waypoints):
                self.stop('Survey route complete.')
                break

            now = rospy.Time.now().to_sec()
            if self.settle_until is not None:
                self.pub.publish(Twist())
                if now >= self.settle_until:
                    self.settle_until = None
                    self.index += 1
                rate.sleep()
                continue

            x, y, yaw = self.pose
            target_x, target_y = self.waypoints[self.index]
            dx, dy = target_x - x, target_y - y
            distance = math.hypot(dx, dy)
            if distance < GOAL_TOLERANCE:
                rospy.loginfo('Reached waypoint %d/%d.', self.index + 1, len(self.waypoints))
                self.settle_until = now + SETTLE_TIME
                self.pub.publish(Twist())
                rate.sleep()
                continue

            heading_error = normalize_angle(math.atan2(dy, dx) - yaw)
            command = Twist()
            if abs(heading_error) > 0.22:
                command.angular.z = clamp(1.2 * heading_error, -MAX_ANGULAR_SPEED, MAX_ANGULAR_SPEED)
            elif self.front_distance() < FRONT_STOP_DISTANCE:
                self.stop('Obstacle is too close; survey stopped safely.')
                break
            else:
                command.linear.x = LINEAR_SPEED * max(0.35, math.cos(heading_error))
                command.angular.z = clamp(0.8 * heading_error, -MAX_ANGULAR_SPEED, MAX_ANGULAR_SPEED)
            self.pub.publish(command)
            rate.sleep()

        self.pub.publish(Twist())


if __name__ == '__main__':
    rospy.init_node('waypoint_mapper')
    try:
        WaypointMapper().run()
    finally:
        rospy.Publisher('/my_car/cmd_vel', Twist, queue_size=1).publish(Twist())
