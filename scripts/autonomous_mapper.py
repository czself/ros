#!/usr/bin/env python3
"""Low-speed reactive wall follower for collecting a Gazebo SLAM map."""
import math

import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan


class WallFollower:
    def __init__(self):
        self.scan = None
        self.pub = rospy.Publisher('/my_car/cmd_vel', Twist, queue_size=1)
        rospy.Subscriber('/scan', LaserScan, self.on_scan, queue_size=1)
        self.duration = rospy.get_param('~duration', 300.0)
        self.started_at = rospy.Time.now()

    def on_scan(self, scan):
        self.scan = scan

    @staticmethod
    def sector_distance(scan, lower_degrees, upper_degrees):
        values = []
        lower = math.radians(lower_degrees)
        upper = math.radians(upper_degrees)
        for index, distance in enumerate(scan.ranges):
            angle = scan.angle_min + index * scan.angle_increment
            if lower <= angle <= upper and math.isfinite(distance):
                values.append(distance)
        return min(values) if values else scan.range_max

    def command(self):
        cmd = Twist()
        if self.scan is None:
            return cmd

        front = self.sector_distance(self.scan, -25, 25)
        right = self.sector_distance(self.scan, -110, -70)

        if front < 0.42:
            cmd.angular.z = 0.85  # Turn left away from an obstacle.
        elif right < 0.30:
            cmd.linear.x = 0.16
            cmd.angular.z = 0.48
        elif right > 0.62:
            cmd.linear.x = 0.18
            cmd.angular.z = -0.42
        else:
            cmd.linear.x = 0.22
            cmd.angular.z = (0.45 - right) * 1.15
        return cmd

    def run(self):
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            elapsed = (rospy.Time.now() - self.started_at).to_sec()
            if elapsed >= self.duration:
                break
            self.pub.publish(self.command())
            rate.sleep()
        self.pub.publish(Twist())


if __name__ == '__main__':
    rospy.init_node('autonomous_mapper')
    try:
        WallFollower().run()
    finally:
        rospy.Publisher('/my_car/cmd_vel', Twist, queue_size=1).publish(Twist())
