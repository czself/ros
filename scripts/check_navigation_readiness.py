#!/usr/bin/env python3
"""Fail startup if Gazebo sensor and encoder timestamps are misaligned."""

import time

import actionlib
import rospy
import tf
from move_base_msgs.msg import MoveBaseAction
from nav_msgs.msg import Odometry
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import LaserScan


def main():
    rospy.init_node('check_navigation_readiness', anonymous=True)
    listener = tf.TransformListener()
    deadline = time.monotonic() + 25.0
    last_error = 'waiting for simulation data'
    while not rospy.is_shutdown() and time.monotonic() < deadline:
        try:
            rospy.wait_for_message('/clock', Clock, timeout=2.0)
            scan = rospy.wait_for_message('/scan', LaserScan, timeout=2.0)
            wheel = rospy.wait_for_message('/my_car/wheel_odom', Odometry, timeout=2.0)
            odom = rospy.wait_for_message('/odom', Odometry, timeout=2.0)
            stamps = [scan.header.stamp.to_sec(), wheel.header.stamp.to_sec(),
                      odom.header.stamp.to_sec()]
            sim_now = rospy.Time.now().to_sec()
            if max(stamps) - min(stamps) > 0.5:
                raise RuntimeError(
                    'laser and wheel odometry timestamps do not share one clock: '
                    'scan=%.3f wheel=%.3f odom=%.3f' % tuple(stamps))
            if any(abs(stamp - sim_now) > 1.0 for stamp in stamps):
                raise RuntimeError(
                    'sensor timestamps differ from /clock: now=%.3f scan=%.3f '
                    'wheel=%.3f odom=%.3f' % (sim_now, *stamps))
            if not listener.canTransform('map', 'base_footprint', rospy.Time(0)):
                raise RuntimeError('AMCL has not published map -> base_footprint')
            client = actionlib.SimpleActionClient('/move_base', MoveBaseAction)
            if not client.wait_for_server(rospy.Duration(1.0)):
                raise RuntimeError('move_base action server is not ready')
            print('Navigation ready: sim time, laser, wheel odometry, AMCL TF, and move_base agree.',
                  flush=True)
            return 0
        except Exception as error:
            last_error = str(error)
            time.sleep(0.2)
    raise SystemExit('Navigation readiness check failed: ' + last_error)


if __name__ == '__main__':
    main()
