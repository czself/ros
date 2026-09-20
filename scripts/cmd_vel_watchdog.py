#!/usr/bin/env python3
"""Forward navigation velocity commands with a wall-clock dead-man switch.

Gazebo's classic diff-drive plugin keeps the last command if its publisher
dies.  That is unsafe during a move_base crash or a ROS mode transition.  The
watchdog is the sole navigation-mode publisher of ``/my_car/cmd_vel`` and
forwards ``/my_car/cmd_vel_nav`` only while fresh commands arrive.
"""

import threading
import time

import rospy
from geometry_msgs.msg import Twist


class CmdVelWatchdog:
    def __init__(self):
        self.timeout = max(0.15, float(rospy.get_param("~timeout", 0.45)))
        self.lock = threading.Lock()
        self.latest = Twist()
        self.last_wall = 0.0
        self.received = False
        self.pub = rospy.Publisher("/my_car/cmd_vel", Twist, queue_size=1)
        rospy.Subscriber("/my_car/cmd_vel_nav", Twist, self.callback, queue_size=1)
        self.timer = rospy.Timer(rospy.Duration(0.05), self.tick)

    @staticmethod
    def zero():
        return Twist()

    def callback(self, message):
        with self.lock:
            self.latest = message
            self.last_wall = time.monotonic()
            self.received = True

    def tick(self, _event):
        with self.lock:
            fresh = self.received and time.monotonic() - self.last_wall <= self.timeout
            message = self.latest if fresh else self.zero()
        self.pub.publish(message)


if __name__ == "__main__":
    rospy.init_node("cmd_vel_watchdog")
    CmdVelWatchdog()
    rospy.spin()
