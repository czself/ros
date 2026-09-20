#!/usr/bin/env python3
"""用于精细建图的脉冲式键盘遥控。"""
import select
import sys
import termios
import time
import tty
import rospy
from geometry_msgs.msg import Twist

LINEAR = 0.10   # m/s
ANGULAR = 0.42  # rad/s
LINEAR_PULSE = 0.30   # 约 3 cm
ANGULAR_PULSE = 0.30  # 约 7 度

KEYMAP = {
    'w': (LINEAR, 0.0), 's': (-LINEAR, 0.0),
    'a': (0.0, ANGULAR), 'd': (0.0, -ANGULAR),
    ' ': (0.0, 0.0),
}

def main():
    rospy.init_node('car_teleop')
    pub = rospy.Publisher('/my_car/cmd_vel', Twist, queue_size=1)
    print(
        "建图遥控: 轻按 w/s 前进后退约 3cm, 轻按 a/d 转向约 7度, "
        "连续按键可连续移动, 空格急停, q退出"
    )
    fd = sys.stdin.fileno()
    old = termios.tcgetattr(fd)
    command = Twist()
    command_until = 0.0
    try:
        tty.setraw(fd)
        while not rospy.is_shutdown():
            ready, _, _ = select.select([sys.stdin], [], [], 0.02)
            if ready:
                ch = sys.stdin.read(1)
                if ch == 'q':
                    break
                if ch in KEYMAP:
                    command = Twist()
                    command.linear.x, command.angular.z = KEYMAP[ch]
                    duration = LINEAR_PULSE if ch in 'ws' else ANGULAR_PULSE
                    if ch == ' ':
                        duration = 0.0
                    command_until = time.monotonic() + duration

            if time.monotonic() < command_until:
                pub.publish(command)
            else:
                pub.publish(Twist())
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old)
        pub.publish(Twist())  # 退出前停车
        print("\n已退出并停车")

if __name__ == '__main__':
    main()
