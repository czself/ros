#!/usr/bin/env python3
"""Long-corner navigation test: start -> far goal, report success or stall context."""

import math

import numpy as np
import rospy
import tf2_ros
from actionlib_msgs.msg import GoalID, GoalStatusArray
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import OccupancyGrid

GOAL = (0.825, 0.375)
WATCH = 260.0


def main():
    rospy.init_node("corner_test", anonymous=True)
    tf = tf2_ros.Buffer()
    tf2_ros.TransformListener(tf)
    rospy.wait_for_message("/move_base/global_costmap/costmap", OccupancyGrid, 10)
    status_pub = rospy.Publisher("/move_base/cancel", GoalID, queue_size=1)

    g = PoseStamped(header=rospy.Header(frame_id="map"))
    g.pose.position.x, g.pose.position.y = GOAL
    g.pose.orientation.w = 1.0
    pub = rospy.Publisher("/move_base_simple/goal", PoseStamped, queue_size=1)
    rospy.sleep(0.5)
    pub.publish(g)
    rospy.loginfo("goal sent %.3f %.3f", GOAL[0], GOAL[1])

    spin_start = None
    spin_pose = None
    last_report = time.time()

    while not rospy.is_shutdown():
        try:
            t = tf.lookup_transform("map", "base_footprint", rospy.Time(0))
        except Exception:
            rospy.sleep(0.4)
            continue
        x, y = t.transform.translation.x, t.transform.translation.y
        q = t.transform.rotation
        yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
        dgoal = math.hypot(x - GOAL[0], y - GOAL[1])
        elapsed = rospy.get_time()
        if dgoal < 0.25 and abs(q.z) < 0.30:
            print("SUCCEEDED pose=(%.3f,%.3f) yaw=%.2f dgoal=%.3f at_t=%.0f" % (x, y, yaw, dgoal, elapsed))
            return 0
        if elapsed > WATCH:
            print("TIMEOUT pose=(%.3f,%.3f) dgoal=%.3f" % (x, y, dgoal))
            return 3
        vel = 0.0
        try:
            cmd = rospy.wait_for_message("/my_car/cmd_vel", Twist, timeout=0.4)
            vel = math.hypot(cmd.linear.x, cmd.linear.y)
        except Exception:
            pass
        status = 1
        try:
            s = rospy.wait_for_message("/move_base/status", GoalStatusArray, timeout=0.4)
            if s.status_list:
                status = s.status_list[-1].status
        except Exception:
            pass
        if (vel < 0.005 and dgoal > 0.30):
            if spin_start is None:
                spin_start = elapsed
                spin_pose = (x, y, yaw)
            elif elapsed - spin_start > 12.0:
                lc = rospy.wait_for_message("/move_base/local_costmap/costmap", OccupancyGrid, 2)
                gc = rospy.wait_for_message("/move_base/global_costmap/costmap", OccupancyGrid, 2)
                print("STUCK pose=(%.3f,%.3f) yaw=%.2f dgoal=%.3f stall_start=%s status=%d" % (
                    x, y, yaw, dgoal, spin_pose, status))
                for label, cm in (("GLOBAL", gc), ("LOCAL", lc)):
                    ox = cm.info.origin.position.x
                    oy = cm.info.origin.position.y
                    res = cm.info.resolution
                    h, w = cm.info.height, cm.info.width
                    d = np.array(cm.data).reshape(h, w)
                    vals = []
                    for xx, yy in ((3.5, 2.9), (3.5, 3.1), (3.9, 3.0), (4.1, 3.0),
                                   (2.3, 3.0), (2.6, 3.0), (2.9, 3.0), (3.2, 3.0), (x, y)):
                        c = round((xx - ox) / res)
                        r = h - 1 - round((yy - oy) / res)
                        if 0 <= r < h and 0 <= c < w:
                            vals.append("(%.1f,%.1f)=%d" % (xx, yy, int(d[r, c])))
                    print(" %s: %s" % (label, " ".join(vals)))
                return 2
        else:
            spin_start = None
        if elapsed - last_report > 40 and dgoal > 0.3 and spin_start is None:
            last_report = elapsed
            print("t=%ds pose=(%.3f,%.3f) yaw=%.2f dgoal=%.3f" % (elapsed, x, y, yaw, dgoal))
        rospy.sleep(0.4)


import time

if __name__ == "__main__":
    raise SystemExit(main())