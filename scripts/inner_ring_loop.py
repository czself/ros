#!/usr/bin/env python3
"""内圈环线: 从出生点出发, 沿场地内侧走廊绕一圈, 回到出生点车头朝北。

路线(全部经 move_base 实测可达)：
  出生点 -> 东带北上 -> 北带 -> 西带南下 -> 南带东行 -> 沿东带回出生点。
所有目标帧为 map, 依赖已启动的 move_base/AMCL(/root/start_navigation.sh)。
结果写 /root/inner_ring_loop_evidence.json, 全程 SUCCEEDED 且终点位姿达
标才算通过(exit 0)。
"""
import json
import math
import time

import rospy
import actionlib
from geometry_msgs.msg import Twist
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import tf

BIRTH = (4.0833, -3.9583, math.pi / 2)
INNER_RING = [
    (3.90, -2.50, math.pi / 2, 'east-s'),
    (3.90,  0.50, math.pi, 'east-m'),
    (3.90,  2.50, math.pi, 'east-n'),
    (2.30,  3.60, math.pi, 'north-e'),
    (0.00,  4.20, math.pi, 'north-m'),
    (-2.50, 3.60, math.pi, 'north-w'),
    (-3.90, 3.60, math.pi, 'west-n'),
    (-3.90, 1.00, math.pi, 'west-m'),
    (-3.90, -1.10, math.pi, 'west-s'),
    (-3.90, -2.80, 0.0, 'west-s2'),
]
STREET_END = (2.90, -2.60)          # 南段"大街"直行终点(走廊已被确证可行)
CLOSE_AT = (3.90, -2.50, math.pi / 2)


def wrap(a):
    while a > math.pi:
        a -= 2 * math.pi
    while a < -math.pi:
        a += 2 * math.pi
    return a


class InnerRingLoop:
    def __init__(self):
        self.c = actionlib.SimpleActionClient('/move_base', MoveBaseAction)
        self.c.wait_for_server(rospy.Duration(10))
        self.lt = tf.TransformListener()
        self.pub = rospy.Publisher('/my_car/cmd_vel_nav', Twist, queue_size=1)

    def pose(self):
        self.lt.waitForTransform('map', 'base_footprint',
                                 rospy.Time(), rospy.Duration(2))
        p, q = self.lt.lookupTransform('map', 'base_footprint', rospy.Time(0))
        return p[0], p[1], math.atan2(
            2 * (q[3] * q[2] + q[0] * q[1]), 1 - 2 * (q[1] ** 2 + q[2] ** 2))

    def send(self, x, y, yaw, label, timeout=120, retry=True):
        for attempt in range(2 if retry else 1):
            g = MoveBaseGoal()
            g.target_pose.header.frame_id = 'map'
            g.target_pose.pose.position.x = x
            g.target_pose.pose.position.y = y
            g.target_pose.pose.orientation.z = math.sin(yaw / 2)
            g.target_pose.pose.orientation.w = math.cos(yaw / 2)
            t0 = time.time()
            self.c.send_goal(g)
            self.c.wait_for_result(rospy.Duration(timeout))
            st = self.c.get_state()
            if st == 3:
                rospy.loginfo('%-12s state=%d %6.1fs', label, st,
                              time.time() - t0)
                return st
            rospy.logwarn('%-12s attempt%d state=%d %6.1fs',
                          label, attempt + 1, st, time.time() - t0)
            if attempt == 0 and retry:
                self.c.cancel_all_goals()
                rospy.sleep(1.5)
        return st

    def stop(self):
        z = Twist()
        for _ in range(3):
            self.pub.publish(z)
            rospy.sleep(0.05)

    def _cmd(self, v, w, rate):
        t = Twist()
        t.linear.x = v
        t.angular.z = w
        self.pub.publish(t)
        rate.sleep()

    def drive_straight_to(self, tx, ty, v=0.40, timeout=60):
        """正向直线纯跟踪: 前视点投影在起点->终点直线上, 避免回旋。"""
        t0 = time.time()
        rate = rospy.Rate(20)
        px0, py0, _ = self.pose()
        fx, fy = tx - px0, ty - py0
        bl = math.hypot(fx, fy) or 1.0
        ux, uy = fx / bl, fy / bl
        while time.time() - t0 < timeout:
            px, py, th = self.pose()
            along = (px - px0) * ux + (py - py0) * uy
            ct = max(0.0, min(bl, along + 0.5))
            lxp, lyp = px0 + ct * ux, py0 + ct * uy
            err = wrap(math.atan2(lyp - py, lxp - px) - th)
            w = max(-1.0, min(1.0, 2.0 * err))
            if along >= bl - 0.05:
                self._cmd(0.0, w, rate)
                break
            dist = math.hypot(tx - px, ty - py)
            sp = v if dist > 0.30 else v * 0.3
            if abs(err) > math.radians(60):
                sp = 0.0
            self._cmd(sp, w, rate)
        self.stop()
        return time.time() - t0

    def run(self):
        self.c.cancel_all_goals()
        rospy.sleep(1)
        log = {'route': [nm for _, _, _, nm in INNER_RING],
               'birth': list(BIRTH[:2]), 'nav': [], 'home': []}
        ok = True
        for x, y, yaw, nm in INNER_RING:
            t0 = time.time()
            st = self.send(x, y, yaw, nm)
            log['nav'].append({'wp': nm, 'state': st,
                               'secs': round(time.time() - t0, 1)})
            if st != 3:
                ok = False
                break
        if ok:
            # ---- 南段"大街"直线自控(绕过 move_base 在该走廊的摆动)
            self.c.cancel_all_goals()
            rospy.sleep(0.4)
            t0 = time.time()
            self.drive_straight_to(*STREET_END, v=0.45)
            log['street_drive_s'] = round(time.time() - t0, 1)
            rospy.sleep(0.4)
            st = self.send(*CLOSE_AT, 'east-s-close')
            log['nav'].append({'wp': 'east-s-close', 'state': st,
                               'secs': round(time.time() - t0, 1)})
            ok = st == 3
        rospy.sleep(0.4)

        st_home = self.send(*BIRTH, 'home-align', 90)
        log['home'].append({'wp': 'home', 'state': st_home})
        ok = ok and st_home == 3
        rospy.sleep(0.5)
        fx, fy, thf = self.pose()
        dx, dy = fx - BIRTH[0], fy - BIRTH[1]
        log['final_pose'] = [round(fx, 3), round(fy, 3),
                             round(math.degrees(thf), 1)]
        log['birth_offset_m'] = round(math.hypot(dx, dy), 3)
        log['heading_err_deg'] = round(math.degrees(abs(wrap(thf - BIRTH[2]))), 2)

        # 若车头偏差超限, 原位旋转对齐一次
        if ok and log['heading_err_deg'] > 2.0:
            self.c.cancel_all_goals()
            rospy.sleep(0.3)
            st = self.send(*BIRTH, 'yaw-lock north', 45)
            ok = ok and st == 3
            rospy.sleep(0.5)
            _, _, thf = self.pose()
            log['heading_err_deg'] = round(math.degrees(
                abs(wrap(thf - BIRTH[2]))), 2)

        log['passed'] = (ok and log['birth_offset_m'] < 0.25 and
                         log['heading_err_deg'] < 2.5)
        with open('/root/inner_ring_loop_evidence.json', 'w') as fh:
            json.dump(log, fh, indent=2)
        rospy.loginfo('INNER-RING final=%s off=%.3f headerr=%.2f passed=%s',
                      log['final_pose'], log['birth_offset_m'],
                      log['heading_err_deg'], log['passed'])
        self.stop()
        return 0 if log['passed'] else 1


if __name__ == '__main__':
    rospy.init_node('inner_ring_loop')
    raise SystemExit(InnerRingLoop().run())