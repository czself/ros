#!/usr/bin/env python3
"""围绕地图跑一圈回到出发车位: 车头方向与出发一致(朝北), 尾段"倒车入库"。

流程:
  1. move_base 依次访问 NE/NW/SW, 再到街道 (2.80,-4.45) 车头朝东。
  2. 自研控制器沿街道正向快开至东侧越位点 (4.55,-4.28) 车头朝东。
  3. 反向纯跟踪, 沿预计算圆弧(后轴路径, 右打方向)倒车旋入库位。
  4. 库内原位摆正车头到出生方向(北, 90°)。

终点: 出生点 (4.0833,-3.9583)，出发方向 90°，与出发时一致。
"""
import json
import math
import time

import rospy
import actionlib
from geometry_msgs.msg import Twist
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import tf

NAV_CENTERS = [(3.93, 3.93, 0.0, 'w1 NE'),
               (-3.60, 3.93, math.pi, 'w2 NW-west-facing'),
               (-3.77, -2.47, -math.pi / 2, 'w3 SW-south-facing'),
               (4.0833, -2.10, math.pi / 2, 'w4 east-lane')]

LEG_OV = (4.15, -4.30)               # 东车道下方直行终点(越位点入口)
OVER = (4.15, -4.30, 0.0)             # 越位点: 车头朝东、位于库以东
BIRTH = (4.0833, -3.9583, math.pi / 2)

PARK_V = 0.26
DRIVE_V = 0.40


def wrap(a):
    while a > math.pi:
        a -= 2 * math.pi
    while a < -math.pi:
        a += 2 * math.pi
    return a


def reverse_arc(a, e, n=40):
    """两点圆弧(后轴路径): 各自切于航向。返回 (path, center, R, sweep, look_def)。"""
    x1, y1, t1 = a
    x2, y2, t2 = e
    n1 = (-math.sin(t1), math.cos(t1))
    l = math.hypot(x2 - x1, y2 - y1)
    ux, uy = (x2 - x1) / l, (y2 - y1) / l
    mid = ((x1 + x2) / 2, (y1 + y2) / 2)
    lam = -((x1 - mid[0]) * ux + (y1 - mid[1]) * uy) / (n1[0] * ux + n1[1] * uy)
    c = (x1 + lam * n1[0], y1 + lam * n1[1])
    r = math.hypot(c[0] - x1, c[1] - y1)
    p0 = math.atan2(y1 - c[1], x1 - c[0])
    pe = math.atan2(y2 - c[1], x2 - c[0])
    sw = wrap(pe - p0)
    path = [(c[0] + r * math.cos(p0 + sw * k / n),
             c[1] + r * math.sin(p0 + sw * k / n)) for k in range(n + 1)]
    return path, c, r, sw


class ReverseParkLoop:
    def __init__(self):
        self.c = actionlib.SimpleActionClient('/move_base', MoveBaseAction)
        self.c.wait_for_server(rospy.Duration(10))
        self.lt = tf.TransformListener()
        self.pub = rospy.Publisher('/my_car/cmd_vel_nav', Twist, queue_size=1)
        self.path, self.center, self.r, self.sweep = reverse_arc(OVER, BIRTH)
        self.n = len(self.path) - 1

    def pose(self):
        self.lt.waitForTransform('map', 'base_footprint',
                                 rospy.Time(), rospy.Duration(2))
        p, q = self.lt.lookupTransform('map', 'base_footprint', rospy.Time(0))
        x, y, z, w = q
        return p[0], p[1], math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))

    def send(self, x, y, yaw, label, timeout=150):
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
        rospy.loginfo('%-20s state=%d %5.1fs', label, st, time.time() - t0)
        return st

    def stop(self):
        z = Twist()
        z.linear.x = 0.0
        z.angular.z = 0.0
        for _ in range(3):
            self.pub.publish(z)
            rospy.sleep(0.05)

    def _cmd(self, v, w, rate):
        t = Twist()
        t.linear.x = v
        t.angular.z = w
        self.pub.publish(t)
        rate.sleep()

    def drive_straight_to(self, tx, ty, v=0.38, timeout=45):
        """正向直线纯跟踪: 目标为线段终点, 前视点投影在直线上, 避免回旋。"""
        t0 = time.time()
        rate = rospy.Rate(20)
        ret = 0
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
                ret += 1
                break
            dist = math.hypot(tx - px, ty - py)
            sp = v if dist > 0.30 else v * 0.3
            if abs(err) > math.radians(60):
                sp = 0.0
            self._cmd(sp, w, rate)
        self.stop()
        return ret

    def run(self):
        self.c.cancel_all_goals()
        rospy.sleep(1)
        log = {'route': [lab for _, _, _, lab in NAV_CENTERS],
               'overshoot': [round(OVER[0], 3), round(OVER[1], 3),
                             round(math.degrees(OVER[2]), 1)],
               'arc': {'center': [round(self.center[0], 3),
                                  round(self.center[1], 3)],
                       'R': round(self.r, 3),
                       'sweep_deg': round(math.degrees(self.sweep), 1)},
               'nav': [], 'park': [], 'traj': []}
        for x, y, yaw, lab in NAV_CENTERS:
            t0 = time.time()
            st = self.send(x, y, yaw, lab)
            log['nav'].append({'wp': lab, 'state': st,
                               'secs': round(time.time() - t0, 1)})
            if st != 3:
                self.stop()
                log['passed'] = False
                return self._dump(log)
        rospy.sleep(0.6)

        # ---- 接管: 直线纯跟踪沿东车道下行到越位点
        self.c.cancel_all_goals()
        rospy.sleep(0.3)
        t0 = time.time()
        self.drive_straight_to(*LEG_OV)
        log['drive_legs_s'] = round(time.time() - t0, 1)
        rospy.sleep(0.3)
        ox, oy, oth = self.pose()
        log['overshoot_pose'] = [round(ox, 3), round(oy, 3),
                                 round(math.degrees(oth), 1)]
        align_ok = abs(wrap(oth - OVER[2])) < 0.06
        if not align_ok:            # 残余偏航: move_base 原地摆正到朝东
            self.send(ox, oy, OVER[2], 'overshoot align', 45)
            rospy.sleep(0.5)
            _, _, oth = self.pose()
            align_ok = abs(wrap(oth - OVER[2])) < 0.04
        log['overshoot_aligned'] = align_ok

        # ---- 反向纯跟踪倒车圆弧入库
        arc_len = self.r * abs(self.sweep)
        look = max(6, int(0.30 / (arc_len / self.n)))
        rate = rospy.Rate(20)
        rev_n = any_n = 0
        oreq = time.time() + 40
        done = False
        while time.time() < oreq:
            px, py, th = self.pose()
            log['traj'].append([round(px, 3), round(py, 3),
                                round(math.degrees(th), 1)])
            dmin, i_n = 1e9, 0
            for i, (qx, qy) in enumerate(self.path):
                d = (qx - px) ** 2 + (qy - py) ** 2
                if d < dmin:
                    dmin, i_n = d, i
            j = min(i_n + look, self.n)
            tx, ty = self.path[j]
            err = wrap(math.atan2(ty - py, tx - px) - (th + math.pi))
            w = max(-1.2, min(1.2, 4.0 * err))
            v = -0.32 if dmin > 0.09 else (-0.28 if dmin > 0.04 else -PARK_V)
            self._cmd(v, w, rate)
            any_n += 1
            if v < -0.01:
                rev_n += 1
            if (math.hypot(px - BIRTH[0], py - BIRTH[1]) < 0.10 and
                    i_n >= self.n - 2):
                done = True
                break
        self.stop()
        log['park'] += [{'arc_rev_cmds': rev_n, 'arc_cmd_n': any_n,
                         'arc_hit_bay': done}]

        # ---- 摆正车头到出生方向(北): 交给 move_base 原地转向(它处理±180°回绕最稳)
        rospy.sleep(0.3)
        st_t = self.send(*BIRTH, 'yaw-align north', 60)
        align_ok2 = st_t == 3
        # ---- 微调到位: 对准后直行贴合出生点(偏移小), 再次摆正
        self.c.cancel_all_goals()
        rospy.sleep(0.3)
        self.drive_straight_to(BIRTH[0], BIRTH[1], v=0.30, timeout=20)
        st_t2 = self.send(*BIRTH, 'yaw-lock north', 60)
        align_ok2 = align_ok2 and st_t2 == 3
        self.stop()
        rospy.sleep(0.8)
        fx, fy, thf = self.pose()
        log['final_pose'] = [round(fx, 3), round(fy, 3),
                             round(math.degrees(thf), 1)]
        log['birth_offset_m'] = round(math.hypot(fx - BIRTH[0], fy - BIRTH[1]), 3)
        log['heading_err_deg'] = round(
            math.degrees(abs(wrap(thf - BIRTH[2]))), 2)
        log['align_final_yaw'] = align_ok2
        log['passed'] = (log['overshoot_aligned'] and done and align_ok2 and
                         log['birth_offset_m'] < 0.12 and
                         log['heading_err_deg'] < 2.5)
        return self._dump(log)

    def _dump(self, log):
        with open('/root/reverse_park_loop_evidence.json', 'w') as f:
            json.dump(log, f, indent=2)
        park = log['park'][-1] if log['park'] else {}
        rospy.loginfo('PARK arc rev=%s/%s hit_bay=%s final=%s off=%.3f '
                      'headerr=%.2f passed=%s',
                      park.get('arc_rev_cmds'), park.get('arc_cmd_n'),
                      park.get('arc_hit_bay'), log['final_pose'],
                      log['birth_offset_m'], log['heading_err_deg'],
                      log['passed'])
        return 0 if log['passed'] else 1


if __name__ == '__main__':
    rospy.init_node('reverse_park_loop')
    raise SystemExit(ReverseParkLoop().run())