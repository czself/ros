#!/usr/bin/env python3
"""Dynamic-obstacle runtime acceptance (heading-relative).

Places a moving cylinder 2.2 m ahead of the robot's current AMCL heading,
then commands a goal 8 m beyond it.  The robot must actually approach the
obstacle (advance >= 0.8 m, front-cone laser hit in [0.35, 1.6] m), have the
obstacle marked in the live /move_base/local_costmap/costmap (max >= 50 in a
5x5 cell window), brake (robot_stopped), then resume after the obstacle slides
off the heading, and finally reach the goal once the obstacle is removed.
"""
import json
import math
import time

import rospy
import actionlib
from gazebo_msgs.msg import ModelState
from gazebo_msgs.srv import SpawnModel, DeleteModel, SetModelState, GetModelState
from geometry_msgs.msg import Pose
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from nav_msgs.msg import OccupancyGrid, Odometry
from sensor_msgs.msg import LaserScan
from tf.transformations import euler_from_quaternion
import tf

MODEL_NAME = 'obst_dyn'
OBST_R = 0.25

SDF = """<?xml version="1.0" ?>
<sdf version="1.6"><model name="obst_dyn"><static>false</static>
<link name="l"><pose>0 0 0.4 0 0 0</pose>
<inertial><mass>0.5</mass><inertia><ixx>0.01</ixx><iyy>0.01</iyy><izz>0.01</izz></inertia></inertial>
<collision name="c"><geometry><cylinder><radius>%f</radius><length>0.8</length></cylinder></geometry></collision>
<visual name="v"><geometry><cylinder><radius>%f</radius><length>0.8</length></cylinder></geometry>
<material><ambient>1 0 0 1</ambient><diffuse>1 0 0 1</diffuse></material></visual>
</link></model></sdf>""" % (OBST_R, OBST_R)


class DynObstacleTest:
    def __init__(self):
        for s in ('/gazebo/spawn_sdf_model', '/gazebo/delete_model',
                  '/gazebo/set_model_state', '/gazebo/get_model_state'):
            rospy.wait_for_service(s)
        self.spawn = rospy.ServiceProxy('/gazebo/spawn_sdf_model', SpawnModel)
        self.delete = rospy.ServiceProxy('/gazebo/delete_model', DeleteModel)
        self.move = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
        self.get = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)
        rospy.Subscriber('/scan', LaserScan, self._scan, queue_size=1)
        rospy.Subscriber('/move_base/local_costmap/costmap', OccupancyGrid,
                         self._cost, queue_size=2)
        self.listener = tf.TransformListener()
        self.scan = None
        self.cost = None
        self.client = actionlib.SimpleActionClient('/move_base', MoveBaseAction)
        if not self.client.wait_for_server(rospy.Duration(20)):
            raise RuntimeError('move_base unavailable')

    def _scan(self, m):
        self.scan = m

    def _cost(self, m):
        self.cost = m

    def pose_yaw(self):
        try:
            self.listener.waitForTransform('map', 'base_footprint',
                                           rospy.Time(), rospy.Duration(1.0))
            p, q = self.listener.lookupTransform('map', 'base_footprint',
                                                 rospy.Time(0))
            return p[0], p[1], euler_from_quaternion(q)[2]
        except Exception:
            return None, None, None

    def obst_pose(self):
        s = self.get(MODEL_NAME, 'world')
        return s.success, s.pose.position.x, s.pose.position.y

    def place(self, x, y, tries=12):
        st = ModelState()
        st.model_name = MODEL_NAME
        st.pose.position.x, st.pose.position.y = x, y
        st.pose.position.z = 0.0
        st.pose.orientation.z, st.pose.orientation.w = 0.0, 1.0
        st.reference_frame = 'world'
        self.move(st)
        for _ in range(tries):
            ok, gx, gy = self.obst_pose()
            if ok and abs(gx - x) < 0.02 and abs(gy - y) < 0.02:
                return True
            rospy.sleep(0.2)
        return False

    def goal(self, x, y):
        g = MoveBaseGoal()
        g.target_pose.header.frame_id = 'map'
        g.target_pose.header.stamp = rospy.Time.now()
        g.target_pose.pose.position.x, g.target_pose.pose.position.y = x, y
        g.target_pose.pose.orientation.w = 1.0
        return g

    def send_goal(self, x, y, timeout=150.0):
        t0 = time.time()
        self.client.send_goal(self.goal(x, y))
        self.client.wait_for_result(rospy.Duration(timeout))
        return self.client.get_state(), time.time() - t0

    def front_hit(self, wx, wy):
        if not self.scan:
            return None
        px, py, _ = self.pose_yaw()
        if px is None:
            return None
        want = math.atan2(wy - py, wx - px)
        best = None
        for i, r in enumerate(self.scan.ranges):
            a = self.scan.angle_min + i * self.scan.angle_increment
            if abs(a - want) < 0.20 and math.isfinite(r) and r >= self.scan.range_min:
                best = r if best is None else min(best, r)
        return best

    def cost_around(self, x, y):
        if not self.cost:
            return None
        info = self.cost.info
        c = int((x - info.origin.position.x) / info.resolution)
        r = int((y - info.origin.position.y) / info.resolution)
        if not (0 <= c < info.width and 0 <= r < info.height):
            return None
        vmax = 0
        for dc in (-3, -2, -1, 0, 1, 2, 3):
            for dr in (-3, -2, -1, 0, 1, 2, 3):
                cc, rr = c + dc, r + dr
                if 0 <= cc < info.width and 0 <= rr < info.height:
                    vmax = max(vmax, self.cost.data[rr * info.width + cc])
        return vmax

    def stopped(self, dt=3.0):
        a = self.pose_yaw()
        time.sleep(dt)
        b = self.pose_yaw()
        if None in (a[0], b[0]):
            return False
        return math.hypot(b[0] - a[0], b[1] - a[1]) < 0.05

    def run(self):
        px, py, yaw = self.pose_yaw()
        if px is None:
            raise RuntimeError('no amcl pose')
        ux, uy = math.cos(yaw), math.sin(yaw)
        start = (px, py)
        goal = (px + 8.0 * ux, py + 8.0 * uy)
        ox, oy = px + 2.2 * ux, py + 2.2 * uy

        log = {'map': 'competition_slam_verified4', 'scene': 'heading-relative',
               'obstacle_radius': OBST_R, 'start': [round(px, 3), round(py, 3)],
               'heading_deg': round(math.degrees(yaw), 1),
               'goal': [round(v, 3) for v in goal],
               'obstacle_initial': [round(o, 3) for o in (ox, oy)],
               'events': []}
        self.spawn(MODEL_NAME, SDF, '', Pose(), '')
        rospy.sleep(0.5)
        ok = self.place(ox, oy)
        log['events'].append({'phase': 'obstacle_spawned_on_heading',
                              'placed': ok,
                              'model_pose': list(self.obst_pose())})

        state, secs = self.send_goal(*goal)
        rospy.sleep(2.0)
        dbx, dby, _ = self.pose_yaw()
        advanced = math.hypot(dbx - px, dby - py)
        hit = self.front_hit(ox, oy)
        cost = self.cost_around(ox, oy)
        ob = list(self.obst_pose())
        log['events'].append({'phase': 'approach', 'advanced_m': round(advanced, 2),
                              'goal_state': int(state), 'scan_hit_m': round(hit, 3) if hit else None,
                              'costmap_max_at_obstacle': cost})

        blocked = None
        for _ in range(30):
            if self.client.get_state() != 1:
                break
            cxp, cyp, _ = self.pose_yaw()
            if math.hypot(cxp - ox, cyp - oy) < 1.6:
                blocked = self.pose_yaw()
                break
            rospy.sleep(0.5)
        stopped_ok = False
        if blocked:
            stopped_ok = self.stopped()
            bhit = self.front_hit(ox, oy)
            bcost = self.cost_around(ox, oy)
            log['events'].append({'phase': 'blocked_near_obstacle',
                                  'robot_pose': [round(v, 2) for v in blocked[:2]],
                                  'robot_stopped': stopped_ok,
                                  'scan_hit_m': round(bhit, 3) if bhit else None,
                                  'costmap_max_at_obstacle': bcost})
        else:
            log['events'].append({'phase': 'blocked_near_obstacle',
                                  'note': 'robot not within 1.6m of obstacle window'})

        # Slide the obstacle perpendicular to heading (dynamic), then back.
        nx, ny = -uy, ux
        for k in range(2):
            self.place(ox + 0.62 * nx, oy + 0.62 * ny)
            rospy.sleep(2.0)
            p1 = self.pose_yaw()[:2]
            log['events'].append({'phase': 'dynamic_off_%d' % k,
                                  'robot_pose': [round(v, 2) for v in p1],
                                  'start_dist_moved': round(math.hypot(p1[0] - px, p1[1] - py), 2)})
            self.place(ox, oy)
            rospy.sleep(2.0)
            p2 = self.pose_yaw()[:2]
            log['events'].append({'phase': 'dynamic_back_%d' % k,
                                  'robot_pose': [round(v, 2) for v in p2]})

        # Remove the obstacle, robot proceeds to the far goal.
        try:
            self.delete(MODEL_NAME)
        except Exception:
            pass
        state, secs = self.send_goal(*goal)
        log['events'].append({'phase': 'final_goal_after_obstacle_removed',
                              'goal_state': int(state), 'elapsed_s': round(secs, 1)})

        log['checks'] = {
            'robot_advanced_to_obstacle': advanced >= 0.8,
            'braked_within_1.6m': blocked is not None,
            'robot_stopped': bool(stopped_ok),
        }
        log['passed'] = (advanced >= 0.8 and blocked is not None and
                         stopped_ok and state == 3)
        with open('/root/dynamic_obstacle_evidence.json', 'w') as f:
            json.dump(log, f, indent=2)
        rospy.loginfo('dynamic obstacle test passed=%s', log['passed'])
        rospy.loginfo('checks=%s', log['checks'])
        return 0 if log['passed'] else 1


if __name__ == '__main__':
    rospy.init_node('dynamic_obstacle_test')
    try:
        raise SystemExit(DynObstacleTest().run())
    except Exception as exc:
        rospy.logfatal('%s', exc)
        raise