#!/usr/bin/env python3
"""Run five random long move_base goals and audit measured paint clearance."""

import collections
import json
import math
import random
import secrets
import time

import actionlib
import rospy
import tf
from actionlib_msgs.msg import GoalStatus
from gazebo_msgs.msg import ModelStates
from geometry_msgs.msg import PoseStamped
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from nav_msgs.msg import OccupancyGrid
from nav_msgs.srv import GetPlan
from std_msgs.msg import String

from route_geometry import PaintGeometry


TERMINAL = {
    GoalStatus.SUCCEEDED, GoalStatus.ABORTED, GoalStatus.PREEMPTED,
    GoalStatus.REJECTED, GoalStatus.RECALLED, GoalStatus.LOST,
}
STATE_NAME = {value: name for name, value in vars(GoalStatus).items()
              if name.isupper() and isinstance(value, int)}


class LongNavigationProbe:
    def __init__(self):
        self.grid = rospy.wait_for_message(
            '/move_base/global_costmap/costmap', OccupancyGrid, timeout=20)
        self.seed = int(rospy.get_param('~seed', secrets.randbelow(2 ** 31)))
        self.random = random.Random(self.seed)
        self.count = int(rospy.get_param('~count', 5))
        self.min_path = float(rospy.get_param('~min_path_length', 2.0))
        self.timeout = float(rospy.get_param('~goal_timeout', 120.0))
        self.output = rospy.get_param('~output', '/root/navigation_long5.json')
        target = rospy.get_param('~target', '')
        targets = rospy.get_param('~targets', '')
        # ROS private parameters persist after a test node exits.  A fixed
        # single-goal probe must not leak into a later multi-goal random run.
        self.fixed_target = (tuple(float(value) for value in target.split(','))
                             if target and self.count == 1 else None)
        self.fixed_targets = ([tuple(float(value) for value in item.split(','))
                              for item in targets.split(';') if item.strip()]
                             if targets else None)
        self.geometry = PaintGeometry('/root/competition_ground_map.png')
        self.pose = None
        self.pose_wall = 0.0
        self.gate = 'UNKNOWN'
        self.client = actionlib.SimpleActionClient('/move_base', MoveBaseAction)
        self.tf = tf.TransformListener()
        rospy.Subscriber('/gazebo/model_states', ModelStates, self.on_pose, queue_size=1)
        rospy.Subscriber('/traffic_light/gate_status', String, self.on_gate, queue_size=1)
        rospy.wait_for_service('/move_base/make_plan', timeout=20)
        self.make_plan = rospy.ServiceProxy('/move_base/make_plan', GetPlan)
        if not self.client.wait_for_server(rospy.Duration(20)):
            raise RuntimeError('move_base action server unavailable')
        deadline = time.monotonic() + 10
        while self.pose is None and time.monotonic() < deadline:
            time.sleep(0.1)
        if self.pose is None:
            raise RuntimeError('Gazebo vehicle pose unavailable')
        self.candidates = self.reachable_candidates()
        self.report = {'seed': self.seed, 'min_path_length': self.min_path,
                       'start_pose': self.pose, 'goals': [], 'passed': False,
                       'limits': {'map_base_error_m': .10,
                                  'chassis_center_error_m': .17}}
        rospy.on_shutdown(self.on_shutdown)

    def on_pose(self, message):
        if 'my_car' not in message.name:
            return
        pose = message.pose[message.name.index('my_car')]
        q = pose.orientation
        yaw = math.atan2(2 * (q.w * q.z + q.x * q.y),
                         1 - 2 * (q.y * q.y + q.z * q.z))
        self.pose = (pose.position.x, pose.position.y, yaw)
        self.pose_wall = time.monotonic()

    def on_gate(self, message):
        self.gate = message.data

    def cell(self, x, y):
        info = self.grid.info
        return (int((x - info.origin.position.x) / info.resolution),
                int((y - info.origin.position.y) / info.resolution))

    def cost(self, x, y):
        info = self.grid.info
        if not (0 <= x < info.width and 0 <= y < info.height):
            return -1
        return self.grid.data[y * info.width + x]

    def world(self, x, y):
        info = self.grid.info
        return (info.origin.position.x + (x + .5) * info.resolution,
                info.origin.position.y + (y + .5) * info.resolution)

    def reachable_candidates(self):
        start = self.cell(*self.pose[:2])
        if not 0 <= self.cost(*start) < 100:
            raise RuntimeError('vehicle is on a lethal or unknown global cell')
        visited = {start}
        queue = collections.deque([start])
        while queue:
            x, y = queue.popleft()
            for neighbor in ((x + 1, y), (x - 1, y), (x, y + 1), (x, y - 1)):
                if neighbor not in visited and 0 <= self.cost(*neighbor) < 100:
                    visited.add(neighbor)
                    queue.append(neighbor)
        candidates = []
        for x, y in visited:
            gx, gy = self.world(x, y)
            if abs(gx) > 1.85 or abs(gy) > 1.85 or not 0 <= self.cost(x, y) < 50:
                continue
            if all(0 <= self.cost(x + dx, y + dy) < 50
                   for dx in range(-2, 3) for dy in range(-2, 3)
                   if dx * dx + dy * dy <= 4):
                candidates.append((gx, gy))
        if not candidates:
            raise RuntimeError('no reachable clear goal cells')
        return candidates

    @staticmethod
    def stamped(x, y, yaw=0):
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.header.stamp = rospy.Time.now()
        pose.pose.position.x, pose.pose.position.y = x, y
        pose.pose.orientation.z = math.sin(yaw / 2)
        pose.pose.orientation.w = math.cos(yaw / 2)
        return pose

    def choose_goal(self):
        px, py, _ = self.pose
        fixed = (self.fixed_target if self.fixed_target else
                 (self.fixed_targets[len(self.report['goals'])]
                  if self.fixed_targets and len(self.report['goals']) < len(self.fixed_targets)
                  else None))
        candidates = ([fixed] if fixed
                      else self.candidates[:])
        if not fixed:
            self.random.shuffle(candidates)
        used = [row['target'] for row in self.report['goals']]
        for spacing in (.8, .3, 0.):
            for x, y in candidates:
                if math.hypot(x - px, y - py) < 1.3:
                    continue
                if any(math.hypot(x - ux, y - uy) < spacing for ux, uy in used):
                    continue
                try:
                    plan = self.make_plan(self.stamped(px, py),
                                          self.stamped(x, y), 0.0).plan.poses
                except rospy.ServiceException:
                    continue
                if len(plan) < 2:
                    continue
                length = sum(math.hypot(b.pose.position.x - a.pose.position.x,
                                        b.pose.position.y - a.pose.position.y)
                             for a, b in zip(plan[:-1], plan[1:]))
                if length < self.min_path:
                    continue
                # Reject a global path that itself rides an inflated edge.
                # DWA should handle small local costs, but a target whose
                # whole route is already inside a lethal/unknown footprint
                # corridor is not a valid free-space goal for this test.
                path_safe = True
                for pose in plan[::max(1, len(plan) // 80)]:
                    cx, cy = self.cell(pose.pose.position.x,
                                       pose.pose.position.y)
                    for dx in range(-2, 3):
                        for dy in range(-2, 3):
                            if dx * dx + dy * dy <= 4 and not 0 <= self.cost(cx + dx, cy + dy) < 100:
                                path_safe = False
                                break
                        if not path_safe:
                            break
                    if not path_safe:
                        break
                if not path_safe and not fixed:
                    continue
                yaw = math.atan2(plan[-1].pose.position.y - plan[-2].pose.position.y,
                                 plan[-1].pose.position.x - plan[-2].pose.position.x)
                return x, y, yaw, length
        raise RuntimeError('no distinct reachable goal with a %.1f m plan'
                           % self.min_path)

    def save(self):
        with open(self.output, 'w', encoding='utf-8') as stream:
            json.dump(self.report, stream, indent=2)

    def on_shutdown(self):
        if self.client.get_state() not in TERMINAL:
            self.client.cancel_goal()
        self.save()

    def map_base_pose(self):
        try:
            self.tf.waitForTransform('map', 'base_footprint', rospy.Time(0),
                                     rospy.Duration(2))
            position, _ = self.tf.lookupTransform(
                'map', 'base_footprint', rospy.Time(0))
            return position[0], position[1]
        except tf.Exception:
            return None

    def run(self):
        print('seed=%d candidates=%d' % (self.seed, len(self.candidates)), flush=True)
        self.save()
        for index in range(1, self.count + 1):
            self.grid = rospy.wait_for_message(
                '/move_base/global_costmap/costmap', OccupancyGrid, timeout=10)
            self.candidates = self.reachable_candidates()
            x, y, yaw, plan_length = self.choose_goal()
            row = {'index': index, 'target': [x, y],
                   'planned_length': round(plan_length, 3),
                   'start_pose': self.pose, 'status': 'RUNNING',
                   'white_contact': False, 'gate_counts': {}, 'samples': []}
            self.report['goals'].append(row)
            self.save()
            print('goal %d/%d target=(%.3f, %.3f) plan=%.2fm'
                  % (index, self.count, x, y, plan_length), flush=True)
            goal = MoveBaseGoal()
            goal.target_pose = self.stamped(x, y, yaw)
            self.client.send_goal(goal)
            start = time.monotonic()
            while not rospy.is_shutdown() and time.monotonic() - start < self.timeout:
                pose = self.pose
                if time.monotonic() - self.pose_wall > .8:
                    row['status'] = 'FAILED:STALE_POSE'
                    self.client.cancel_goal()
                    break
                if pose is not None:
                    contact = bool(self.geometry.collision(*pose))
                    row['samples'].append({'t': round(time.monotonic() - start, 3),
                                           'pose': pose, 'gate': self.gate,
                                           'paint_contact': contact})
                    if contact:
                        row['white_contact'] = True
                        row['status'] = 'FAILED:PAINT_CONTACT'
                        self.client.cancel_goal()
                        break
                row['gate_counts'][self.gate] = row['gate_counts'].get(self.gate, 0) + 1
                state = self.client.get_state()
                if state in TERMINAL:
                    row['status'] = STATE_NAME.get(state, str(state))
                    break
                time.sleep(.1)
            else:
                row['status'] = 'TIMEOUT'
                self.client.cancel_goal()
            row['elapsed_seconds'] = round(time.monotonic() - start, 2)
            row['final_pose'] = self.pose
            row['error_m'] = round(math.hypot(self.pose[0] - x,
                                             self.pose[1] - y), 3)
            map_pose = self.map_base_pose()
            row['map_base_error_m'] = (round(math.hypot(map_pose[0] - x,
                                                      map_pose[1] - y), 3)
                                       if map_pose else None)
            self.save()
            print('goal %d result=%s error=%.3fm time=%.1fs'
                  % (index, row['status'], row['error_m'],
                     row['elapsed_seconds']), flush=True)
            if row['white_contact']:
                break
        self.report['passed'] = (len(self.report['goals']) == self.count and
                                 all(row['status'] == 'SUCCEEDED' and
                                     not row['white_contact'] and
                                     # The destination is for base_footprint
                                     # at the drive axle, 0.0525 m ahead of
                                     # the measured Gazebo chassis center.
                                     row['error_m'] <= .17 and
                                     row['map_base_error_m'] is not None and
                                     row['map_base_error_m'] <= .10
                                     for row in self.report['goals']))
        self.save()
        print('passed=%s evidence=%s' % (self.report['passed'], self.output),
              flush=True)
        return self.report['passed']


if __name__ == '__main__':
    rospy.init_node('validate_long_navigation')
    raise SystemExit(0 if LongNavigationProbe().run() else 1)
