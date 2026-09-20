#!/usr/bin/env python3
"""Run a fresh random, obstacle-clear patrol through the birth component.

The navigation launcher resets the simulated chassis once at the fixed birth
pose. This node never resets or teleports the robot: it samples five goals
from the live inflated global costmap and sends them sequentially to move_base.
Pass ``_seed:=...`` when a route needs to be reproduced.
"""
import collections
import math
import random
import secrets
import time

import actionlib
import rospy
import tf2_ros
from actionlib_msgs.msg import GoalStatus
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from nav_msgs.msg import OccupancyGrid
from tf.transformations import quaternion_from_euler


BIRTH = (4.0833, -3.9583)
GOAL_COUNT = 5


class RandomPatrol:
    def __init__(self):
        self.grid = rospy.wait_for_message(
            '/move_base/global_costmap/costmap', OccupancyGrid, timeout=20.0)
        configured_seed = rospy.get_param('~seed', None)
        self.seed = (int(configured_seed) if configured_seed is not None
                     else secrets.randbits(64))
        self.rng = random.Random(self.seed)
        self.timeout = float(rospy.get_param('~waypoint_timeout', 120.0))
        self.count = int(rospy.get_param('~count', GOAL_COUNT))
        self.client = actionlib.SimpleActionClient('/move_base', MoveBaseAction)
        self.tf = tf2_ros.Buffer(cache_time=rospy.Duration(30.0))
        self.listener = tf2_ros.TransformListener(self.tf)
        if not self.client.wait_for_server(rospy.Duration(20.0)):
            raise RuntimeError('move_base action server unavailable')

    def cell(self, x, y):
        info = self.grid.info
        return (int((x - info.origin.position.x) / info.resolution),
                int((y - info.origin.position.y) / info.resolution))

    def world(self, cell):
        info = self.grid.info
        return (info.origin.position.x + (cell[0] + 0.5) * info.resolution,
                info.origin.position.y + (cell[1] + 0.5) * info.resolution)

    def traversable(self, x, y):
        info = self.grid.info
        return (0 <= x < info.width and 0 <= y < info.height and
                0 <= self.grid.data[y * info.width + x] < 50)

    def clear_goal(self, cell, radius_cells=8):
        cx, cy = cell
        info = self.grid.info
        for dy in range(-radius_cells, radius_cells + 1):
            for dx in range(-radius_cells, radius_cells + 1):
                if dx * dx + dy * dy > radius_cells * radius_cells:
                    continue
                x, y = cx + dx, cy + dy
                if not (0 <= x < info.width and 0 <= y < info.height):
                    return False
                # Goal cells must be truly free, not merely below lethal cost.
                # The 0.40 m global inflation radius is the route safety
                # boundary; this extra raster check rejects goals on a map
                # edge even when a planner update has not arrived yet.
                if self.grid.data[y * info.width + x] != 0:
                    return False
        return True

    def sample(self):
        start = self.cell(*BIRTH)
        if not self.traversable(*start):
            raise RuntimeError('fixed birth cell is not traversable in global costmap')
        reachable = {start}
        queue = collections.deque([start])
        while queue:
            x, y = queue.popleft()
            for nxt in ((x + 1, y), (x - 1, y), (x, y + 1), (x, y - 1)):
                if nxt not in reachable and self.traversable(*nxt):
                    reachable.add(nxt)
                    queue.append(nxt)
        candidates = list(reachable)
        self.rng.shuffle(candidates)
        goals = []
        for cell in candidates:
            gx, gy = self.world(cell)
            if math.hypot(gx - BIRTH[0], gy - BIRTH[1]) < 0.9:
                continue
            if not self.clear_goal(cell):
                continue
            if all(math.hypot(gx - x, gy - y) > 1.0 for x, y in goals):
                goals.append((gx, gy))
            if len(goals) >= self.count:
                return goals
        raise RuntimeError('could not sample %d separated reachable goals' % self.count)

    @staticmethod
    def make_goal(x, y, yaw):
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = 'map'
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose.position.x = x
        goal.target_pose.pose.position.y = y
        q = quaternion_from_euler(0.0, 0.0, yaw)
        goal.target_pose.pose.orientation.x = q[0]
        goal.target_pose.pose.orientation.y = q[1]
        goal.target_pose.pose.orientation.z = q[2]
        goal.target_pose.pose.orientation.w = q[3]
        return goal

    def run(self):
        goals = self.sample()
        rospy.loginfo('Random patrol seed=%d goals=%s', self.seed,
                      ['(%.2f, %.2f)' % goal for goal in goals])
        results = []
        for index, (x, y) in enumerate(goals, 1):
            if rospy.is_shutdown():
                break
            try:
                pose = self.tf.lookup_transform(
                    'map', 'base_footprint', rospy.Time(0), rospy.Duration(2.0))
                px, py = pose.transform.translation.x, pose.transform.translation.y
            except Exception:
                px, py = BIRTH
            yaw = math.atan2(y - py, x - px)
            started = time.monotonic()
            self.client.send_goal(self.make_goal(x, y, yaw))
            deadline = started + self.timeout
            terminal = (GoalStatus.SUCCEEDED, GoalStatus.ABORTED,
                        GoalStatus.PREEMPTED, GoalStatus.REJECTED,
                        GoalStatus.RECALLED, GoalStatus.LOST)
            while (not rospy.is_shutdown() and time.monotonic() < deadline and
                   self.client.get_state() not in terminal):
                time.sleep(0.1)
            state = self.client.get_state()
            if state not in terminal:
                self.client.cancel_goal()
                state = GoalStatus.PREEMPTED
            elapsed = time.monotonic() - started
            ok = state == GoalStatus.SUCCEEDED
            results.append(ok)
            rospy.loginfo('Random goal %d/%d (%.3f, %.3f): %s %.1fs',
                          index, len(goals), x, y,
                          'SUCCEEDED' if ok else 'FAILED(state=%d)' % state,
                          elapsed)
            if not ok:
                break
        rospy.loginfo('Random patrol result %d/%d succeeded seed=%d',
                      sum(results), len(goals), self.seed)
        return all(results) and len(results) == len(goals)


if __name__ == '__main__':
    rospy.init_node('patrol_controller')
    try:
        raise SystemExit(0 if RandomPatrol().run() else 1)
    except Exception as exc:
        rospy.logfatal('%s', exc)
        raise
