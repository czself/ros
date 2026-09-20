#!/usr/bin/env python3
"""Run five random, clearance-checked navigation goals from the fixed birth pose."""
import json
import math
import random
import secrets
import sys
import time
from collections import deque

import actionlib
import rospy
from actionlib_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from nav_msgs.msg import OccupancyGrid, Odometry
import tf2_ros
from tf.transformations import euler_from_quaternion, quaternion_from_euler


BIRTH = (4.0833, -3.9583, 1.5708)


class Validator:
    def __init__(self):
        # Sample the same inflated, unknown-aware grid that Navfn/DWA actually
        # use.  Raw SLAM pixels can be free while the runtime footprint is not.
        self.map = rospy.wait_for_message('/move_base/global_costmap/costmap', OccupancyGrid, timeout=20.0)
        self.odom = rospy.wait_for_message('/odom', Odometry, timeout=10.0)
        self.client = actionlib.SimpleActionClient('/move_base', MoveBaseAction)
        self.tf = tf2_ros.Buffer(cache_time=rospy.Duration(30.0))
        self.tf_listener = tf2_ros.TransformListener(self.tf)
        # A fresh seed makes every validation run exercise a different set of
        # separated goals.  A ROS parameter can still pin a seed when a failed
        # route needs to be reproduced exactly.
        configured_seed = rospy.get_param('~seed', None)
        self.seed = int(configured_seed) if configured_seed is not None else secrets.randbits(64)
        self.rng = random.Random(self.seed)
        if not self.client.wait_for_server(rospy.Duration(20.0)):
            raise RuntimeError('move_base action server unavailable')

    def pose_xy(self):
        try:
            t = self.tf.lookup_transform('map', 'base_footprint', rospy.Time(0), rospy.Duration(2.0))
            return t.transform.translation.x, t.transform.translation.y
        except Exception:
            msg = rospy.wait_for_message('/odom', Odometry, timeout=5.0)
            return msg.pose.pose.position.x, msg.pose.pose.position.y

    def pose_yaw(self):
        try:
            t = self.tf.lookup_transform('map', 'base_footprint', rospy.Time(0), rospy.Duration(2.0))
            q = t.transform.rotation
            return euler_from_quaternion((q.x, q.y, q.z, q.w))[2]
        except Exception:
            return 0.0

    def free_with_clearance(self, gx, gy, radius_cells=8):
        info = self.map.info
        cx = int((gx - info.origin.position.x) / info.resolution)
        cy = int((gy - info.origin.position.y) / info.resolution)
        for dy in range(-radius_cells, radius_cells + 1):
            for dx in range(-radius_cells, radius_cells + 1):
                if dx * dx + dy * dy > radius_cells * radius_cells:
                    continue
                x, y = cx + dx, cy + dy
                if x < 0 or y < 0 or x >= info.width or y >= info.height:
                    return False
                value = self.map.data[y * info.width + x]
                if value < 0 or value >= 50:
                    return False
        return True

    def sample_goals(self, count):
        info = self.map.info
        start_x = int((BIRTH[0] - info.origin.position.x) / info.resolution)
        start_y = int((BIRTH[1] - info.origin.position.y) / info.resolution)
        # Restrict random targets to the same free-space component as the
        # birth pose; a free cell behind a wall is not a valid navigation goal.
        reachable = set([(start_x, start_y)])
        queue = deque([(start_x, start_y)])
        while queue:
            x, y = queue.popleft()
            for nx, ny in ((x + 1, y), (x - 1, y), (x, y + 1), (x, y - 1)):
                if nx < 0 or ny < 0 or nx >= info.width or ny >= info.height:
                    continue
                if (nx, ny) in reachable:
                    continue
                value = self.map.data[ny * info.width + nx]
                # Unknown cells are not traversable just because their
                # signed OccupancyGrid value (-1) is below the occupied
                # threshold.  Keeping them out prevents random goals behind
                # unmapped/closed regions from entering the test set.
                if value < 0 or value >= 50:
                    continue
                reachable.add((nx, ny))
                queue.append((nx, ny))
        candidates = list(reachable)
        self.rng.shuffle(candidates)
        goals = []
        # Select random, dispersed cells so each run exercises a different
        # part of the birth-connected component rather than fixed waypoints.
        # The clearance check is against the live inflated global costmap, so
        # randomization cannot deliberately choose an occupied/unknown cell.
        for cx, cy in candidates:
            gx = info.origin.position.x + (cx + 0.5) * info.resolution
            gy = info.origin.position.y + (cy + 0.5) * info.resolution
            if math.hypot(gx - BIRTH[0], gy - BIRTH[1]) < 0.9:
                continue
            if not self.free_with_clearance(gx, gy):
                continue
            if all(math.hypot(gx - x, gy - y) > 1.0 for x, y in goals):
                goals.append((gx, gy))
            if len(goals) == count:
                return goals
        raise RuntimeError('could not sample enough reachable-looking map cells')

    def send(self, x, y, timeout=90.0):
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = 'map'
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose.position.x = x
        goal.target_pose.pose.position.y = y
        px, py = self.pose_xy()
        q = quaternion_from_euler(0.0, 0.0, math.atan2(y - py, x - px))
        goal.target_pose.pose.orientation.x, goal.target_pose.pose.orientation.y = q[0], q[1]
        goal.target_pose.pose.orientation.z, goal.target_pose.pose.orientation.w = q[2], q[3]
        self.client.send_goal(goal)
        deadline = time.monotonic() + timeout
        while not rospy.is_shutdown() and time.monotonic() < deadline:
            state = self.client.get_state()
            if state in (GoalStatus.SUCCEEDED, GoalStatus.ABORTED, GoalStatus.PREEMPTED,
                         GoalStatus.REJECTED, GoalStatus.RECALLED, GoalStatus.LOST):
                break
            time.sleep(0.2)
        state = self.client.get_state()
        if state not in (GoalStatus.SUCCEEDED, GoalStatus.PREEMPTED, GoalStatus.RECALLED):
            self.client.cancel_goal()
            cancel_deadline = time.monotonic() + 3.0
            while time.monotonic() < cancel_deadline and self.client.get_state() not in (
                    GoalStatus.PREEMPTED, GoalStatus.RECALLED, GoalStatus.ABORTED,
                    GoalStatus.REJECTED, GoalStatus.LOST):
                time.sleep(0.1)
            state = self.client.get_state()
        return state == GoalStatus.SUCCEEDED, state, self.pose_xy()


def main():
    rospy.init_node('validate_navigation')
    validator = Validator()
    start = validator.pose_xy()
    if math.hypot(start[0] - BIRTH[0], start[1] - BIRTH[1]) > 0.35:
        raise RuntimeError('birth pose changed: %.3f %.3f' % start)
    goals = validator.sample_goals(5)
    result = {'seed': validator.seed, 'birth': start, 'goals': goals, 'results': []}
    print('seed: %d' % validator.seed)
    print('birth: %.3f %.3f' % start)
    failures = 0
    for index, (x, y) in enumerate(goals, 1):
        ok, state, pose = validator.send(x, y)
        error = math.hypot(pose[0] - x, pose[1] - y)
        print('goal %d: %.3f %.3f -> %s (state=%d, final=%.3f %.3f)' %
              (index, x, y, 'SUCCEEDED' if ok else 'FAILED', state, pose[0], pose[1]))
        result['results'].append({'goal': [x, y], 'ok': ok, 'state': int(state), 'final': pose, 'error': error})
        if not ok:
            failures += 1
    result['passed'] = failures == 0
    with open('/root/navigation_validation.json', 'w') as stream:
        json.dump(result, stream, indent=2)
    if failures:
        return 1
    return 0


if __name__ == '__main__':
    try:
        sys.exit(main())
    except Exception as exc:
        print('VALIDATION ERROR:', exc, file=sys.stderr)
        sys.exit(2)
