#!/usr/bin/env python3
"""Run ten random, obstacle-clear navigation goals and print a compact report."""
import math
import random
import secrets
import rospy
import actionlib
import tf2_ros
from actionlib_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseWithCovarianceStamped
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from nav_msgs.msg import OccupancyGrid
from tf.transformations import euler_from_quaternion


def yaw_quat(yaw):
    return math.sin(yaw / 2.0), math.cos(yaw / 2.0)


class Tester:
    def __init__(self):
        self.map = rospy.wait_for_message('/map', OccupancyGrid, timeout=15.0)
        self.tf = tf2_ros.Buffer(cache_time=rospy.Duration(10))
        self.listener = tf2_ros.TransformListener(self.tf)
        self.initial_pub = rospy.Publisher('/initialpose', PoseWithCovarianceStamped, queue_size=1, latch=True)
        configured_seed = rospy.get_param('~seed', None)
        self.seed = int(configured_seed) if configured_seed is not None else secrets.randbits(64)
        self.rng = random.Random(self.seed)
        self.client = actionlib.SimpleActionClient('/move_base', MoveBaseAction)
        if not self.client.wait_for_server(rospy.Duration(30)):
            raise RuntimeError('move_base action server unavailable')

    def pose(self):
        tr = self.tf.lookup_transform('odom', 'chassis', rospy.Time(0), rospy.Duration(5))
        q = tr.transform.rotation
        return tr.transform.translation.x, tr.transform.translation.y, euler_from_quaternion((q.x, q.y, q.z, q.w))[2]

    def set_initial_pose(self):
        x, y, yaw = self.pose()
        msg = PoseWithCovarianceStamped()
        msg.header.frame_id = 'map'
        msg.header.stamp = rospy.Time.now()
        msg.pose.pose.position.x, msg.pose.pose.position.y = x, y
        msg.pose.pose.orientation.z, msg.pose.pose.orientation.w = yaw_quat(yaw)
        msg.pose.covariance[0] = 0.04
        msg.pose.covariance[7] = 0.04
        msg.pose.covariance[35] = 0.09
        self.initial_pub.publish(msg)
        rospy.sleep(3.0)
        return x, y

    def free_points(self, count=10):
        info = self.map.info
        candidates = []
        # Keep a robot-sized clearance from occupied/unknown cells.
        radius = int(math.ceil(0.35 / info.resolution))
        for iy in range(radius, info.height - radius):
            if iy % 3:
                continue
            for ix in range(radius, info.width - radius):
                if ix % 3:
                    continue
                idx = iy * info.width + ix
                if self.map.data[idx] != 0:
                    continue
                wx = info.origin.position.x + (ix + 0.5) * info.resolution
                wy = info.origin.position.y + (iy + 0.5) * info.resolution
                if not (-3.0 <= wx <= 3.0 and -2.0 <= wy <= 3.0):
                    continue
                clear = True
                for dy in range(-radius, radius + 1, 3):
                    for dx in range(-radius, radius + 1, 3):
                        if dx * dx + dy * dy > radius * radius:
                            continue
                        v = self.map.data[(iy + dy) * info.width + ix + dx]
                        if v != 0:
                            clear = False
                            break
                    if not clear:
                        break
                if clear:
                    candidates.append((wx, wy))
        self.rng.shuffle(candidates)
        chosen = []
        for p in candidates:
            if all((p[0] - q[0]) ** 2 + (p[1] - q[1]) ** 2 > 0.7 ** 2 for q in chosen):
                chosen.append(p)
                if len(chosen) == count:
                    break
        if len(chosen) < count:
            raise RuntimeError('could not sample ten separated free points')
        return chosen

    def run(self):
        self.set_initial_pose()
        count = int(rospy.get_param('~count', 10))
        # Do not fall back to the historical fixed five-point route.  Each
        # invocation samples a fresh set; pass ~seed:=... to reproduce one.
        points = self.free_points(count)
        rospy.loginfo('Random navigation seed: %d', self.seed)
        results = []
        for i, (x, y) in enumerate(points, 1):
            goal = MoveBaseGoal()
            goal.target_pose.header.frame_id = 'map'
            goal.target_pose.header.stamp = rospy.Time.now()
            goal.target_pose.pose.position.x = x
            goal.target_pose.pose.position.y = y
            goal.target_pose.pose.orientation.w = 1.0
            start = rospy.Time.now()
            self.client.send_goal(goal)
            done = self.client.wait_for_result(rospy.Duration(60))
            state = self.client.get_state()
            if not done:
                self.client.cancel_goal()
            label = 'SUCCEEDED' if done and state == GoalStatus.SUCCEEDED else ('TIMEOUT' if not done else 'ABORTED')
            elapsed = (rospy.Time.now() - start).to_sec()
            results.append(label)
            rospy.loginfo('RANDOM %02d target=(%.2f, %.2f) %s %.1fs', i, x, y, label, elapsed)
            rospy.sleep(1.0)
        rospy.loginfo('RESULT %d/%d succeeded; %d failed', results.count('SUCCEEDED'), count, count - results.count('SUCCEEDED'))


if __name__ == '__main__':
    rospy.init_node('random_navigation_test')
    try:
        Tester().run()
    except Exception as exc:
        rospy.logfatal('%s', exc)
        raise
