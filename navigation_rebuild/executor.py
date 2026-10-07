"""One move_base goal at a time, with measured progress and arrival checks."""

import math
import time
from dataclasses import dataclass

import actionlib
import rospy
import tf
from actionlib_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from nav_msgs.msg import Odometry


def angle_error(target, actual):
    return math.atan2(math.sin(target - actual), math.cos(target - actual))


@dataclass(frozen=True)
class NavResult:
    ok: bool
    reason: str
    action_state: int
    duration_s: float
    position_error_m: float = None
    heading_error_rad: float = None


class NavExecutor:
    def __init__(self):
        self.client = actionlib.SimpleActionClient("/move_base", MoveBaseAction)
        self.tf = tf.TransformListener()
        self.odom = None
        rospy.Subscriber("/odom", Odometry, self._on_odom, queue_size=1)
        rospy.on_shutdown(self.cancel)

    def _on_odom(self, message):
        self.odom = message

    def cancel(self):
        self.client.cancel_all_goals()

    def pose(self):
        stamp = self.tf.getLatestCommonTime("map", "base_footprint")
        if (rospy.Time.now() - stamp).to_sec() > 0.8:
            raise RuntimeError("map pose is stale")
        position, quaternion = self.tf.lookupTransform("map", "base_footprint", stamp)
        yaw = math.atan2(2 * (quaternion[3] * quaternion[2] + quaternion[0] * quaternion[1]),
                         1 - 2 * (quaternion[1] ** 2 + quaternion[2] ** 2))
        return position[0], position[1], yaw

    def motion_pose(self):
        message = self.odom
        if message is None or (rospy.Time.now() - message.header.stamp).to_sec() > 0.8:
            raise RuntimeError("wheel odometry is stale")
        pose = message.pose.pose
        q = pose.orientation
        yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y ** 2 + q.z ** 2))
        return pose.position.x, pose.position.y, yaw

    @staticmethod
    def _message(goal):
        result = MoveBaseGoal()
        result.target_pose = PoseStamped()
        result.target_pose.header.frame_id = "map"
        result.target_pose.header.stamp = rospy.Time.now()
        result.target_pose.pose.position.x = goal.x
        result.target_pose.pose.position.y = goal.y
        result.target_pose.pose.orientation.z = math.sin(goal.yaw / 2)
        result.target_pose.pose.orientation.w = math.cos(goal.yaw / 2)
        return result

    def navigate(self, goal, no_progress_s=25.0):
        if not self.client.wait_for_server(rospy.Duration(15)):
            return NavResult(False, "MOVE_BASE_UNAVAILABLE", GoalStatus.LOST, 0)
        start = time.monotonic()
        last_motion = start
        last_pose = None
        self.client.send_goal(self._message(goal))
        terminal = {GoalStatus.SUCCEEDED, GoalStatus.ABORTED, GoalStatus.PREEMPTED,
                    GoalStatus.REJECTED, GoalStatus.RECALLED, GoalStatus.LOST}
        reason = "TIMEOUT"
        while not rospy.is_shutdown() and time.monotonic() - start < goal.timeout_s:
            state = self.client.get_state()
            if state in terminal:
                reason = "ACTION_RESULT"
                break
            try:
                pose = self.motion_pose()
                if (last_pose is None or math.hypot(pose[0] - last_pose[0], pose[1] - last_pose[1]) > 0.025
                        or abs(angle_error(pose[2], last_pose[2])) > 0.06):
                    last_motion, last_pose = time.monotonic(), pose
            except RuntimeError:
                pass
            if time.monotonic() - last_motion > no_progress_s:
                reason = "NO_PROGRESS"
                break
            time.sleep(0.1)
        duration = time.monotonic() - start
        state = self.client.get_state()
        if state != GoalStatus.SUCCEEDED or reason != "ACTION_RESULT":
            self.client.cancel_goal()
            self.client.wait_for_result(rospy.Duration(2))
            return NavResult(False, reason, state, duration)
        try:
            x, y, yaw = self.pose()
        except (tf.Exception, RuntimeError):
            return NavResult(False, "ARRIVAL_POSE_UNAVAILABLE", state, duration)
        xy_error = math.hypot(x - goal.x, y - goal.y)
        yaw_error = abs(angle_error(goal.yaw, yaw))
        if xy_error > 0.08 or yaw_error > 0.14:
            return NavResult(False, "ARRIVAL_MISMATCH", state, duration, xy_error, yaw_error)
        return NavResult(True, "ARRIVED", state, duration, xy_error, yaw_error)
