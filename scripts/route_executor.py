#!/usr/bin/env python3
"""Execute the fixed 4.2 m route from the competition diagram.

The route is read from one contract rather than duplicated as ad-hoc Python
coordinates.  Its segments must join end-to-end, so a planner can never make
an unapproved diagonal jump from one diagram arrow to another.
"""

import math
import time

import actionlib
import rospy
import yaml
import tf
from actionlib_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped, Twist
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from std_msgs.msg import String
from tf.transformations import quaternion_from_euler


CONTRACT_PATH = '/root/navigation/inner_route.yaml'


def load_route(path):
    """Load and validate the diagram's ordered, connected lane segments."""
    with open(path, encoding='utf-8') as stream:
        contract = yaml.safe_load(stream)
    if isinstance(contract, dict) and isinstance(contract.get('points'), list):
        points=[tuple(float(v) for v in p) for p in contract['points']]
        if len(points)<2 or points[0] != points[-1]:
            raise ValueError('inner route must return to its start')
        route=[]
        for i,(start,end) in enumerate(zip(points[:-1],points[1:])):
            dx,dy=end[0]-start[0],end[1]-start[1]
            heading=math.atan2(dy,dx)
            distance=math.hypot(dx,dy)
            steps=max(1,int(math.ceil(distance/.25)))
            for step in range(1,steps+1):
                f=step/float(steps)
                route.append(('INNER_%02d_%02d'%(i+1,step),start[0]+dx*f,start[1]+dy*f,heading))
        return route
    segments = contract.get('route_order', []) if isinstance(contract, dict) else []
    if not segments:
        raise ValueError('route contract has no route_order')

    route = []
    previous_end = None
    for index, segment in enumerate(segments, 1):
        name = segment.get('id')
        start, end = segment.get('from'), segment.get('to')
        heading = segment.get('heading')
        if (not name or not isinstance(start, list) or not isinstance(end, list)
                or len(start) != 2 or len(end) != 2 or heading is None):
            raise ValueError('invalid route segment %d' % index)
        start = tuple(float(value) for value in start)
        end = tuple(float(value) for value in end)
        heading = float(heading)
        if previous_end is not None and math.hypot(
                start[0] - previous_end[0], start[1] - previous_end[1]) > 0.03:
            raise ValueError('%s does not connect to the preceding segment' % name)
        if math.hypot(end[0] - start[0], end[1] - start[1]) < 0.03:
            raise ValueError('%s has no usable length' % name)
        if previous_end is None:
            route.append(('START', start[0], start[1], heading))
        distance = math.hypot(end[0] - start[0], end[1] - start[1])
        steps = max(1, int(math.ceil(distance / 0.20)))
        for step in range(1, steps + 1):
            fraction = step / float(steps)
            suffix = '_%02d' % step if steps > 1 else ''
            route.append((name + suffix,
                          start[0] + (end[0] - start[0]) * fraction,
                          start[1] + (end[1] - start[1]) * fraction,
                          heading))
        previous_end = end
    return route


class RouteExecutor:
    def __init__(self):
        self.timeout = float(rospy.get_param('~waypoint_timeout', 90.0))
        contract = rospy.get_param('~route_contract', CONTRACT_PATH)
        self.route = load_route(contract)
        with open(contract, encoding='utf-8') as stream:
            raw_contract = yaml.safe_load(stream)
        self.birth_x, self.birth_y = (float(v) for v in raw_contract['points'][0])
        self.status_pub = rospy.Publisher('/route/status', String, queue_size=1, latch=True)
        self.client = actionlib.SimpleActionClient('/move_base', MoveBaseAction)
        self.cmd_pub = rospy.Publisher('/my_car/cmd_vel_nav', Twist, queue_size=1)

    @staticmethod
    def goal(x, y, yaw):
        goal = MoveBaseGoal()
        goal.target_pose = PoseStamped()
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
        self.status_pub.publish('WAIT_MOVE_BASE')
        ready = self.client.wait_for_server(rospy.Duration(30.0))
        if not ready:
            self.status_pub.publish('FAILED:NO_MOVE_BASE')
            return False
        terminal = {
            GoalStatus.SUCCEEDED, GoalStatus.ABORTED, GoalStatus.PREEMPTED,
            GoalStatus.REJECTED, GoalStatus.RECALLED, GoalStatus.LOST,
        }
        start_index = int(rospy.get_param('~start_waypoint', 0))
        for index, (name, x, y, yaw) in enumerate(self.route[start_index:], start_index + 1):
            if rospy.is_shutdown():
                return False
            self.status_pub.publish('GO:%02d:%s' % (index, name))
            rospy.loginfo('route waypoint %d/%d start: %s (%.2f, %.2f)',
                          index, len(self.route), name, x, y)
            state = GoalStatus.LOST
            for attempt in range(1, 4):
                self.client.send_goal(self.goal(x, y, yaw))
                deadline = time.monotonic() + self.timeout
                while not rospy.is_shutdown() and time.monotonic() < deadline:
                    if self.client.get_state() in terminal:
                        break
                    time.sleep(0.1)
                state = self.client.get_state()
                if state == GoalStatus.SUCCEEDED:
                    break
                rospy.logwarn('route waypoint %d attempt %d ended state=%d', index, attempt, state)
                if state not in (GoalStatus.PREEMPTED, GoalStatus.RECALLED, GoalStatus.LOST):
                    break
                time.sleep(0.5)
            if state != GoalStatus.SUCCEEDED:
                self.client.cancel_goal()
                self.status_pub.publish('FAILED:%02d:%s:%d' % (index, name, state))
                rospy.logerr('route waypoint %d/%d failed: %s state=%d',
                             index, len(self.route), name, state)
                return False
            rospy.loginfo('route waypoint %d/%d reached: %s (%.2f, %.2f)',
                          index, len(self.route), name, x, y)
        return self.precise_park()

    def precise_park(self):
        """Trim the final pose so every completed lap ends in the birth slot."""
        target_x, target_y = self.birth_x, self.birth_y
        target_yaw = float(rospy.get_param('~birth_yaw', 1.606236))
        listener = tf.TransformListener()
        rate = rospy.Rate(20)
        deadline = time.monotonic() + float(rospy.get_param('~park_timeout', 45.0))
        while not rospy.is_shutdown() and time.monotonic() < deadline:
            try:
                stamp = listener.getLatestCommonTime('map', 'base_footprint')
                (x, y, _), (qx, qy, qz, qw) = listener.lookupTransform(
                    'map', 'base_footprint', stamp)
            except (tf.Exception, tf.LookupException, tf.ConnectivityException):
                rate.sleep(); continue
            distance = math.hypot(target_x - x, target_y - y)
            yaw = math.atan2(2.0 * (qw * qz + qx * qy),
                             1.0 - 2.0 * (qy * qy + qz * qz))
            yaw_error = math.atan2(math.sin(target_yaw - yaw),
                                   math.cos(target_yaw - yaw))
            command = Twist()
            if distance > 0.025:
                aim = math.atan2(target_y - y, target_x - x)
                heading_error = math.atan2(math.sin(aim - yaw), math.cos(aim - yaw))
                command.linear.x = max(-0.10, min(0.10, 0.55 * distance * math.cos(heading_error)))
                command.angular.z = max(-0.25, min(0.25, heading_error))
            elif abs(yaw_error) > 0.025:
                command.angular.z = max(-0.28, min(0.28, 1.2 * yaw_error))
            else:
                self.cmd_pub.publish(Twist())
                self.status_pub.publish('COMPLETE_PARKED')
                rospy.loginfo('precise parking complete: x=%.3f y=%.3f yaw=%.3f', x, y, yaw)
                return True
            self.cmd_pub.publish(command)
            rate.sleep()
        self.cmd_pub.publish(Twist())
        self.status_pub.publish('FAILED:PRECISE_PARK')
        rospy.logerr('precise parking failed to reach %.3f %.3f yaw %.3f', target_x, target_y, target_yaw)
        return False


if __name__ == '__main__':
    # A stable name lets runtime_control cancel this mission before a new
    # navigation mode starts; anonymous names leave stale action clients.
    rospy.init_node('route_executor')
    raise SystemExit(0 if RouteExecutor().run() else 1)
