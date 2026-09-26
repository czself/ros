#!/usr/bin/env python3
"""Reject unsafe RViz goals without silently moving their requested poses."""
import copy
import threading
import rospy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid
from map_msgs.msg import OccupancyGridUpdate
from std_msgs.msg import String
from navigation_goal_safety import (DEFAULT_FOOTPRINT, GridFootprintChecker,
                                    parse_footprint, quaternion_yaw)


class GoalSanitizer:
    def __init__(self):
        self.grid = None
        self.grid_lock = threading.Lock()
        self.safety_margin = float(rospy.get_param('~safety_margin', 0.04))
        self.max_grid_age = float(rospy.get_param('~max_grid_age', 3.0))
        self.output = rospy.Publisher('/sanitized_goal', PoseStamped, queue_size=1)
        self.status = rospy.Publisher('/navigation/goal_sanitizer', String,
                                       queue_size=1, latch=True)
        rospy.Subscriber('/move_base/global_costmap/costmap', OccupancyGrid,
                         self.grid_cb, queue_size=1)
        rospy.Subscriber('/move_base/global_costmap/costmap_updates', OccupancyGridUpdate,
                         self.grid_update_cb, queue_size=10)
        self.input = rospy.Subscriber('/move_base_simple/goal', PoseStamped,
                                      self.goal_cb, queue_size=1)
        self.status.publish('WAIT_GLOBAL_COSTMAP')

    def grid_cb(self, message):
        with self.grid_lock:
            self.grid = copy.deepcopy(message)
            self.grid.data = list(message.data)

    def grid_update_cb(self, message):
        with self.grid_lock:
            grid = self.grid
            if grid is None:
                return
            if (message.header.frame_id != grid.header.frame_id or
                    message.x + message.width > grid.info.width or
                    message.y + message.height > grid.info.height or
                    len(message.data) != message.width * message.height):
                self.grid = None
                self.reject('INVALID_COSTMAP_UPDATE')
                return
            if message.header.stamp < grid.header.stamp:
                return
            for row in range(message.height):
                offset = (message.y + row) * grid.info.width + message.x
                grid.data[offset:offset + message.width] = message.data[
                    row * message.width:(row + 1) * message.width]
            grid.header.stamp = message.header.stamp

    def reject(self, reason):
        self.status.publish('REJECTED:' + reason)
        rospy.logwarn('Navigation goal rejected: %s', reason)

    def goal_cb(self, message):
        with self.grid_lock:
            grid = copy.deepcopy(self.grid)
        if grid is None:
            self.reject('NO_GLOBAL_COSTMAP')
            return
        if message.header.frame_id != grid.header.frame_id:
            self.reject('GOAL_FRAME_%s_EXPECTED_%s' %
                        (message.header.frame_id, grid.header.frame_id))
            return
        age = (rospy.Time.now() - grid.header.stamp).to_sec()
        if age < -0.1 or age > self.max_grid_age:
            self.reject('STALE_GLOBAL_COSTMAP:%.2fs' % age)
            return
        try:
            footprint = parse_footprint(rospy.get_param(
                '/move_base/global_costmap/footprint', DEFAULT_FOOTPRINT))
            padding = float(rospy.get_param('/move_base/global_costmap/footprint_padding', 0.0))
            checker = GridFootprintChecker.from_message(
                grid, footprint, safety_margin=self.safety_margin + padding)
            pose = message.pose
            result = checker.check_pose(pose.position.x, pose.position.y,
                                        quaternion_yaw(pose.orientation))
        except (ValueError, TypeError, SyntaxError, IndexError) as error:
            self.reject('INVALID_INPUT:%s' % error)
            return
        if not result.safe:
            self.reject('%s:CELL=%s' % (result.reason, result.cell))
            return
        accepted = copy.deepcopy(message)
        accepted.header.stamp = rospy.Time.now()
        self.output.publish(accepted)
        self.status.publish('ACCEPTED:%.3f:%.3f' %
                            (accepted.pose.position.x, accepted.pose.position.y))


if __name__ == '__main__':
    rospy.init_node('goal_sanitizer')
    GoalSanitizer()
    rospy.spin()
