"""Dead-man output and independent map/traffic checks for move_base velocity."""

import math
import time

import rospy
import tf
from geometry_msgs.msg import Twist
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import Float32, String

from .signal import parse_signal


FOOTPRINT = (-0.147, 0.042, -0.085, 0.085)
STOP_LINES = (("NORTH", "y", 1.0, -0.290, 1.70, 0.32),
              ("WEST", "x", -1.0, -0.515, 0.00, 0.38))


def footprint_samples(step=0.01):
    low_x, high_x, low_y, high_y = FOOTPRINT
    nx = int(math.ceil((high_x - low_x) / step))
    ny = int(math.ceil((high_y - low_y) / step))
    return tuple((low_x + (high_x - low_x) * i / nx,
                  low_y + (high_y - low_y) * j / ny)
                 for i in range(nx + 1) for j in range(ny + 1))


SAMPLES = footprint_samples()


class VelocityGate:
    def __init__(self):
        rospy.init_node("navigation_rebuild_velocity_gate")
        self.tf = tf.TransformListener()
        self.grid = None
        self.command = Twist()
        self.command_wall = 0.0
        self.signal = None
        self.signal_count = 0
        self.signal_wall = 0.0
        self.sim_signal = None
        self.remaining_s = 0.0
        self.committed = {line[0]: False for line in STOP_LINES}
        self.status_pub = rospy.Publisher("/navigation_rebuild/gate_status", String, queue_size=1, latch=True)
        self.motor_pub = rospy.Publisher("/my_car/cmd_vel", Twist, queue_size=1)
        rospy.Subscriber("/my_car/cmd_vel_rebuild", Twist, self._on_command, queue_size=1)
        rospy.Subscriber("/map", OccupancyGrid, self._on_map, queue_size=1)
        rospy.Subscriber("/inspection/traffic_light", String, self._on_signal, queue_size=1)
        rospy.Subscriber("/traffic_light/state", String, self._on_sim_signal, queue_size=1)
        rospy.Subscriber("/traffic_light/time_remaining", Float32, self._on_remaining, queue_size=1)
        rospy.Timer(rospy.Duration(0.05), self.tick)
        rospy.on_shutdown(lambda: self.motor_pub.publish(Twist()))

    def _on_command(self, message):
        self.command = message
        self.command_wall = time.monotonic()

    def _on_map(self, message):
        self.grid = message

    def _on_signal(self, message):
        value = parse_signal(message.data)
        self.signal_count = self.signal_count + 1 if value == self.signal else 1
        self.signal = value
        self.signal_wall = time.monotonic()

    def _on_sim_signal(self, message):
        self.sim_signal = message.data.upper()

    def _on_remaining(self, message):
        self.remaining_s = float(message.data)

    def _pose(self):
        stamp = self.tf.getLatestCommonTime("map", "base_footprint")
        if (rospy.Time.now() - stamp).to_sec() > 0.6:
            raise RuntimeError("POSE_STALE")
        p, q = self.tf.lookupTransform("map", "base_footprint", stamp)
        yaw = math.atan2(2 * (q[3] * q[2] + q[0] * q[1]), 1 - 2 * (q[1] ** 2 + q[2] ** 2))
        return p[0], p[1], yaw

    @staticmethod
    def _future(pose, command, duration):
        x, y, yaw = pose
        v, w = command.linear.x, command.angular.z
        if abs(w) < 1e-8:
            return x + v * duration * math.cos(yaw), y + v * duration * math.sin(yaw), yaw
        next_yaw = yaw + w * duration
        return (x + v / w * (math.sin(next_yaw) - math.sin(yaw)),
                y - v / w * (math.cos(next_yaw) - math.cos(yaw)), next_yaw)

    def _map_clear(self, pose):
        grid = self.grid
        if grid is None or grid.info.resolution <= 0:
            return False
        x, y, yaw = pose
        c, s = math.cos(yaw), math.sin(yaw)
        origin = grid.info.origin.position
        resolution = grid.info.resolution
        for dx, dy in SAMPLES:
            wx, wy = x + c * dx - s * dy, y + s * dx + c * dy
            col = math.floor((wx - origin.x) / resolution)
            row = math.floor((wy - origin.y) / resolution)
            if col < 0 or row < 0 or col >= grid.info.width or row >= grid.info.height:
                return False
            value = grid.data[row * grid.info.width + col]
            if value < 0 or value >= 65:
                return False
        return True

    def _stable_green(self, now):
        return (self.signal == "GREEN" and self.signal_count >= 3
                and now - self.signal_wall < 0.7 and self.sim_signal == "GREEN"
                and self.remaining_s >= 2.0)

    def _traffic_command(self, pose, command, now):
        x, y, yaw = pose
        for name, axis, direction, line, lateral_center, lateral_half in STOP_LINES:
            coordinate, lateral = (x, y) if axis == "x" else (y, x)
            if abs(lateral - lateral_center) > lateral_half:
                continue
            direction_component = math.cos(yaw) if axis == "x" else math.sin(yaw)
            if direction * direction_component < 0.65:
                continue
            progress = direction * (coordinate - line)
            front = progress + 0.05
            rear = progress - 0.15
            if front < -0.30:
                self.committed[name] = False
            if self.committed[name]:
                if rear > 0.05:
                    self.committed[name] = False
                else:
                    return command, name + ":CLEARING"
            if front >= -0.10:
                if self._stable_green(now):
                    if command.linear.x > 0:
                        self.committed[name] = True
                    return command, name + ":GREEN"
                stopped = Twist()
                stopped.angular.z = command.angular.z
                return stopped, name + ":WAIT_GREEN"
        return command, "CLEAR"

    def tick(self, _event):
        now = time.monotonic()
        command = self.command if now - self.command_wall <= 0.4 else Twist()
        try:
            pose = self._pose()
        except (tf.Exception, RuntimeError):
            self.motor_pub.publish(Twist())
            self.status_pub.publish(String("POSE_UNAVAILABLE"))
            return
        command, status = self._traffic_command(pose, command, now)
        for index in range(8):
            candidate = self._future(pose, command, index * 0.05)
            if not self._map_clear(candidate):
                command, status = Twist(), "MAP_BLOCKED"
                break
        self.motor_pub.publish(command)
        self.status_pub.publish(String(status))


if __name__ == "__main__":
    VelocityGate()
    rospy.spin()
