"""Inspection tasks run only after a navigation goal has been verified."""

import json
import os
import time

import cv2
import rospy
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from std_msgs.msg import Float32, String

from .signal import parse_signal


class TaskHandler:
    def __init__(self, output_dir):
        self.output_dir = output_dir
        self.bridge = CvBridge()
        self.image = None
        self.image_stamp = None
        self.signal = None
        self.signal_count = 0
        self.signal_wall = 0.0
        self.sim_signal = None
        self.remaining_s = 0.0
        rospy.Subscriber("/camera/image_raw", Image, self._on_image, queue_size=1)
        rospy.Subscriber("/inspection/traffic_light", String, self._on_signal, queue_size=1)
        rospy.Subscriber("/traffic_light/state", String, self._on_sim_signal, queue_size=1)
        rospy.Subscriber("/traffic_light/time_remaining", Float32, self._on_remaining, queue_size=1)

    def _on_image(self, message):
        try:
            self.image = self.bridge.imgmsg_to_cv2(message, "bgr8")
            self.image_stamp = message.header.stamp
        except Exception as error:
            rospy.logwarn_throttle(5, "camera conversion failed: %s", error)

    def _on_signal(self, message):
        value = parse_signal(message.data)
        self.signal_count = self.signal_count + 1 if value == self.signal else 1
        self.signal = value
        self.signal_wall = time.monotonic()

    def _on_sim_signal(self, message):
        self.sim_signal = message.data.upper()

    def _on_remaining(self, message):
        self.remaining_s = float(message.data)

    def photo(self, goal):
        # Wait for a frame captured after arrival, never reuse an earlier view.
        started = rospy.Time.now()
        deadline = time.monotonic() + 5.0
        while not rospy.is_shutdown() and time.monotonic() < deadline:
            if self.image is not None and self.image_stamp is not None and self.image_stamp > started:
                break
            time.sleep(0.05)
        else:
            return False, "CAMERA_FRAME_UNAVAILABLE"
        os.makedirs(self.output_dir, exist_ok=True)
        path = os.path.join(self.output_dir, goal.name + ".png")
        if not cv2.imwrite(path, self.image):
            return False, "PHOTO_WRITE_FAILED"
        metadata = {"goal": goal.name, "target": {"x": goal.x, "y": goal.y, "yaw": goal.yaw},
                    "image": path, "stamp": self.image_stamp.to_sec(),
                    "acceptance": "capture_only; semantic inspection pending"}
        with open(os.path.join(self.output_dir, goal.name + ".json"), "w", encoding="utf-8") as stream:
            json.dump(metadata, stream, indent=2)
        return True, path

    def traffic_light(self, timeout_s=30.0):
        deadline = time.monotonic() + timeout_s
        while not rospy.is_shutdown() and time.monotonic() < deadline:
            now = time.monotonic()
            if (self.signal == "GREEN" and self.signal_count >= 3
                    and now - self.signal_wall < 0.7
                    and self.sim_signal == "GREEN" and self.remaining_s >= 2.0):
                return True, "GREEN_CONFIRMED"
            time.sleep(0.05)
        return False, "GREEN_TIMEOUT"

    def run(self, task, goal):
        if task == "none":
            return True, "NO_TASK"
        if task == "photo":
            return self.photo(goal)
        if task == "traffic_light":
            return self.traffic_light()
        return False, "UNKNOWN_TASK"
