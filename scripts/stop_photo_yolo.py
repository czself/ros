#!/usr/bin/env python3
"""Drive to a standoff pose in front of a standee, stop, snapshot the camera,
run YOLOv8 (person) inference on the frame, save annotated image + JSON evidence
and publish the result on /inspection/yolo.

Usage: stop_photo_yolo.py <x> <y> <yaw> <label> [conf_thr]
"""
import json
import os
import sys
import time

import cv2
import numpy as np
import rospy
import tf
import math
from cv_bridge import CvBridge
from actionlib import SimpleActionClient
from geometry_msgs.msg import Twist
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from sensor_msgs.msg import Image
from std_msgs.msg import String
from ultralytics import YOLO

MODEL = os.environ.get("YOLO_MODEL", "/root/yolo/yolov8n.pt")
CAMERA_TOPIC = os.environ.get("CAMERA_TOPIC", "/camera_rgb/image_raw")


class Capture:
    def __init__(self):
        self.bridge = CvBridge()
        self.frame = None
        self.stamp = None
        rospy.Subscriber(CAMERA_TOPIC, Image, self.on_img, queue_size=1)
        rospy.wait_for_message(CAMERA_TOPIC, Image, timeout=15)

    def on_img(self, msg):
        try:
            self.frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except Exception:
            return
        self.stamp = msg.header.stamp

    def grab(self, n=3):
        frames = []
        for _ in range(n):
            if self.frame is not None:
                frames.append(self.frame.copy())
            rospy.sleep(0.1)
        return frames


def get_yaw():
    tm = tf.TransformListener()
    tm.waitForTransform("map", "base_footprint", rospy.Time(), rospy.Duration(2))
    _, q = tm.lookupTransform("map", "base_footprint", rospy.Time(0))
    return math.atan2(2 * (q[3] * q[2] + q[0] * q[1]), 1 - 2 * (q[1] ** 2 + q[2] ** 2))


def move_to(ac, x, y, yaw, timeout=90):
    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id = "map"
    goal.target_pose.header.stamp = rospy.Time.now()
    goal.target_pose.pose.position.x = x
    goal.target_pose.pose.position.y = y
    goal.target_pose.pose.orientation.z = math.sin(yaw / 2.0)
    goal.target_pose.pose.orientation.w = math.cos(yaw / 2.0)
    t0 = time.time()
    ac.send_goal(goal)
    done = ac.wait_for_result(rospy.Duration(timeout))
    state = ac.get_state()
    return done and state == 3, time.time() - t0, state


def settled(cmd_pub):
    """Small yaw correction (externally safe when nav left a modest residual
    offset <~1.0 rad), then hold still so the frame is sharp."""
    try:
        err = get_yaw()
    except Exception:
        err = 0.0
    rate = rospy.Rate(20)
    deadline = time.time() + 6.0
    while time.time() < deadline and abs(err) > 0.05:
        w = 0.45 * math.copysign(1.0, err) if abs(err) > 0.35 else 0.35 * err
        tw = Twist()
        tw.angular.z = w
        cmd_pub.publish(tw)
        rate.sleep()
        try:
            err = get_yaw()
        except Exception:
            break
    cmd_pub.publish(Twist())
    rospy.sleep(1.0)


def run(yolo, x, y, yaw, label, conf_thr):
    ac = SimpleActionClient("/move_base", MoveBaseAction)
    ac.wait_for_server(rospy.Duration(10))
    cap = Capture()
    solved, dt, state = move_to(ac, x, y, yaw)
    cmd_pub = rospy.Publisher("/my_car/cmd_vel_nav", Twist, queue_size=1)
    settled(cmd_pub)
    frames = cap.grab(3)
    if not frames:
        rospy.logerr("no camera frame")
        return False

    best = None
    ann = None
    for frame in frames:
        res = yolo.predict(frame, conf=conf_thr, verbose=False)[0]
        det = []
        for box in res.boxes:
            cls = int(box.cls[0])
            conf = float(box.conf[0])
            x1, y1, x2, y2 = (float(v) for v in box.xyxy[0])
            name = res.names.get(cls, cls)
            det.append({"class": name, "conf": round(conf, 3),
                        "box": [round(x1), round(y1), round(x2), round(y2)]})
            cv2.rectangle(frame, (int(x1), int(y1)), (int(x2), int(y2)),
                          (0, 255, 0), 2)
            cv2.putText(frame, f"{name} {conf:.2f}", (int(x1), max(22, int(y1) - 6)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
        cv2.putText(frame, f"standoff {label} nav={dt:.1f}s", (10, 20),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
        if not det:
            continue
        if best is None or det[0]["conf"] > best["conf"]:
            ann = frame.copy()
            best = {"label": label, "nav_ok": solved, "nav_s": round(dt, 1),
                    "nav_state": state, "detections": det}
            best["conf"] = max(d["conf"] for d in det)

    if best is None and ann is None:
        ann = frames[0].copy()
        best = {"label": label, "nav_ok": solved, "nav_s": round(dt, 1),
                "nav_state": state, "detections": []}

    base = f"/root/yolo/photos/{label}_{int(time.time())}"
    os.makedirs(os.path.dirname(base), exist_ok=True)
    cv2.imwrite(base + ".png", ann)
    with open(base + ".json", "w") as fh:
        json.dump(best, fh, ensure_ascii=False, indent=2)
    pub = rospy.Publisher("/inspection/yolo", String, queue_size=1)
    pub.publish(String(data=json.dumps(best, ensure_ascii=False)))
    rospy.loginfo("RESULT %s conf=%.0f%% det=%d saved=%s",
                  best["label"],
                  (best.get("conf") or 0) * 100,
                  len(best["detections"]), base + ".png")
    return len(best["detections"]) > 0


if __name__ == "__main__":
    x, y, yaw, label = float(sys.argv[1]), float(sys.argv[2]), float(sys.argv[3]), sys.argv[4]
    conf_thr = float(sys.argv[5]) if len(sys.argv) > 5 else 0.3
    rospy.init_node("stop_photo_yolo")
    model = YOLO(MODEL)
    ok = run(model, x, y, yaw, label, conf_thr)
    sys.exit(0 if ok else 1)