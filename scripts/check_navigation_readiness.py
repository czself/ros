#!/usr/bin/env python3
"""Fail startup if Gazebo sensor and encoder timestamps are misaligned."""

import json
import time

import actionlib
import rospy
import tf
from move_base_msgs.msg import MoveBaseAction
from nav_msgs.msg import Odometry
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import Image, LaserScan, PointCloud2
from std_msgs.msg import String


BEST_PT_SHA256 = 'fe502091a4e964371eee3b08ec26029ad653019250d5406e13dc68ce8969a2ad'
EXPECTED_CLASSES = sorted([
    'resident', 'stranger', 'red_on', 'red_off', 'yellow_on', 'yellow_off',
    'green_on', 'green_off', 'license_plate'])


def validate_yolo_stream():
    if not rospy.get_param('/cmd_vel_watchdog/enforce_white_lines', False):
        raise RuntimeError('white-line gate is disabled')
    if not rospy.get_param('/cmd_vel_watchdog/enforce_traffic', False):
        raise RuntimeError('traffic-light gate is disabled')
    recent_detections = []
    recent_traffic = []
    recent_images = []
    subscribers = [
        rospy.Subscriber('/inspection/detections', String,
                         lambda msg: recent_detections.append(json.loads(msg.data)), queue_size=10),
        rospy.Subscriber('/inspection/traffic_light', String,
                         lambda msg: recent_traffic.append(json.loads(msg.data)), queue_size=10),
        rospy.Subscriber('/inspection/image', Image,
                         lambda msg: recent_images.append(msg.header.stamp.to_sec()), queue_size=10),
    ]
    error = 'waiting for a fresh matched YOLO frame'
    deadline = time.monotonic() + 6.0
    try:
        while not rospy.is_shutdown() and time.monotonic() < deadline:
            for values in (recent_detections, recent_traffic, recent_images):
                if len(values) > 10:
                    del values[:-10]
            for detections in reversed(recent_detections):
                if detections.get('checkpoint_sha256') != BEST_PT_SHA256:
                    error = 'live YOLO detections are not from the audited best.pt'
                    continue
                if sorted(detections.get('classes', [])) != EXPECTED_CLASSES:
                    error = 'live YOLO class map differs from the audited checkpoint'
                    continue
                source_stamp = float(detections['source_stamp']['seconds'])
                traffic_match = next((item for item in reversed(recent_traffic)
                                      if item.get('checkpoint_sha256') == BEST_PT_SHA256 and
                                      abs(float(item['source_stamp']['seconds'])-source_stamp) <= 1e-6),
                                     None)
                image_match = next((stamp for stamp in reversed(recent_images)
                                    if abs(stamp-source_stamp) <= 1e-6), None)
                now = rospy.Time.now().to_sec()
                if traffic_match is None or image_match is None:
                    error = 'raw-source YOLO record, watchdog state, and annotation are not same-stamp'
                    continue
                if not (-0.05 <= now-source_stamp <= 0.50):
                    error = 'YOLO detection source frame is stale'
                    continue
                return True
            time.sleep(0.02)
    finally:
        for subscriber in subscribers:
            subscriber.unregister()
    raise RuntimeError(error)


def main():
    rospy.init_node('check_navigation_readiness', anonymous=True)
    listener = tf.TransformListener()
    if not rospy.get_param('/cmd_vel_watchdog/enforce_white_lines', False):
        raise SystemExit('Navigation readiness failed: white-line gate must be enabled.')
    if not rospy.get_param('/cmd_vel_watchdog/enforce_traffic', False):
        raise SystemExit('Navigation readiness failed: traffic gate must be enabled.')
    deadline = time.monotonic() + 25.0
    last_error = 'waiting for simulation data'
    while not rospy.is_shutdown() and time.monotonic() < deadline:
        try:
            rospy.wait_for_message('/clock', Clock, timeout=2.0)
            scan = rospy.wait_for_message('/scan', LaserScan, timeout=2.0)
            depth_image = rospy.wait_for_message('/camera/depth/image_raw', Image, timeout=2.0)
            depth_cloud = rospy.wait_for_message('/camera/depth/points', PointCloud2, timeout=2.0)
            wheel = rospy.wait_for_message('/my_car/wheel_odom', Odometry, timeout=2.0)
            odom = rospy.wait_for_message('/odom', Odometry, timeout=2.0)
            stamps = [scan.header.stamp.to_sec(), depth_image.header.stamp.to_sec(),
                      depth_cloud.header.stamp.to_sec(), wheel.header.stamp.to_sec(),
                      odom.header.stamp.to_sec()]
            sim_now = rospy.Time.now().to_sec()
            if max(stamps) - min(stamps) > 0.5:
                raise RuntimeError(
                    'laser and wheel odometry timestamps do not share one clock: '
                    'scan=%.3f wheel=%.3f odom=%.3f' % tuple(stamps))
            if any(abs(stamp - sim_now) > 1.0 for stamp in stamps):
                raise RuntimeError(
                    'sensor timestamps differ from /clock: now=%.3f scan=%.3f '
                    'wheel=%.3f odom=%.3f' % (sim_now, *stamps))
            if not listener.canTransform('map', 'base_footprint', rospy.Time(0)):
                raise RuntimeError('AMCL has not published map -> base_footprint')
            client = actionlib.SimpleActionClient('/move_base', MoveBaseAction)
            if not client.wait_for_server(rospy.Duration(1.0)):
                raise RuntimeError('move_base action server is not ready')
            if rospy.get_param('/move_base/base_local_planner', '') != \
                    'forward_path_follower/ForwardPathFollower':
                raise RuntimeError('forward path follower is not the active local planner')
            observation_sources = rospy.get_param(
                '/move_base/local_costmap/obstacle_layer/observation_sources', '')
            if 'laser' not in observation_sources or 'depth' not in observation_sources:
                raise RuntimeError('local obstacle layer must combine laser and depth')
            validate_yolo_stream()
            print('Navigation ready: white/traffic gates, best.pt, laser+depth, sim time, AMCL TF, and forward path follower agree.',
                  flush=True)
            return 0
        except Exception as error:
            last_error = str(error)
            time.sleep(0.2)
    raise SystemExit('Navigation readiness check failed: ' + last_error)


if __name__ == '__main__':
    main()
