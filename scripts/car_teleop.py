#!/usr/bin/env python3
"""用于精细建图的脉冲式键盘遥控。"""
import select
import sys
import termios
import time
import tty
import rospy
import os
import json
import math
import cv2
import tf
import numpy as np
from cv_bridge import CvBridge
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image, CameraInfo

LINEAR = 0.10   # m/s
ANGULAR = 0.42  # rad/s
LINEAR_PULSE = 0.30   # 约 3 cm
ANGULAR_PULSE = 0.30  # 约 7 度

KEYMAP = {
    'w': (LINEAR, 0.0), 's': (-LINEAR, 0.0),
    'a': (0.0, ANGULAR), 'd': (0.0, -ANGULAR),
    ' ': (0.0, 0.0),
}


def _image(bridge, message):
    try:
        return bridge.imgmsg_to_cv2(message, 'bgr8')
    except Exception:
        return None


def _depth(bridge, message):
    try:
        depth = np.asarray(bridge.imgmsg_to_cv2(message, 'passthrough'))
        if depth.dtype == np.uint16:
            return depth.astype(np.float32) * 0.001
        return depth.astype(np.float32)
    except Exception:
        return None


def _view_direction(q):
    """World direction of the optical camera's forward (+Z) axis."""
    x, y, z, w = q
    direction = (2 * (x * z + w * y),
                 2 * (y * z - w * x),
                 1 - 2 * (x * x + y * y))
    norm = math.sqrt(sum(value * value for value in direction)) or 1.0
    return [round(value / norm, 6) for value in direction]


def _center_depth_summary(depth):
    if depth is None:
        return None
    h, w = depth.shape[:2]
    patch = depth[max(0, h // 2 - 8):min(h, h // 2 + 8),
                  max(0, w // 2 - 8):min(w, w // 2 + 8)]
    valid = patch[np.isfinite(patch) & (patch > 0.10) & (patch < 10.0)]
    if not valid.size:
        return None
    return {'region': 'image_center_16x16_pixels',
            'valid_pixels': int(valid.size),
            'p10_m': round(float(np.percentile(valid, 10)), 4),
            'median_m': round(float(np.median(valid)), 4),
            'p90_m': round(float(np.percentile(valid, 90)), 4)}

def main():
    rospy.init_node('car_teleop')
    # Feed the safety watchdog input.  Publishing directly to /my_car/cmd_vel
    # races the watchdog's gated output and can make every pulse get replaced
    # by zero before Gazebo applies it.
    pub = rospy.Publisher('/my_car/cmd_vel_nav', Twist, queue_size=1)
    bridge = CvBridge()
    latest_image = [None]
    latest_rgb_stamp = [None]
    latest_depth = [None]
    latest_depth_stamp = [None]
    rgb_info = [None]
    depth_info = [None]

    def on_rgb(message):
        latest_image[0] = _image(bridge, message)
        latest_rgb_stamp[0] = message.header.stamp.to_sec()

    def on_depth(message):
        latest_depth[0] = _depth(bridge, message)
        latest_depth_stamp[0] = message.header.stamp.to_sec()

    rospy.Subscriber('/camera/image_raw', Image, on_rgb, queue_size=1)
    rospy.Subscriber('/camera/depth/image_raw', Image, on_depth, queue_size=1)
    rospy.Subscriber('/camera/camera_info', CameraInfo,
                     lambda msg: rgb_info.__setitem__(0, msg), queue_size=1)
    rospy.Subscriber('/camera/depth/camera_info', CameraInfo,
                     lambda msg: depth_info.__setitem__(0, msg), queue_size=1)
    listener = tf.TransformListener()
    output_dir = '/root/ros1_ws/photo_stops/standee_capture_20260924'
    os.makedirs(output_dir, exist_ok=True)
    manifest_path = os.path.join(output_dir, 'points.json')
    points = []
    if os.path.exists(manifest_path):
        try:
            with open(manifest_path, encoding='utf-8') as stream:
                points = json.load(stream)
        except (OSError, ValueError):
            points = []
    print(
        "遥控: w/s 前后约3cm, a/d 转向约7度, 空格急停; "
        "目标完整入镜并对准中央后，按 1-9 或 0 停车记录(0为POINT_10), q退出"
    )
    fd = sys.stdin.fileno()
    old = termios.tcgetattr(fd)
    command = Twist()
    command_until = 0.0
    try:
        tty.setraw(fd)
        while not rospy.is_shutdown():
            ready, _, _ = select.select([sys.stdin], [], [], 0.02)
            if ready:
                ch = sys.stdin.read(1)
                if ch == 'q':
                    break
                if ch in '1234567890':
                    pub.publish(Twist())
                    rospy.sleep(0.25)
                    try:
                        stamp = rospy.Time(0)
                        listener.waitForTransform('map', 'base_footprint', stamp, rospy.Duration(2))
                        (x, y, _), q = listener.lookupTransform('map', 'base_footprint', stamp)
                        yaw = math.atan2(2 * (q[3] * q[2] + q[0] * q[1]),
                                         1 - 2 * (q[1] * q[1] + q[2] * q[2]))
                        listener.waitForTransform('map', 'camera_optical_frame', stamp, rospy.Duration(2))
                        camera_position, camera_q = listener.lookupTransform(
                            'map', 'camera_optical_frame', stamp)
                        camera_yaw = math.atan2(
                            2 * (camera_q[3] * camera_q[2] + camera_q[0] * camera_q[1]),
                            1 - 2 * (camera_q[1] * camera_q[1] + camera_q[2] * camera_q[2]))
                        point_number = '10' if ch == '0' else ch
                        name = 'POINT_%s' % point_number
                        now = rospy.Time.now().to_sec()
                        if (latest_image[0] is None or latest_depth[0] is None
                                or latest_rgb_stamp[0] is None or latest_depth_stamp[0] is None
                                or abs(now - latest_rgb_stamp[0]) > 0.5
                                or abs(now - latest_depth_stamp[0]) > 0.5
                                or abs(latest_rgb_stamp[0] - latest_depth_stamp[0]) > 0.1):
                            raise RuntimeError('RGB/深度帧缺失或不同步，请稍停后重按数字键')
                        capture_stamp = time.strftime('%Y%m%d_%H%M%S')
                        image_path = os.path.join(output_dir, name + '_' + capture_stamp + '.png')
                        depth_path = os.path.join(output_dir, name + '_' + capture_stamp + '_depth.npy')
                        if not cv2.imwrite(image_path, latest_image[0]):
                            raise RuntimeError('RGB 照片保存失败')
                        np.save(depth_path, latest_depth[0])
                        depth_summary = _center_depth_summary(latest_depth[0])
                        entry = {
                            'name': name,
                            'sequence': int(point_number),
                            'base_pose': {'x': round(x, 4), 'y': round(y, 4),
                                          'yaw': round(yaw, 4), 'frame': 'base_footprint'},
                            'camera_pose': {
                                'frame': 'camera_optical_frame',
                                'x': round(camera_position[0], 4),
                                'y': round(camera_position[1], 4),
                                'z': round(camera_position[2], 4),
                                'yaw': round(camera_yaw, 4),
                                'quaternion': [round(float(v), 6) for v in camera_q],
                                'view_direction_world': _view_direction(camera_q),
                            },
                            'camera_topic': '/camera/image_raw',
                            'rgb_stamp_sec': latest_rgb_stamp[0],
                            'image': image_path,
                            'depth_topic': '/camera/depth/image_raw',
                            'depth_stamp_sec': latest_depth_stamp[0],
                            'depth_file': depth_path,
                            'center_depth_estimate': depth_summary,
                            'rgb_camera_info': ({'width': rgb_info[0].width,
                                                 'height': rgb_info[0].height,
                                                 'K': list(rgb_info[0].K),
                                                 'D': list(rgb_info[0].D)}
                                                if rgb_info[0] else None),
                            'depth_camera_info': ({'width': depth_info[0].width,
                                                   'height': depth_info[0].height,
                                                   'K': list(depth_info[0].K),
                                                   'D': list(depth_info[0].D)}
                                                  if depth_info[0] else None),
                        }
                        points = [p for p in points if p.get('name') != name]
                        points.append(entry)
                        points.sort(key=lambda item: item.get('sequence', 999))
                        with open(manifest_path, 'w', encoding='utf-8') as stream:
                            json.dump(points, stream, ensure_ascii=False, indent=2)
                        print('\n已记录 %s: 底盘(%.3f, %.3f, %.3f), 相机(%.3f, %.3f, %.3f), 中心深度中位数=%s m' %
                              (name, x, y, yaw, camera_position[0], camera_position[1], camera_yaw,
                               (depth_summary or {}).get('median_m', '无有效深度')))
                    except Exception as error:
                        print('\n记录失败: %s' % error)
                    command = Twist()
                    command_until = 0.0
                    continue
                if ch in KEYMAP:
                    command = Twist()
                    command.linear.x, command.angular.z = KEYMAP[ch]
                    duration = LINEAR_PULSE if ch in 'ws' else ANGULAR_PULSE
                    if ch == ' ':
                        duration = 0.0
                    command_until = time.monotonic() + duration

            if time.monotonic() < command_until:
                pub.publish(command)
            else:
                pub.publish(Twist())
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old)
        pub.publish(Twist())  # 退出前停车
        print("\n已退出并停车")

if __name__ == '__main__':
    main()
