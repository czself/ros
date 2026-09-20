#!/usr/bin/env python3
"""Camera-based traffic-light detector used during the patrol demonstration."""
import cv2
import numpy as np
import rospy
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from std_msgs.msg import String


class VisualInspector:
    def __init__(self):
        self.bridge = CvBridge()
        self.result_pub = rospy.Publisher('/inspection/traffic_light', String, queue_size=1)
        self.image_pub = rospy.Publisher('/inspection/image', Image, queue_size=1)
        rospy.Subscriber('/camera/image_raw', Image, self.on_image, queue_size=1)

    @staticmethod
    def largest_blob(mask):
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        candidates = [cv2.boundingRect(c) for c in contours if cv2.contourArea(c) >= 80]
        return max(candidates, key=lambda box: box[2] * box[3]) if candidates else None

    def on_image(self, message):
        try:
            frame = self.bridge.imgmsg_to_cv2(message, 'bgr8')
        except CvBridgeError as error:
            rospy.logwarn_throttle(5, 'Camera conversion failed: %s', error)
            return

        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        masks = {
            'RED': cv2.inRange(hsv, (0, 100, 100), (10, 255, 255)) |
                   cv2.inRange(hsv, (170, 100, 100), (180, 255, 255)),
            'YELLOW': cv2.inRange(hsv, (18, 100, 110), (38, 255, 255)),
            'GREEN': cv2.inRange(hsv, (42, 90, 80), (90, 255, 255)),
        }
        detections = [(name, self.largest_blob(mask)) for name, mask in masks.items()]
        detections = [(name, box) for name, box in detections if box]
        label = 'NO_TRAFFIC_LIGHT'
        if detections:
            label, box = max(detections, key=lambda item: item[1][2] * item[1][3])
            x, y, width, height = box
            color = {'RED': (0, 0, 255), 'YELLOW': (0, 255, 255), 'GREEN': (0, 255, 0)}[label]
            cv2.rectangle(frame, (x, y), (x + width, y + height), color, 2)
            cv2.putText(frame, 'TRAFFIC: ' + label, (x, max(22, y - 8)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2)
        self.result_pub.publish(String(data=label))
        self.image_pub.publish(self.bridge.cv2_to_imgmsg(frame, 'bgr8'))


if __name__ == '__main__':
    rospy.init_node('visual_inspector')
    VisualInspector()
    rospy.spin()
