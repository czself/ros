#!/usr/bin/env python3
"""Camera-based traffic-light detector used during the patrol demonstration."""
import cv2
import numpy as np
import rospy
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from std_msgs.msg import Float32, String


class VisualInspector:
    def __init__(self):
        self.bridge = CvBridge()
        self.result_pub = rospy.Publisher('/inspection/traffic_light', String, queue_size=1)
        self.confidence_pub = rospy.Publisher(
            '/inspection/traffic_light_confidence', Float32, queue_size=1
        )
        self.image_pub = rospy.Publisher('/inspection/image', Image, queue_size=1)
        rospy.Subscriber('/camera/image_raw', Image, self.on_image, queue_size=1)

    @staticmethod
    def largest_blob(mask):
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        candidates = [cv2.boundingRect(c) for c in contours if cv2.contourArea(c) >= 16]
        return max(candidates, key=lambda box: box[2] * box[3]) if candidates else None

    def on_image(self, message):
        try:
            frame = self.bridge.imgmsg_to_cv2(message, 'bgr8')
        except CvBridgeError as error:
            rospy.logwarn_throttle(5, 'Camera conversion failed: %s', error)
            return

        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        # The supplied OFF photographs still contain their nominal colour.
        # V>=180 separates an illuminated LED face from that coloured plastic.
        masks = {
            'RED': cv2.inRange(hsv, (0, 100, 180), (10, 255, 255)) |
                   cv2.inRange(hsv, (170, 100, 180), (180, 255, 255)),
            'YELLOW': cv2.inRange(hsv, (18, 100, 180), (38, 255, 255)),
            'GREEN': cv2.inRange(hsv, (42, 90, 180), (90, 255, 255)),
        }
        kernel = np.ones((3, 3), np.uint8)
        masks = {
            name: cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
            for name, mask in masks.items()
        }
        detections = [
            (name, self.largest_blob(mask), cv2.countNonZero(mask))
            for name, mask in masks.items()
        ]
        detections = [(name, box, score) for name, box, score in detections if box]
        label = 'NO_TRAFFIC_LIGHT'
        confidence = 0.0
        if detections:
            label, box, best_score = max(detections, key=lambda item: item[2])
            total_score = sum(item[2] for item in detections)
            confidence = float(best_score) / max(1.0, float(total_score))
            x, y, width, height = box
            color = {'RED': (0, 0, 255), 'YELLOW': (0, 255, 255), 'GREEN': (0, 255, 0)}[label]
            cv2.rectangle(frame, (x, y), (x + width, y + height), color, 2)
            cv2.putText(frame, 'TRAFFIC: %s %.2f' % (label, confidence),
                        (x, max(22, y - 8)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2)
        self.result_pub.publish(String(data=label))
        self.confidence_pub.publish(Float32(data=confidence))
        self.image_pub.publish(self.bridge.cv2_to_imgmsg(frame, 'bgr8'))


if __name__ == '__main__':
    rospy.init_node('visual_inspector')
    VisualInspector()
    rospy.spin()
