#!/usr/bin/env python3
"""Live best.pt detector for people, traffic lamps, and plate boxes.

The traffic topic is structured JSON so the watchdog can require both a fresh
camera source stamp and a confidence value before authorizing GREEN.
"""
import json
import hashlib
import math
import os
import threading
import time
from collections import deque

import cv2
import numpy as np
import rospy
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from std_msgs.msg import Float32, String
from ultralytics import YOLO


EXPECTED_CLASSES = {
    'resident', 'stranger', 'red_on', 'red_off', 'yellow_on', 'yellow_off',
    'green_on', 'green_off', 'license_plate',
}
LAMP_STATE = {'red_on': 'RED', 'yellow_on': 'YELLOW', 'green_on': 'GREEN'}
COLORS = {
    'resident': (255, 220, 0), 'stranger': (0, 165, 255),
    'red_on': (0, 0, 255), 'red_off': (0, 0, 150),
    'yellow_on': (0, 255, 255), 'yellow_off': (0, 160, 160),
    'green_on': (0, 255, 0), 'green_off': (0, 130, 0),
    'license_plate': (255, 0, 255),
}


class YoloInspector:
    def __init__(self):
        self.model_path = rospy.get_param('~model', '/root/yolo/best.pt')
        self.conf_threshold = float(rospy.get_param('~conf_threshold', 0.25))
        self.traffic_min_confidence = float(rospy.get_param('~traffic_min_confidence', 0.50))
        self.process_hz = float(rospy.get_param('~process_hz', 5.0))
        if not os.path.isfile(self.model_path):
            raise RuntimeError('YOLO checkpoint is missing: %s' % self.model_path)
        if self.process_hz < 5.0 or not 0.0 < self.conf_threshold < 1.0:
            raise ValueError('process_hz must be >=5 and confidence must be between 0 and 1')
        self.bridge = CvBridge()
        digest = hashlib.sha256()
        with open(self.model_path, 'rb') as stream:
            for block in iter(lambda: stream.read(1024 * 1024), b''):
                digest.update(block)
        self.checkpoint_sha256 = digest.hexdigest()
        self.model = YOLO(self.model_path)
        self.model.to('cpu')
        names = self.model.names
        names = {int(k): str(v) for k, v in names.items()} if isinstance(names, dict) else dict(enumerate(names))
        self.names = names
        absent = EXPECTED_CLASSES - set(names.values())
        if absent:
            raise RuntimeError('checkpoint class map is missing: %s' % sorted(absent))

        self.annotation_pub = rospy.Publisher('/inspection/image', Image, queue_size=1)
        self.detections_pub = rospy.Publisher('/inspection/detections', String, queue_size=2)
        self.traffic_pub = rospy.Publisher('/inspection/traffic_light', String, queue_size=1)
        self.traffic_confidence_pub = rospy.Publisher(
            '/inspection/traffic_light_confidence', Float32, queue_size=1)
        self.metrics_pub = rospy.Publisher('/inspection/yolo/metrics', String,
                                           queue_size=1, latch=True)
        self.lock = threading.Lock()
        self.last_processed_wall = 0.0
        self.last_metric_wall = time.monotonic()
        self.processed = 0
        self.latencies_ms = deque(maxlen=25)
        self.frame_ages_ms = deque(maxlen=25)
        self.source_stamps = deque(maxlen=1000)
        rospy.Subscriber('/camera/image_raw', Image, self.on_image,
                         queue_size=1, buff_size=4 * 1024 * 1024)
        rospy.loginfo('YOLO ready: model=%s classes=%s conf=%.2f traffic_conf=%.2f rate=%.1fHz',
                      self.model_path, sorted(names.values()), self.conf_threshold,
                      self.traffic_min_confidence, self.process_hz)

    @staticmethod
    def stamp_record(stamp):
        return {'secs': int(stamp.secs), 'nsecs': int(stamp.nsecs),
                'seconds': stamp.to_sec()}

    def traffic_state(self, detections):
        candidates = [(LAMP_STATE[d['class']], d['confidence'])
                      for d in detections
                      if d['class'] in LAMP_STATE and
                      d['confidence'] >= self.traffic_min_confidence]
        states = {state for state, _confidence in candidates}
        if len(states) != 1:
            return 'UNKNOWN', 0.0
        state = next(iter(states))
        return state, max(conf for label, conf in candidates if label == state)

    @staticmethod
    def suppress_duplicate_person_boxes(detections, overlap_threshold=0.85):
        """Remove cross-class boxes that describe the same person standee."""
        people = [item for item in detections
                  if item.get('class') in ('resident', 'stranger')]
        keep = []
        for item in sorted(people, key=lambda value: float(value.get('confidence', 0.0)),
                           reverse=True):
            x1, y1, x2, y2 = map(float, item['box'])
            area = max(0.0, x2 - x1) * max(0.0, y2 - y1)
            duplicate = False
            for selected in keep:
                sx1, sy1, sx2, sy2 = map(float, selected['box'])
                intersection = (max(0.0, min(x2, sx2) - max(x1, sx1)) *
                                max(0.0, min(y2, sy2) - max(y1, sy1)))
                selected_area = max(0.0, sx2 - sx1) * max(0.0, sy2 - sy1)
                union = area + selected_area - intersection
                if union > 0.0 and intersection / union >= overlap_threshold:
                    duplicate = True
                    break
            if not duplicate:
                keep.append(item)
        non_people = [item for item in detections
                      if item.get('class') not in ('resident', 'stranger')]
        return non_people + sorted(keep, key=lambda value: float(value['box'][0]))

    def annotate(self, frame, detections, latency_ms, age_ms):
        output = frame.copy()
        for item in detections:
            x1, y1, x2, y2 = item['box']
            color = COLORS.get(item['class'], (255, 255, 255))
            cv2.rectangle(output, (x1, y1), (x2, y2), color, 2)
            cv2.putText(output, '%s %.2f' % (item['class'], item['confidence']),
                        (x1, max(18, y1 - 6)), cv2.FONT_HERSHEY_SIMPLEX,
                        0.48, color, 2, cv2.LINE_AA)
        cv2.putText(output, 'YOLO %.0f ms  age %.0f ms' % (latency_ms, age_ms),
                    (8, 18), cv2.FONT_HERSHEY_SIMPLEX, 0.42, (255, 255, 255), 1,
                    cv2.LINE_AA)
        return output

    def publish_metrics(self, now_wall):
        if now_wall - self.last_metric_wall < 1.0:
            return
        with self.lock:
            lats = np.asarray(self.latencies_ms, dtype=np.float64)
            ages = np.asarray(self.frame_ages_ms, dtype=np.float64)
            stamps = list(self.source_stamps)
            processed = self.processed
            self.processed = 0
            self.last_metric_wall = now_wall
        metrics = {
            'window_s': 1.0,
            'processed_hz': float(processed),
            'p95_latency_ms': float(np.percentile(lats, 95)) if lats.size else None,
            'max_frame_age_ms': float(np.max(ages)) if ages.size else None,
            'last_source_stamp': stamps[-1] if stamps else None,
            'source': 'best.pt',
            'checkpoint_sha256': self.checkpoint_sha256,
            'classes': sorted(self.names.values()),
        }
        self.metrics_pub.publish(String(data=json.dumps(metrics, separators=(',', ':'))))
        rospy.loginfo_throttle(5.0, 'YOLO %.1fHz p95=%.1fms max_age=%.1fms',
                               metrics['processed_hz'], metrics['p95_latency_ms'] or -1.0,
                               metrics['max_frame_age_ms'] or -1.0)

    def on_image(self, message):
        wall_start = time.monotonic()
        if wall_start - self.last_processed_wall < 1.0 / self.process_hz:
            return
        self.last_processed_wall = wall_start
        try:
            frame = self.bridge.imgmsg_to_cv2(message, 'bgr8')
            result = self.model.predict(frame, conf=self.conf_threshold,
                                        imgsz=640, device='cpu', verbose=False)[0]
            detections = []
            for box in result.boxes:
                class_id = int(box.cls[0])
                name = self.names.get(class_id, str(class_id))
                confidence = float(box.conf[0])
                x1, y1, x2, y2 = [int(round(float(v))) for v in box.xyxy[0]]
                detections.append({'class': name, 'confidence': round(confidence, 5),
                                   'box': [x1, y1, x2, y2]})
            detections = self.suppress_duplicate_person_boxes(detections)
            state, confidence = self.traffic_state(detections)
            try:
                age_ms = max(0.0, (rospy.Time.now() - message.header.stamp).to_sec() * 1000.0)
            except Exception:
                age_ms = float('inf')
            annotated = self.annotate(frame, detections,
                                      (time.monotonic()-wall_start)*1000.0, age_ms)
            record = {
                'source_stamp': self.stamp_record(message.header.stamp),
                'frame_seq': int(message.header.seq),
                'width': int(frame.shape[1]), 'height': int(frame.shape[0]),
                'checkpoint_sha256': self.checkpoint_sha256,
                'classes': sorted(self.names.values()),
                'traffic_state': state, 'traffic_confidence': round(confidence, 5),
                'frame_age_ms': round(age_ms, 3),
                'latency_ms': round((time.monotonic()-wall_start)*1000.0, 3),
                'detections': detections,
            }
            payload = json.dumps(record, ensure_ascii=False, separators=(',', ':'))
            traffic = {'state': state, 'confidence': round(confidence, 5),
                       'source_stamp': self.stamp_record(message.header.stamp),
                       'checkpoint_sha256': self.checkpoint_sha256,
                       'frame_age_ms': round(age_ms, 3)}
            annotated_msg = self.bridge.cv2_to_imgmsg(annotated, 'bgr8')
            annotated_msg.header = message.header
            self.annotation_pub.publish(annotated_msg)
            self.detections_pub.publish(String(data=payload))
            self.traffic_pub.publish(String(data=json.dumps(traffic, separators=(',', ':'))))
            self.traffic_confidence_pub.publish(Float32(data=confidence))
            for item in detections:
                rospy.loginfo('DETECTION stamp=%.9f class=%s confidence=%.3f box=%s',
                              message.header.stamp.to_sec(), item['class'],
                              item['confidence'], item['box'])
            with self.lock:
                self.processed += 1
                self.latencies_ms.append(record['latency_ms'])
                self.frame_ages_ms.append(record['frame_age_ms'])
                self.source_stamps.append(record['source_stamp'])
            self.publish_metrics(time.monotonic())
        except Exception as error:
            stamp = self.stamp_record(message.header.stamp)
            traffic = {'state': 'UNKNOWN', 'confidence': 0.0,
                       'source_stamp': stamp, 'checkpoint_sha256': self.checkpoint_sha256,
                       'error': str(error)}
            self.traffic_pub.publish(String(data=json.dumps(traffic, separators=(',', ':'))))
            self.traffic_confidence_pub.publish(Float32(data=0.0))
            rospy.logerr_throttle(1.0, 'YOLO inference failed closed: %s', error)


if __name__ == '__main__':
    rospy.init_node('yolo_inspector')
    YoloInspector()
    rospy.spin()
