#!/usr/bin/env python3
"""Capture a subscriber-activated, co-located RGB camera at plate stops."""
from collections import OrderedDict
import json
import os
import threading

import cv2
import rospy
from sensor_msgs.msg import CameraInfo, Image


class HDPlateCapture:
    def __init__(self, bridge, tf_listener):
        self.bridge, self.tf_listener = bridge, tf_listener
        self.frames = OrderedDict()
        self.info = None
        self.lock = threading.Lock()
        self.subscribers = []
        self.publisher = rospy.Publisher('/inspection/plate_capture', Image, queue_size=1)

    def start(self):
        with self.lock:
            self.frames.clear()
            self.info = None
        self.subscribers = [
            rospy.Subscriber('/ocr_camera/image_raw', Image, self._image,
                             queue_size=1, buff_size=16*1024*1024),
            rospy.Subscriber('/ocr_camera/camera_info', CameraInfo, self._info, queue_size=1)]

    def stop(self):
        for subscriber in self.subscribers:
            subscriber.unregister()
        self.subscribers = []

    def _image(self, message):
        try:
            image = self.bridge.imgmsg_to_cv2(message, 'bgr8')
            key = (message.header.stamp.secs, message.header.stamp.nsecs)
            with self.lock:
                self.frames[key] = image
                while len(self.frames) > 8:
                    self.frames.popitem(last=False)
        except Exception as error:
            rospy.logwarn_throttle(5, 'HD plate image unavailable: %s', error)

    def _info(self, message):
        with self.lock:
            self.info = message

    def save(self, waypoint, record, directory, capture_start):
        plates = [p for p in record.get('detections', [])
                  if p.get('class') == 'license_plate' and p.get('confidence', 0) >= .25
                  and p['box'][0] >= 3 and p['box'][1] >= 3
                  and p['box'][2] <= record['width']-3
                  and p['box'][3] <= record['height']-3]
        if not plates:
            raise RuntimeError('HD_PLATE_NO_COMPLETE_ROI')
        plate = max(plates, key=lambda p: p['confidence'])
        with self.lock:
            frames, info = list(self.frames.items()), self.info
        candidates = []
        low_stamp = record['source_stamp']['seconds']
        for stamp, image in frames:
            seconds = stamp[0]+stamp[1]*1e-9
            if seconds < capture_start or abs(seconds-low_stamp) > .45:
                continue
            h, w = image.shape[:2]
            sx, sy = w/float(record['width']), h/float(record['height'])
            if (w,h) != (1920,1440) or abs(sx-sy) > 1e-6:
                raise RuntimeError('HD_CAMERA_UNEXPECTED_GEOMETRY')
            box = [int(round(v*(sx if i%2==0 else sy))) for i,v in enumerate(plate['box'])]
            x1,y1,x2,y2 = box
            crop = image[y1:y2,x1:x2]
            if crop.size == 0:
                continue
            quality = float(cv2.Laplacian(cv2.cvtColor(crop,cv2.COLOR_BGR2GRAY),cv2.CV_64F).var())
            stamp_ros = rospy.Time(stamp[0],stamp[1])
            try:
                position, quaternion = self.tf_listener.lookupTransform('map','base_footprint',stamp_ros)
            except Exception:
                continue
            candidates.append((quality,stamp,image,box,position,quaternion))
        candidates.sort(key=lambda item: item[0], reverse=True)
        candidates = candidates[:3]
        if len(candidates) < 2 or info is None:
            raise RuntimeError('HD_CAMERA_NEEDS_TWO_FRESH_FRAMES_AND_INTRINSICS')
        job_dir = os.path.join(directory,'ocr')
        os.makedirs(job_dir,exist_ok=True)
        os.chown(job_dir, int(rospy.get_param('~ocr_worker_uid',1000)),
                 int(rospy.get_param('~ocr_worker_gid',1000)))
        job = {'waypoint':waypoint,'version':1,'frames':[],
               'detector_source_stamp':record['source_stamp'],
               'detector_box':plate['box'],'detector_confidence':plate['confidence'],
               'roi_source':'640x480 detector box scaled onto co-located same-FOV 1920x1440 camera',
               'camera_topic':'/ocr_camera/image_raw','width':1920,'height':1440,
               'camera_intrinsics':list(info.K), 'frame':'camera_optical_frame'}
        for index,(quality,stamp,image,box,position,quaternion) in enumerate(candidates):
            stem = os.path.join(job_dir,'%s_frame_%d' % (waypoint,index))
            x1,y1,x2,y2 = box
            if not cv2.imwrite(stem+'.raw.png',image) or not cv2.imwrite(stem+'.crop.png',image[y1:y2,x1:x2]):
                raise RuntimeError('HD_PLATE_IMAGE_WRITE_FAILED')
            job['frames'].append({'raw_image':stem+'.raw.png','crop_image':stem+'.crop.png',
                                  'source_stamp':{'secs':stamp[0],'nsecs':stamp[1],
                                                  'seconds':stamp[0]+stamp[1]*1e-9},
                                  'box':box,'local_sharpness':quality,
                                  'actual_base_pose':{'position':list(position),'quaternion':list(quaternion)},
                                  'rgb_detector_delta_s':abs(stamp[0]+stamp[1]*1e-9-low_stamp)})
        path = os.path.join(job_dir,waypoint+'.job.json')
        with open(path+'.tmp','w',encoding='utf-8') as stream:
            json.dump(job,stream,ensure_ascii=False,indent=2)
        os.replace(path+'.tmp',path)
        selected = candidates[0]
        message = self.bridge.cv2_to_imgmsg(selected[2],'bgr8')
        message.header.stamp = rospy.Time(selected[1][0],selected[1][1])
        message.header.frame_id = 'camera_optical_frame'
        self.publisher.publish(message)
        rospy.loginfo('HD plate %s: %dx%d crop, %d distinct frames, local sharpness %.1f',
                      waypoint,selected[3][2]-selected[3][0],selected[3][3]-selected[3][1],
                      len(candidates),selected[0])
        return {'job':path,'selected':job['frames'][0],'frame_count':len(candidates)}
