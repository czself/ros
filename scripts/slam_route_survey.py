#!/usr/bin/env python3
"""Scan a prescribed corridor loop using SLAM feedback and live laser clearance."""
import json
import math
import os
import time

import numpy as np
import rospy
import tf
import yaml
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry, OccupancyGrid
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
from run_artifacts import write_run_summary
from navigation_goal_safety import DEFAULT_FOOTPRINT


def norm(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def anchored_route(points, birth_yaw, anchor):
    """Express the configured relative route in the initial SLAM frame."""
    c, s = math.cos(anchor[2]-birth_yaw), math.sin(anchor[2]-birth_yaw)
    bx, by = points[0]
    return [(anchor[0]+c*(x-bx)-s*(y-by),
             anchor[1]+s*(x-bx)+c*(y-by)) for x, y in points[1:]]


def laser_motion_clear(scan, laser_offset, speed, yaw_rate):
    """Reject a half-second swept footprint that intersects measured returns."""
    ranges = np.asarray(scan.ranges, dtype=float)
    valid = np.isfinite(ranges) & (ranges >= scan.range_min) & (ranges <= scan.range_max)
    if valid.sum() < 80:
        return False
    angles = scan.angle_min+np.arange(len(ranges))*scan.angle_increment
    px = ranges[valid]*np.cos(angles[valid])+laser_offset[0]
    py = ranges[valid]*np.sin(angles[valid])+laser_offset[1]
    margin = .035
    xmin, xmax = min(p[0] for p in DEFAULT_FOOTPRINT)-margin, max(p[0] for p in DEFAULT_FOOTPRINT)+margin
    ymin, ymax = min(p[1] for p in DEFAULT_FOOTPRINT)-margin, max(p[1] for p in DEFAULT_FOOTPRINT)+margin
    for elapsed in np.linspace(0., .6, 13):
        angle = yaw_rate*elapsed
        dx = speed*elapsed if abs(yaw_rate)<1e-6 else speed*math.sin(angle)/yaw_rate
        dy = 0. if abs(yaw_rate)<1e-6 else speed*(1-math.cos(angle))/yaw_rate
        c, s = math.cos(angle), math.sin(angle)
        x, y = c*(px-dx)+s*(py-dy), -s*(px-dx)+c*(py-dy)
        if np.any((x>=xmin)&(x<=xmax)&(y>=ymin)&(y<=ymax)):
            return False
    return True


class SlamSurvey:
    def __init__(self):
        with open(rospy.get_param('~route_contract', '/root/navigation/inner_route.yaml')) as stream:
            self.contract=yaml.safe_load(stream)
        self.directory=rospy.get_param('~output_dir')
        self.odom=None
        self.scan=None
        self.listener=tf.TransformListener()
        self.publisher=rospy.Publisher('/my_car/cmd_vel',Twist,queue_size=1)
        self.status=rospy.Publisher('/slam_survey/status',String,queue_size=1,latch=True)
        rospy.Subscriber('/odom',Odometry,lambda msg:setattr(self,'odom',msg),queue_size=1)
        rospy.Subscriber('/scan',LaserScan,lambda msg:setattr(self,'scan',msg),queue_size=1)
        rospy.on_shutdown(self.stop)
        self.records=[]
        self.index=0
        self.goals=[]

    def stop(self):
        self.publisher.publish(Twist())

    def pose(self):
        now=rospy.Time.now()
        if self.scan is None or self.odom is None:
            raise RuntimeError('waiting for laser and encoder odometry')
        for msg in (self.scan,self.odom):
            if not -.1 <= (now-msg.header.stamp).to_sec() <= .5:
                raise RuntimeError('stale sensor')
        stamp=self.listener.getLatestCommonTime('map','base_footprint')
        if not -.2 <= (now-stamp).to_sec() <= .6:
            raise RuntimeError('stale SLAM transform')
        p,q=self.listener.lookupTransform('map','base_footprint',rospy.Time(0))
        return p[0],p[1],tf.transformations.euler_from_quaternion(q)[2]

    def run(self):
        started=time.monotonic()
        success=False
        reason='shutdown'
        try:
            deadline=started+10
            while time.monotonic()<deadline:
                try:
                    anchor=self.pose()
                    break
                except (RuntimeError,tf.Exception):
                    self.stop()
                    time.sleep(.1)
            else:
                raise RuntimeError('SLAM startup sensors unavailable')
            self.goals=anchored_route(self.contract['points'],self.contract['birth_yaw'],anchor)
            offset,_=self.listener.lookupTransform('base_footprint',self.scan.header.frame_id,rospy.Time(0))
            self.status.publish('MAPPING:0/%d'%len(self.goals))
            progress_time=time.monotonic()
            best_distance=float('inf')
            last_record=0.
            while not rospy.is_shutdown() and self.index<len(self.goals):
                now=time.monotonic()
                if now-started>600:
                    raise RuntimeError('survey timeout')
                x,y,yaw=self.pose()
                gx,gy=self.goals[self.index]
                distance=math.hypot(gx-x,gy-y)
                if now-last_record>.5:
                    self.records.append({'elapsed_s':now-started,'x':x,'y':y,'yaw':yaw,'waypoint':self.index,'distance_m':distance})
                    last_record=now
                if distance<.06:
                    self.index+=1
                    self.stop()
                    best_distance=float('inf')
                    progress_time=now
                    self.status.publish('MAPPING:%d/%d'%(self.index,len(self.goals)))
                    rospy.loginfo('SLAM survey waypoint %d/%d',self.index,len(self.goals))
                    time.sleep(.15)
                    continue
                error=norm(math.atan2(gy-y,gx-x)-yaw)
                command=Twist()
                command.angular.z=max(-.6,min(.6,1.8*error))
                if abs(error)<.22:
                    command.linear.x=min(.25,max(.06,1.0*distance))
                # Heading settles use their own bounded time budget.
                if distance<best_distance-.015:
                    best_distance=distance
                    progress_time=now
                if now-progress_time>25:
                    raise RuntimeError('no route progress')
                if not laser_motion_clear(self.scan,offset[:2],command.linear.x,command.angular.z):
                    raise RuntimeError('laser footprint clearance blocked')
                self.publisher.publish(command)
                time.sleep(.067)
            success=self.index==len(self.goals)
            reason='route_complete' if success else 'shutdown'
            return success
        except Exception as error:
            reason=type(error).__name__+': '+str(error)
            rospy.logerr('SLAM survey stopped: %s',reason)
            return False
        finally:
            for _ in range(5):
                self.stop()
                time.sleep(.067)
            report={'route_status':'COMPLETE_SURVEY' if success else 'FAILED',
                    'stop_reason':reason,'completed_waypoints':self.index,
                    'goal_count':len(self.goals),'duration_s':time.monotonic()-started,
                    'localization':'wheel_encoders+laser_GMapping',
                    'controller_uses_simulator_pose':False,
                    'route_mode':'prescribed_relative_corridor_survey',
                    'goals_map':self.goals,'trajectory':self.records}
            write_run_summary(self.directory,report)
            self.status.publish(report['route_status'])


if __name__=='__main__':
    rospy.init_node('slam_route_survey')
    node=SlamSurvey()
    raise SystemExit(0 if node.run() else 1)
