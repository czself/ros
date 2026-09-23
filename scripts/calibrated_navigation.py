#!/usr/bin/env python3
"""Ordered inner-corridor tracking with live range vetoes and trajectory audit.

Simulation localization uses Gazebo chassis poses, explicitly recorded in the
evidence. No claim of real-robot localization or depth-only paint detection.
Only cmd_vel_nav is published; the separate watchdog owns the Gazebo command.
"""
import json
import math
import time
import threading
from pathlib import Path
import numpy as np
import rospy
import rosgraph
from gazebo_msgs.msg import ModelStates
from geometry_msgs.msg import Twist, PoseStamped, PolygonStamped, Point32
from nav_msgs.msg import Path as RosPath
from sensor_msgs.msg import LaserScan, PointCloud2
from sensor_msgs import point_cloud2
from std_msgs.msg import String
from route_geometry import PaintGeometry,load_contract,preflight,segment_error,HALF_LENGTH,HALF_WIDTH

def wrap(v): return math.atan2(math.sin(v),math.cos(v))

class Navigation:
    def __init__(self):
        self.contract,self.points=load_contract(rospy.get_param('~contract','/root/navigation/inner_route.yaml'))
        self.finish_yaw=float(self.contract.get('birth_yaw', 1.606236))
        self.geometry=PaintGeometry('/root/competition_ground_map.png')
        failures=preflight(self.geometry,self.points)
        if failures: raise RuntimeError('Offline geometry rejected: %s'%failures)
        self.output=Path(rospy.get_param('~output','/root/calibrated_run.json'))
        self.pose=None; self.twist=None; self.pose_time=0.; self.scan_time=0.; self.depth_time=0.
        self.laser=float('inf'); self.depth=float('inf'); self.turn_clearance=float('inf'); self.lock=threading.Lock()
        self.authorities=[]
        self.segment=0; self.completed=[]; self.samples=[]; self.turns=[]; self.gate=''; self.result=None
        self.started=time.monotonic(); self.last_progress=self.started; self.best_distance=999.
        self.stop_since=None; self.last_save=0.
        self.cmd=rospy.Publisher('/my_car/cmd_vel_nav',Twist,queue_size=1)
        self.status=rospy.Publisher('/route/status',String,queue_size=1,latch=True)
        self.plan_pub=rospy.Publisher('/route/planned',RosPath,queue_size=1,latch=True)
        self.track_pub=rospy.Publisher('/route/actual',RosPath,queue_size=1,latch=True)
        self.foot_pub=rospy.Publisher('/route/footprint',PolygonStamped,queue_size=1)
        self.path=RosPath(); self.path.header.frame_id='odom'
        rospy.Subscriber('/gazebo/model_states',ModelStates,self.on_pose,queue_size=1)
        rospy.Subscriber('/scan',LaserScan,self.on_scan,queue_size=1)
        rospy.Subscriber('/camera/depth/points',PointCloud2,self.on_depth,queue_size=1)
        rospy.Subscriber('/traffic_light/gate_status',String,lambda m:setattr(self,'gate',m.data),queue_size=1)
        rospy.on_shutdown(self.on_shutdown)
        plan=RosPath();plan.header.frame_id='odom'
        for x,y in self.points:
            p=PoseStamped();p.header.frame_id='odom';p.pose.position.x=x;p.pose.position.y=y;p.pose.orientation.w=1.;plan.poses.append(p)
        self.plan_pub.publish(plan)

    def on_pose(self,m):
        if 'my_car' not in m.name: return
        i=m.name.index('my_car');p=m.pose[i];q=p.orientation
        pose=(p.position.x,p.position.y,math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z)))
        with self.lock: self.pose=pose;self.twist=m.twist[i];self.pose_time=time.monotonic()

    def on_scan(self,m):
        # Coordinates relative to chassis, including actual lidar mounting.
        vals=[];radius=[]
        for i,r in enumerate(m.ranges):
            if not math.isfinite(r) or not m.range_min<=r<=m.range_max:continue
            a=m.angle_min+i*m.angle_increment;x=-.035+r*math.cos(a);y=r*math.sin(a)
            if x>.10 and abs(y)<.12: vals.append(x-.094)
            if not(abs(x)<.095 and abs(y)<.085):radius.append(math.hypot(x,y))
        self.laser=min(vals,default=float('inf'));self.scan_time=time.monotonic()
        self.turn_clearance=min(radius,default=float('inf'))

    def on_depth(self,m):
        # Optical xyz -> chassis: forward=.0735+z, left=-x, height=.1645-y.
        # Ignore floor and objects above the chassis, retain upright obstacles.
        uvs=[(u,v) for v in range(0,m.height,6) for u in range(0,m.width,6)]
        values=[]
        for x,y,z in point_cloud2.read_points(m,field_names=('x','y','z'),skip_nans=True,uvs=uvs):
            forward=.0735+z;height=.1645-y
            if .10<forward<2. and abs(x)<.12 and .05<height<.30:
                values.append(forward-.094)
        self.depth=min(values,default=float('inf'));self.depth_time=time.monotonic()

    def publish(self,v=0.,w=0.,status=''):
        m=Twist();m.linear.x=v;m.angular.z=w;self.cmd.publish(m)
        if status:self.status.publish(status)

    def save(self):
        p=self.pose
        record={'result':self.result or 'RUNNING','localization':'Gazebo chassis ground truth (simulation)',
                'signals':'ignored','planned':self.points.tolist(),'completed_segments':self.completed,
                'turns':self.turns,'samples':self.samples,'final_pose':p,
                'authorities':self.authorities,
                'home_error':float(np.linalg.norm(np.array(p[:2])-self.points[0])) if p else None,
                'white_contacts':sum(s['paint'] for s in self.samples),
                'max_cross_track':max((s['cross'] for s in self.samples),default=0.)}
        self.output.write_text(json.dumps(record,indent=2,allow_nan=False))

    def finish(self,result):
        self.result=result;self.publish(status=result);self.save();rospy.loginfo(result)

    def on_shutdown(self):
        self.publish()
        if not self.result:self.result='FAILED:SHUTDOWN'
        self.save()

    def run(self):
        rate=rospy.Rate(20)
        while not rospy.is_shutdown() and not self.result:
            now=time.monotonic()
            if not self.pose or min(self.scan_time,self.depth_time)==0:
                self.publish(status='WAIT_SENSORS');rate.sleep();continue
            with self.lock:pose=self.pose;twist=self.twist;pose_age=now-self.pose_time
            ages=[pose_age,now-self.scan_time,now-self.depth_time]
            if max(ages)>.8:
                self.finish('FAILED:STALE_SENSOR');break
            i=min(self.segment,len(self.points)-2);a,b=self.points[i:i+2]
            along,cross,length=segment_error(pose,a,b)
            paint=self.geometry.collision(*pose)
            self.samples.append({'t':now-self.started,'sim':rospy.Time.now().to_sec(),
                'pose':list(pose),'segment':i,'along':along,'cross':cross,'paint':paint,
                'ages':ages,'gate':self.gate,'laser':self.laser if math.isfinite(self.laser) else None,
                'depth':self.depth if math.isfinite(self.depth) else None,
                'speed':math.hypot(twist.linear.x,twist.linear.y),'yaw_rate':twist.angular.z})
            if paint:self.finish('FAILED:PAINT_CONTACT');break
            if cross>.17 or along<-.17 or along>length+.17:
                self.finish('FAILED:WRONG_CORRIDOR');break
            if now-self.last_save>1.:
                pubs,_,_=rosgraph.Master(rospy.get_name()).getSystemState()
                authorities=dict(pubs)
                raw=authorities.get('/my_car/cmd_vel',[]); nav=authorities.get('/my_car/cmd_vel_nav',[])
                self.authorities.append({'t':now-self.started,'raw':raw,'nav':nav})
                if raw!=['/cmd_vel_watchdog'] or nav!=['/calibrated_navigation']:
                    self.finish('FAILED:COMMAND_OWNERSHIP');break
                self.save();self.last_save=now
                p=PoseStamped();p.header.frame_id='odom';p.pose.position.x=pose[0];p.pose.position.y=pose[1];p.pose.orientation.w=1
                self.path.poses.append(p);self.track_pub.publish(self.path)
            poly=PolygonStamped();poly.header.frame_id='odom'
            c,s=math.cos(pose[2]),math.sin(pose[2])
            for lx,ly in [(HALF_LENGTH,HALF_WIDTH),(HALF_LENGTH,-HALF_WIDTH),(-HALF_LENGTH,-HALF_WIDTH),(-HALF_LENGTH,HALF_WIDTH)]:
                poly.polygon.points.append(Point32(pose[0]+c*lx-s*ly,pose[1]+s*lx+c*ly,0.))
            self.foot_pub.publish(poly)
            if self.segment>=len(self.points)-1:
                home=np.linalg.norm(np.array(pose[:2])-self.points[0])
                yaw_error=wrap(self.finish_yaw-pose[2])
                # The final bay is entered from the southbound lane. Rotate in
                # place until the nose faces the street before accepting finish.
                if home < .10 and abs(yaw_error) > .025:
                    self.publish(0., max(-.45,min(.45,2.0*yaw_error)),
                                 'PARK_ALIGN:yaw_error=%.3f'%yaw_error)
                    self.stop_since=None
                    rate.sleep();continue
                if home > .025:
                    target_angle=math.atan2(self.points[0][1]-pose[1],
                                            self.points[0][0]-pose[0])
                    along=math.cos(target_angle-pose[2])
                    self.publish(max(-.08,min(.08,.7*home*along)), 0.,
                                 'PARK_TRIM:distance=%.3f'%home)
                    self.stop_since=None
                    rate.sleep();continue
                self.publish(status='VERIFY_STOPPED')
                stationary=math.hypot(twist.linear.x,twist.linear.y)<.01 and abs(twist.angular.z)<.03
                if not stationary:self.stop_since=None
                elif self.stop_since is None:self.stop_since=now
                if self.stop_since is not None and now-self.stop_since>1.:
                    home=np.linalg.norm(np.array(pose[:2])-self.points[0])
                    heading_ok=abs(wrap(self.finish_yaw-pose[2]))<.05
                    self.finish('COMPLETE' if home<.08 and heading_ok else
                                ('FAILED:HEADING_ERROR' if home<.08 else 'FAILED:HOME_ERROR'))
                rate.sleep();continue
            dist=float(np.linalg.norm(b-np.array(pose[:2])))
            if dist<.045:
                self.publish();self.completed.append(i);self.segment+=1;self.best_distance=999.;self.last_progress=now
                self.turns.append({'segment_completed':i,'pose':list(pose),'t':now-self.started})
                rospy.loginfo('Segment %d/7 reached at %.3f %.3f',i+1,*pose[:2]);rate.sleep();continue
            if dist<self.best_distance-.008:self.best_distance=dist;self.last_progress=now
            if now-self.last_progress>30.:
                self.finish('FAILED:NO_PROGRESS');break
            # Begin turning before each corner.  A short look-ahead target
            # prevents the faster chassis from overshooting the waypoint.
            lookahead=0.14 if self.segment < len(self.points)-2 else 0.08
            if dist < lookahead and self.segment + 2 < len(self.points):
                nb=self.points[self.segment+2]
                target_angle=math.atan2(nb[1]-pose[1],nb[0]-pose[0])
            else:
                target_angle=math.atan2(b[1]-pose[1],b[0]-pose[0])
            error=wrap(target_angle-pose[2]);w=max(-.55,min(.55,2.0*error))
            v=0. if abs(error)>.15 else min(.34,max(.04,1.15*dist))
            if self.segment < len(self.points)-2 and dist < .32:
                v=min(v,.22)
            # Enter the final bay already aligned with the street-facing
            # heading: rotate before the bay mouth, then reverse the last
            # short distance so the chassis stops square in the slot.
            if self.segment == len(self.points)-2 and dist < .45:
                error=wrap(self.finish_yaw-pose[2])
                w=max(-.55,min(.55,2.0*error))
                v=0. if abs(error)>.08 else -min(.12,max(.035,.8*dist))
            # Brake using live laser and depth clearances; neither is disabled.
            clearance=min(self.laser,self.depth)
            if clearance<.14:v=0.
            elif clearance<.32:v=min(v,.10)
            if abs(w)>.1 and self.turn_clearance<.19:v=w=0.
            # Independent predicted paint veto, including axle rotation drift.
            px,py,theta=pose
            for _ in range(12):
                dt=.035;px+=(v*math.cos(theta)+.0525*w*math.sin(theta))*dt
                py+=(v*math.sin(theta)-.0525*w*math.cos(theta))*dt;theta+=w*dt
                if self.geometry.collision(px,py,theta,.012):v=w=0.;break
            self.publish(v,w,'SEGMENT:%d/7:distance=%.2f:clearance=%.2f'%(i+1,dist,clearance))
            rate.sleep()
        for _ in range(5):self.publish();time.sleep(.05)

if __name__=='__main__':
    rospy.init_node('calibrated_navigation'); Navigation().run()
