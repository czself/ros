#!/usr/bin/env python3
import math, time, yaml
import rospy, tf
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String

def wrap(a): return math.atan2(math.sin(a), math.cos(a))

class ContinuousRoute:
    def __init__(self):
        with open(rospy.get_param('~route','/root/navigation/inner_route.yaml')) as f:
            self.points=[tuple(map(float,p)) for p in yaml.safe_load(f)['points']]
        self.tf=tf.TransformListener(); self.i=1; self.pub=rospy.Publisher('/my_car/cmd_vel_nav',Twist,queue_size=1)
        self.status=rospy.Publisher('/route/status',String,queue_size=1,latch=True); self.scan=None
        rospy.Subscriber('/scan',LaserScan,lambda m:setattr(self,'scan',m),queue_size=1)
    def pose(self):
        try:
            t=self.tf.getLatestCommonTime('map','base_footprint')
            (x,y,_),(qx,qy,qz,qw)=self.tf.lookupTransform('map','base_footprint',t)
            yaw=math.atan2(2*(qw*qz+qx*qy),1-2*(qy*qy+qz*qz)); return x,y,yaw
        except (tf.Exception, tf.LookupException, tf.ConnectivityException): return None
    def front(self):
        if self.scan is None:return 9.
        vals=[r for j,r in enumerate(self.scan.ranges) if abs(self.scan.angle_min+j*self.scan.angle_increment)<.25 and math.isfinite(r) and r>=self.scan.range_min]
        return min(vals,default=9.)
    def run(self):
        rate=rospy.Rate(15); started=time.monotonic()
        while not rospy.is_shutdown():
            p=self.pose(); cmd=Twist()
            if p is None or self.scan is None: self.pub.publish(cmd); rate.sleep(); continue
            x,y,yaw=p; tx,ty=self.points[self.i]; d=math.hypot(tx-x,ty-y)
            if self.i==len(self.points)-1 and d<.10:
                # Final parking: settle at the exact birth point and restore
                # the departure heading before declaring the loop complete.
                home_yaw=1.606236
                yaw_err=wrap(home_yaw-yaw)
                if abs(yaw_err)>.035:
                    cmd.angular.z=max(-.35,min(.35,1.4*yaw_err))
                elif d>.025:
                    cmd.linear.x=max(-.07,min(.07,.8*d))
                else:
                    self.pub.publish(Twist()); self.status.publish('COMPLETE_PARKED'); return
                self.pub.publish(cmd); rate.sleep(); continue
            if d<.10:
                self.i=min(self.i+1,len(self.points)-1); continue
            # Aim at the next corner early so the chassis follows one smooth route.
            aim=(tx,ty)
            if d<.10 and self.i+1<len(self.points): aim=self.points[self.i+1]
            err=wrap(math.atan2(aim[1]-y,aim[0]-x)-yaw)
            cmd.angular.z=max(-.50,min(.50,1.15*err))
            cmd.linear.x=0. if abs(err)>.40 else min(.42,max(.08,.65*d))
            if self.front()<.28: cmd.linear.x=0.
            self.pub.publish(cmd); self.status.publish('CONTINUOUS_SEGMENT_%d/%d'%(self.i,len(self.points)-1))
            if time.monotonic()-started>300: self.status.publish('FAILED:TIMEOUT'); return
            rate.sleep()
        self.pub.publish(Twist())

if __name__=='__main__':
    rospy.init_node('continuous_route_nav'); ContinuousRoute().run()
