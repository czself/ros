#!/usr/bin/env python3
"""Short continuation survey from the operator's current pose to the remaining frontier."""
import math, time, rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan

ROUTE=[(3.75,-3.70),(3.75,-2.45),(2.50,-2.35),(1.50,-2.35),(0.50,-2.35),
       (-0.50,-2.35),(-1.50,-2.35),(-2.50,-2.35),(-3.50,-2.35),
       (-3.50,-1.50),(-3.50,-0.60),(-2.50,0.58),(-1.50,0.58),
       (-0.50,0.58),(0.30,0.80),(0.30,1.10)]

def norm(a): return math.atan2(math.sin(a), math.cos(a))

class Survey:
    def __init__(self):
        self.pose=None; self.scan=None; self.i=0
        self.pub=rospy.Publisher('/my_car/cmd_vel',Twist,queue_size=1)
        rospy.Subscriber('/my_car/odom',Odometry,self.odom,queue_size=1)
        rospy.Subscriber('/scan',LaserScan,self.laser,queue_size=1)
    def odom(self,m):
        q=m.pose.pose.orientation
        yaw=math.atan2(2*q.w*q.z,1-2*q.z*q.z)
        self.pose=(m.pose.pose.position.x,m.pose.pose.position.y,yaw)
    def laser(self,m): self.scan=m
    def front(self):
        if not self.scan:return None
        v=[r for i,r in enumerate(self.scan.ranges)
           if abs(self.scan.angle_min+i*self.scan.angle_increment)<.25
           and math.isfinite(r) and r>=self.scan.range_min]
        return min(v) if v else None
    def run(self):
        period=1.0/15.0; start=time.monotonic()
        try:
            while not rospy.is_shutdown() and self.i<len(ROUTE):
                if time.monotonic()-start>360: raise RuntimeError('frontier survey timeout')
                if not self.pose or not self.scan:
                    self.pub.publish(Twist()); time.sleep(period); continue
                x,y,yaw=self.pose; tx,ty=ROUTE[self.i]
                d=math.hypot(tx-x,ty-y)
                if d<.12:
                    self.i+=1; self.pub.publish(Twist()); time.sleep(.5); continue
                front=self.front()
                e=norm(math.atan2(ty-y,tx-x)-yaw); cmd=Twist()
                if front is not None and front<.42 and abs(e)<.20: raise RuntimeError('frontier route blocked')
                if abs(e)>.22: cmd.angular.z=max(-.45,min(.45,1.4*e))
                else: cmd.linear.x=.20; cmd.angular.z=max(-.28,min(.28,1.0*e))
                self.pub.publish(cmd); time.sleep(period)
        finally:
            for _ in range(6): self.pub.publish(Twist()); time.sleep(period)
            rospy.loginfo('frontier survey complete route=%d/%d',self.i,len(ROUTE))

if __name__=='__main__':
    rospy.init_node('frontier_survey'); Survey().run()
