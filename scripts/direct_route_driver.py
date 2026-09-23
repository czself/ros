#!/usr/bin/env python3
import math, rospy
from geometry_msgs.msg import Twist
from gazebo_msgs.msg import ModelStates
from sensor_msgs.msg import LaserScan

ROUTE = [(1.70,-0.40),(1.70,0.0),(-1.40,0.0),(-1.55,-0.55),(-0.85,-0.70),(-0.70,-1.25),(1.35,-1.40),(1.70,-0.80),(1.70,-1.60)]
LANE_HALF_WIDTH = 0.30
VEHICLE_HALF_WIDTH = 0.085
LINE_MARGIN = 0.02
MAX_CROSS_TRACK = LANE_HALF_WIDTH - VEHICLE_HALF_WIDTH - LINE_MARGIN

def norm(a): return math.atan2(math.sin(a), math.cos(a))

class Driver:
    def __init__(self):
        self.pose = None; self.scan = None; self.i = 0
        self.pub = rospy.Publisher('/my_car/cmd_vel', Twist, queue_size=1)
        rospy.Subscriber('/gazebo/model_states', ModelStates, self.on_states)
        rospy.Subscriber('/scan', LaserScan, self.on_scan)
    def on_states(self, m):
        if "my_car" not in m.name: return
        m.pose = m.pose[m.name.index("my_car")]
        q=m.pose.orientation
        yaw=math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))
        self.pose=(m.pose.position.x,m.pose.position.y,yaw)
    def on_scan(self,m): self.scan=m
    def front(self):
        if not self.scan: return 99.0
        n=len(self.scan.ranges); vals=self.scan.ranges[n//2-18:n//2+19]
        vals=[x for x in vals if math.isfinite(x) and x>=self.scan.range_min]
        return min(vals) if vals else 99.0
    def cross_track(self, x, y):
        if self.i == 0:
            return 0.0
        px, py = ROUTE[self.i - 1]
        tx, ty = ROUTE[self.i]
        sx, sy = tx - px, ty - py
        length = max(1e-6, math.hypot(sx, sy))
        return abs((x - px) * sy - (y - py) * sx) / length
    def run(self):
        rate=rospy.Rate(15)
        while not rospy.is_shutdown() and self.i<len(ROUTE):
            c=Twist()
            if self.pose is None or self.scan is None: self.pub.publish(c); rate.sleep(); continue
            x,y,yaw=self.pose; tx,ty=ROUTE[self.i]; dx,dy=tx-x,ty-y; d=math.hypot(dx,dy)
            if self.cross_track(x, y) > MAX_CROSS_TRACK + 0.015:
                rospy.logerr('white-line boundary exceeded: cross_track=%.3f m', self.cross_track(x, y))
                break
            if d<0.10:
                rospy.loginfo('direct route waypoint %d/%d reached',self.i+1,len(ROUTE)); self.i+=1; self.pub.publish(c); rate.sleep(); continue
            e=norm(math.atan2(dy,dx)-yaw)
            if self.front()<0.32 and abs(e)<0.35:
                self.pub.publish(c); rate.sleep(); continue
            if abs(e)>0.22: c.angular.z=max(-0.8,min(0.8,1.8*e))
            else:
                c.linear.x=min(0.42,0.35*d); c.angular.z=max(-0.45,min(0.45,1.2*e))
            self.pub.publish(c); rate.sleep()
        self.pub.publish(Twist()); rospy.loginfo('DIRECT_ROUTE_COMPLETE')

if __name__=='__main__':
    raise SystemExit('Retired: guessed route bypassed paint checks. Use start_calibrated_navigation.sh.')
