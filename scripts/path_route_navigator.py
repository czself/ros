#!/usr/bin/env python3
import math, yaml, rospy, tf
from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Path
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String

def wrap(a): return math.atan2(math.sin(a), math.cos(a))

class PathNavigator:
    def __init__(self):
        with open(rospy.get_param('~route','/root/navigation/inner_route.yaml')) as f:
            pts=[tuple(map(float,p)) for p in yaml.safe_load(f)['points']]
        self.home=pts[0]; self.yaw_home=1.606236; self.path=[]
        for a,b in zip(pts[:-1],pts[1:]):
            n=max(2,int(math.ceil(math.hypot(b[0]-a[0],b[1]-a[1])/.06)))
            for j in range(n):
                u=j/float(n); self.path.append((a[0]+u*(b[0]-a[0]),a[1]+u*(b[1]-a[1])))
        self.path.append(self.home); self.tf=tf.TransformListener(); self.scan=None; self.done=False
        self.pub=rospy.Publisher('/my_car/cmd_vel_nav',Twist,queue_size=1); self.path_pub=rospy.Publisher('/route/planned',Path,queue_size=1,latch=True); self.status=rospy.Publisher('/route/status',String,queue_size=1,latch=True)
        rospy.Subscriber('/scan',LaserScan,lambda m:setattr(self,'scan',m),queue_size=1)
        msg=Path(); msg.header.frame_id='map'
        for x,y in self.path:
            p=PoseStamped(); p.header.frame_id='map'; p.pose.position.x=x; p.pose.position.y=y; p.pose.orientation.w=1.; msg.poses.append(p)
        self.path_pub.publish(msg)
    def pose(self):
        try:
            t=self.tf.getLatestCommonTime('map','base_footprint'); (x,y,_),(qx,qy,qz,qw)=self.tf.lookupTransform('map','base_footprint',t)
            return x,y,math.atan2(2*(qw*qz+qx*qy),1-2*(qy*qy+qz*qz))
        except: return None
    def front(self):
        if self.scan is None:return 9.
        return min((r for i,r in enumerate(self.scan.ranges) if abs(self.scan.angle_min+i*self.scan.angle_increment)<.25 and math.isfinite(r)),default=9.)
    def run(self):
        rate=rospy.Rate(20)
        while not rospy.is_shutdown():
            p=self.pose(); c=Twist()
            if p is None: self.pub.publish(c); rate.sleep(); continue
            x,y,yaw=p; dists=[(math.hypot(px-x,py-y),i) for i,(px,py) in enumerate(self.path)]; _,idx=min(dists)
            if idx>=len(self.path)-2:
                d=math.hypot(self.home[0]-x,self.home[1]-y); e=wrap(self.yaw_home-yaw)
                if d>.025:
                    a=math.atan2(self.home[1]-y,self.home[0]-x); h=wrap(a-yaw); c.linear.x=max(-.08,min(.08,.55*d*math.cos(h))); c.angular.z=max(-.22,min(.22,h))
                elif abs(e)>.025: c.angular.z=max(-.25,min(.25,1.2*e))
                else: self.pub.publish(Twist()); self.status.publish('COMPLETE_PARKED'); return
            else:
                look=min(len(self.path)-1,idx+max(4,int(0.22/.06))); tx,ty=self.path[look]; h=wrap(math.atan2(ty-y,tx-x)-yaw); c.angular.z=max(-.55,min(.55,1.8*h)); c.linear.x=0. if abs(h)>.65 else .34
                if self.front()<.28:c.linear.x=0.
                self.status.publish('PATH_TRACKING:%d/%d'%(idx,len(self.path)))
            self.pub.publish(c); rate.sleep()
        self.pub.publish(Twist())
if __name__=='__main__': rospy.init_node('path_route_navigator'); PathNavigator().run()
