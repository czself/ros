#!/usr/bin/env python3
"""Deterministic SLAM survey of the birth-connected corridors."""
import json, math, time, rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
# The fixed birth pose is in the east lane.  First cross above the south
# horizontal wall endpoint to survey the birth-connected lower corridor, then
# return to the east lane and trace the full outer loop.  Waypoints stay well
# clear of collision boxes; SLAM observes walls from the live laser only.
# Collision-clearance route generated from the static wall geometry. The map
# itself is still produced only by live GMapping laser observations.
ROUTE=[
 (4.08,-3.92),(4.08,-3.42),(4.03,-2.92),(3.53,-2.88),(3.03,-2.88),(2.53,-2.88),(2.03,-2.88),(1.53,-2.88),(1.03,-2.88),(0.53,-2.88),(0.03,-2.88),(-0.47,-2.88),(-0.97,-2.88),(-1.47,-2.88),(-1.97,-2.88),(-2.47,-2.88),(-2.98,-2.88),(-3.52,-2.88),(-3.97,-2.62),
 (-3.47,-2.47),(-2.92,-2.47),(-2.42,-2.47),(-1.92,-2.47),(-1.42,-2.47),(-0.92,-2.47),(-0.42,-2.47),(0.08,-2.47),(0.58,-2.47),(1.08,-2.47),(1.58,-2.47),(2.08,-2.27),(2.58,-2.27),(3.08,-2.27),(3.58,-2.12),(3.72,-1.62),(3.72,-1.12),(3.72,-0.62),(3.72,-0.12),(4.03,0.28),(4.03,0.78),(4.03,1.28),(4.03,1.78),(4.03,2.28),(4.03,2.78),(4.03,3.28),(4.03,3.78),
 (3.58,4.03),(3.08,4.03),(2.58,4.03),(2.08,4.03),(1.58,4.03),(1.08,4.03),(0.58,4.03),(0.08,4.03),(-0.42,4.03),(-0.92,4.03),(-1.42,4.03),(-1.92,4.03),(-2.42,4.03),(-2.92,4.03),(-3.42,4.03),(-3.92,4.03),(-3.97,3.53),(-3.97,3.03),(-3.97,2.53),(-3.77,2.03),(-3.77,1.53),(-3.77,1.03),(-3.47,0.62),(-2.98,0.58),(-2.47,0.58),(-1.97,0.58),(-1.47,0.58),(-0.97,0.58),(-0.47,0.58),(0.03,0.58),(0.53,0.58),(0.88,0.98),(0.88,1.48),(1.03,1.98),(1.03,2.48),(1.53,2.53),(1.03,2.53),(0.88,2.03),(0.88,1.53),(0.88,1.03),(0.58,0.62),(0.08,0.58),(-0.42,0.58),(-0.92,0.58),(-1.42,0.58),(-1.92,0.58),(-2.42,0.58),(-2.92,0.58),(-3.42,0.58),(-3.92,0.58),(-3.97,0.08)]
MAX_DURATION=600.0
def norm(a): return math.atan2(math.sin(a),math.cos(a))
class Survey:
 def __init__(self):
  self.pose=None;self.pose_wall=0.0;self.scan=None;self.scan_wall=0.0;self.i=0;self.trajectory=[];self.last_record=0.0;self.stop_reason='route_complete';self.pub=rospy.Publisher('/my_car/cmd_vel',Twist,queue_size=1);rospy.Subscriber('/my_car/odom',Odometry,self.odom,queue_size=1);rospy.Subscriber('/scan',LaserScan,self.laser,queue_size=1)
 def odom(self,m):
  q=m.pose.pose.orientation;y=math.atan2(2*q.w*q.z,1-2*q.z*q.z);p=m.pose.pose.position;self.pose=(p.x,p.y,y);self.pose_wall=time.monotonic()
 def laser(self,m): self.scan=m;self.scan_wall=time.monotonic()
 def front(self):
  if not self.scan:return None
  v=[r for n,r in enumerate(self.scan.ranges) if abs(self.scan.angle_min+n*self.scan.angle_increment)<.25 and math.isfinite(r) and r>=self.scan.range_min]
  return min(v) if v else None
 def run(self):
  period=1.0/15.0
  started=time.monotonic()
  try:
   while not rospy.is_shutdown() and self.i<len(ROUTE):
    if time.monotonic()-started>MAX_DURATION: self.stop_reason='timeout';raise RuntimeError('survey timeout')
    now=time.monotonic()
    if not self.pose or not self.scan:
     self.pub.publish(Twist());time.sleep(period);continue
    if now-self.pose_wall>.5 or now-self.scan_wall>.5:
     self.stop_reason='stale_sensor';raise RuntimeError('stale sensor')
    x,y,yaw=self.pose;tx,ty=ROUTE[self.i];d=math.hypot(tx-x,ty-y)
    now=time.monotonic()
    if now-self.last_record >= 0.5:
     self.trajectory.append({'t':now-started,'x':x,'y':y,'yaw':yaw,'waypoint':self.i});self.last_record=now
    if d<.12:self.i+=1;self.pub.publish(Twist());time.sleep(.8);continue
    e=norm(math.atan2(ty-y,tx-x)-yaw);c=Twist()
    front=self.front()
    if front is None: self.stop_reason='invalid_scan';raise RuntimeError('invalid scan')
    if front<.55 and abs(e)<.8: self.stop_reason='blocked';raise RuntimeError('obstacle blocked route')
    elif abs(e)>.20:c.angular.z=max(-.45,min(.45,1.4*e))
    else:c.linear.x=.25;c.angular.z=max(-.35,min(.35,1.1*e))
    self.pub.publish(c);time.sleep(period)
   rospy.loginfo('survey complete route=%s',self.i)
  finally:
   for _ in range(5): self.pub.publish(Twist()); time.sleep(period)
   try:
    with open('/root/survey_trajectory.json','w') as stream: json.dump({'route':ROUTE,'points':self.trajectory,'stop_reason':self.stop_reason,'completed_waypoints':self.i},stream,indent=2)
   except OSError: pass
if __name__=='__main__':rospy.init_node('survey_mapper');Survey().run()
