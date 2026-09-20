#!/usr/bin/env python3
"""Bounded simulation checks; --motion explicitly exercises the fixed spawn lane."""
import argparse
import collections
import json
import math
import time
import rospy
from gazebo_msgs.srv import GetLinkState, GetModelState
from nav_msgs.msg import Odometry, OccupancyGrid
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan, Image, PointCloud2
from tf2_msgs.msg import TFMessage
from tf.transformations import euler_from_quaternion


def main():
    parser=argparse.ArgumentParser()
    parser.add_argument('--motion',action='store_true')
    parser.add_argument('--output',default='/root/foundation_checks.json')
    args=parser.parse_args()
    rospy.init_node('foundation_check',anonymous=True)
    counts=collections.Counter(); latest={}; authorities=collections.defaultdict(set)
    def receive(m,topic):
        counts[topic]+=1; latest[topic]=m
    def transforms(m):
        for t in m.transforms:
            authorities[t.child_frame_id].add((t.header.frame_id,m._connection_header.get('callerid','')))
    for t,cls in [('/odom',Odometry),('/scan',LaserScan),('/camera/image_raw',Image),
                  ('/camera/depth/image_raw',Image),('/camera/depth/points',PointCloud2),
                  ('/move_base/local_costmap/costmap',OccupancyGrid)]:
        rospy.Subscriber(t,cls,receive,t,queue_size=1)
    rospy.Subscriber('/tf',TFMessage,transforms,queue_size=100)
    rospy.Subscriber('/tf_static',TFMessage,transforms,queue_size=100)
    time.sleep(5)
    result={'samples':dict(counts),'tf_authorities':{k:sorted(v) for k,v in authorities.items()},'motion':[]}
    assert all(counts[t]>=20 for t in ['/odom','/scan','/camera/image_raw','/camera/depth/image_raw','/camera/depth/points']),result
    assert all(len(v)==1 for v in authorities.values()),result
    assert latest['/odom'].child_frame_id=='base_footprint'
    assert latest['/camera/depth/points'].header.frame_id=='camera_optical_frame'
    link=rospy.ServiceProxy('/gazebo/get_link_state',GetLinkState)
    lidar=link('my_car::lidar_link','world')
    assert lidar.success
    result['lidar_world_z']=lidar.link_state.pose.position.z
    assert .36<result['lidar_world_z']<.41
    if '/move_base/local_costmap/costmap' in latest:
        hist=dict(collections.Counter(latest['/move_base/local_costmap/costmap'].data))
        result['local_costmap_histogram']=hist
        assert hist.get(100,0)>0,'No obstacle marks'
    if args.motion:
        import rosgraph
        pubs=dict(rosgraph.Master(rospy.get_name()).getSystemState()[0])
        assert not pubs.get('/my_car/cmd_vel'),pubs.get('/my_car/cmd_vel')
        pub=rospy.Publisher('/my_car/cmd_vel',Twist,queue_size=1)
        time.sleep(.4)
        def drive(v,w,duration):
            samples=[]; begin=time.monotonic();cmd=Twist();cmd.linear.x=v;cmd.angular.z=w
            start=latest['/odom'].pose.pose
            try:
                while time.monotonic()-begin<duration:
                    scan=latest['/scan']
                    assert (rospy.Time.now()-scan.header.stamp).to_sec()<.5,'stale scan'
                    if v>0:
                        front=[r for i,r in enumerate(scan.ranges) if abs(scan.angle_min+i*scan.angle_increment)<.25 and math.isfinite(r)]
                        assert not front or min(front)>.50,'obstacle ahead'
                    pub.publish(cmd); samples.append(latest['/odom']);time.sleep(.04)
            finally:
                for _ in range(10):pub.publish(Twist());time.sleep(.04)
            end=latest['/odom'].pose.pose
            yaw=lambda p:euler_from_quaternion((p.orientation.x,p.orientation.y,p.orientation.z,p.orientation.w))[2]
            dyaw=math.atan2(math.sin(yaw(end)-yaw(start)),math.cos(yaw(end)-yaw(start)))
            steady=samples[len(samples)//2:]
            item={'command':[v,w], 'duration':duration,'distance':math.hypot(end.position.x-start.position.x,end.position.y-start.position.y),'yaw_change':dyaw,
                  'mean_vx':sum(m.twist.twist.linear.x for m in steady)/len(steady),'mean_vy':sum(m.twist.twist.linear.y for m in steady)/len(steady),
                  'mean_wz':sum(m.twist.twist.angular.z for m in steady)/len(steady)}
            result['motion'].append(item)
            assert abs(item['mean_vy'])<.02,item
            if v:assert abs(item['mean_vx']-v)<.05 and item['distance']>.15,item
            else:assert item['distance']<.05 and abs(dyaw-w*duration)<.20,item
        drive(.15,0,2)
        drive(0,.4,math.pi/2/.4)
        drive(.15,0,2)
        result['stopped_speed']=latest['/odom'].twist.twist.linear.x
        assert abs(result['stopped_speed'])<.02
    result['passed']=True
    with open(args.output,'w') as f:json.dump(result,f,indent=2)
    print(json.dumps(result,indent=2),flush=True)


if __name__=='__main__':
    main()
