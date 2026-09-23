#!/usr/bin/env python3
import json, math, random, time
import actionlib, rospy
from actionlib_msgs.msg import GoalStatus
from nav_msgs.msg import OccupancyGrid
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import PoseStamped
from tf.transformations import quaternion_from_euler
import tf2_ros

def main():
    rospy.init_node('random_reachability_test')
    grid=rospy.wait_for_message('/move_base/global_costmap/costmap', OccupancyGrid, 20)
    tfbuf=tf2_ros.Buffer(cache_time=rospy.Duration(30)); tf2_ros.TransformListener(tfbuf)
    ac=actionlib.SimpleActionClient('/move_base',MoveBaseAction)
    if not ac.wait_for_server(rospy.Duration(20)): raise RuntimeError('move_base unavailable')
    info=grid.info; rng=random.Random(rospy.get_param('~seed', int(time.time())))
    candidates=[]
    radius=5
    for cy in range(radius,info.height-radius):
        for cx in range(radius,info.width-radius):
            if grid.data[cy*info.width+cx] >= 50: continue
            ok=True
            for dy in range(-radius,radius+1):
                for dx in range(-radius,radius+1):
                    if dx*dx+dy*dy<=radius*radius and (grid.data[(cy+dy)*info.width+cx+dx] < 0 or grid.data[(cy+dy)*info.width+cx+dx] >= 50):
                        ok=False; break
                if not ok: break
            if ok: candidates.append((info.origin.position.x+(cx+.5)*info.resolution,
                                     info.origin.position.y+(cy+.5)*info.resolution))
    rng.shuffle(candidates); goals=[]
    for p in candidates:
        if all(math.hypot(p[0]-q[0],p[1]-q[1])>.3 for q in goals): goals.append(p)
        if len(goals)>=5: break
    if len(goals)<5: raise RuntimeError('only %d clear map goals available'%len(goals))
    result={'goals':goals,'results':[]}
    for x,y in goals:
        t=tfbuf.lookup_transform('map','base_footprint',rospy.Time(0),rospy.Duration(3))
        yaw=math.atan2(y-t.transform.translation.y,x-t.transform.translation.x)
        g=MoveBaseGoal(); g.target_pose.header=PoseStamped().header; g.target_pose.header.frame_id='map'; g.target_pose.header.stamp=rospy.Time.now()
        g.target_pose.pose.position.x=x; g.target_pose.pose.position.y=y
        q=quaternion_from_euler(0,0,yaw); g.target_pose.pose.orientation.z=q[2]; g.target_pose.pose.orientation.w=q[3]
        ac.send_goal(g); ac.wait_for_result(rospy.Duration(120)); state=ac.get_state()
        try:
            end=tfbuf.lookup_transform('map','base_footprint',rospy.Time(0),rospy.Duration(2)).transform.translation
            err=math.hypot(end.x-x,end.y-y); final=[end.x,end.y]
        except Exception: err=None; final=None
        ok=state==GoalStatus.SUCCEEDED; result['results'].append({'goal':[x,y],'state':int(state),'ok':ok,'final':final,'error':err})
        print('goal %.3f %.3f -> %s state=%d error=%s'%(x,y,'PASS' if ok else 'FAIL',state,err),flush=True)
    result['passed']=all(r['ok'] for r in result['results'])
    open('/root/random_reachability_test.json','w').write(json.dumps(result,indent=2))
    return 0 if result['passed'] else 1
if __name__=='__main__': raise SystemExit(main())
