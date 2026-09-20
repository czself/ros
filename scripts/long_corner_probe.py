import rospy, actionlib, math, time
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from gazebo_msgs.msg import ModelStates
from geometry_msgs.msg import Twist
from nav_msgs.msg import Path, OccupancyGrid
from tf.transformations import quaternion_from_euler

rospy.init_node("long_corner", anonymous=True)
s = {}

def model(m):
    try:
        i = m.name.index("my_car")
        p = m.pose[i]
        q = p.orientation
        s["p"] = (round(p.position.x, 2), round(p.position.y, 2),
                  round(math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z)), 2))
    except Exception:
        pass

def cmd(m):
    s["cmd"] = (round(m.linear.x, 3), round(m.angular.z, 3))

def plan(m):
    s["n"] = len(m.poses)

def cost(m):
    s["cost"] = (m.info.origin.position.y, max(m.data), sum(1 for x in m.data if x >= 98))

rospy.Subscriber("/gazebo/model_states", ModelStates, model, queue_size=1)
rospy.Subscriber("/my_car/cmd_vel", Twist, cmd, queue_size=1)
rospy.Subscriber("/move_base/DWAPlannerROS/local_plan", Path, plan, queue_size=1)
rospy.Subscriber("/move_base/local_costmap/costmap", OccupancyGrid, cost, queue_size=1)

cli = actionlib.SimpleActionClient("/move_base", MoveBaseAction)
assert cli.wait_for_server(rospy.Duration(10))
g = MoveBaseGoal()
g.target_pose.header.frame_id = "map"
g.target_pose.header.stamp = rospy.Time.now()
g.target_pose.pose.position.x = 0.825
g.target_pose.pose.position.y = 0.375
y = math.atan2(.375 + 3.9583, .825 - 4.0833)
q = quaternion_from_euler(0, 0, y)
g.target_pose.pose.orientation.z = q[2]
g.target_pose.pose.orientation.w = q[3]
cli.send_goal(g)
start = time.monotonic()
last = start
prev = None
idle = start
while time.monotonic() - start < 150 and not rospy.is_shutdown():
    now = time.monotonic()
    p = s.get("p")
    if p and (prev is None or abs(p[0] - prev[0]) + abs(p[1] - prev[1]) > .05):
        idle = now
        prev = p
    if now - last > .5:
        last = now
        print("t=%.1f p=%s cmd=%s n=%s cost=%s st=%s idle=%.1f" % (now - start, s.get("p"), s.get("cmd"), s.get("n"), s.get("cost"), cli.get_state(), now - idle), flush=True)
    if cli.get_state() in (2, 3, 4, 5):
        break
    rospy.sleep(.05)
print("TERMINAL", cli.get_state(), flush=True)
cli.cancel_goal()
rospy.sleep(.6)