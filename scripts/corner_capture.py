import rospy, actionlib, math, time, tf2_ros
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from nav_msgs.msg import Path, OccupancyGrid

rospy.init_node("corner_capture", anonymous=True)
s = {}
tf_buf = tf2_ros.Buffer()
tf2_ros.TransformListener(tf_buf)

def pose():
    try:
        t = tf_buf.lookup_transform("map", "base_footprint", rospy.Time(0))
        q = t.transform.rotation
        yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
        return (t.transform.translation.x, t.transform.translation.y, yaw)
    except Exception:
        return None

def gplan(m):
    s["gplan"] = [(x.pose.position.x, x.pose.position.y) for x in m.poses]

def nplan(m):
    s["nplan"] = [(x.pose.position.x, x.pose.position.y) for x in m.poses]

def lcost(m):
    s["lcost"] = m

def gcost(m):
    s["gcost"] = m

rospy.Subscriber("/move_base/global_plan", Path, gplan, queue_size=1)
rospy.Subscriber("/move_base/NavfnROS/plan", Path, nplan, queue_size=1)
rospy.Subscriber("/move_base/local_costmap/costmap", OccupancyGrid, lcost, queue_size=1)
rospy.Subscriber("/move_base/global_costmap/costmap", OccupancyGrid, gcost, queue_size=1)
rospy.sleep(1.0)

cli = actionlib.SimpleActionClient("/move_base", MoveBaseAction)
assert cli.wait_for_server(rospy.Duration(10))
g = MoveBaseGoal()
g.target_pose.header.frame_id = "map"
g.target_pose.header.stamp = rospy.Time.now()
g.target_pose.pose.position.x = 0.825
g.target_pose.pose.position.y = 0.375
g.target_pose.pose.orientation.w = 1.0
cli.send_goal(g)

def cost_snapshot(c, cx, cy, label):
    if c is None:
        print(label, "NONE", flush=True)
        return
    x0 = c.info.origin.position.x
    y0 = c.info.origin.position.y
    r = c.info.resolution
    w = c.info.width
    h = c.info.height
    hits = []
    for j in range(h):
        for i in range(w):
            v = c.data[j * w + i]
            if v >= 50 and abs(x0 + i * r - cx) < 1.6 and abs(y0 + j * r - cy) < 1.6:
                hits.append((round(x0 + i * r, 2), round(y0 + j * r, 2), v))
    leth = [q for q in hits if q[2] >= 98]
    print(label, "n50=%d lethal=%d" % (len(hits), len(leth)), flush=True)
    print(label, "LE_THAL", sorted(leth), flush=True)

start = time.monotonic()
last = start
prev = None
stuck_since = None
while time.monotonic() - start < 120 and not rospy.is_shutdown():
    now = time.monotonic()
    p = pose()
    if p:
        if prev is None or abs(p[0] - prev[0]) + abs(p[1] - prev[1]) > .05:
            prev = p
            stuck_since = now
    if cli.get_state() in (2, 3, 4, 5):
        print("TERMINAL", cli.get_state(), "pose", p, flush=True)
        break
    if stuck_since and now - stuck_since > 8 and not s.get("dumped"):
        print("STUCK at", p, "st=", cli.get_state(), flush=True)
        if p:
            cost_snapshot(s.get("lcost"), p[0], p[1], "LCOST")
            cost_snapshot(s.get("gcost"), p[0], p[1], "GCOST")
            if s.get("gplan"):
                gp = s["gplan"]
                near = sorted(gp, key=lambda q: (q[0] - p[0]) ** 2 + (q[1] - p[1]) ** 2)[:6]
                print("GPLAN_N", len(gp), flush=True)
                print("GPLAN_HEAD", [tuple(round(a, 2) for a in q) for q in gp[:8]], flush=True)
                print("GPLAN_NEAR", [tuple(round(a, 2) for a in q) for q in near], flush=True)
                print("GPLAN_TAIL", [tuple(round(a, 2) for a in q) for q in gp[-3:]], flush=True)
            if s.get("nplan"):
                npv = s["nplan"]
                print("NPLAN_N", len(npv), "HEAD", [tuple(round(a, 2) for a in q) for q in npv[:6]], flush=True)
        s["dumped"] = True
        break
    if now - last > 2:
        last = now
        print("t=%.0f p=%s st=%s" % (now - start, None if not p else (round(p[0], 2), round(p[1], 2), round(p[2], 2)), cli.get_state()), flush=True)
    rospy.sleep(.05)

if not s.get("dumped") and not cli.get_state() in (2, 3, 4, 5):
    p = pose()
    print("STUCK-END", p, "st", cli.get_state(), flush=True)
    if p:
        cost_snapshot(s.get("lcost"), p[0], p[1], "LCOST")
        cost_snapshot(s.get("gcost"), p[0], p[1], "GCOST")

cli.cancel_goal()
rospy.sleep(.5)