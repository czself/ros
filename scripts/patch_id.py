import rospy, re
from nav_msgs.msg import OccupancyGrid
from PIL import Image
import numpy as np

rospy.init_node("patch_id", anonymous=True)
m = rospy.wait_for_message('/map', OccupancyGrid, timeout=8)
lm = np.array(m.data).reshape(m.info.height, m.info.width).astype(np.int8)
print('live /map: %dx%d origin=(%.3f,%.3f) res=%.3f  free=%d occ=%d unk=%d' % (
    m.info.width, m.info.height, m.info.origin.position.x, m.info.origin.position.y, m.info.resolution,
    int((lm == 0).sum()), int((lm == 100).sum()), int((lm == -1).sum())))

def decode(name, base):
    im = np.array(Image.open('%s/%s.pgm' % (base, name)))
    y = open('%s/%s.yaml' % (base, name)).read()
    res = float(re.search(r'resolution:\s*([\d.]+)', y).group(1))
    ox, oy = [float(v) for v in re.search(r'origin:\s*\[([-\d.\s,]+)\]', y).group(1).split(',')][:2]
    h, w = im.shape
    grid = np.full((h, w), -1, dtype=np.int8)
    # map_server semantic: pixel<=free_thresh*256 -> free, >=occupied_thresh*256 -> occupied
    ft = float(re.search(r'free_thresh:\s*([\d.]+)', y).group(1)) * 256
    ot = float(re.search(r'occupied_thresh:\s*([\d.]+)', y).group(1)) * 256
    grid[im <= ft] = 0
    grid[im >= ot] = 100
    return grid, res, ox, oy

for name in ['competition_navigation_safe', 'competition_hybrid_navigation', 'competition_slam_verified', 'competition_slam_complete']:
    g, res, ox, oy = decode(name, '/root/ros1_ws/maps')
    if (g.shape[0], g.shape[1]) != (m.info.height, m.info.width):
        print('%-28s shape %s != live %s  (skip)' % (name, g.shape, (m.info.height, m.info.width)))
        continue
    # align by origin -> live cells for occupied
    c0 = round((ox - m.info.origin.position.x) / m.info.resolution)
    r0 = round((oy - m.info.origin.position.y) / m.info.resolution)
    occc = g == 100
    locc = lm == 100
    freec = g == 0
    lfree = lm == 0
    same_occ = (occc & locc).sum()
    same_free = (freec & lfree).sum()
    tot_occ = int(occc.sum()); ltot = int(locc.sum())
    print('%-28s occ_file=%d occ_live=%d  FILEocc∩LIVEocc=%d  FILEfree∩LIVEfree=%d' % (
        name, tot_occ, ltot, int(same_occ), int(same_free)))