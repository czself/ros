import rospy, re
from nav_msgs.msg import OccupancyGrid
from PIL import Image
import numpy as np

rospy.init_node("patch_id2", anonymous=True)
m = rospy.wait_for_message('/map', OccupancyGrid, timeout=8)
lm = np.array(m.data).reshape(m.info.height, m.info.width).astype(np.int8)
locc = (lm == 100)
lfree = (lm == 0)

def decode(name, base, conv):
    im = np.array(Image.open('%s/%s.pgm' % (base, name)))
    y = open('%s/%s.yaml' % (base, name)).read()
    res = float(re.search(r'resolution:\s*([\d.]+)', y).group(1))
    ox, oy = [float(v) for v in re.search(r'origin:\s*\[([-\d.\s,]+)\]', y).group(1).split(',')][:2]
    h, w = im.shape
    grid = np.full((h, w), -1, dtype=np.int8)
    ft = float(re.search(r'free_thresh:\s*([\d.]+)', y).group(1)) * 255
    ot = float(re.search(r'occupied_thresh:\s*([\d.]+)', y).group(1)) * 255
    if conv == 'low=occ':
        grid[im < ft] = 0
        grid[im >= ot] = 100
    else:
        grid[im >= ot] = 0
        grid[im < ft] = 100
    g = grid[int(m.info.origin.position.y - oy) / res:]  # not used
    return grid, res, ox, oy

base = '/root/ros1_ws/maps'
for conv in ['low=occ', 'high=occ']:
    print('CONVERSION pixel-' + conv)
    for name in ['competition_navigation_safe', 'competition_hybrid_navigation', 'competition_slam_verified', 'competition_slam_demo', 'competition_slam', 'competition_ground_truth']:
        try:
            g, res, ox, oy = decode(name, base, conv)
        except Exception:
            continue
        if (g.shape[0], g.shape[1]) != (m.info.height, m.info.width):
            continue
        occc = (g == 100); freec = (g == 0); unkc = (g == -1)
        agree = int((occc & locc).sum() + (freec & lfree).sum() + (unkc & (lm == -1)).sum())
        total = g.size
        print('  %-28s agree=%d/%d (%.1f%%)  occ=%d free=%d unk=%d' % (
            name, agree, total, 100.0 * agree / total, int(occc.sum()), int(freec.sum()), int(unkc.sum())))