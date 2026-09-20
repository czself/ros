import rospy, re
from nav_msgs.msg import OccupancyGrid

rospy.init_node("map_verify", anonymous=True)

def host_occ():
    from PIL import Image
    import numpy as np
    im = np.array(Image.open('/home/sz/game/maps/competition_navigation_safe.pgm'))
    txt = open('/home/sz/game/maps/competition_navigation_safe.yaml').read()
    ot = float(re.search(r'occupied_thresh:\s*([\d.]+)', txt).group(1))
    ft = float(re.search(r'free_thresh:\s*([\d.]+)', txt).group(1))
    res = float(re.search(r'resolution:\s*([\d.]+)', txt).group(1))
    origin = [float(v) for v in re.search(r'origin:\s*\[([-\d.\s,]+)\]', txt).group(1).split(',')]
    ox, oy = origin[0], origin[1]
    h, w = im.shape
    grid = np.full((h, w), -1, dtype=np.int8)
    prob = (255 - im.astype(float)) / 255.0
    grid[prob >= ot] = 100
    grid[prob <= ft] = 0
    return grid, res, ox, oy

def sample(grid, res, ox, oy, x, y):
    h = grid.shape[0]
    c = round((x - ox) / res)
    r = h - 1 - round((y - oy) / res)
    return int(grid[r, c])

grid, res, ox, oy = host_occ()
m = rospy.wait_for_message('/map', OccupancyGrid, timeout=8)
mmap = {round((x - m.info.origin.position.x) / m.info.resolution): x for x in range(m.info.width)}
mr = {round((y - m.info.origin.position.y) / m.info.resolution): y for y in range(m.info.height)}
def mget(mm_, x, y):
    c = round((x - mm_.info.origin.position.x) / mm_.info.resolution)
    r = mm_.info.height - 1 - round((y - mm_.info.origin.position.y) / mm_.info.resolution)
    return int(mm_.data[r * mm_.info.width + c])

print('CELLS (x,y): hostPGM map /map ==  equal?')
bad = 0
for x in [2.15, 2.3, 2.5, 2.75, 3.0, 3.2, 3.35]:
    for y in [2.3, 2.55, 2.9, 3.1, 3.3, 3.5]:
        hv = sample(grid, res, ox, oy, x, y)
        mv = mget(m, x, y)
        flag = '' if hv == mv else '  <-- DIFF'
        if hv != mv:
            bad += 1
        print(' (%5.2f,%5.2f) host=%d live=%d%s' % (x, y, hv, mv, flag))
print('diffs:', bad)

g = rospy.wait_for_message('/move_base/global_costmap/costmap', OccupancyGrid, timeout=8)
print('global_costmap info: origin=(%.2f,%.2f) %dx%d res=%.3f' % (g.info.origin.position.x, g.info.origin.position.y, g.info.width, g.info.height, g.info.resolution))
print('GCOST at block (host/map compare cell set):')
for x in [2.15, 2.3, 2.5, 2.75, 3.0, 3.2, 3.35]:
    for y in [2.3, 2.55, 2.9, 3.1, 3.3, 3.5]:
        gv = mget(g, x, y)
        hv = sample(grid, res, ox, oy, x, y)
        if gv > 0 or hv > 0:
            print(' (%5.2f,%5.2f) host=%d gcost=%d%s' % (x, y, hv, gv, '' if (gv >= 98) == (hv >= 98) else ' <-- GCOST extra/missing'))