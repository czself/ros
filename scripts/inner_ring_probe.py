#!/usr/bin/env python3
"""Probe candidate inner-ring waypoints against the verified map and move_base.

Each candidate is first checked on the verified4 occupancy grid (pixel free,
with a small clearance), then the robot is asked to navigate there via
move_base.  The goal is to learn which corridor nodes actually form a
closed inner loop back to the birth pose.
"""
import json
import math
import time

import rospy
import actionlib
from geometry_msgs.msg import Twist
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from nav_msgs.msg import OccupancyGrid

MAP = rospy.get_param('/map_server/_map_file', None)
CANDIDATES = [
    (3.90, -2.50, 0.0, 'east-s'),
    (3.90,  0.50, 0.0, 'east-m'),
    (3.90,  2.50, 0.0, 'east-n'),
    (2.30,  3.60, 0.0, 'north-e'),
    (0.00,  3.60, 0.0, 'north-m'),
    (-2.50, 3.60, 0.0, 'north-w'),
    (-3.90, 3.60, 0.0, 'west-n'),
    (-3.90, 1.00, 0.0, 'west-m'),
    (-3.90, -1.10, 0.0, 'west-s'),
    (-3.90, -2.80, 0.0, 'west-s2'),
    (-2.00, -3.90, 0.0, 'south-w'),
    (0.50, -3.90, 0.0, 'south-m'),
    (2.30, -3.90, 0.0, 'south-e'),
    (3.90, -3.20, 0.0, 'east-s2'),
]
BIRTH = (4.0833, -3.9583, math.pi / 2)


def free_at(grid, ox, oy, res, X, Y, clearm=0.03):
    w = grid.info.width
    h = grid.info.height
    top = oy + h * res
    c = int((X - ox) / res)
    r = int((top - Y) / res)
    if not (0 <= c < w and 0 <= r < h):
        return False
    if grid.data[r * w + c] >= 55:
        return False
    step = max(1, int(clearm / res))
    for dy in range(-step, step + 1):
        for dx in range(-step, step + 1):
            cc, rr = c + dx, r + dy
            if not (0 <= cc < w and 0 <= rr < h):
                continue
            if grid.data[rr * w + cc] >= 55:
                return False
    return True


def nav(client, x, y, yaw, timeout=90):
    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id = 'map'
    goal.target_pose.pose.position.x = x
    goal.target_pose.pose.position.y = y
    goal.target_pose.pose.orientation.z = math.sin(yaw / 2)
    goal.target_pose.pose.orientation.w = math.cos(yaw / 2)
    t0 = time.time()
    client.send_goal(goal)
    client.wait_for_result(rospy.Duration(timeout))
    return client.get_state(), time.time() - t0


def main():
    rospy.init_node('inner_ring_probe')
    grid = rospy.wait_for_message('/map', OccupancyGrid, timeout=15)
    ox, oy = grid.info.origin.position.x, grid.info.origin.position.y
    res = grid.info.resolution
    client = actionlib.SimpleActionClient('/move_base', MoveBaseAction)
    client.wait_for_server(rospy.Duration(10))

    log = []
    for x, y, yaw, name in CANDIDATES:
        if not free_at(grid, ox, oy, res, x, y):
            print('SKIP  %-8s (%.2f,%.2f) not free' % (name, x, y))
            log.append({'wp': name, 'x': x, 'y': y, 'state': -1, 'secs': 0.0,
                        'free': False})
            continue
        st, dt = nav(client, x, y, yaw)
        print('%s %-8s (%.2f,%.2f) state=%d %5.1fs' %
              ('OK' if st == 3 else 'FAIL', name, x, y, st, dt))
        log.append({'wp': name, 'x': x, 'y': y, 'state': st,
                    'secs': round(dt, 1), 'free': True})
    ok = [p['wp'] for p in log if p['state'] == 3]
    print('RESULT try order:', ' -> '.join(ok))
    with open('/root/inner_ring_probe.json', 'w') as fh:
        json.dump({'birth': BIRTH[:2], 'candidates': log, 'succeeded': ok},
                  fh, indent=2)


if __name__ == '__main__':
    main()