#!/usr/bin/env python3
"""Publish permanent real-white-line walls as an RViz-only map overlay.

GMapping remains the sole publisher of /map during manual mapping.  This
separate, latched OccupancyGrid makes the same permanent lines visible without
altering any live SLAM cells.
"""
import math

import cv2
import numpy as np
import rospy
from nav_msgs.msg import OccupancyGrid

from build_white_line_navigation_map import (
    GROUND_SIZE_M, STOP_LINES, START_LINE, WHITE_THRESHOLD,
    ZEBRA_PIXEL_BOXES, texture_box,
)


def make_overlay(width, height, resolution, origin_x, origin_y):
    texture = cv2.imread(
        '/root/competition_ground_map.png', cv2.IMREAD_GRAYSCALE)
    if texture is None:
        raise RuntimeError('cannot read /root/competition_ground_map.png')
    white = texture >= WHITE_THRESHOLD
    size = texture.shape[0]
    for line in (START_LINE,) + STOP_LINES:
        r0, r1, c0, c1 = texture_box(*line, size)
        white[r0:r1, c0:c1] = False
    for r0, r1, c0, c1 in ZEBRA_PIXEL_BOXES:
        white[r0:r1 + 1, c0:c1 + 1] = False

    image_order = np.zeros((height, width), dtype=bool)
    for row in range(height):
        y_low = origin_y + (height - row - 1) * resolution
        y_high = y_low + resolution
        for col in range(width):
            x_low, x_high = origin_x + col * resolution, origin_x + (col + 1) * resolution
            if x_high < -GROUND_SIZE_M / 2 or x_low > GROUND_SIZE_M / 2:
                continue
            if y_high < -GROUND_SIZE_M / 2 or y_low > GROUND_SIZE_M / 2:
                continue
            r0 = max(0, int(math.floor((.5 - x_high / GROUND_SIZE_M) * size)))
            r1 = min(size, int(math.ceil((.5 - x_low / GROUND_SIZE_M) * size)))
            c0 = max(0, int(math.floor((.5 - y_high / GROUND_SIZE_M) * size)))
            c1 = min(size, int(math.ceil((.5 - y_low / GROUND_SIZE_M) * size)))
            image_order[row, col] = r0 < r1 and c0 < c1 and white[r0:r1, c0:c1].any()
    # OccupancyGrid rows begin at its lower origin, unlike a PGM image.
    return np.where(np.flipud(image_order), 100, -1).astype(np.int8).ravel().tolist()


def main():
    rospy.init_node('white_line_wall_overlay')
    map_info = rospy.wait_for_message('/map', OccupancyGrid, timeout=20.0).info
    grid = OccupancyGrid()
    grid.header.frame_id = 'map'
    grid.info = map_info
    grid.data = make_overlay(map_info.width, map_info.height, map_info.resolution,
                             map_info.origin.position.x, map_info.origin.position.y)
    publisher = rospy.Publisher('/white_line_walls', OccupancyGrid, queue_size=1, latch=True)
    rate = rospy.Rate(1)
    while not rospy.is_shutdown():
        grid.header.stamp = rospy.Time.now()
        publisher.publish(grid)
        rate.sleep()


if __name__ == '__main__':
    main()
