#!/usr/bin/env python3
"""Rasterize static collision boxes from the competition SDF into a ROS map."""
import argparse
import math
import os
import xml.etree.ElementTree as ET


RESOLUTION = 0.05
ORIGIN_X = -5.0
ORIGIN_Y = -5.0
WIDTH = 200
HEIGHT = 200


def pose(node):
    values = [float(value) for value in (node.findtext('pose') or '0 0 0 0 0 0').split()]
    return values[0], values[1], values[5]


def compose(first, second):
    x, y, yaw = first
    dx, dy, dyaw = second
    return (
        x + math.cos(yaw) * dx - math.sin(yaw) * dy,
        y + math.sin(yaw) * dx + math.cos(yaw) * dy,
        yaw + dyaw,
    )


def mark_box(image, center_x, center_y, yaw, size_x, size_y):
    half_x, half_y = size_x / 2.0, size_y / 2.0
    radius = math.hypot(half_x, half_y)
    min_col = max(0, int((center_x - radius - ORIGIN_X) / RESOLUTION))
    max_col = min(WIDTH - 1, int((center_x + radius - ORIGIN_X) / RESOLUTION))
    min_row = max(0, int((ORIGIN_Y + HEIGHT * RESOLUTION - center_y - radius) / RESOLUTION))
    max_row = min(HEIGHT - 1, int((ORIGIN_Y + HEIGHT * RESOLUTION - center_y + radius) / RESOLUTION))
    cos_yaw, sin_yaw = math.cos(yaw), math.sin(yaw)

    for row in range(min_row, max_row + 1):
        world_y = ORIGIN_Y + (HEIGHT - row - 0.5) * RESOLUTION
        for col in range(min_col, max_col + 1):
            world_x = ORIGIN_X + (col + 0.5) * RESOLUTION
            local_x = cos_yaw * (world_x - center_x) + sin_yaw * (world_y - center_y)
            local_y = -sin_yaw * (world_x - center_x) + cos_yaw * (world_y - center_y)
            if abs(local_x) <= half_x and abs(local_y) <= half_y:
                image[row][col] = 0


def build_map(world_path):
    image = [[205 for _ in range(WIDTH)] for _ in range(HEIGHT)]
    # The playable floor is a 10 m square, with unknown space outside it.
    for row in range(HEIGHT):
        world_y = ORIGIN_Y + (HEIGHT - row - 0.5) * RESOLUTION
        for col in range(WIDTH):
            world_x = ORIGIN_X + (col + 0.5) * RESOLUTION
            if -4.9 < world_x < 4.9 and -4.9 < world_y < 4.9:
                image[row][col] = 254

    root = ET.parse(world_path).getroot()
    for model in root.findall('./world/model'):
        if model.get('name') == 'my_car' or model.findtext('static') not in ('1', 'true', 'True'):
            continue
        model_pose = pose(model)
        for link in model.findall('link'):
            link_pose = compose(model_pose, pose(link))
            for collision in link.findall('collision'):
                size_text = collision.findtext('./geometry/box/size')
                if not size_text:
                    continue
                size_x, size_y, size_z = [float(value) for value in size_text.split()]
                if size_z < 0.2:
                    continue
                center_x, center_y, yaw = compose(link_pose, pose(collision))
                mark_box(image, center_x, center_y, yaw, size_x, size_y)
    return image


def write_map(image, output_prefix):
    os.makedirs(os.path.dirname(output_prefix), exist_ok=True)
    with open(output_prefix + '.pgm', 'wb') as pgm:
        pgm.write('P5\n{} {}\n255\n'.format(WIDTH, HEIGHT).encode('ascii'))
        pgm.write(bytes(value for row in image for value in row))
    with open(output_prefix + '.yaml', 'w', encoding='ascii') as yaml_file:
        yaml_file.write(
            'image: {}.pgm\nresolution: {:.6f}\norigin: [{:.6f}, {:.6f}, 0.000000]\n'
            'negate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.196\n'.format(
                os.path.basename(output_prefix), RESOLUTION, ORIGIN_X, ORIGIN_Y
            )
        )


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('world')
    parser.add_argument('output_prefix')
    args = parser.parse_args()
    write_map(build_map(args.world), args.output_prefix)
