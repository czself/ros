#!/usr/bin/env python3
"""Add real permanent white-line obstacles from the 4.2 m ground texture.

The original texture, not an RViz screenshot, is the geometry source.  Normal
white lane/boundary paint becomes lethal in a copy of the latest SLAM map.
The start line, two signal stop lines and two zebra crossings are deliberately
carved out: they have conditional permission and are enforced by the final
traffic-light footprint gate instead of being permanent walls.
"""

import argparse
import hashlib
import json
import re
from pathlib import Path

import cv2
import numpy as np


GROUND_SIZE_M = 4.2
WHITE_THRESHOLD = 220
# axis, coordinate, lateral centre, lateral half-width, half-thickness
START_LINE = ('y', -1.400, 1.700, 0.32, 0.055)
STOP_LINES = (
    ('x', -0.515, 0.00, 0.38, 0.055),
    ('y', -0.290, 1.70, 0.32, 0.055),
)
# Exact pixel boxes of the disconnected zebra strokes in map.png.
ZEBRA_PIXEL_BOXES = ((388, 1934, 6138, 6706), (7543, 8111, 4898, 6444))


def sha256(path):
    digest = hashlib.sha256()
    with open(path, 'rb') as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b''):
            digest.update(chunk)
    return digest.hexdigest()


def yaml_value(text, key):
    found = re.search(r'^%s:\s*(.+?)\s*$' % re.escape(key), text, re.M)
    if not found:
        raise ValueError('missing %s' % key)
    return found.group(1).strip().strip('"\'')


def yaml_origin(text):
    found = re.search(r'^origin:\s*\[([^]]+)\]', text, re.M)
    if not found:
        raise ValueError('missing origin')
    return [float(value.strip()) for value in found.group(1).split(',')]


def texture_box(axis, coordinate, lateral_center, lateral_half, half_thickness, size):
    """Return a clipped texture rectangle for one world-aligned line band."""
    if axis == 'x':
        min_x, max_x = coordinate - half_thickness, coordinate + half_thickness
        min_y, max_y = lateral_center - lateral_half, lateral_center + lateral_half
    else:
        min_x, max_x = lateral_center - lateral_half, lateral_center + lateral_half
        min_y, max_y = coordinate - half_thickness, coordinate + half_thickness
    # Gazebo's ground UV has the texture row axis opposite map/world x.
    row0 = max(0, int(np.floor((.5 - max_x / GROUND_SIZE_M) * size)))
    row1 = min(size, int(np.ceil((.5 - min_x / GROUND_SIZE_M) * size)))
    col0 = max(0, int(np.floor((.5 - max_y / GROUND_SIZE_M) * size)))
    col1 = min(size, int(np.ceil((.5 - min_y / GROUND_SIZE_M) * size)))
    return row0, row1, col0, col1


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('source_yaml', type=Path)
    parser.add_argument('ground_texture', type=Path)
    parser.add_argument('output_basename', type=Path)
    args = parser.parse_args()

    yaml_text = args.source_yaml.read_text(encoding='utf-8')
    source_pgm = args.source_yaml.parent / Path(yaml_value(yaml_text, 'image')).name
    source = cv2.imread(str(source_pgm), cv2.IMREAD_GRAYSCALE)
    texture = cv2.imread(str(args.ground_texture), cv2.IMREAD_GRAYSCALE)
    if source is None or texture is None or texture.shape[0] != texture.shape[1]:
        raise SystemExit('cannot read source map or square ground texture')

    white = texture >= WHITE_THRESHOLD
    size = texture.shape[0]
    # Conditional markings must not become permanent static obstacles.
    for line in (START_LINE,) + STOP_LINES:
        r0, r1, c0, c1 = texture_box(*line, size)
        white[r0:r1, c0:c1] = False
    for r0, r1, c0, c1 in ZEBRA_PIXEL_BOXES:
        white[r0:r1 + 1, c0:c1 + 1] = False

    resolution = float(yaml_value(yaml_text, 'resolution'))
    origin_x, origin_y, _ = yaml_origin(yaml_text)
    output = source.copy()
    marked = np.zeros_like(source, dtype=bool)
    for row in range(source.shape[0]):
        # ROS OccupancyGrid y rises upward; image rows fall downward.
        y_low = origin_y + (source.shape[0] - row - 1) * resolution
        y_high = y_low + resolution
        for col in range(source.shape[1]):
            x_low = origin_x + col * resolution
            x_high = x_low + resolution
            if x_high < -GROUND_SIZE_M / 2 or x_low > GROUND_SIZE_M / 2:
                continue
            if y_high < -GROUND_SIZE_M / 2 or y_low > GROUND_SIZE_M / 2:
                continue
            r0 = max(0, int(np.floor((.5 - x_high / GROUND_SIZE_M) * size)))
            r1 = min(size, int(np.ceil((.5 - x_low / GROUND_SIZE_M) * size)))
            c0 = max(0, int(np.floor((.5 - y_high / GROUND_SIZE_M) * size)))
            c1 = min(size, int(np.ceil((.5 - y_low / GROUND_SIZE_M) * size)))
            if r0 < r1 and c0 < c1 and white[r0:r1, c0:c1].any():
                marked[row, col] = True
    output[marked] = 0

    output_pgm = args.output_basename.with_suffix('.pgm')
    output_yaml = args.output_basename.with_suffix('.yaml')
    output_pgm.parent.mkdir(parents=True, exist_ok=True)
    if not cv2.imwrite(str(output_pgm), output):
        raise SystemExit('failed to write output pgm')
    output_yaml.write_text('image: %s\n%s' % (output_pgm.name, '\n'.join(
        line for line in yaml_text.splitlines() if not line.startswith('image:')
    ) + '\n'), encoding='utf-8')
    manifest = {
        'kind': 'navigation_real_white_line_overlay',
        'source_map': str(source_pgm),
        'source_map_sha256': sha256(source_pgm),
        'source_yaml': str(args.source_yaml),
        'ground_texture': str(args.ground_texture),
        'ground_texture_sha256': sha256(args.ground_texture),
        'permanent_white_line_cells': int(marked.sum()),
        'conditional_exclusions': {
            'start_line': START_LINE,
            'stop_lines': STOP_LINES,
            'zebra_pixel_boxes': ZEBRA_PIXEL_BOXES,
        },
        'policy': 'real ordinary white paint is lethal; conditional traffic markings are gated at runtime',
    }
    args.output_basename.with_suffix('.manifest.json').write_text(
        json.dumps(manifest, ensure_ascii=False, indent=2) + '\n', encoding='utf-8'
    )
    print(json.dumps(manifest, ensure_ascii=False))


if __name__ == '__main__':
    main()
