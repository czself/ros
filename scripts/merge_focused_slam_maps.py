#!/usr/bin/env python3
"""Conservatively merge same-grid focused GMapping snapshots.

The inputs are independently saved `/slam_gmapping` occupancy grids made
from the same reset pose and map geometry.  This utility never invents free
space: occupied wins any disagreement, free is emitted only where at least
one scan observed free space and neither observed occupied.
"""
import argparse
import hashlib
import json
from pathlib import Path

import cv2
import yaml
import re


UNKNOWN, OCCUPIED, FREE = 205, 0, 254


def sha256(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('first_yaml')
    parser.add_argument('second_yaml')
    parser.add_argument('output_stem')
    args = parser.parse_args()

    first_yaml = Path(args.first_yaml).resolve()
    second_yaml = Path(args.second_yaml).resolve()
    first_meta = yaml.safe_load(first_yaml.read_text())
    second_meta = yaml.safe_load(second_yaml.read_text())
    for key in ('resolution', 'origin', 'negate', 'occupied_thresh', 'free_thresh'):
        if first_meta.get(key) != second_meta.get(key):
            raise SystemExit('incompatible map {}: {!r} != {!r}'.format(
                key, first_meta.get(key), second_meta.get(key)))
    # map_saver may write an absolute container path into `image`; maps copied
    # to this workspace retain the PGM beside their YAML, so resolve by name.
    first_pgm = first_yaml.parent / Path(first_meta['image']).name
    second_pgm = second_yaml.parent / Path(second_meta['image']).name
    first = cv2.imread(str(first_pgm), cv2.IMREAD_GRAYSCALE)
    second = cv2.imread(str(second_pgm), cv2.IMREAD_GRAYSCALE)
    if first is None or second is None or first.shape != second.shape:
        raise SystemExit('incompatible occupancy image shapes')

    merged = cv2.UMat(first).get()  # independent writable array
    occupied = (first == OCCUPIED) | (second == OCCUPIED)
    free = ~occupied & ((first == FREE) | (second == FREE))
    merged[:] = UNKNOWN
    merged[free] = FREE
    merged[occupied] = OCCUPIED

    stem = Path(args.output_stem).resolve()
    stem.parent.mkdir(parents=True, exist_ok=True)
    pgm = stem.with_suffix('.pgm')
    yaml_path = stem.with_suffix('.yaml')
    cv2.imwrite(str(pgm), merged)
    # Preserve map_server's inline origin syntax; the white-line overlay tool
    # and existing launch scripts intentionally consume that canonical form.
    yaml_path.write_text(re.sub(
        r'^image:\s*.*$', 'image: ' + pgm.name,
        first_yaml.read_text(), count=1, flags=re.M))
    manifest = {
        'kind': 'conservative_merged_focused_slam',
        'merge_policy': 'occupied_wins; free_only_if_observed_free; unknown_otherwise',
        'inputs': [
            {'yaml': str(first_yaml), 'pgm_sha256': sha256(first_pgm)},
            {'yaml': str(second_yaml), 'pgm_sha256': sha256(second_pgm)},
        ],
        'merged_pgm_sha256': sha256(pgm),
        'resolution': first_meta['resolution'],
        'origin': first_meta['origin'],
        'occupied_cells': int((merged == OCCUPIED).sum()),
        'free_cells': int((merged == FREE).sum()),
        'unknown_cells': int((merged == UNKNOWN).sum()),
    }
    stem.with_suffix('.manifest.json').write_text(json.dumps(manifest, indent=2) + '\n')


if __name__ == '__main__':
    main()
