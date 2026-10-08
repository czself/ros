#!/usr/bin/env python3
"""Prepare the physical SLAM map for AMCL, excluding navigation paint overlays."""

import argparse
import hashlib
import json
from pathlib import Path
import shutil

from PIL import Image
import yaml


def prepare_localization_map(navigation_yaml, output_basename):
    navigation_yaml = Path(navigation_yaml).resolve()
    navigation = yaml.safe_load(navigation_yaml.read_text())
    source_yaml = navigation_yaml
    manifest_path = navigation_yaml.with_suffix('.manifest.json')
    expected_hash = None
    if manifest_path.exists():
        manifest = json.loads(manifest_path.read_text())
        if manifest.get('kind') == 'navigation_real_white_line_overlay':
            source_yaml = Path(manifest['source_yaml']).resolve()
            expected_hash = manifest['source_map_sha256']
        elif manifest.get('kind') == 'navigation_safety_overlay':
            source_yaml = Path(manifest['source_map']['yaml']).resolve()
    source = yaml.safe_load(source_yaml.read_text())
    source_image = source_yaml.parent / source['image']
    navigation_image = navigation_yaml.parent / navigation['image']
    if expected_hash and hashlib.sha256(source_image.read_bytes()).hexdigest() != expected_hash:
        raise ValueError('physical SLAM map hash differs from navigation manifest')
    for field in ('resolution', 'origin', 'negate', 'occupied_thresh', 'free_thresh'):
        if source.get(field) != navigation.get(field):
            raise ValueError('localization/navigation map geometry differs: %s' % field)
    with Image.open(source_image) as physical, Image.open(navigation_image) as safety:
        if physical.size != safety.size:
            raise ValueError('localization/navigation map image dimensions differ')
    output_basename = Path(output_basename)
    output_basename.parent.mkdir(parents=True, exist_ok=True)
    output_image = output_basename.with_suffix('.pgm')
    output_yaml = output_basename.with_suffix('.yaml')
    shutil.copyfile(source_image, output_image)
    source['image'] = output_image.name
    output_yaml.write_text(yaml.safe_dump(source, sort_keys=False))
    return output_yaml


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('navigation_yaml', type=Path)
    parser.add_argument('output_basename', type=Path)
    args = parser.parse_args()
    print(prepare_localization_map(args.navigation_yaml, args.output_basename))
