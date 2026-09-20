#!/usr/bin/env python3
"""Overlay the current Gazebo collision geometry onto a measured map.

The saved SLAM map is useful evidence, but a missed/erased wall cell must not
become a legal navigation route.  This utility therefore keeps every source
map cell and unions it with a rasterization of the *current* world's static
box collisions.  The result is explicitly a navigation safety map, not a
claim that the world was scanned by SLAM.
"""

import argparse
import hashlib
import json
import math
import pathlib
import sys

from PIL import Image


ROOT = pathlib.Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "scripts"))
from generate_ground_truth_map import build_map  # noqa: E402


def sha256(path):
    digest = hashlib.sha256()
    with pathlib.Path(path).open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def read_yaml(path):
    values = {}
    for line in pathlib.Path(path).read_text(encoding="utf-8").splitlines():
        text = line.strip()
        if not text or text.startswith("#") or ":" not in text:
            continue
        key, _, raw = text.partition(":")
        raw = raw.strip()
        if key.strip() == "origin":
            values[key.strip()] = [float(item) for item in raw.strip("[]").split(",")]
        elif key.strip() in ("resolution", "negate", "occupied_thresh", "free_thresh"):
            values[key.strip()] = float(raw)
        else:
            values[key.strip()] = raw.strip("\"'")
    return values


def resolve_image(yaml_path, image):
    candidate = pathlib.Path(image)
    if candidate.is_file():
        return candidate
    candidate = pathlib.Path(yaml_path).parent / candidate.name
    if candidate.is_file():
        return candidate
    raise RuntimeError("source map image not found: %s" % image)


def overlay(args):
    source_yaml = pathlib.Path(args.source_yaml).resolve()
    source_meta = read_yaml(source_yaml)
    source_pgm = resolve_image(source_yaml, source_meta["image"])
    source = Image.open(source_pgm).convert("L")
    width, height = source.size
    resolution = float(source_meta["resolution"])
    origin = [float(value) for value in source_meta["origin"][:3]]
    if abs(origin[2]) > 1e-6:
        raise RuntimeError("rotated source map origins are unsupported")

    collision = build_map(str(pathlib.Path(args.world).resolve()))
    collision_height = len(collision)
    collision_width = len(collision[0])
    collision_res = 0.05
    collision_origin = (-5.0, -5.0)
    pixels = bytearray(source.tobytes())
    changed = 0
    overlay_cells = 0
    padding_cells = int(math.ceil(float(args.padding) / resolution))

    # First collect collision cells in output-map coordinates.  Dilation is
    # intentionally tiny; the costmap inflation layer supplies the planning
    # margin while this layer closes rasterization/registration gaps.
    collision_cells = set()
    for row in range(collision_height):
        for col in range(collision_width):
            if collision[row][col] >= 65:
                continue
            wx = collision_origin[0] + (col + 0.5) * collision_res
            wy = collision_origin[1] + (collision_height - row - 0.5) * collision_res
            out_col = int(math.floor((wx - origin[0]) / resolution))
            out_row_from_bottom = int(math.floor((wy - origin[1]) / resolution))
            out_row = height - 1 - out_row_from_bottom
            if not (0 <= out_col < width and 0 <= out_row < height):
                continue
            for dy in range(-padding_cells, padding_cells + 1):
                for dx in range(-padding_cells, padding_cells + 1):
                    if dx * dx + dy * dy > padding_cells * padding_cells:
                        continue
                    rr, cc = out_row + dy, out_col + dx
                    if 0 <= rr < height and 0 <= cc < width:
                        collision_cells.add(rr * width + cc)

    for index in collision_cells:
        overlay_cells += 1
        if pixels[index] != 0:
            pixels[index] = 0
            changed += 1

    output_prefix = pathlib.Path(args.output_prefix).resolve()
    output_prefix.parent.mkdir(parents=True, exist_ok=True)
    output_pgm = output_prefix.with_suffix(".pgm")
    output_yaml = output_prefix.with_suffix(".yaml")
    output_manifest = pathlib.Path(args.manifest).resolve()
    Image.frombytes("L", (width, height), bytes(pixels)).save(output_pgm, format="PPM")
    output_yaml.write_text(
        "image: %s\nresolution: %.6f\norigin: [%.6f, %.6f, %.6f]\n"
        "negate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.196\n"
        % (output_pgm.name, resolution, origin[0], origin[1], origin[2]),
        encoding="ascii",
    )

    report = {
        "kind": "navigation_safety_overlay",
        "pure_slam": False,
        "source_map": {
            "yaml": str(source_yaml),
            "pgm": str(source_pgm),
            "yaml_sha256": sha256(source_yaml),
            "pgm_sha256": sha256(source_pgm),
        },
        "collision_world": {
            "sdf": str(pathlib.Path(args.world).resolve()),
            "sha256": sha256(args.world),
            "padding_m": float(args.padding),
        },
        "output": {
            "yaml": str(output_yaml),
            "pgm": str(output_pgm),
            "yaml_sha256": sha256(output_yaml),
            "pgm_sha256": sha256(output_pgm),
            "size": [width, height],
            "origin": origin,
            "resolution": resolution,
        },
        "collision_overlay_cells": len(collision_cells),
        "source_cells_changed_to_occupied": changed,
        "warning": "Use for navigation safety only; it is not a pure SLAM artifact.",
    }
    output_manifest.write_text(json.dumps(report, indent=2) + "\n", encoding="utf-8")
    print(json.dumps(report, indent=2))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--source-yaml", required=True)
    parser.add_argument("--world", required=True)
    parser.add_argument("--output-prefix", required=True)
    parser.add_argument("--manifest", required=True)
    parser.add_argument("--padding", type=float, default=0.03)
    args = parser.parse_args()
    overlay(args)


if __name__ == "__main__":
    main()
