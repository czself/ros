#!/usr/bin/env python3
"""Build an explicitly labelled hybrid navigation map.

Known cells from a measured SLAM map are authoritative. Unknown SLAM cells
inside the aligned world/truth map are filled from that map only so navigation
can represent physically closed areas that the robot cannot enter. This output
is not a pure SLAM artifact and must never be used as SLAM provenance evidence.
"""
import argparse
import ast
import hashlib
import json
import math
import pathlib
import sys

from PIL import Image


OCCUPIED_MAX = 64
UNKNOWN_MAX = 229
FREE_MIN = 230


class MergeError(Exception):
    pass


def sha256(path):
    return hashlib.sha256(pathlib.Path(path).read_bytes()).hexdigest()


def parse_yaml(path):
    result = {}
    for line in pathlib.Path(path).read_text(encoding="utf-8").splitlines():
        text = line.strip()
        if not text or text.startswith("#") or ":" not in text:
            continue
        key, _, value = text.partition(":")
        value = value.strip()
        try:
            result[key.strip()] = ast.literal_eval(value)
        except (ValueError, SyntaxError):
            result[key.strip()] = value.strip("\"'")
    return result


def classify(value):
    if value <= OCCUPIED_MAX:
        return "occupied"
    if value <= UNKNOWN_MAX:
        return "unknown"
    return "free"


def resolve_image(yaml_path, explicit):
    meta = parse_yaml(yaml_path)
    image = explicit if explicit else meta.get("image")
    if not image:
        raise MergeError("YAML has no image and no explicit image path was given")
    raw = pathlib.Path(str(image))
    candidates = [raw]
    if not raw.is_absolute():
        candidates.append(pathlib.Path(yaml_path).parent / raw)
    else:
        candidates.append(pathlib.Path(yaml_path).parent / raw.name)
    for path in candidates:
        try:
            if path.is_file() and path.stat().st_size > 0:
                return path, meta
        except OSError:
            continue
    raise MergeError("image not found or unreadable: %s" % image)


def origin(meta, label):
    try:
        value = [float(item) for item in meta["origin"]]
        if len(value) < 3:
            raise ValueError
        return value
    except (KeyError, TypeError, ValueError):
        raise MergeError("%s YAML origin must be [x,y,yaw]" % label)


def resolution(meta, label):
    try:
        value = float(meta["resolution"])
    except (KeyError, TypeError, ValueError):
        raise MergeError("%s YAML resolution is invalid" % label)
    if value <= 0 or not math.isfinite(value):
        raise MergeError("%s YAML resolution must be positive" % label)
    return value


def world_to_pixel(wx, wy, meta, width, height):
    res = resolution(meta, "source")
    ox, oy, _ = origin(meta, "source")
    col = round((wx - ox) / res)
    row_from_bottom = round((wy - oy) / res)
    return col, height - 1 - row_from_bottom


def load_map(yaml_path, explicit_pgm, label):
    pgm, meta = resolve_image(yaml_path, explicit_pgm)
    try:
        with Image.open(pgm) as opened:
            opened.load()
            image = opened.convert("L")
    except Exception as exc:
        raise MergeError("cannot read %s image %s: %s" % (label, pgm, exc))
    return pgm, meta, image


def write_yaml(path, image_name, source_meta):
    text = (
        "image: %s\n"
        "resolution: %.6f\n"
        "origin: [%.6f, %.6f, %.6f]\n"
        "negate: %d\n"
        "occupied_thresh: %.6f\n"
        "free_thresh: %.6f\n"
    ) % (
        image_name,
        float(source_meta.get("resolution", 0.05)),
        *[float(v) for v in source_meta.get("origin", [-6.0, -6.0, 0.0])[:3]],
        int(source_meta.get("negate", 0)),
        float(source_meta.get("occupied_thresh", 0.65)),
        float(source_meta.get("free_thresh", 0.196)),
    )
    pathlib.Path(path).write_text(text, encoding="utf-8")


def build(args):
    slam_pgm, slam_meta, slam_image = load_map(args.slam_yaml, args.slam_pgm, "SLAM")
    truth_pgm, truth_meta, truth_image = load_map(args.truth_yaml, args.truth_pgm, "world")
    slam_res = resolution(slam_meta, "SLAM")
    truth_res = resolution(truth_meta, "world")
    if abs(slam_res - truth_res) > 1e-9:
        raise MergeError("resolution mismatch: SLAM=%s world=%s" % (slam_res, truth_res))
    slam_origin = origin(slam_meta, "SLAM")
    truth_origin = origin(truth_meta, "world")
    if abs(slam_origin[2]) > 1e-6 or abs(truth_origin[2]) > 1e-6:
        raise MergeError("rotated map origins are not supported")
    if int(slam_meta.get("negate", 0)) != int(truth_meta.get("negate", 0)):
        raise MergeError("negate mismatch between SLAM and world maps")

    width, height = slam_image.size
    truth_width, truth_height = truth_image.size
    slam_pixels = bytearray(slam_image.tobytes())
    truth_pixels = truth_image.tobytes()
    counts_before = {"occupied": 0, "unknown": 0, "free": 0}
    counts_after = {"occupied": 0, "unknown": 0, "free": 0}
    filled = {"occupied": 0, "unknown": 0, "free": 0}
    unresolved = 0
    overlap_unknown = 0

    for row in range(height):
        for col in range(width):
            index = row * width + col
            old_class = classify(slam_pixels[index])
            counts_before[old_class] += 1
            if old_class != "unknown":
                continue
            wx = slam_origin[0] + col * slam_res
            wy = slam_origin[1] + (height - 1 - row) * slam_res
            truth_col = round((wx - truth_origin[0]) / truth_res)
            truth_row = truth_height - 1 - round((wy - truth_origin[1]) / truth_res)
            if not (0 <= truth_col < truth_width and 0 <= truth_row < truth_height):
                unresolved += 1
                continue
            overlap_unknown += 1
            replacement = truth_pixels[truth_row * truth_width + truth_col]
            replacement_class = classify(replacement)
            if replacement_class == "unknown":
                unresolved += 1
                continue
            slam_pixels[index] = replacement
            filled[replacement_class] += 1

    for value in slam_pixels:
        counts_after[classify(value)] += 1
    output_pgm = pathlib.Path(args.output_pgm)
    output_yaml = pathlib.Path(args.output_yaml)
    output_manifest = pathlib.Path(args.manifest)
    forbidden = {slam_pgm.resolve(), truth_pgm.resolve()}
    if output_pgm.resolve() in forbidden or output_yaml.resolve() in {
        pathlib.Path(args.slam_yaml).resolve(), pathlib.Path(args.truth_yaml).resolve()
    }:
        raise MergeError("refusing to overwrite a source map")
    output_pgm.parent.mkdir(parents=True, exist_ok=True)
    output_yaml.parent.mkdir(parents=True, exist_ok=True)
    output_manifest.parent.mkdir(parents=True, exist_ok=True)
    Image.frombytes("L", (width, height), bytes(slam_pixels)).save(output_pgm, format="PPM")
    write_yaml(output_yaml, output_pgm.name, slam_meta)
    report = {
        "kind": "hybrid_navigation_map",
        "pure_slam": False,
        "warning": (
            "World/truth occupancy was used only to fill unknown SLAM cells. "
            "This artifact is for navigation convenience and is not a pure SLAM map."
        ),
        "slam_source": {
            "pgm": str(slam_pgm),
            "yaml": str(args.slam_yaml),
            "pgm_sha256": sha256(slam_pgm),
            "yaml_sha256": sha256(args.slam_yaml),
            "size": [width, height],
            "origin": slam_origin,
            "resolution": slam_res,
        },
        "world_source": {
            "pgm": str(truth_pgm),
            "yaml": str(args.truth_yaml),
            "pgm_sha256": sha256(truth_pgm),
            "yaml_sha256": sha256(args.truth_yaml),
            "size": [truth_width, truth_height],
            "origin": truth_origin,
            "resolution": truth_res,
        },
        "output": {
            "pgm": str(output_pgm),
            "yaml": str(output_yaml),
            "pgm_sha256": sha256(output_pgm),
            "yaml_sha256": sha256(output_yaml),
            "size": [width, height],
            "origin": slam_origin,
            "resolution": slam_res,
        },
        "merge_policy": "preserve_known_slam_cells_fill_unknown_from_world",
        "counts_before": counts_before,
        "filled_from_world": filled,
        "unknown_cells_in_world_overlap": overlap_unknown,
        "unresolved_unknown_cells": unresolved,
        "counts_after": counts_after,
    }
    output_manifest.write_text(json.dumps(report, indent=2) + "\n", encoding="utf-8")
    return report


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--slam-yaml", required=True)
    parser.add_argument("--slam-pgm")
    parser.add_argument("--truth-yaml", required=True)
    parser.add_argument("--truth-pgm")
    parser.add_argument("--output-pgm", required=True)
    parser.add_argument("--output-yaml", required=True)
    parser.add_argument("--manifest", required=True)
    args = parser.parse_args(argv)
    try:
        report = build(args)
    except (MergeError, OSError, ValueError) as exc:
        print("merge_hybrid_map: error: %s" % exc, file=sys.stderr)
        return 1
    print(json.dumps(report, indent=2))
    return 0


if __name__ == "__main__":
    sys.exit(main())
