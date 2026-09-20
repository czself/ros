#!/usr/bin/env python3
"""Static, offline coverage report for a saved ROS map_server map.

Reads a map image (PGM) and its YAML manifest, then reports birth-connected
coverage computed from the pixels alone: global occupied/free/unknown counts,
the inflated free component reachable from the birth pose, unknown frontier
cells adjacent to that component, and the connected components of the unknown
region with each classified as touching a reachable frontier or blocked /
unreachable.

Offline-only: this tool never starts ROS/Gazebo, never subscribes to a topic,
and never writes under maps/. It is a static map report and is NOT runtime
evidence for Gate A coverage.
"""
import argparse
import ast
import hashlib
import json
import math
import pathlib
import sys

from PIL import Image, ImageDraw

# Fixed classification bands on the raw PGM pixel value (after optional
# negate inversion). These match map_saver's 0/254/205 convention.
OCCUPIED_MAX = 64
UNKNOWN_MAX = 229
FREE_MIN = 230

OCCUPIED = 0
UNKNOWN = 1
FREE = 2

WARNING = (
    "STATIC MAP REPORT ONLY: this report is derived from a saved map image. "
    "It does not run the live mapper, the /map publisher, the frontier "
    "planner, or any costmap, and it cannot prove runtime coverage. Gate A "
    "acceptance still requires runtime frontier/costmap evidence captured "
    "from the live session."
)


class ReportError(Exception):
    """Fatal input/problem error: report is not produced and exit is non-zero."""


def sha256_hex(path):
    data = pathlib.Path(path).read_bytes()
    return hashlib.sha256(data).hexdigest()


def parse_yaml(path):
    """Minimal ROS map_server YAML reader (image/resolution/origin/negate)."""
    meta = {}
    for line in pathlib.Path(path).read_text(encoding="utf-8").splitlines():
        text = line.strip()
        if not text or text.startswith("#") or ":" not in text:
            continue
        key, _, value = text.partition(":")
        key = key.strip()
        value = value.strip()
        if len(value) >= 2 and value[0] == '"' and value[-1] == '"':
            value = value[1:-1]
        try:
            meta[key] = ast.literal_eval(value)
        except (ValueError, SyntaxError):
            meta[key] = value
    return meta


def resolve_pgm(pgm_arg, yaml_path, image_field):
    yaml_dir = pathlib.Path(yaml_path).parent
    raw_names = []
    if pgm_arg:
        raw_names.append(pgm_arg)
    elif image_field:
        raw_names.append(image_field)
        raw_names.append(pathlib.Path(image_field).name)
    for name in raw_names:
        for path in (pathlib.Path(name), yaml_dir / name):
            try:
                if path.is_file() and path.stat().st_size > 0:
                    return path
            except OSError:
                continue
    raise ReportError("PGM image not found/unreadable; tried: %s"
                      % "; ".join(raw_names))


def classification_table(negate):
    table = []
    for value in range(256):
        pixel = (255 - value) if negate else value
        if pixel <= OCCUPIED_MAX:
            table.append(OCCUPIED)
        elif pixel <= UNKNOWN_MAX:
            table.append(UNKNOWN)
        else:
            table.append(FREE)
    return bytes(table)


def world_to_pixel(wx, wy, origin, resolution, height):
    ox, oy = origin[0], origin[1]
    col = round((wx - ox) / resolution)
    row_from_bottom = round((wy - oy) / resolution)
    return col, (height - 1) - row_from_bottom


def pixels_to_world(col, row, origin, resolution, height):
    ox, oy = origin[0], origin[1]
    wx = ox + col * resolution
    wy = oy + (height - 1 - row) * resolution
    return wx, wy


def neighbors8(x, y, width, height):
    x0 = x - 1 if x > 0 else x
    x1 = x + 1 if x < width - 1 else x
    y0 = y - 1 if y > 0 else y
    y1 = y + 1 if y < height - 1 else y
    out = []
    for yy in range(y0, y1 + 1):
        for xx in range(x0, x1 + 1):
            if yy == y and xx == x:
                continue
            out.append(yy * width + xx)
    return out


def inflate_occupied_mask(cells, width, height, radius):
    """Inflate occupied cells by a filled disk of the given cell radius."""
    canvas = Image.new("L", (width, height), 0)
    draw = ImageDraw.Draw(canvas)
    for idx, cell in enumerate(cells):
        if cell == OCCUPIED:
            x, y = idx % width, idx // width
            draw.ellipse([x - radius, y - radius, x + radius, y + radius], fill=255)
    return bytearray(1 if byte else 0 for byte in canvas.tobytes())


def flood_reachable(cells, blocked, seed, width, height):
    """8-neighbour flood fill from seed over raw-free, non-blocked cells."""
    reachable = bytearray(width * height)
    if cells[seed] != FREE or blocked[seed]:
        return reachable
    reachable[seed] = 1
    stack = [seed]
    while stack:
        idx = stack.pop()
        x = idx % width
        y = idx // width
        for nidx in neighbors8(x, y, width, height):
            if not reachable[nidx] and cells[nidx] == FREE and not blocked[nidx]:
                reachable[nidx] = 1
                stack.append(nidx)
    return reachable


def mark_frontier_from(cell_index, cells, frontier, width, height):
    """Mark unknown 8-neighbours of one reachable cell as frontier."""
    x = cell_index % width
    y = cell_index // width
    for nidx in neighbors8(x, y, width, height):
        if cells[nidx] == UNKNOWN:
            frontier[nidx] = 1


def compute_frontier(cells, reachable, width, height):
    """Unknown cells with at least one 8-neighbour reachable-free cell."""
    frontier = bytearray(width * height)
    for index, reached in enumerate(reachable):
        if reached:
            mark_frontier_from(index, cells, frontier, width, height)
    return frontier


def unknown_components(cells, frontier, width, height):
    """Connected components of the unknown region; label order is raster scan.

    Uses scanline flood fill (per-run, not per-cell adjacency) so large
    all-unknown canvasses stay tractable while keeping 8-neighbour
    connectivity.
    """
    visited = bytearray(width * height)
    components = {}
    next_id = 1
    total = width * height
    for start in range(total):
        if cells[start] != UNKNOWN or visited[start]:
            continue
        stack = [(start % width, start // width)]
        count = 0
        frontier_count = 0
        while stack:
            x, y = stack.pop()
            while x > 0 and cells[y * width + x - 1] == UNKNOWN and not visited[y * width + x - 1]:
                x -= 1
            left = x
            while x < width and cells[y * width + x] == UNKNOWN and not visited[y * width + x]:
                visited[y * width + x] = 1
                count += 1
                frontier_count += frontier[y * width + x]
                x += 1
            right = x - 1
            for ny in (y - 1, y + 1):
                if not (0 <= ny < height):
                    continue
                # Push the start of each unvisited unknown run overlapping
                # the widened interval, so each cell is pushed only once
                # per spanning row instead of once per covering column.
                nx = max(0, left - 1)
                end = min(width, right + 2)
                while nx < end:
                    nidx = ny * width + nx
                    if cells[nidx] == UNKNOWN and not visited[nidx]:
                        stack.append((nx, ny))
                        while nx < end and cells[ny * width + nx] == UNKNOWN and not visited[ny * width + nx]:
                            nx += 1
                    else:
                        nx += 1
        components[next_id] = {
            "id": next_id,
            "cells": count,
            "frontier_cells": frontier_count,
            "touches_reachable_frontier": frontier_count > 0,
        }
        next_id += 1
    return components


def build_report(args):
    yaml_path = pathlib.Path(args.yaml)
    if not yaml_path.is_file():
        raise ReportError("yaml file not found: %s" % yaml_path)
    meta = parse_yaml(yaml_path)
    for key in ("image", "resolution", "origin"):
        if key not in meta:
            raise ReportError("yaml missing required key %r" % key)
    resolution = float(meta["resolution"])
    if resolution <= 0 or not math.isfinite(resolution):
        raise ReportError("invalid yaml resolution: %r" % meta["resolution"])
    origin = list(meta["origin"])
    if len(origin) < 2:
        raise ReportError("yaml origin must have at least x,y")
    negate = 1 if int(meta.get("negate", 0)) else 0
    if args.inflate_radius_m <= 0 or not math.isfinite(args.inflate_radius_m):
        raise ReportError("invalid --inflate-radius-m: %r" % args.inflate_radius_m)

    pgm_path = resolve_pgm(args.pgm, yaml_path, str(meta.get("image", "")))
    try:
        with Image.open(pgm_path) as loaded:
            loaded.load()
            image = loaded.convert("L")
    except Exception as exc:  # Pillow decode/header errors
        raise ReportError("cannot read PGM %s: %s" % (pgm_path, exc))
    width, height = image.size
    raw = image.tobytes()
    cells = raw.translate(classification_table(negate))

    try:
        birth_parts = [token.strip() for token in args.birth.replace("(", "").replace(")", "").split(",")]
        if len(birth_parts) < 2:
            raise ReportError("--birth expects \"x,y\" or \"x,y,yaw\"")
        bx, by = float(birth_parts[0]), float(birth_parts[1])
        birth_yaw = float(birth_parts[2]) if len(birth_parts) > 2 else None
    except ValueError as exc:
        raise ReportError("cannot parse --birth %r: %s" % (args.birth, exc))

    col, row = world_to_pixel(bx, by, origin, resolution, height)
    if not (0 <= col < width and 0 <= row < height):
        raise ReportError(
            "birth world (%.4f, %.4f) maps to pixel (%d, %d), outside map %dx%d"
            % (bx, by, col, row, width, height)
        )
    seed = row * width + col
    seed_class = cells[seed]

    inflate_cells = max(1, int(math.ceil(args.inflate_radius_m / resolution)))
    blocked = inflate_occupied_mask(cells, width, height, inflate_cells)
    if seed_class == OCCUPIED:
        raise ReportError("birth pixel %s is an occupied cell" % ([col, row]))
    if seed_class == UNKNOWN:
        raise ReportError("birth pixel %s is an unknown cell" % ([col, row]))
    if blocked[seed]:
        raise ReportError(
            "birth pixel %s is inside the inflated robot-clearance envelope "
            "of an occupied cell (clearance %.3f m / %d cells)"
            % ([col, row], args.inflate_radius_m, inflate_cells)
        )

    reachable = flood_reachable(cells, blocked, seed, width, height)
    frontier = compute_frontier(cells, reachable, width, height)
    components = unknown_components(cells, frontier, width, height)

    reachable_free_cells = sum(reachable)
    frontier_cells = sum(frontier)
    reachable_frontier_ids = [c["id"] for c in sorted(components.values(), key=lambda c: c["id"])
                              if c["touches_reachable_frontier"]]
    blocked_ids = [c["id"] for c in sorted(components.values(), key=lambda c: c["id"])
                   if not c["touches_reachable_frontier"]]

    occupied_cells = sum(1 for c in cells if c == OCCUPIED)
    unknown_cells = sum(1 for c in cells if c == UNKNOWN)
    free_cells = len(cells) - occupied_cells - unknown_cells

    gate_a_candidate = frontier_cells == 0

    return {
        "tool": "scripts/coverage_report.py",
        "kind": "static_map_coverage_report",
        "warning": WARNING,
        "input_hashes": {
            "pgm": sha256_hex(pgm_path),
            "yaml": sha256_hex(yaml_path),
            "pgm_file": str(pgm_path),
            "yaml_file": str(yaml_path),
        },
        "size": [width, height],
        "resolution": resolution,
        "negate": negate,
        "origin": [float(v) for v in origin],
        "classification_thresholds": {
            "occupied_max": OCCUPIED_MAX,
            "unknown_max": UNKNOWN_MAX,
            "free_min": FREE_MIN,
        },
        "birth": {
            "world": [bx, by] if birth_yaw is None else [bx, by, birth_yaw],
            "pixel": [col, row],
            "cell": "free",
        },
        "inflate_radius_m": args.inflate_radius_m,
        "inflate_cells": inflate_cells,
        "counts": {
            "occupied_cells": occupied_cells,
            "free_cells": free_cells,
            "unknown_cells": unknown_cells,
            "total_cells": width * height,
        },
        "reachable_free_cells": reachable_free_cells,
        "frontier_cells": frontier_cells,
        "unknown_component_count": len(components),
        "reachable_frontier_component_ids": reachable_frontier_ids,
        "blocked_or_unreachable_component_ids": blocked_ids,
        "unknown_components": [
            {k: c[k] for k in ("id", "cells", "frontier_cells", "touches_reachable_frontier")}
            for c in sorted(components.values(), key=lambda c: c["id"])
        ],
        "note": (
            "Counts are over the full map canvas. reachable_free_cells counts "
            "raw-free cells reachable from the birth seed on the 8-neighbour "
            "grid after occupied inflation. frontier_cells counts unknown cells "
            "8-adjacent to that reachable component. gate_a_candidate is true "
            "only when frontier_cells == 0 and the birth seed is free; it is a "
            "static precondition and not runtime acceptance evidence."
        ),
        "gate_a_candidate": gate_a_candidate,
    }


def main(argv=None):
    parser = argparse.ArgumentParser(
        prog="coverage_report.py",
        description=(
            "Static offline coverage report for a saved ROS map_server map "
            "(PGM + YAML). Pure pixel analysis; never starts ROS/Gazebo and "
            "never writes under maps/."
        ),
        epilog=(
            "examples:\n"
            "  coverage_report.py --yaml maps/competition_slam_verified4.yaml "
            "--birth 4.0833,-4.0833\n"
            "  coverage_report.py --yaml maps/competition_slam_verified4.yaml "
            "--pgm maps/competition_slam_verified4.pgm --birth 4.0833,-4.0833 "
            "--inflate-radius-m 0.40 --output /tmp/coverage.json\n"
        ),
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    parser.add_argument("--yaml", required=True,
                        help="ROS map_server YAML manifest (image/resolution/origin)")
    parser.add_argument("--pgm", default=None,
                        help="map image path; defaults to image: from the yaml")
    parser.add_argument("--birth", required=True,
                        help='birth pose in world coordinates as "x,y" or "x,y,yaw"')
    parser.add_argument("--inflate-radius-m", type=float, default=0.40,
                        help=("conservative robot-clearance radius used to inflate "
                              "occupied cells (default 0.40; mapping-gate envelope; "
                              "runtime navigation inflation is configured separately)"))
    parser.add_argument("--output", default=None,
                        help="optional JSON output path; default prints to stdout")
    args = parser.parse_args(argv)

    try:
        report = build_report(args)
    except ReportError as exc:
        print("coverage_report: error: %s" % exc, file=sys.stderr)
        return 1
    except Exception as exc:  # Pillow or other surprise: no pass conclusion
        print("coverage_report: unexpected error: %s" % exc, file=sys.stderr)
        return 1

    text = json.dumps(report, indent=2)
    if args.output:
        pathlib.Path(args.output).write_text(text + "\n", encoding="utf-8")
    print(text)
    return 0


if __name__ == "__main__":
    sys.exit(main())
