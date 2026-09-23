#!/usr/bin/env python3
"""Uniformly scale metric SDF positions, geometry, and sensor ranges.

Rotations, rates, mass, and inertia deliberately remain unchanged.  The tool
writes to a separate output path so callers can validate the result before
replacing an active world.
"""
import argparse
import re
from pathlib import Path


VECTOR_TAGS = ("size", "scale")
SCALAR_TAGS = ("radius", "length", "wheelSeparation", "wheelDiameter")


def scaled(value, factor):
    return format(float(value) * factor, ".9g")


def scale_pose(match, factor):
    values = match.group(1).split()
    if len(values) != 6:
        return match.group(0)
    values[:3] = [scaled(value, factor) for value in values[:3]]
    return "<pose>" + " ".join(values) + "</pose>"


def scale_vector(match, factor):
    values = match.group(2).split()
    if len(values) != 3:
        return match.group(0)
    return match.group(1) + " ".join(scaled(value, factor) for value in values) + match.group(3)


def scale_scalar(match, factor):
    return match.group(1) + scaled(match.group(2), factor) + match.group(3)


def scale_ray_ranges(text, factor):
    def replace_range(match):
        block = match.group(0)
        return re.sub(
            r"(<(?:min|max)>)([-+0-9.eE]+)(</(?:min|max)>)",
            lambda scalar: scale_scalar(scalar, factor),
            block,
        )

    return re.sub(r"<range>.*?</range>", replace_range, text, flags=re.DOTALL)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("source", type=Path)
    parser.add_argument("target", type=Path)
    parser.add_argument("--factor", type=float, required=True)
    args = parser.parse_args()
    if not 0 < args.factor:
        raise SystemExit("--factor must be positive")

    text = args.source.read_text(encoding="utf-8")
    text = re.sub(r"<pose>([^<]+)</pose>", lambda match: scale_pose(match, args.factor), text)
    vector_names = "|".join(VECTOR_TAGS)
    text = re.sub(
        rf"(<(?:{vector_names})>)([^<]+)(</(?:{vector_names})>)",
        lambda match: scale_vector(match, args.factor),
        text,
    )
    scalar_names = "|".join(SCALAR_TAGS)
    text = re.sub(
        rf"(<(?:{scalar_names})>)([-+0-9.eE]+)(</(?:{scalar_names})>)",
        lambda match: scale_scalar(match, args.factor),
        text,
    )
    args.target.write_text(scale_ray_ranges(text, args.factor), encoding="utf-8")


if __name__ == "__main__":
    main()
