#!/usr/bin/env python3
"""Restore standees that were inserted at runtime but absent from the base world."""
import re
from pathlib import Path

WORLD = Path("worlds/competition_classic.world")
ROOT = Path("insert/person_standees")
POSES = {
    "01": (-1.350, 1.500, 0.083, 0.000),
    "02": (-0.810, 1.500, 0.083, 0.000),
    "07": (-1.350, 0.500, 0.083, 0.000),
    "08": (-0.810, 0.500, 0.083, 0.000),
    "13": (-1.350, -0.500, 0.083, 0.000),
    "14": (-0.810, -0.500, 0.083, 0.000),
    "15": (-0.270, -0.500, 0.083, 0.000),
    "16": (0.270, -0.500, 0.083, 0.000),
}

text = WORLD.read_text()
blocks = []
for number, (x, y, z, yaw) in POSES.items():
    name = f"person_standee_{number}"
    if re.search(rf"<model name=['\"]{name}['\"]>", text):
        continue
    sdf = (ROOT / f"model_{name}" / "model.sdf").read_text()
    model = re.search(r"<model name=\"[^\"]+\">.*?</model>", sdf, re.DOTALL).group(0)
    model = model.replace(f'<model name="{name}">', f"<model name='{name}'>", 1)
    model = model.replace("<static>1</static>", f"<static>1</static>\n      <pose>{x:.9f} {y:.9f} {z:.9f} 0 0 {yaw:.9f}</pose>", 1)
    blocks.append("    " + model.replace("\n", "\n    "))

if blocks:
    text = text.replace("    <gravity>", "\n".join(blocks) + "\n    <gravity>", 1)
    WORLD.write_text(text)
print(f"restored {len(blocks)} standees")
