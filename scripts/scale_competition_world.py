#!/usr/bin/env python3
"""Scale the competition SDF uniformly in metric dimensions and positions."""
import re
import sys

S = 10.0 / 6.0

def number(value):
    return f"{float(value) * S:.8g}"

def scale_pose(match):
    values = match.group(1).split()
    if len(values) < 6:
        return match.group(0)
    values[:3] = [number(v) for v in values[:3]]
    return "<pose>" + " ".join(values) + "</pose>"

def scale_vector(match):
    values = match.group(2).split()
    if len(values) != 3:
        return match.group(0)
    return match.group(1) + " ".join(number(v) for v in values) + match.group(3)

def scale_scalar(match):
    return match.group(1) + number(match.group(2)) + match.group(3)

def main(source, target):
    text = open(source, encoding="utf-8").read()
    text = re.sub(r"<pose>([^<]+)</pose>", scale_pose, text)
    text = re.sub(r"(<(?:size|scale)>)([^<]+)(</(?:size|scale)>)", scale_vector, text)
    text = re.sub(r"(<(?:radius|length|wheelSeparation|wheelDiameter)>)([-+0-9.eE]+)(</(?:radius|length|wheelSeparation|wheelDiameter)>)", scale_scalar, text)
    text = text.replace("<mesh><uri>", f"<mesh><scale>{S:.8g} {S:.8g} {S:.8g}</scale><uri>")
    text = text.replace("<near>0.05</near>", f"<near>{0.05*S:.8g}</near>")
    text = text.replace("<far>10</far>", f"<far>{10*S:.8g}</far>")
    open(target, "w", encoding="utf-8").write(text)

if __name__ == "__main__":
    if len(sys.argv) != 3:
        raise SystemExit("usage: scale_competition_world.py INPUT OUTPUT")
    main(sys.argv[1], sys.argv[2])
