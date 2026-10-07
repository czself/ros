"""Load a fixed, reviewable sequence of navigation and inspection goals."""

import json
import math
from dataclasses import dataclass


@dataclass(frozen=True)
class Goal:
    name: str
    x: float
    y: float
    yaw: float
    tasks: tuple = ("none",)
    timeout_s: float = 75.0


def load_goals(path):
    with open(path, encoding="utf-8") as stream:
        document = json.load(stream)
    if document.get("frame") != "map":
        raise ValueError("route frame must be map")
    entries = document.get("goals")
    if not isinstance(entries, list) or not entries:
        raise ValueError("route requires a nonempty goals list")
    goals = []
    names = set()
    for entry in entries:
        name = entry["name"]
        if not isinstance(name, str) or not name or name in names:
            raise ValueError("goal names must be unique and nonempty")
        values = [float(entry[key]) for key in ("x", "y", "yaw")]
        if not all(math.isfinite(value) for value in values):
            raise ValueError("goal pose must be finite: " + name)
        tasks = tuple(entry.get("tasks", ["none"]))
        if not tasks or any(task not in ("none", "photo", "traffic_light") for task in tasks):
            raise ValueError("unknown goal task: " + str(tasks))
        timeout = float(entry.get("timeout_s", 75.0))
        if not math.isfinite(timeout) or timeout <= 0:
            raise ValueError("invalid goal timeout: " + name)
        goals.append(Goal(name, *values, tasks, timeout))
        names.add(name)
    if goals[-1].name != "HOME":
        raise ValueError("route must end at HOME")
    return goals
