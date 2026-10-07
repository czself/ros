"""Normalize the existing detector's JSON status to a traffic color."""

import json


def parse_signal(value):
    try:
        decoded = json.loads(value)
    except (TypeError, ValueError):
        decoded = value
    if isinstance(decoded, dict):
        decoded = decoded.get("state", "")
    result = str(decoded).upper()
    return result if result in ("RED", "YELLOW", "GREEN") else "UNKNOWN"
