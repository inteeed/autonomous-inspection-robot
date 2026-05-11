"""Pure helpers shared by waypoint mission nodes."""
from __future__ import annotations

import re


def marker_id_from_label(label: str):
    match = re.search(r'marker_(\d+)', label)
    return int(match.group(1)) if match else None


def should_skip_label(label: str, skip_marker_ids) -> bool:
    marker_id = marker_id_from_label(label)
    return marker_id is not None and marker_id in set(skip_marker_ids)


def choose_avoid_direction(left_clearance: float, right_clearance: float) -> int:
    return 1 if left_clearance >= right_clearance else -1
