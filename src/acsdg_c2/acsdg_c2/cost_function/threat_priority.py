"""Threat priority scoring — extracted verbatim from the legacy c2_engine `_score`.

Score = 0.5·proximity + 0.3·heading + 0.2·speed, all in [0, 1].
Behavior must match the legacy function for any track values that fed it.
"""

from __future__ import annotations

import math

from acsdg_c2.weapons.types import Track

MAX_RANGE = 400.0   # m — same value as legacy c2_engine_node.MAX_RANGE
MAX_SPEED = 20.0    # m/s — same value as legacy c2_engine_node.MAX_SPEED


def score_threat(track: Track) -> float:
    """Return the priority score in [0, 1] for a fused target track."""
    px, py, pz = track.position
    vx, vy, vz = track.velocity

    dist = math.sqrt(px * px + py * py + pz * pz)
    speed = math.sqrt(vx * vx + vy * vy + vz * vz)

    proximity = max(0.0, 1.0 - dist / MAX_RANGE)
    speed_s = min(1.0, speed / MAX_SPEED)

    if dist > 1e-6 and speed > 1e-6:
        toward_x = -px / dist
        toward_y = -py / dist
        toward_z = -pz / dist
        v_unit_x = vx / speed
        v_unit_y = vy / speed
        v_unit_z = vz / speed
        heading = max(0.0,
                      toward_x * v_unit_x
                      + toward_y * v_unit_y
                      + toward_z * v_unit_z)
    else:
        heading = 0.0

    return 0.5 * proximity + 0.3 * heading + 0.2 * speed_s
