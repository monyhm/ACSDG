import math
from typing import Dict

from core.constants import GRID_UNIT_M


def score_threat(t: Dict) -> Dict:
    dist_grid = math.hypot(50 - t["x"], 50 - t["y"])
    dist_m = dist_grid * GRID_UNIT_M
    proximity_score = max(0, 40 - (dist_m / 20))
    dx, dy = 50 - t["x"], 50 - t["y"]
    angle_to_asset = math.degrees(math.atan2(dy, dx)) % 360
    heading_diff = abs((t["heading_deg"] - angle_to_asset + 180) % 360 - 180)
    heading_score = max(0, 25 - (heading_diff / 7.2))
    speed_score = min(15, (t["speed_mps"] / 70) * 15)
    tti = dist_m / t["speed_mps"] if t["speed_mps"] > 0 else 99
    tti_score = max(0, 10 - (tti / 10))
    rcs_score = max(0, 5 - (t["rcs_m2"] * 50))
    conf_score = t.get("confidence", 0.9) * 5
    total_score = round(
        min(
            100,
            proximity_score
            + heading_score
            + speed_score
            + tti_score
            + rcs_score
            + conf_score,
        )
    )

    classification = "UNKNOWN"
    if t["rcs_m2"] < 0.03 and t["speed_mps"] < 20:
        classification = "MULTIROTOR"
        threat_type = "FPV_DRONE"
    elif t["rcs_m2"] < 0.06 and t["speed_mps"] < 60:
        classification = "LOITERING"
        threat_type = "SHAHED_136"
    elif t["speed_mps"] >= 60:
        classification = "FIXED_WING"
        threat_type = "QASEF_2K"
    else:
        classification = "UNKNOWN"
        threat_type = "GENERIC_UAV"

    result = t.copy()
    result.update(
        {
            "score": total_score,
            "tti_s": round(tti, 1),
            "classification": classification,
            "threat_type": (
                "SHAHED_136" if classification == "LOITERING" else "GENERIC_UAV"
            ),
            "status": t.get("status", "ACTIVE"),
        }
    )
    return result
