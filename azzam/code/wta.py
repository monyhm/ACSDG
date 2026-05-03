import math
from typing import List, Dict, Optional


# Sensor-interceptor pairing map
TOWER_POSITIONS = {
    "RADAR_NORTH": [50, 5],
    "RADAR_SOUTH": [50, 95],
    "RADAR_EAST": [95, 50],
    "RADAR_WEST": [5, 50],
}

INTERCEPTOR_BASE = {
    "INT-NORTH": {"x": 50.0, "y": 15.0, "paired_tower": "RADAR_NORTH"},
    "INT-SOUTH": {"x": 50.0, "y": 85.0, "paired_tower": "RADAR_SOUTH"},
    "INT-EAST": {"x": 85.0, "y": 50.0, "paired_tower": "RADAR_EAST"},
    "INT-WEST": {"x": 15.0, "y": 50.0, "paired_tower": "RADAR_WEST"},
}


class WTA:
    def __init__(self):
        self.interceptors = {
            iid: {
                "id": iid,
                "x": data["x"],
                "y": data["y"],
                "base_x": data["x"],
                "base_y": data["y"],
                "speed_mps": 0.0,
                "heading_deg": 0.0,
                "status": "AVAILABLE",
                "target": None,
                "proximity": 0.0,
                "paired_tower": data["paired_tower"],
            }
            for iid, data in INTERCEPTOR_BASE.items()
        }

    def _closest_available(self, threat: Dict) -> Optional[str]:
        """Find closest available interceptor to threat position."""
        best_iid = None
        best_dist = float("inf")

        for iid, state in self.interceptors.items():
            if state["status"] != "AVAILABLE":
                continue
            dist = math.hypot(state["x"] - threat["x"], state["y"] - threat["y"])
            if dist < best_dist:
                best_dist = dist
                best_iid = iid

        return best_iid

    def _paired_interceptor(self, threat: Dict, threat_origin: Dict) -> Optional[str]:
        """
        Return the interceptor paired with the tower that first detected
        this threat. Falls back to closest available if paired is busy.
        """
        origin_tower = threat_origin.get(threat["id"])
        if not origin_tower:
            return self._closest_available(threat)

        # Find interceptor paired to that tower
        paired_iid = next(
            (
                iid
                for iid, data in INTERCEPTOR_BASE.items()
                if data["paired_tower"] == origin_tower
            ),
            None,
        )

        if paired_iid and self.interceptors[paired_iid]["status"] == "AVAILABLE":
            return paired_iid

        # Paired is busy — find closest available
        return self._closest_available(threat)

    def assign(
        self,
        scored_tracks: List[Dict],
        current_interceptor_states: Dict,
        threat_origin: Optional[Dict] = None,
    ) -> List[Dict]:
        threat_origin = threat_origin or {}

        # Sync interceptor states from guidance engine
        for iid, state in current_interceptor_states.items():
            if iid in self.interceptors:
                # Preserve base positions — never overwrite from external state
                bx = self.interceptors[iid]["base_x"]
                by = self.interceptors[iid]["base_y"]
                self.interceptors[iid].update(state)
                self.interceptors[iid]["base_x"] = bx
                self.interceptors[iid]["base_y"] = by

        assignments = []

        # Sort by score descending — highest threat first
        active = []
        for t in scored_tracks:
            score_ready = t["score"] > 20 and t.get("status") == "ACTIVE"
            tti_urgent = (
                t.get("tti_s", 99) < 90
                and t.get("confidence", 0) > 0.35
                and t.get("status") in ["TRACKING", "ACTIVE"]
            )
            if score_ready or tti_urgent:
                active.append(t)
        active = sorted(active, key=lambda x: x["score"], reverse=True)

        already_targeted = {
            state["target"]
            for state in self.interceptors.values()
            if state["target"] is not None
        }

        for threat in active:
            # Skip if already being engaged
            if threat["id"] in already_targeted:
                continue

            # Layer 1: paired interceptor
            # Layer 2: closest available fallback
            iid = self._paired_interceptor(threat, threat_origin)

            if not iid:
                continue  # no interceptors available

            self.interceptors[iid]["status"] = "DISPATCHED"
            self.interceptors[iid]["target"] = threat["id"]
            already_targeted.add(threat["id"])

            assignments.append(
                {
                    "interceptor_id": iid,
                    "threat_id": threat["id"],
                    "intercept_x": threat["x"],
                    "intercept_y": threat["y"],
                    "decision_layer": "rule",
                    "paired": self.interceptors[iid]["paired_tower"]
                    == threat_origin.get(threat["id"], ""),
                }
            )

        return assignments

    def free_interceptor(self, iid: str, return_to_base: bool = True):
        """Call this after a kill or miss to reset interceptor."""
        if iid not in self.interceptors:
            return
        self.interceptors[iid]["status"] = "AVAILABLE"
        self.interceptors[iid]["target"] = None
        self.interceptors[iid]["proximity"] = 0.0

        if return_to_base:
            self.interceptors[iid]["x"] = self.interceptors[iid]["base_x"]
            self.interceptors[iid]["y"] = self.interceptors[iid]["base_y"]
            self.interceptors[iid]["speed_mps"] = 0.0
