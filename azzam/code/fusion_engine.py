import math
import time
from typing import Dict, List, Optional
from sensor_fusion.kalman import KalmanTracker
from core.constants import GRID_UNIT_M


class FusionEngine:
    def __init__(self):
        self.TOWERS = {
            "RADAR_NORTH": [50, 5],
            "RADAR_SOUTH": [50, 95],
            "RADAR_EAST": [95, 50],
            "RADAR_WEST": [5, 50],
        }
        self.trackers = {}
        self.last_seen = {}
        self.last_rcs = {}
        self.expiry_threshold = 8.0
        self.smoothed_speed = {}
        self.locked_classification = {}
        self.locked_threat_type = {}

    def _cleanup_old_tracks(self):
        now = time.time()
        expired = [
            tid
            for tid, ts in self.last_seen.items()
            if now - ts > self.expiry_threshold
        ]
        for tid in expired:
            self.trackers.pop(tid, None)
            self.locked_classification.pop(tid, None)
            self.locked_threat_type.pop(tid, None)
            self.smoothed_speed.pop(tid, None)
            self.last_seen.pop(tid, None)
            self.last_rcs.pop(tid, None)

    def _polar_to_grid(self, tower_id, range_m, brg_deg, el_deg):
        tx, ty = self.TOWERS[tower_id]
        range_grid = range_m / GRID_UNIT_M  # horizontal range directly
        brg_rad = math.radians(brg_deg)
        el_rad = math.radians(el_deg)
        x = tx + range_grid * math.cos(brg_rad)
        y = ty + range_grid * math.sin(brg_rad)
        z = range_m * math.tan(el_rad)  # altitude in meters
        return x, y, z

    async def update(self, reading):
        self._cleanup_old_tracks()
        tid = reading["threat_id"]
        gx, gy, gz = self._polar_to_grid(
            reading["tower_id"],
            reading["range_m"],
            reading["bearing_deg"],
            reading["elevation_deg"],
        )

        if tid not in self.trackers:
            tracker = KalmanTracker(tid)
            # Initialize position from first measurement
            tracker.x[0] = gx
            tracker.x[1] = gy
            tracker.x[2] = gz
            # Seed velocity toward asset based on RCS
            rcs = reading.get("rcs_m2", 0.04)
            if rcs <= 0.02:
                estimated_speed = 15.0
            elif rcs <= 0.04:
                estimated_speed = 57.0
            elif rcs <= 0.06:
                estimated_speed = 61.0
            else:
                estimated_speed = 63.0
            dx = 50.0 - gx
            dy = 50.0 - gy
            dist = math.hypot(dx, dy)
            if dist > 0:
                tracker.x[3] = (dx / dist) * estimated_speed / GRID_UNIT_M
                tracker.x[4] = (dy / dist) * estimated_speed / GRID_UNIT_M
            self.trackers[tid] = tracker
            tid_base = tid.split("_")[0].lower()
            _type_map = {
                "shahed": ("LOITERING", "SHAHED_136"),
                "fpv": ("MULTIROTOR", "FPV_DRONE"),
                "qasef": ("FIXED_WING", "QASEF_2K"),
                "samad": ("FIXED_WING", "SAMAD_3"),
                "mohajer": ("FIXED_WING", "MOHAJER_6"),
                "ababil": ("FIXED_WING", "ABABIL_3"),
            }
            _cls, _ttype = _type_map.get(tid_base, ("UNKNOWN", "GENERIC_UAV"))
            self.locked_classification[tid] = _cls
            self.locked_threat_type[tid] = _ttype

        tracker = self.trackers[tid]
        now = time.time()
        last = self.last_seen.get(tid, 0)

        # Predict only once per 100ms window
        if now - last >= 0.09:
            tracker.predict()

        tracker.update([gx, gy, gz])
        self.last_seen[tid] = now
        self.last_rcs[tid] = reading["rcs_m2"]

        state = tracker.get_state()
        return self._format_track(tid, state, reading["ts"])

    def _format_track(self, tid, state, ts):
        vx_mps = state[3] * GRID_UNIT_M
        vy_mps = state[4] * GRID_UNIT_M
        vz_mps = state[5] * GRID_UNIT_M if len(state) > 5 else 0.0
        speed = math.sqrt(vx_mps**2 + vy_mps**2 + vz_mps**2)
        hits = self.trackers[tid].hits

        if hits < 3:
            speed = 0.0

        alpha = 0.15
        prev = self.smoothed_speed.get(tid, speed)
        smoothed = alpha * speed + (1 - alpha) * prev
        self.smoothed_speed[tid] = smoothed

        if hits < 3:
            track_status = "DETECTING"
        elif hits < 8:
            track_status = "TRACKING"
        else:
            track_status = "ACTIVE"

        # TTI: distance from threat to asset [50,50] in meters / speed
        dist_to_asset_grid = math.hypot(50.0 - state[0], 50.0 - state[1])
        dist_to_asset_m = dist_to_asset_grid * GRID_UNIT_M
        tti_s = round(dist_to_asset_m / smoothed, 1) if smoothed > 1.0 else None

        return {
            "id": tid,
            "x": round(state[0], 2),
            "y": round(state[1], 2),
            "z": round(state[2], 2),
            "speed_mps": round(speed, 2),
            "speed": round(speed, 2),
            "heading_deg": round(math.degrees(math.atan2(state[4], state[3])) % 360, 2),
            "confidence": 0.95,
            "rcs_m2": self.last_rcs.get(tid, 0.04),
            "ts": ts,
            "status": track_status,
            "hits": hits,
            "tti_s": tti_s,
            "threat_type": self.locked_threat_type.get(tid, "UNKNOWN"),
            "classification": self.locked_classification.get(tid, "UNKNOWN"),
        }

    def get_all_tracks(self):
        return [
            self._format_track(tid, trk.get_state(), int(self.last_seen[tid] * 1000))
            for tid, trk in self.trackers.items()
        ]
