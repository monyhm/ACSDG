"""ADAMANT v2 RL environment — variable-config, Dict-observation, MultiDiscrete actions.

Standalone from rl/environment.py. Physics matches the live backend grid (1 unit = 200 m,
20 km × 20 km field, 2 Hz tick). Step reward is the 6-component shaped reward from
rl/v2/reward.py; per-step components are exposed via info["reward_components"], and
per-episode totals via info["episode_reward_components"] on terminal/truncated steps.
"""

from __future__ import annotations

import math
from typing import Any, Optional

import gymnasium as gym
import numpy as np

from rl.v2.reward import compute_reward_v2

REWARD_COMPONENT_KEYS = (
    "total", "survival", "coverage", "depletion",
    "kill_bonus", "waste_penalty", "geometry",
)


# Physics constants
ASSET = (50.0, 50.0)
GRID_UNIT_M = 200.0
DT = 0.5
MAX_EPISODE_STEPS = 350
INTERCEPT_RADIUS = 0.5
ASSET_HIT_RADIUS = 0.5
INTERCEPTOR_MAX_SPEED_MPS = 80.0
INTERCEPTOR_ACCEL_MPSS = 20.0
HIT_PROBABILITY = 0.85

# Threat type specs (locked)
THREAT_SPECS: dict[str, dict[str, Any]] = {
    "shahed":  {"speed_mps": 57.0, "rcs_m2": 0.04, "alt_m": 200.0,  "classification": "LOITERING"},
    "fpv":     {"speed_mps": 15.0, "rcs_m2": 0.02, "alt_m": 100.0,  "classification": "MULTIROTOR"},
    "qasef":   {"speed_mps": 61.0, "rcs_m2": 0.05, "alt_m": 2500.0, "classification": "FIXED_WING"},
    "samad":   {"speed_mps": 63.0, "rcs_m2": 0.10, "alt_m": 5000.0, "classification": "FIXED_WING"},
    "mohajer": {"speed_mps": 55.0, "rcs_m2": 0.06, "alt_m": 3000.0, "classification": "FIXED_WING"},
    "ababil":  {"speed_mps": 50.0, "rcs_m2": 0.03, "alt_m": 1500.0, "classification": "FIXED_WING"},
}
THREAT_TYPES = list(THREAT_SPECS.keys())

CLASSIFICATION_IDX = {
    "MULTIROTOR": 0,
    "LOITERING": 1,
    "FIXED_WING": 2,
    "UNKNOWN": 3,
}

STATUS_IDX = {
    "READY": 0,
    "DISPATCHED": 1,
    "INTERCEPT": 2,
    "RELOAD": 3,
}

INTERCEPTOR_BASES = [
    ("INT-NORTH", 50.0, 15.0),
    ("INT-SOUTH", 50.0, 85.0),
    ("INT-EAST",  85.0, 50.0),
    ("INT-WEST",  15.0, 50.0),
]

# Per-scenario defaults. `attack_axes=None` means "use whatever the user passed".
SCENARIO_DEFAULTS = {
    "foundation": {"decoy_ratio": 0.0,  "attack_axes": 1,    "elevated_rcs": False, "degraded": False},
    "scarcity":   {"decoy_ratio": 0.20, "attack_axes": 1,    "elevated_rcs": False, "degraded": False},
    "multi_axis": {"decoy_ratio": 0.0,  "attack_axes": None, "elevated_rcs": False, "degraded": False},
    "spoofing":   {"decoy_ratio": 0.40, "attack_axes": 1,    "elevated_rcs": True,  "degraded": False},
    "saturation": {"decoy_ratio": 0.0,  "attack_axes": 1,    "elevated_rcs": False, "degraded": False},
    "degraded":   {"decoy_ratio": 0.0,  "attack_axes": 1,    "elevated_rcs": False, "degraded": True},
}
SCENARIO_LIST = list(SCENARIO_DEFAULTS.keys())


class AdamantEnvV2(gym.Env):
    metadata = {"render_modes": []}

    def __init__(
        self,
        n_threats: int = 6,
        n_interceptors: int = 4,
        scenario: str = "mixed",
        max_threats: int = 20,
        max_interceptors: int = 16,
        decoy_ratio: float = 0.0,
        attack_axes: int = 1,
        seed: Optional[int] = None,
    ):
        super().__init__()

        if n_threats > max_threats:
            raise ValueError(f"n_threats ({n_threats}) exceeds max_threats ({max_threats})")
        if n_interceptors > max_interceptors:
            raise ValueError(f"n_interceptors ({n_interceptors}) exceeds max_interceptors ({max_interceptors})")
        if scenario != "mixed" and scenario not in SCENARIO_DEFAULTS:
            raise ValueError(f"Unknown scenario: {scenario}")

        self.n_threats = n_threats
        self.n_interceptors = n_interceptors
        self.scenario_name = scenario
        self.max_threats = max_threats
        self.max_interceptors = max_interceptors
        # User overrides only kick in when truthy (>0). Side-effect: explicit 0.0 cannot
        # zero out a scenario default (e.g. spoofing's 0.4); accepted for v2 simplicity.
        self.user_decoy_ratio = decoy_ratio
        self.user_attack_axes = attack_axes

        self.observation_space = gym.spaces.Dict({
            "threats": gym.spaces.Box(
                low=-np.inf, high=np.inf, shape=(max_threats, 9), dtype=np.float32
            ),
            "interceptors": gym.spaces.Box(
                low=-np.inf, high=np.inf, shape=(max_interceptors, 5), dtype=np.float32
            ),
            "threat_mask": gym.spaces.Box(
                low=0.0, high=1.0, shape=(max_threats,), dtype=np.float32
            ),
            "interceptor_mask": gym.spaces.Box(
                low=0.0, high=1.0, shape=(max_interceptors,), dtype=np.float32
            ),
        })

        self.action_space = gym.spaces.MultiDiscrete([max_threats + 1] * max_interceptors)
        self.HOLD_ACTION = max_threats

        self.threats: list[dict] = []
        self.interceptors: list[dict] = []
        self.tick = 0
        self.asset_hit = False
        self.active_scenario = scenario
        self.scenario_config: dict = {}
        self.degraded_quadrant: Optional[int] = None
        self._next_threat_id = 0

        self.reset(seed=seed)

    # ------------------------------------------------------------------ reset
    def reset(self, seed: Optional[int] = None, options: Optional[dict] = None):
        super().reset(seed=seed)

        if self.scenario_name == "mixed":
            self.active_scenario = SCENARIO_LIST[int(self.np_random.integers(0, len(SCENARIO_LIST)))]
        else:
            self.active_scenario = self.scenario_name

        defaults = SCENARIO_DEFAULTS[self.active_scenario]
        decoy_ratio = self.user_decoy_ratio if self.user_decoy_ratio > 0 else defaults["decoy_ratio"]
        attack_axes = defaults["attack_axes"] if defaults["attack_axes"] is not None else self.user_attack_axes
        attack_axes = max(1, min(4, int(attack_axes)))
        elevated_rcs = defaults["elevated_rcs"]
        degraded = defaults["degraded"]

        self.degraded_quadrant = (
            int(self.np_random.integers(0, 4)) if degraded else None
        )

        self.scenario_config = {
            "active_scenario": self.active_scenario,
            "decoy_ratio": decoy_ratio,
            "attack_axes": attack_axes,
            "elevated_rcs": elevated_rcs,
            "degraded": degraded,
            "degraded_quadrant": self.degraded_quadrant,
            "n_threats": self.n_threats,
            "n_interceptors": self.n_interceptors,
        }

        # Pick allowed spawn sides
        sides_all = ["N", "S", "E", "W"]
        if attack_axes >= 4:
            allowed_sides = sides_all
        else:
            chosen = self.np_random.choice(len(sides_all), size=attack_axes, replace=False)
            allowed_sides = [sides_all[int(i)] for i in chosen]

        self._next_threat_id = 0
        self.tick = 0
        self.asset_hit = False
        self.threats = []
        self._episode_reward_components = {k: 0.0 for k in REWARD_COMPONENT_KEYS}

        n_decoys = int(round(self.n_threats * decoy_ratio))
        n_real = self.n_threats - n_decoys
        for _ in range(n_real):
            self.threats.append(self._spawn_threat(False, allowed_sides, elevated_rcs))
        for _ in range(n_decoys):
            self.threats.append(self._spawn_threat(True, allowed_sides, elevated_rcs))

        self.interceptors = []
        for i in range(self.n_interceptors):
            base_id, bx, by = INTERCEPTOR_BASES[i % len(INTERCEPTOR_BASES)]
            inst_idx = i // len(INTERCEPTOR_BASES)
            iid = base_id if inst_idx == 0 else f"{base_id}-{inst_idx}"
            self.interceptors.append({
                "id": iid,
                "x": bx, "y": by,
                "base_x": bx, "base_y": by,
                "speed_mps": 0.0,
                "heading_deg": 0.0,
                "status": "READY",
                "target_idx": None,
            })

        return self._get_obs(), {"scenario_config": self.scenario_config}

    # ------------------------------------------------------------------ spawn
    def _spawn_threat(self, is_decoy: bool, sides: list[str], elevated_rcs: bool) -> dict:
        spec_name = THREAT_TYPES[int(self.np_random.integers(0, len(THREAT_TYPES)))]
        spec = THREAT_SPECS[spec_name]

        side = sides[int(self.np_random.integers(0, len(sides)))]
        if side == "N":
            x = float(self.np_random.uniform(0.0, 100.0))
            y = float(self.np_random.uniform(0.0, 5.0))
        elif side == "S":
            x = float(self.np_random.uniform(0.0, 100.0))
            y = float(self.np_random.uniform(95.0, 100.0))
        elif side == "E":
            x = float(self.np_random.uniform(95.0, 100.0))
            y = float(self.np_random.uniform(0.0, 100.0))
        else:  # W
            x = float(self.np_random.uniform(0.0, 5.0))
            y = float(self.np_random.uniform(0.0, 100.0))

        speed_grid = spec["speed_mps"] / GRID_UNIT_M
        ang_to_asset = math.atan2(ASSET[1] - y, ASSET[0] - x)
        if is_decoy:
            offset_sign = 1.0 if self.np_random.random() < 0.5 else -1.0
            offset_deg = float(self.np_random.uniform(60.0, 120.0))
            heading = ang_to_asset + offset_sign * math.radians(offset_deg)
        else:
            heading = ang_to_asset

        vx = speed_grid * math.cos(heading)
        vy = speed_grid * math.sin(heading)

        rcs = spec["rcs_m2"]
        if elevated_rcs and is_decoy:
            rcs *= 3.0  # spoofing: decoys mimic larger threats

        confidence = float(min(0.98, 0.7 + spec["rcs_m2"] * 3.0))

        threat_id = f"t_{self._next_threat_id}_{spec_name}"
        self._next_threat_id += 1

        return {
            "id": threat_id,
            "type": spec_name,
            "x": x, "y": y,
            "vx": vx, "vy": vy,
            "speed_mps": spec["speed_mps"],
            "speed_grid": speed_grid,
            "heading_rad": heading,
            "rcs_m2": rcs,
            "alt_m": spec["alt_m"],
            "classification": spec["classification"],
            "confidence": confidence,
            "is_decoy": is_decoy,
        }

    # ------------------------------------------------------------------ action
    def _apply_action(self, action) -> None:
        a = np.asarray(action).flatten()
        for inter_idx in range(self.n_interceptors):
            cmd = int(a[inter_idx])
            if cmd == self.HOLD_ACTION:
                continue
            if cmd >= len(self.threats):
                continue  # invalid (padding slot or stale) — treated as hold
            intc = self.interceptors[inter_idx]
            if intc["status"] != "READY":
                continue  # already engaged
            intc["status"] = "DISPATCHED"
            intc["target_idx"] = cmd

    # ------------------------------------------------------------------ physics
    def _step_physics(self) -> dict:
        # kills/misses/wasted are lists of {"threat_id", "interceptor_id"}; reward.py
        # consumes these directly and uses interceptor_id for per-engagement geometry.
        events = {
            "kills": [],
            "misses": [],
            "wasted_interceptors": [],
            "intercepts": [],
            "asset_hit": False,
            "decoys_exited": 0,
        }

        # Move threats
        for t in self.threats:
            t["x"] += t["vx"] * DT
            t["y"] += t["vy"] * DT

        # Phase A: pursue or queue engagement
        engagements_by_target: dict[int, list[int]] = {}
        for inter_idx, intc in enumerate(self.interceptors):
            if intc["status"] in ("READY", "RELOAD"):
                continue
            target_idx = intc["target_idx"]
            if target_idx is None or target_idx >= len(self.threats):
                self._return_to_base(intc)
                continue
            target = self.threats[target_idx]
            dx = target["x"] - intc["x"]
            dy = target["y"] - intc["y"]
            dist = math.hypot(dx, dy)

            if dist < INTERCEPT_RADIUS:
                engagements_by_target.setdefault(target_idx, []).append(inter_idx)
            else:
                self._move_toward(intc, target, dist, dx, dy)

        # Phase B: resolve engagements; multi-engagement → first hit credited, others wasted
        kills_set: set[int] = set()
        for target_idx, inter_idxs in engagements_by_target.items():
            target = self.threats[target_idx]
            if len(inter_idxs) == 1:
                inter_idx = inter_idxs[0]
                intc = self.interceptors[inter_idx]
                hit = self.np_random.random() < HIT_PROBABILITY
                if hit:
                    kills_set.add(target_idx)
                    events["kills"].append(self._build_kill_event(intc, target))
                    events["intercepts"].append({
                        "interceptor_id": intc["id"],
                        "threat_id": target["id"],
                        "outcome": "KILL",
                    })
                else:
                    events["misses"].append({
                        "threat_id": target["id"],
                        "interceptor_id": intc["id"],
                    })
                    events["intercepts"].append({
                        "interceptor_id": intc["id"],
                        "threat_id": target["id"],
                        "outcome": "MISS",
                    })
                self._return_to_base(intc)
            else:
                # Multi-engagement: roll in iteration order until a hit; later interceptors
                # don't roll (target already dead). All non-killers are "wasted".
                killer_idx_in_list: Optional[int] = None
                for i, inter_idx in enumerate(inter_idxs):
                    if killer_idx_in_list is None and self.np_random.random() < HIT_PROBABILITY:
                        killer_idx_in_list = i
                        intc = self.interceptors[inter_idx]
                        kills_set.add(target_idx)
                        events["kills"].append(self._build_kill_event(intc, target))
                        events["intercepts"].append({
                            "interceptor_id": intc["id"],
                            "threat_id": target["id"],
                            "outcome": "KILL",
                        })
                for i, inter_idx in enumerate(inter_idxs):
                    intc = self.interceptors[inter_idx]
                    if i != killer_idx_in_list:
                        events["wasted_interceptors"].append({
                            "interceptor_id": intc["id"],
                            "threat_id": target["id"],
                        })
                        events["intercepts"].append({
                            "interceptor_id": intc["id"],
                            "threat_id": target["id"],
                            "outcome": "WASTED",
                        })
                    self._return_to_base(intc)

        # Asset hit check (skip already-killed and decoys)
        for i, t in enumerate(self.threats):
            if i in kills_set or t.get("is_decoy", False):
                continue
            if math.hypot(t["x"] - ASSET[0], t["y"] - ASSET[1]) < ASSET_HIT_RADIUS:
                self.asset_hit = True
                events["asset_hit"] = True
                break

        # Remove killed threats and reindex interceptor target_idx
        if kills_set:
            new_threats: list[dict] = []
            old_to_new: dict[int, int] = {}
            for old_idx, t in enumerate(self.threats):
                if old_idx in kills_set:
                    continue
                old_to_new[old_idx] = len(new_threats)
                new_threats.append(t)
            for intc in self.interceptors:
                if intc["target_idx"] is None:
                    continue
                if intc["target_idx"] in kills_set:
                    self._return_to_base(intc)
                else:
                    intc["target_idx"] = old_to_new[intc["target_idx"]]
            self.threats = new_threats

        # Remove off-grid threats (decoys flying past)
        kept = [
            i for i, t in enumerate(self.threats)
            if -10.0 <= t["x"] <= 110.0 and -10.0 <= t["y"] <= 110.0
        ]
        if len(kept) < len(self.threats):
            keep_set = set(kept)
            old_to_new = {old: new for new, old in enumerate(kept)}
            for intc in self.interceptors:
                if intc["target_idx"] is None:
                    continue
                if intc["target_idx"] not in keep_set:
                    self._return_to_base(intc)
                else:
                    intc["target_idx"] = old_to_new[intc["target_idx"]]
            events["decoys_exited"] = sum(
                1 for i, t in enumerate(self.threats)
                if i not in keep_set and t.get("is_decoy", False)
            )
            self.threats = [self.threats[i] for i in kept]

        return events

    @staticmethod
    def _return_to_base(intc: dict) -> None:
        intc["status"] = "READY"
        intc["target_idx"] = None
        intc["x"] = intc["base_x"]
        intc["y"] = intc["base_y"]
        intc["speed_mps"] = 0.0

    @classmethod
    def _build_kill_event(cls, intc: dict, target: dict) -> dict:
        return {
            "threat_id": target["id"],
            "interceptor_id": intc["id"],
            "kill_x": target["x"],
            "kill_y": target["y"],
            "geometry_score": cls._compute_geometry_score(intc, target),
        }

    @staticmethod
    def _compute_geometry_score(intc: dict, target: dict) -> float:
        # 1.0 - |cos(angle)| between interceptor and threat velocity vectors.
        # Returns 0.0 when either is stationary (no meaningful angle).
        intc_speed_grid = intc["speed_mps"] / GRID_UNIT_M
        threat_v_mag = math.hypot(target["vx"], target["vy"])
        if intc_speed_grid < 1e-9 or threat_v_mag < 1e-9:
            return 0.0
        intc_h = math.radians(intc["heading_deg"])
        intc_vx = intc_speed_grid * math.cos(intc_h)
        intc_vy = intc_speed_grid * math.sin(intc_h)
        intc_v_mag = math.hypot(intc_vx, intc_vy)
        cos_a = (intc_vx * target["vx"] + intc_vy * target["vy"]) / (intc_v_mag * threat_v_mag)
        cos_a = max(-1.0, min(1.0, cos_a))
        return 1.0 - abs(cos_a)

    @staticmethod
    def _move_toward(intc: dict, target: dict, dist: float, dx: float, dy: float) -> None:
        interceptor_speed = min(
            INTERCEPTOR_MAX_SPEED_MPS,
            intc["speed_mps"] + INTERCEPTOR_ACCEL_MPSS * DT,
        )
        speed_grid = interceptor_speed / GRID_UNIT_M
        tti_est = dist / speed_grid if speed_grid > 0 else 1.0
        lead_x = target["x"] + target["vx"] * tti_est * 0.5
        lead_y = target["y"] + target["vy"] * tti_est * 0.5
        ldx = lead_x - intc["x"]
        ldy = lead_y - intc["y"]
        lead_dist = math.hypot(ldx, ldy)
        if lead_dist > 1e-9:
            nx, ny = ldx / lead_dist, ldy / lead_dist
        else:
            nx, ny = dx / max(dist, 1e-9), dy / max(dist, 1e-9)
        intc["x"] += nx * speed_grid * DT
        intc["y"] += ny * speed_grid * DT
        intc["speed_mps"] = interceptor_speed
        intc["heading_deg"] = math.degrees(math.atan2(ny, nx)) % 360
        intc["status"] = "DISPATCHED"

    # ------------------------------------------------------------------ step
    def step(self, action):
        self.tick += 1
        self._apply_action(action)
        events = self._step_physics()

        terminated = self.asset_hit or len(self.threats) == 0
        truncated = (not terminated) and self.tick >= MAX_EPISODE_STEPS

        state = {
            "threats": self.threats,
            "interceptors": self.interceptors,
            "tick": self.tick,
            "asset_hit": self.asset_hit,
        }
        reward_components = compute_reward_v2(events, state, terminated, truncated)
        for k, v in reward_components.items():
            self._episode_reward_components[k] += v

        info = {
            "reward_components": reward_components,
            "scenario_config": self.scenario_config,
            "events": events,
            "tick": self.tick,
            "n_threats_remaining": len(self.threats),
            "n_real_threats_remaining": sum(
                1 for t in self.threats if not t.get("is_decoy", False)
            ),
        }
        if terminated or truncated:
            info["episode_reward_components"] = dict(self._episode_reward_components)

        return self._get_obs(), float(reward_components["total"]), terminated, truncated, info

    # ------------------------------------------------------------------ obs
    def _get_obs(self) -> dict[str, np.ndarray]:
        threats_arr = np.zeros((self.max_threats, 9), dtype=np.float32)
        threat_mask = np.zeros(self.max_threats, dtype=np.float32)

        for i, t in enumerate(self.threats[: self.max_threats]):
            tti = self._compute_tti(t)
            score = self._compute_score(t, tti)

            x_obs, y_obs = t["x"], t["y"]
            vx_obs, vy_obs = t["vx"], t["vy"]
            if self.degraded_quadrant is not None:
                if self._quadrant_of(t["x"], t["y"]) == self.degraded_quadrant:
                    # +50% noise on this quadrant's observed kinematics
                    noise_scale = 1.5
                    x_obs += float(self.np_random.normal(0.0, 0.3 * noise_scale))
                    y_obs += float(self.np_random.normal(0.0, 0.3 * noise_scale))
                    vx_obs += float(self.np_random.normal(0.0, 0.05 * noise_scale))
                    vy_obs += float(self.np_random.normal(0.0, 0.05 * noise_scale))

            cls_idx = CLASSIFICATION_IDX.get(t["classification"], CLASSIFICATION_IDX["UNKNOWN"])

            threats_arr[i] = [
                x_obs, y_obs, vx_obs, vy_obs,
                tti, score, t["rcs_m2"], t["confidence"], float(cls_idx),
            ]
            threat_mask[i] = 1.0

        intc_arr = np.zeros((self.max_interceptors, 5), dtype=np.float32)
        intc_mask = np.zeros(self.max_interceptors, dtype=np.float32)
        for i, intc in enumerate(self.interceptors[: self.max_interceptors]):
            status = STATUS_IDX[intc["status"]]
            available = 1.0 if intc["status"] == "READY" else 0.0
            intc_arr[i] = [intc["x"], intc["y"], intc["speed_mps"], float(status), available]
            intc_mask[i] = 1.0

        return {
            "threats": threats_arr,
            "interceptors": intc_arr,
            "threat_mask": threat_mask,
            "interceptor_mask": intc_mask,
        }

    @staticmethod
    def _compute_tti(t: dict) -> float:
        dist_m = math.hypot(ASSET[0] - t["x"], ASSET[1] - t["y"]) * GRID_UNIT_M
        speed = t["speed_mps"]
        if speed <= 0.0:
            return 99.0
        return dist_m / speed

    @staticmethod
    def _compute_score(t: dict, tti: float) -> float:
        dist_m = math.hypot(ASSET[0] - t["x"], ASSET[1] - t["y"]) * GRID_UNIT_M
        speed = t["speed_mps"]

        proximity_score = max(0.0, 40.0 - (dist_m / 400.0))
        ang_to_asset = math.degrees(math.atan2(ASSET[1] - t["y"], ASSET[0] - t["x"])) % 360.0
        heading_deg = math.degrees(math.atan2(t["vy"], t["vx"])) % 360.0
        heading_diff = abs((heading_deg - ang_to_asset + 180.0) % 360.0 - 180.0)
        heading_score = max(0.0, 25.0 - (heading_diff / 7.2))
        speed_score = min(15.0, (speed / 70.0) * 15.0)
        tti_score = max(0.0, 10.0 - (tti / 10.0))
        rcs_score = max(0.0, 5.0 - (t["rcs_m2"] * 50.0))
        conf_score = t["confidence"] * 5.0
        return min(
            100.0,
            proximity_score + heading_score + speed_score + tti_score + rcs_score + conf_score,
        )

    @staticmethod
    def _quadrant_of(x: float, y: float) -> int:
        if x >= ASSET[0] and y >= ASSET[1]:
            return 0
        if x < ASSET[0] and y >= ASSET[1]:
            return 1
        if x < ASSET[0] and y < ASSET[1]:
            return 2
        return 3

    def render(self):
        pass
