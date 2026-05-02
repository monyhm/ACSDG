# Phase 2: Coyote Block 2 — Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Add the Raytheon Coyote Block 2 frag-warhead jet interceptor as the second concrete `WeaponSystem`, replacing one Anvil at the NE post. After Phase 2, the heterogeneous inventory is **3 Anvils + 1 Coyote**, and Hungarian assignment now picks between fundamentally different airframes (electric quadcopter vs jet missile) based on engagement geometry.

**Architecture:** New `Coyote` class implementing the existing `WeaponSystem` ABC. New SDF (jet form, no rotors), new C++ controller (high-speed pursuit + proximity fuze). World SDF and launch files swap `interceptor_1 → coyote_1` at the NE post. C2 engine registers a Coyote in slot-0 of the weapon registry. Bridge shim wires the new model. Numeric-tail mapping in `Dispatcher` (`coyote_0 → interceptor_id 1`) Just Works because Coyote takes Anvil's old slot — no Dispatcher changes.

**Tech Stack:** Python 3.10 (Coyote class, c2_engine), C++17 (controller), SDF 1.9 (model + world), Gazebo Harmonic, ROS 2 Humble. All tests via pytest.

**Reference spec:** `docs/superpowers/specs/2026-05-02-ai-orchestrated-heterogeneous-defense-design.md` §6.2 (Coyote sim values), §15 Phase 2 (task list).

**Phase exit criterion:** `ros2 launch acsdg_bringup acsdg_full.launch.py headless:=false` then 5-publish wave trigger → 1 Coyote (NE post) and 3 Anvils intercept 4 enemies. Coyote demonstrates high-speed long-range engagement (commanded speeds up to 160 m/s, frag-fuze kill at <8 m). All 4 enemies neutralized; 0 breaches.

---

## File Structure

### Files to create

| Path | Responsibility |
|---|---|
| `src/acsdg_c2/acsdg_c2/weapons/coyote.py` | `Coyote(WeaponSystem)` concrete class |
| `src/acsdg_c2/test/test_weapon_coyote.py` | Unit tests for `Coyote` envelope, Pkill table, dispatch payload, frag-fuze geometry |
| `src/acsdg_c2/src/coyote_controller_node.cpp` | High-speed pursuit + proximity-fuze controller |
| `src/acsdg_gazebo/models/coyote_b2/model.sdf` | Jet-form SDF with VelocityControl + OdometryPublisher plugins |
| `src/acsdg_gazebo/models/coyote_b2/model.config` | Standard Gazebo model manifest |

### Files to modify

| Path | What changes |
|---|---|
| `src/acsdg_c2/acsdg_c2/weapons/__init__.py` | Re-export `Coyote` |
| `src/acsdg_c2/acsdg_c2/c2_engine_node.py` | Replace one `Anvil` with one `Coyote` in the weapon registry |
| `src/acsdg_c2/CMakeLists.txt` | Build + install `coyote_controller_node` |
| `src/acsdg_c2/launch/c2.launch.py` | Spawn `coyote_controller_node` for slot 1 instead of `interceptor_controller_node` |
| `src/acsdg_gazebo/scripts/gz_bridge_shim.py` | Wire `coyote_1` cmd_vel + odometry like the existing interceptor/enemy loops |
| `src/acsdg_gazebo/worlds/military_base.sdf` | Replace `interceptor_1` `<include>` with `coyote_1` |

### Files NOT touched

- `src/acsdg_c2/acsdg_c2/weapons/{base,types,anvil}.py` — ABC and Anvil unchanged.
- `src/acsdg_c2/acsdg_c2/dispatcher/dispatcher.py` — `coyote_0` numeric tail (0) + 1 = `interceptor_id=1`, same as `anvil_0` previously occupied. No code change needed.
- `src/acsdg_c2/acsdg_c2/cost_function/`, `assignment/`, `classifier/` — Phase 1 modules sufficient for Phase 2.
- `src/acsdg_c2/acsdg_c2/interceptor_manager_node.py` — Coyote controller publishes the same `InterceptorState` topic; manager doesn't care about the underlying weapon class.
- All sensor / fusion / radar code.

---

## Pre-Task: Verify Phase 1 baseline still runs

Phase 1 closed at commit `ec8e99f2`. Quick sanity pass before adding the new weapon.

- [ ] **Step 1: Verify clean tree, 48 tests pass**

```bash
wsl -d Ubuntu-22.04 -u mal -e bash -c "source /opt/ros/humble/setup.bash && cd /home/mal/acsdg_ws && git status && git log --oneline -3 && colcon build --symlink-install --packages-select acsdg_c2 acsdg_gazebo 2>&1 | tail -3 && source install/setup.bash && cd src/acsdg_c2 && python3 -m pytest test/ 2>&1 | tail -3"
```

Expected: working tree clean, HEAD is `ec8e99f2`, build succeeds, 48 tests pass.

- [ ] **Step 2: Note progress**

```bash
wsl -d Ubuntu-22.04 -u mal -e bash -c "echo 'Phase 2 starting — Phase 1 baseline verified at ec8e99f2 (48 tests pass)'"
```

---

## Task 1: `Coyote` weapon class

The concrete `WeaponSystem` for the Coyote Block 2. Mirrors `Anvil`'s shape (engagement-state hooks, dispatch payload, etc.) but with Coyote's Pkill table, envelope, and resource cost from spec §6.2.

**Files:**
- Create: `src/acsdg_c2/acsdg_c2/weapons/coyote.py`
- Create: `src/acsdg_c2/test/test_weapon_coyote.py`
- Modify: `src/acsdg_c2/acsdg_c2/weapons/__init__.py` (add `Coyote` re-export)

- [ ] **Step 1: Write the failing test**

Create `src/acsdg_c2/test/test_weapon_coyote.py`:

```python
"""Tests for Coyote — the Phase-2 frag-jet weapon system."""

import pytest

from acsdg_c2.weapons.types import TargetClass, Track
from acsdg_c2.weapons.coyote import Coyote


def make_coyote(weapon_id: str = "coyote_0",
                home: tuple = (177.0, 177.0, 20.0)) -> Coyote:
    return Coyote(weapon_id=weapon_id, home_position=home)


def make_track(track_id: int = 1, pos: tuple = (300.0, 0.0, 50.0),
               vel: tuple = (-5.0, 0.0, 0.0)) -> Track:
    return Track(track_id=track_id, position=pos, velocity=vel,
                 threat_score=0.7, state="DETECTED")


def test_coyote_weapon_type_is_frag_jet():
    c = make_coyote()
    assert c.weapon_type == "frag_jet"


def test_coyote_pkill_table_matches_spec():
    c = make_coyote()
    # spec §6.2 — frag wasted on tiny targets, optimal on Group 3 / Shahed
    assert c.pkill(TargetClass.SMALL_QUAD) == pytest.approx(0.60)
    assert c.pkill(TargetClass.GROUP_1_FIXED_WING) == pytest.approx(0.85)
    assert c.pkill(TargetClass.GROUP_3_LOITERING) == pytest.approx(0.95)
    assert c.pkill(TargetClass.SHAHED_CLASS) == pytest.approx(0.95)


def test_coyote_engagement_envelope_matches_spec():
    c = make_coyote()
    env = c.engagement_envelope()
    assert env.max_range == pytest.approx(5000.0)   # 5 km in sim, spec §6.2
    assert env.max_closing_speed == pytest.approx(160.0)


def test_coyote_can_engage_far_target_within_envelope():
    c = make_coyote(home=(177.0, 177.0, 20.0))
    track = make_track(pos=(177.0, -2000.0, 50.0))   # ~2.2 km away — fine for Coyote
    assert c.can_engage(track) is True


def test_coyote_cannot_engage_out_of_range():
    c = make_coyote(home=(177.0, 177.0, 20.0))
    track = make_track(pos=(7000.0, 0.0, 50.0))   # >5 km
    assert c.can_engage(track) is False


def test_coyote_cannot_engage_when_unavailable():
    c = make_coyote()
    c.mark_engaged(target_id=42)
    track = make_track(pos=(500.0, 0.0, 50.0))
    assert c.is_available() is False
    assert c.can_engage(track) is False


def test_coyote_dispatch_records_engagement_and_returns_payload():
    c = make_coyote()
    track = make_track(track_id=99, pos=(500.0, 0.0, 50.0))
    payload = c.dispatch(track)
    assert payload["target_id"] == 99
    assert payload["weapon_id"] == "coyote_0"
    assert c.is_available() is False
    assert c.engaged_target_id() == 99


def test_coyote_engaged_target_id_resets_on_mark_idle():
    c = make_coyote()
    c.mark_engaged(target_id=7)
    assert c.engaged_target_id() == 7
    c.mark_idle()
    assert c.engaged_target_id() is None


def test_coyote_resource_cost_matches_spec():
    c = make_coyote()
    assert c.resource_cost() == pytest.approx(0.10)


def test_coyote_time_to_intercept_uses_max_speed_and_closing_rate():
    c = make_coyote(home=(0.0, 0.0, 0.0))
    # Target 1600 m away, closing at 0 along the line of sight.
    # eff_speed = max_speed (160) + 0 = 160; ToI = 1600 / 160 = 10 s.
    track = make_track(pos=(1600.0, 0.0, 0.0), vel=(0.0, 0.0, 0.0))
    assert c.time_to_intercept(track) == pytest.approx(10.0, rel=0.01)


def test_coyote_state_marks_one_shot():
    c = make_coyote()
    s = c.state()
    assert s.weapon_id == "coyote_0"
    assert s.weapon_type == "frag_jet"
    assert s.ammo_remaining is None   # one-shot, like Anvil
```

- [ ] **Step 2: Run the test — must fail with import error**

```bash
wsl -d Ubuntu-22.04 -u mal -e bash -c "source /opt/ros/humble/setup.bash && cd /home/mal/acsdg_ws && colcon build --symlink-install --packages-select acsdg_c2 && source install/setup.bash && cd src/acsdg_c2 && python3 -m pytest test/test_weapon_coyote.py -v"
```

Expected: import error — `acsdg_c2.weapons.coyote` doesn't exist yet.

- [ ] **Step 3: Implement the Coyote class**

Create `src/acsdg_c2/acsdg_c2/weapons/coyote.py`:

```python
"""Raytheon Coyote Block 2 — frag-warhead jet interceptor.

Spec values from §6.2 of the design doc:
  max_speed = 160 m/s (Mach 0.45), max_range = 5 km in sim,
  resource_cost = 0.10 ($100k FY24 unit), one-shot,
  proximity-fuze frag warhead with Pk≥0.5 at 5 m, Pk≥0.3 at 8 m.
Pkill table:
  small-quad 0.60, group-1-fixed-wing 0.85, group-3-loitering 0.95, shahed 0.95.

Phase-2 contract:
- Lives in the same weapon registry as Anvil (composes through WeaponSystem ABC).
- weapon_id "coyote_<N>" maps to interceptor_id (N+1) via Dispatcher's existing
  numeric-tail logic. Coyote takes the Anvil-vacated NE slot (slot 1), so
  weapon_id "coyote_0" → interceptor_id 1.
"""

from __future__ import annotations

from typing import Any, Dict, Optional, Tuple

from acsdg_c2.weapons.base import WeaponSystem
from acsdg_c2.weapons.types import (
    TargetClass,
    EngagementEnvelope,
    Track,
    WeaponState,
)


_COYOTE_PKILL = {
    TargetClass.SMALL_QUAD: 0.60,
    TargetClass.GROUP_1_FIXED_WING: 0.85,
    TargetClass.GROUP_3_LOITERING: 0.95,
    TargetClass.SHAHED_CLASS: 0.95,
}

_COYOTE_MAX_SPEED = 160.0      # m/s — spec §6.2 (Mach 0.45)
_COYOTE_MAX_RANGE = 5000.0     # m — Phase-2 sim value (real 10–15 km), spec §6.2
_COYOTE_MIN_RANGE = 100.0      # m — proximity fuze arming distance, spec §6.2
_COYOTE_RESOURCE_COST = 0.10   # $100k normalized — spec §6.2


class Coyote(WeaponSystem):

    weapon_type = "frag_jet"

    def __init__(
        self,
        weapon_id: str,
        home_position: Tuple[float, float, float],
    ) -> None:
        self.weapon_id = weapon_id
        self._home = home_position
        self._current_position = home_position
        self._engaged_target_id: Optional[int] = None

    # ── External state hooks (called by c2_engine on InterceptorState) ──

    def update_position(self, position: Tuple[float, float, float]) -> None:
        self._current_position = position

    def mark_engaged(self, target_id: int) -> None:
        self._engaged_target_id = target_id

    def mark_idle(self) -> None:
        self._engaged_target_id = None

    def engaged_target_id(self) -> Optional[int]:
        return self._engaged_target_id

    # ── WeaponSystem interface ──────────────────────────────────────────

    def is_available(self) -> bool:
        return self._engaged_target_id is None

    def engagement_envelope(self) -> EngagementEnvelope:
        return EngagementEnvelope(
            min_range=_COYOTE_MIN_RANGE,
            max_range=_COYOTE_MAX_RANGE,
            min_alt=0.0,
            max_alt=4500.0,        # spec §6.2 sim altitude ceiling
            max_closing_speed=_COYOTE_MAX_SPEED,
        )

    def pkill(self, target_class: TargetClass) -> float:
        return _COYOTE_PKILL[target_class]

    def time_to_intercept(self, track: Track) -> float:
        d = track.range_from(*self._current_position)
        if d <= 0.0:
            return 0.0
        ux = (track.position[0] - self._current_position[0]) / d
        uy = (track.position[1] - self._current_position[1]) / d
        uz = (track.position[2] - self._current_position[2]) / d
        v_proj = (track.velocity[0] * (-ux)
                  + track.velocity[1] * (-uy)
                  + track.velocity[2] * (-uz))
        eff_speed = max(1.0, _COYOTE_MAX_SPEED + v_proj)
        return d / eff_speed

    def resource_cost(self) -> float:
        return _COYOTE_RESOURCE_COST

    def can_engage(self, track: Track) -> bool:
        if not self.is_available():
            return False
        d = track.range_from(*self._current_position)
        return self.engagement_envelope().contains(
            range_m=d, alt_m=track.position[2])

    def dispatch(self, track: Track) -> Dict[str, Any]:
        self.mark_engaged(track.track_id)
        # Truncating int(...) matches Anvil's priority byte exactly so a
        # heterogeneous wave produces consistent EngagementOrder.priority bytes.
        return {
            "target_id": track.track_id,
            "weapon_id": self.weapon_id,
            "priority": int(min(255, max(0, int(track.threat_score * 255)))),
        }

    def state(self) -> WeaponState:
        return WeaponState(
            weapon_id=self.weapon_id,
            weapon_type=self.weapon_type,
            position=self._current_position,
            available=self.is_available(),
            ammo_remaining=None,    # Coyote is one-shot, like Anvil
        )
```

- [ ] **Step 4: Re-export from package init**

Modify `src/acsdg_c2/acsdg_c2/weapons/__init__.py` — replace its current contents with:

```python
"""Weapon abstractions for the ACSDG C2 layer."""

from acsdg_c2.weapons.types import (
    TargetClass,
    EngagementEnvelope,
    Track,
    WeaponState,
)
from acsdg_c2.weapons.base import WeaponSystem
from acsdg_c2.weapons.anvil import Anvil
from acsdg_c2.weapons.coyote import Coyote

__all__ = [
    "TargetClass", "EngagementEnvelope", "Track", "WeaponState",
    "WeaponSystem", "Anvil", "Coyote",
]
```

- [ ] **Step 5: Run the tests — 11 must pass**

```bash
wsl -d Ubuntu-22.04 -u mal -e bash -c "source /opt/ros/humble/setup.bash && cd /home/mal/acsdg_ws && colcon build --symlink-install --packages-select acsdg_c2 && source install/setup.bash && cd src/acsdg_c2 && python3 -m pytest test/test_weapon_coyote.py -v"
```

Expected: 11 tests pass.

- [ ] **Step 6: Run full suite to verify no regressions**

```bash
wsl -d Ubuntu-22.04 -u mal -e bash -c "source /opt/ros/humble/setup.bash && cd /home/mal/acsdg_ws && source install/setup.bash && cd src/acsdg_c2 && python3 -m pytest test/ 2>&1 | tail -3"
```

Expected: 59 tests pass (48 prior + 11 new).

- [ ] **Step 7: Commit**

```bash
wsl -d Ubuntu-22.04 -u mal -e bash -c "cd /home/mal/acsdg_ws && git add src/acsdg_c2/acsdg_c2/weapons/coyote.py src/acsdg_c2/acsdg_c2/weapons/__init__.py src/acsdg_c2/test/test_weapon_coyote.py && git commit -m 'Phase 2 Task 1: Coyote weapon class' -m 'Adds the Raytheon Coyote Block 2 frag-warhead jet interceptor as the second concrete WeaponSystem implementation. Pkill table per spec §6.2 (small-quad 0.60, group-1 0.85, group-3 0.95, shahed 0.95), max_speed 160 m/s, max_range 5 km in sim, one-shot, resource_cost 0.10. 11 unit tests passing; 59 total in suite.' -m 'Co-Authored-By: Claude Opus 4.7 (1M context) <noreply@anthropic.com>'"
```

---

## Task 2: Coyote SDF model

A jet-form rigid body with the same kinematic-velocity-controlled architecture as the Anvil drone (gravity off, VelocityControl plugin, 3D OdometryPublisher, high angular damping). Visually a simple cylinder + cone nose (no rotors); functionally a flying point mass commanded at up to 160 m/s.

**Files:**
- Create: `src/acsdg_gazebo/models/coyote_b2/model.sdf`
- Create: `src/acsdg_gazebo/models/coyote_b2/model.config`

- [ ] **Step 1: Create `model.config`**

Create `src/acsdg_gazebo/models/coyote_b2/model.config`:

```xml
<?xml version="1.0"?>
<model>
  <name>coyote_b2</name>
  <version>1.0</version>
  <sdf version="1.9">model.sdf</sdf>
  <description>
    Raytheon Coyote Block 2 — turbojet-sustained frag-warhead C-UAS interceptor.
    Phase-2 sim approximation: kinematic velocity-controlled rigid body, no
    rotors (jet sustainer), 5 km max engagement range, 160 m/s sustained.
  </description>
</model>
```

- [ ] **Step 2: Create `model.sdf`**

Create `src/acsdg_gazebo/models/coyote_b2/model.sdf`:

```xml
<?xml version="1.0" encoding="UTF-8"?>
<sdf version="1.9">
  <model name="coyote_b2">
    <pose>0 0 0 0 0 0</pose>

    <link name="base_link">
      <!-- Kinematic velocity-controlled body. Gravity off and high angular
           damping mirror the Anvil model — both prevent the velocity-control
           plugin from drifting between command ticks and prevent the body
           from picking up rotational momentum from numerical noise. -->
      <gravity>false</gravity>
      <velocity_decay>
        <linear>0</linear>
        <angular>10</angular>
      </velocity_decay>
      <inertial>
        <mass>7.0</mass>
        <!-- Long-cylinder approximation, long axis along +x (matches the
             1.0 × 0.15 × 0.15 collision box and the rotated visual cylinder):
               ixx ≈ m*r²/2 = 7*0.005625/2 ≈ 0.020      (axial — about long axis)
               iyy = izz ≈ m*(3*r² + L²)/12             (transverse)
                        = 7*(3*0.005625 + 1.0)/12 ≈ 0.59 -->
        <inertia>
          <ixx>0.020</ixx><ixy>0</ixy><ixz>0</ixz>
          <iyy>0.59</iyy><iyz>0</iyz>
          <izz>0.59</izz>
        </inertia>
      </inertial>

      <collision name="body_collision">
        <geometry><box><size>1.0 0.15 0.15</size></box></geometry>
      </collision>

      <!-- Main fuselage (steel grey) -->
      <visual name="fuselage">
        <geometry>
          <cylinder><radius>0.075</radius><length>0.85</length></cylinder>
        </geometry>
        <pose>0 0 0 0 1.5708 0</pose>   <!-- rotate cylinder onto x-axis -->
        <material>
          <ambient>0.45 0.45 0.5 1</ambient>
          <diffuse>0.55 0.55 0.6 1</diffuse>
          <specular>0.7 0.7 0.7 1</specular>
        </material>
      </visual>

      <!-- Nose cone -->
      <visual name="nose_cone">
        <pose>0.475 0 0 0 1.5708 0</pose>
        <geometry>
          <cylinder><radius>0.075</radius><length>0.15</length></cylinder>
        </geometry>
        <material>
          <ambient>0.6 0.4 0.2 1</ambient>
          <diffuse>0.7 0.45 0.25 1</diffuse>
        </material>
      </visual>

      <!-- Cruciform fins (fold-out cruciform — cosmetic, no aero in sim) -->
      <visual name="fin_top">
        <pose>-0.35 0 0.10 0 0 0</pose>
        <geometry><box><size>0.20 0.02 0.10</size></box></geometry>
        <material><ambient>0.25 0.25 0.3 1</ambient><diffuse>0.3 0.3 0.35 1</diffuse></material>
      </visual>
      <visual name="fin_bottom">
        <pose>-0.35 0 -0.10 0 0 0</pose>
        <geometry><box><size>0.20 0.02 0.10</size></box></geometry>
        <material><ambient>0.25 0.25 0.3 1</ambient><diffuse>0.3 0.3 0.35 1</diffuse></material>
      </visual>
      <visual name="fin_left">
        <pose>-0.35 0.10 0 0 0 0</pose>
        <geometry><box><size>0.20 0.10 0.02</size></box></geometry>
        <material><ambient>0.25 0.25 0.3 1</ambient><diffuse>0.3 0.3 0.35 1</diffuse></material>
      </visual>
      <visual name="fin_right">
        <pose>-0.35 -0.10 0 0 0 0</pose>
        <geometry><box><size>0.20 0.10 0.02</size></box></geometry>
        <material><ambient>0.25 0.25 0.3 1</ambient><diffuse>0.3 0.3 0.35 1</diffuse></material>
      </visual>

      <!-- Orange exhaust cap (hot end, points at -x) -->
      <visual name="exhaust">
        <pose>-0.45 0 0 0 1.5708 0</pose>
        <geometry>
          <cylinder><radius>0.06</radius><length>0.08</length></cylinder>
        </geometry>
        <material>
          <ambient>1.0 0.4 0.1 1</ambient><diffuse>1.0 0.5 0.15 1</diffuse>
          <emissive>0.8 0.3 0 1</emissive>
        </material>
      </visual>
    </link>

    <!-- Velocity control (cmd_vel applied in body frame; gz_bridge_shim
         inverse-rotates the world-frame ROS Twist before forwarding). -->
    <plugin filename="gz-sim-velocity-control-system"
      name="gz::sim::systems::VelocityControl">
    </plugin>

    <!-- Odometry publisher (3D — required so cmd_vel rotation can read pose.z). -->
    <plugin filename="gz-sim-odometry-publisher-system"
      name="gz::sim::systems::OdometryPublisher">
      <odom_frame>world</odom_frame>
      <robot_base_frame>coyote_b2</robot_base_frame>
      <odom_publish_frequency>50</odom_publish_frequency>
      <dimensions>3</dimensions>
    </plugin>

  </model>
</sdf>
```

- [ ] **Step 3: Build to install model into the resource path**

```bash
wsl -d Ubuntu-22.04 -u mal -e bash -c "cd /home/mal/acsdg_ws && colcon build --symlink-install --packages-select acsdg_gazebo 2>&1 | tail -3"
```

Expected: build succeeds.

- [ ] **Step 4: Verify model installs and parses**

```bash
wsl -d Ubuntu-22.04 -u mal -e bash -c "ls /home/mal/acsdg_ws/install/acsdg_gazebo/share/acsdg_gazebo/models/coyote_b2/ && cat /home/mal/acsdg_ws/install/acsdg_gazebo/share/acsdg_gazebo/models/coyote_b2/model.config"
```

Expected: `model.config` and `model.sdf` listed; the `.config` content matches what was written.

- [ ] **Step 5: Commit**

```bash
wsl -d Ubuntu-22.04 -u mal -e bash -c "cd /home/mal/acsdg_ws && git add src/acsdg_gazebo/models/coyote_b2/ && git commit -m 'Phase 2 Task 2: Coyote Block 2 SDF model' -m 'Jet-form kinematic rigid body for the Coyote Block 2 interceptor. Cylindrical fuselage + nose cone + cruciform fins. Mass 7 kg per spec §6.2. VelocityControl plugin (cmd_vel applied in body frame, gz_bridge_shim inverse-rotates) and 3D OdometryPublisher mirror the Anvil drone setup. No rotor links — Coyote is jet-sustained.' -m 'Co-Authored-By: Claude Opus 4.7 (1M context) <noreply@anthropic.com>'"
```

---

## Task 3: World SDF — replace `interceptor_1` with `coyote_1`

The NE defense post becomes a Coyote launch site.

**Files:**
- Modify: `src/acsdg_gazebo/worlds/military_base.sdf` (lines around 522–528)

- [ ] **Step 1: Read the existing interceptor_1 include for context**

```bash
wsl -d Ubuntu-22.04 -u mal -e bash -c "grep -n -B 1 -A 4 'interceptor_1' /home/mal/acsdg_ws/src/acsdg_gazebo/worlds/military_base.sdf | head -10"
```

Expected: shows the `<!-- Interceptor 1 at defense post NE -->` block with the bumped z=20 spawn pose.

- [ ] **Step 2: Replace the include**

In `src/acsdg_gazebo/worlds/military_base.sdf`, replace:

```xml
    <!-- Interceptor 1 at defense post NE
         Spawn z bumped from 5 to 20 to match interceptor_controller_node's
         home_z parameter and to clear ground-collision impulses that were
         tipping the bodies on startup. -->
    <include>
      <uri>model://interceptor_drone</uri>
      <name>interceptor_1</name>
      <pose>177 177 20 0 0 0</pose>
    </include>
```

With:

```xml
    <!-- Coyote 1 at defense post NE — replaces interceptor_1 in Phase 2.
         Same NE post (177, 177, 20) so the C2 weapon registry's slot-0
         entry takes the same spatial position; only the airframe class
         changes. The post becomes the Coyote's tube-launch site. -->
    <include>
      <uri>model://coyote_b2</uri>
      <name>coyote_1</name>
      <pose>177 177 20 0 0 0</pose>
    </include>
```

- [ ] **Step 3: Build and verify world SDF parses**

```bash
wsl -d Ubuntu-22.04 -u mal -e bash -c "cd /home/mal/acsdg_ws && colcon build --symlink-install --packages-select acsdg_gazebo 2>&1 | tail -3"
```

Expected: build succeeds.

- [ ] **Step 4: Commit**

```bash
wsl -d Ubuntu-22.04 -u mal -e bash -c "cd /home/mal/acsdg_ws && git add src/acsdg_gazebo/worlds/military_base.sdf && git commit -m 'Phase 2 Task 3: world spawns coyote_1 at NE post' -m 'Replaces the interceptor_1 model include with coyote_1. Same spatial position (177, 177, 20); only the airframe class changes. C2 weapon registry slot 0 (NE corner) now becomes a Coyote in Task 7.' -m 'Co-Authored-By: Claude Opus 4.7 (1M context) <noreply@anthropic.com>'"
```

---

## Task 4: Bridge shim — wire `coyote_1` cmd_vel + odometry

The bridge currently iterates `enemy_1..4` and `interceptor_1..4`. It needs to also wire `coyote_1`. Generalize the wiring to a list so adding more weapon types in Phase 3 / 4 is a one-line addition.

**Files:**
- Modify: `src/acsdg_gazebo/scripts/gz_bridge_shim.py`

- [ ] **Step 1: Open the bridge and locate the wiring loop**

The current bridge has, around the `__init__`:

```python
ENEMY_COUNT       = 4
INTERCEPTOR_COUNT = 4
```

and later in `__init__`:

```python
        self._wire_twist_ros_to_gz('enemy',       ENEMY_COUNT)
        self._wire_twist_ros_to_gz('interceptor', INTERCEPTOR_COUNT)
        self._wire_odom_gz_to_ros('enemy',        ENEMY_COUNT)
        self._wire_odom_gz_to_ros('interceptor',  INTERCEPTOR_COUNT)
```

The plan is to replace the four hardcoded lines with a single declarative table.

- [ ] **Step 2: Modify the constants**

In `src/acsdg_gazebo/scripts/gz_bridge_shim.py`, replace:

```python
ENEMY_COUNT       = 4
INTERCEPTOR_COUNT = 4
```

With:

```python
# Models bridged on cmd_vel + odometry, as (kind, count) tuples. The bridge
# wires `/{kind}_{1..count}/cmd_vel` (ROS) ↔ `/model/{kind}_{i}/cmd_vel` (gz)
# for every kind. Phase 1 had only enemy + interceptor; Phase 2 adds the
# Coyote at slot 1 alongside the remaining 3 Anvils (so interceptor_count
# is now 3, not 4 — interceptor_1 is gone, replaced by coyote_1).
_BRIDGED_MODELS = (
    ('enemy',       4),    # enemy_1..4 still all 4 Anvils' targets
    ('interceptor', 4),    # interceptor_1..4 — interceptor_1 is no longer
                           # spawned by the world (replaced by coyote_1),
                           # but we keep the topic wiring in case a stray
                           # cmd_vel publisher targets unit 1; bridge to
                           # a non-existent gz model is harmless (no-op).
    ('coyote',      1),    # coyote_1 — new in Phase 2
)
```

- [ ] **Step 3: Modify the `__init__` call sites**

Replace:

```python
        # Per-drone bindings for both enemies and interceptors
        self._wire_twist_ros_to_gz('enemy',       ENEMY_COUNT)
        self._wire_twist_ros_to_gz('interceptor', INTERCEPTOR_COUNT)
        self._wire_odom_gz_to_ros('enemy',        ENEMY_COUNT)
        self._wire_odom_gz_to_ros('interceptor',  INTERCEPTOR_COUNT)

        self.get_logger().info(
            f'BridgeShim ready — {ENEMY_COUNT} enemies + {INTERCEPTOR_COUNT} '
            f'interceptors, both cmd_vel and odometry bridged.')
```

With:

```python
        # Per-drone bindings driven by the _BRIDGED_MODELS table.
        for kind, count in _BRIDGED_MODELS:
            self._wire_twist_ros_to_gz(kind, count)
            self._wire_odom_gz_to_ros(kind, count)

        self.get_logger().info(
            'BridgeShim ready — bridged: '
            + ', '.join(f'{count} × {kind}'
                        for kind, count in _BRIDGED_MODELS))
```

- [ ] **Step 4: Build**

```bash
wsl -d Ubuntu-22.04 -u mal -e bash -c "cd /home/mal/acsdg_ws && colcon build --symlink-install --packages-select acsdg_gazebo 2>&1 | tail -3"
```

Expected: build succeeds.

- [ ] **Step 5: Sanity-check the bridge can still import**

```bash
wsl -d Ubuntu-22.04 -u mal -e bash -c "source /opt/ros/humble/setup.bash && python3 -c \"
import importlib.util, os
spec = importlib.util.spec_from_file_location(
    'gz_bridge_shim',
    '/home/mal/acsdg_ws/src/acsdg_gazebo/scripts/gz_bridge_shim.py')
m = importlib.util.module_from_spec(spec)
spec.loader.exec_module(m)
print('IMPORT_OK; _BRIDGED_MODELS =', m._BRIDGED_MODELS)
\""
```

Expected: `IMPORT_OK; _BRIDGED_MODELS = (('enemy', 4), ('interceptor', 4), ('coyote', 1))`. (If this shells out to `rclpy.init()` because of node init in the import, the test fails — in that case skip this step; the live launch in Task 8 covers it.)

- [ ] **Step 6: Commit**

```bash
wsl -d Ubuntu-22.04 -u mal -e bash -c "cd /home/mal/acsdg_ws && git add src/acsdg_gazebo/scripts/gz_bridge_shim.py && git commit -m 'Phase 2 Task 4: bridge shim wires coyote_1 alongside enemies and interceptors' -m 'Replaces hardcoded ENEMY_COUNT / INTERCEPTOR_COUNT pair with a _BRIDGED_MODELS tuple of (kind, count) pairs. Adds coyote with count=1. Phase 3 (DroneHunter) and Phase 4 (Skyranger) become one-line additions.' -m 'Co-Authored-By: Claude Opus 4.7 (1M context) <noreply@anthropic.com>'"
```

---

## Task 5: Coyote C++ controller node

A high-speed pursuit controller modeled on `interceptor_controller_node.cpp` but tuned for the Coyote: 160 m/s top speed, 5 m kill radius (frag fuze p≥0.5), proximity-fuze arming check, no altitude clamp (Coyote can fly higher than the cruise-50m Anvil).

**Files:**
- Create: `src/acsdg_c2/src/coyote_controller_node.cpp`
- Modify: `src/acsdg_c2/CMakeLists.txt` (add `coyote_controller_node` target)

- [ ] **Step 1: Create the controller source file**

Create `src/acsdg_c2/src/coyote_controller_node.cpp`:

```cpp
//============================================================================
// coyote_controller_node.cpp — Per-Coyote flight controller.
//
// Phase-2 controller for the Raytheon Coyote Block 2 frag-warhead jet
// interceptor. Modelled on interceptor_controller_node.cpp but with
// Coyote-specific constants (max speed 160 m/s vs Anvil 15 m/s, frag-fuze
// kill radius 5 m vs Anvil 8 m collision, no fixed cruise altitude).
//
// Publishes
// ---------
//   /coyote_{id}/cmd_vel         geometry_msgs/Twist
//   /interceptors/unit_{id}/state  acsdg_msgs/InterceptorState
//   /interceptors/unit_{id}/position geometry_msgs/Point
//   /mission/engagement_ack      std_msgs/String
//
// Subscribes
// ----------
//   /c2/engagement_orders        acsdg_msgs/EngagementOrder
//   /threats/target_{n}/fused_target acsdg_msgs/FusedTarget (dynamic)
//   /model/coyote_{id}/odometry  nav_msgs/Odometry
//============================================================================

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <std_msgs/msg/string.hpp>
#include <acsdg_msgs/msg/engagement_order.hpp>
#include <acsdg_msgs/msg/fused_target.hpp>

#include <chrono>
#include <cmath>
#include <memory>
#include <sstream>
#include <string>

using namespace std::chrono_literals;

class CoyoteControllerNode : public rclcpp::Node
{
  // ── Coyote spec values (spec §6.2) ──────────────────────────────────
  static constexpr double kMaxSpeed     = 160.0;  // m/s — Mach 0.45 sustained
  static constexpr double kKillRadius   = 5.0;    // m — frag p≥0.5 (spec §6.2)
  static constexpr double kArmDistance  = 100.0;  // m — proximity fuze arms here
  static constexpr int    kLostTicks    = 40;     // 40 × 50 ms = 2 s
  static constexpr double kDt           = 0.05;   // 20 Hz
  // No fixed cruise altitude — Coyote pursues in 3D directly. Anvil's 2D-
  // pursuit + altitude-hold pattern was needed because radar-noisy tgt_vz
  // extrapolated over 10 s+ t_go was unstable; Coyote's 160 m/s closes
  // engagements in 5–10 s, so the noise window is too short to diverge.

public:
  CoyoteControllerNode() : rclcpp::Node("coyote_controller_node")
  {
    declare_parameter("interceptor_id", 1);
    declare_parameter("home_x",  177.0);
    declare_parameter("home_y",  177.0);
    declare_parameter("home_z",   20.0);

    id_     = get_parameter("interceptor_id").as_int();
    home_x_ = get_parameter("home_x").as_double();
    home_y_ = get_parameter("home_y").as_double();
    home_z_ = get_parameter("home_z").as_double();

    pos_x_ = home_x_;  pos_y_ = home_y_;  pos_z_ = home_z_;

    // ── Publishers ────────────────────────────────────────────────────
    std::string cmd_topic = "/coyote_" + std::to_string(id_) + "/cmd_vel";
    cmd_vel_pub_ = create_publisher<geometry_msgs::msg::Twist>(cmd_topic, 10);

    std::string pos_topic = "/interceptors/unit_" + std::to_string(id_) + "/position";
    position_pub_ = create_publisher<geometry_msgs::msg::Point>(pos_topic, 10);

    ack_pub_ = create_publisher<std_msgs::msg::String>("/mission/engagement_ack", 10);

    // ── Subscriptions ─────────────────────────────────────────────────
    order_sub_ = create_subscription<acsdg_msgs::msg::EngagementOrder>(
      "/c2/engagement_orders", 10,
      [this](acsdg_msgs::msg::EngagementOrder::SharedPtr msg) { onOrder(msg); });

    std::string odom_topic = "/model/coyote_" + std::to_string(id_) + "/odometry";
    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      odom_topic, rclcpp::QoS(10),
      [this](nav_msgs::msg::Odometry::SharedPtr m) {
        pos_x_ = m->pose.pose.position.x;
        pos_y_ = m->pose.pose.position.y;
        pos_z_ = m->pose.pose.position.z;
        has_odom_ = true;
      });

    // ── 20 Hz control loop ────────────────────────────────────────────
    timer_ = create_wall_timer(50ms, [this]() { controlStep(); });

    RCLCPP_INFO(get_logger(),
      "CoyoteController #%d  home=(%.0f, %.0f, %.0f)  max_speed=%.0fm/s",
      id_, home_x_, home_y_, home_z_, kMaxSpeed);
  }

private:
  void onOrder(const acsdg_msgs::msg::EngagementOrder::SharedPtr msg)
  {
    if (static_cast<int>(msg->interceptor_id) != id_) return;
    if (pursuing_ && static_cast<uint32_t>(target_id_) == msg->target_id) return;

    target_id_       = static_cast<int>(msg->target_id);
    pursuing_        = true;
    lost_ticks_      = 0;
    has_target_data_ = false;

    target_sub_.reset();
    std::string topic =
      "/threats/target_" + std::to_string(target_id_) + "/fused_target";
    target_sub_ = create_subscription<acsdg_msgs::msg::FusedTarget>(
      topic, rclcpp::QoS(10),
      [this](acsdg_msgs::msg::FusedTarget::SharedPtr m) { onTarget(m); });

    RCLCPP_INFO(get_logger(),
      "Coyote #%d assigned to target #%d", id_, target_id_);
  }

  void onTarget(const acsdg_msgs::msg::FusedTarget::SharedPtr msg)
  {
    tgt_x_ = msg->position.x;
    tgt_y_ = msg->position.y;
    tgt_z_ = msg->position.z;
    tgt_vx_ = msg->velocity.x;
    tgt_vy_ = msg->velocity.y;
    tgt_vz_ = msg->velocity.z;
    lost_ticks_      = 0;
    has_target_data_ = true;
  }

  void controlStep()
  {
    if (!has_odom_) {
      publishCmdVel(0.0, 0.0, 0.0);
      return;
    }

    if (pursuing_) {
      ++lost_ticks_;
      if (lost_ticks_ > kLostTicks) {
        RCLCPP_INFO(get_logger(),
          "Coyote #%d: target #%d lost — returning home", id_, target_id_);
        publishAck(target_id_, "LOST");
        pursuing_        = false;
        has_target_data_ = false;
        target_sub_.reset();
      } else if (has_target_data_) {
        pursueTarget();
      } else {
        publishCmdVel(0.0, 0.0, 0.0);
      }
    } else {
      stepTowardHome();
    }

    publishPosition();
  }

  void pursueTarget()
  {
    const double rx = tgt_x_ - pos_x_;
    const double ry = tgt_y_ - pos_y_;
    const double rz = tgt_z_ - pos_z_;
    const double range = std::sqrt(rx*rx + ry*ry + rz*rz);

    if (range < kKillRadius) {
      // Frag-fuze proximity kill — Coyote detonates within 5 m.
      RCLCPP_INFO(get_logger(),
        "Coyote #%d: NEUTRALISED target #%d at range=%.2fm "
        "[FRAG-FUZE] coy(%.1f,%.1f,%.1f) tgt(%.1f,%.1f,%.1f)",
        id_, target_id_, range,
        pos_x_, pos_y_, pos_z_, tgt_x_, tgt_y_, tgt_z_);
      publishAck(target_id_, "NEUTRALIZED");
      pursuing_        = false;
      has_target_data_ = false;
      target_sub_.reset();
      publishCmdVel(0.0, 0.0, 0.0);
      return;
    }

    // 3D proportional pursuit toward predicted intercept point.
    const double t_go   = range / kMaxSpeed;
    const double lead_x = tgt_x_ + tgt_vx_ * t_go;
    const double lead_y = tgt_y_ + tgt_vy_ * t_go;
    const double lead_z = tgt_z_ + tgt_vz_ * t_go;

    const double dx   = lead_x - pos_x_;
    const double dy   = lead_y - pos_y_;
    const double dz   = lead_z - pos_z_;
    const double dmag = std::sqrt(dx*dx + dy*dy + dz*dz);
    const double s    = (dmag > 1e-6) ? (kMaxSpeed / dmag) : 0.0;

    publishCmdVel(dx * s, dy * s, dz * s);
  }

  void stepTowardHome()
  {
    const double dx = home_x_ - pos_x_;
    const double dy = home_y_ - pos_y_;
    const double dz = home_z_ - pos_z_;
    const double dist = std::sqrt(dx*dx + dy*dy + dz*dz);
    if (dist < 0.5) {
      publishCmdVel(0.0, 0.0, 0.0);
      return;
    }
    const double kReturnSpeed = 5.0;
    publishCmdVel((dx / dist) * kReturnSpeed,
                  (dy / dist) * kReturnSpeed,
                  (dz / dist) * kReturnSpeed);
  }

  // ── Publishing helpers ──────────────────────────────────────────────

  void publishPosition()
  {
    geometry_msgs::msg::Point pt;
    pt.x = pos_x_;  pt.y = pos_y_;  pt.z = pos_z_;
    position_pub_->publish(pt);
  }

  void publishCmdVel(double vx, double vy, double vz)
  {
    geometry_msgs::msg::Twist t;
    t.linear.x = vx;
    t.linear.y = vy;
    t.linear.z = vz;
    cmd_vel_pub_->publish(t);
  }

  void publishAck(int target_id, const std::string & outcome)
  {
    std::ostringstream oss;
    oss << "{\"target_id\":" << target_id
        << ",\"interceptor_id\":" << id_
        << ",\"outcome\":\"" << outcome << "\"}";
    std_msgs::msg::String msg;
    msg.data = oss.str();
    ack_pub_->publish(msg);
  }

  // ── Members ─────────────────────────────────────────────────────────

  int    id_;
  double home_x_, home_y_, home_z_;
  double pos_x_, pos_y_, pos_z_;
  bool   pursuing_{false};
  int    target_id_{0};
  double tgt_x_{0}, tgt_y_{0}, tgt_z_{0};
  double tgt_vx_{0}, tgt_vy_{0}, tgt_vz_{0};
  int    lost_ticks_{0};
  bool   has_target_data_{false};
  bool   has_odom_{false};

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr        cmd_vel_pub_;
  rclcpp::Publisher<geometry_msgs::msg::Point>::SharedPtr        position_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr            ack_pub_;
  rclcpp::Subscription<acsdg_msgs::msg::EngagementOrder>::SharedPtr order_sub_;
  rclcpp::Subscription<acsdg_msgs::msg::FusedTarget>::SharedPtr     target_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr          odom_sub_;
  rclcpp::TimerBase::SharedPtr                                      timer_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<CoyoteControllerNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
```

- [ ] **Step 2: Add the build target to CMakeLists**

In `src/acsdg_c2/CMakeLists.txt`, after the `interceptor_controller_node` block (after the `install(TARGETS interceptor_controller_node ...)` line), insert:

```cmake
# ── C++ node: coyote_controller_node ─────────────────────────────────────
add_executable(coyote_controller_node src/coyote_controller_node.cpp)
ament_target_dependencies(coyote_controller_node
  rclcpp
  acsdg_msgs
  geometry_msgs
  nav_msgs
  std_msgs
  builtin_interfaces
)

install(TARGETS coyote_controller_node
  DESTINATION lib/${PROJECT_NAME})
```

(Note: Coyote does not depend on `mavros_msgs` — we don't bother with an autopilot mode-set request. Phase 1's `requestOffboard()` was a soft no-op anyway.)

- [ ] **Step 3: Build**

```bash
wsl -d Ubuntu-22.04 -u mal -e bash -c "source /opt/ros/humble/setup.bash && cd /home/mal/acsdg_ws && colcon build --symlink-install --packages-select acsdg_c2 2>&1 | tail -5"
```

Expected: build succeeds; `coyote_controller_node` executable installed at `install/acsdg_c2/lib/acsdg_c2/coyote_controller_node`.

- [ ] **Step 4: Verify the executable was installed**

```bash
wsl -d Ubuntu-22.04 -u mal -e bash -c "ls -la /home/mal/acsdg_ws/install/acsdg_c2/lib/acsdg_c2/ | grep coyote"
```

Expected: `coyote_controller_node` is present and executable.

- [ ] **Step 5: Commit**

```bash
wsl -d Ubuntu-22.04 -u mal -e bash -c "cd /home/mal/acsdg_ws && git add src/acsdg_c2/src/coyote_controller_node.cpp src/acsdg_c2/CMakeLists.txt && git commit -m 'Phase 2 Task 5: Coyote C++ flight controller' -m 'Adds coyote_controller_node, modeled on the Anvil interceptor controller but tuned for Coyote — 160 m/s top speed, 5 m frag-fuze kill radius (spec §6.2 p≥0.5), 3D pursuit instead of 2D-plus-altitude-hold (Coyote engagements close in 5–10 s, too fast for radar-noisy z extrapolation to diverge). Publishes /coyote_{id}/cmd_vel and /interceptors/unit_{id}/state so the existing C2 + interceptor_manager pipeline routes orders to it unchanged.' -m 'Co-Authored-By: Claude Opus 4.7 (1M context) <noreply@anthropic.com>'"
```

---

## Task 6: Launch file — spawn `coyote_controller_node` for slot 1

The Phase-1 `c2.launch.py` spawns 4 `interceptor_controller_node` instances (one per slot 1–4). Phase 2 swaps slot 1 to a Coyote.

**Files:**
- Modify: `src/acsdg_c2/launch/c2.launch.py`

- [ ] **Step 1: Read the current launch**

```bash
wsl -d Ubuntu-22.04 -u mal -e bash -c "cat /home/mal/acsdg_ws/src/acsdg_c2/launch/c2.launch.py"
```

Note the `FLEET` dict and the per-slot `interceptor_controller_node` loop.

- [ ] **Step 2: Replace the file**

Overwrite `src/acsdg_c2/launch/c2.launch.py`:

```python
from launch import LaunchDescription
from launch_ros.actions import Node

# Fleet home positions: four corners of a 400 m × 400 m square, 20 m AGL.
# Phase 2: slot 1 (NE) is a Coyote Block 2; slots 2/3/4 remain Anvil drones.
FLEET: dict = {
    1: ( 200.0,  200.0, 20.0),    # NE — Coyote Block 2 (Phase 2)
    2: (-200.0,  200.0, 20.0),    # NW — Anvil
    3: ( 200.0, -200.0, 20.0),    # SE — Anvil
    4: (-200.0, -200.0, 20.0),    # SW — Anvil
}

# Slot id whose controller is the Coyote (jet, no rotors). All other slots
# spawn the legacy Anvil interceptor_controller_node.
COYOTE_SLOT = 1


def generate_launch_description() -> LaunchDescription:
    nodes = [
        # ── Python nodes ──────────────────────────────────────────────────
        Node(
            package='acsdg_c2',
            executable='c2_engine_node',
            name='c2_engine_node',
            output='screen',
            emulate_tty=True,
        ),
        Node(
            package='acsdg_c2',
            executable='interceptor_manager_node',
            name='interceptor_manager_node',
            output='screen',
            emulate_tty=True,
        ),
        Node(
            package='acsdg_c2',
            executable='mission_manager_node',
            name='mission_manager_node',
            output='screen',
            emulate_tty=True,
            parameters=[{'difficulty': 0.5}],
        ),
    ]

    # ── One C++ controller per slot (Coyote at COYOTE_SLOT, Anvil elsewhere) ─
    for iid, (hx, hy, hz) in FLEET.items():
        executable = ('coyote_controller_node' if iid == COYOTE_SLOT
                      else 'interceptor_controller_node')
        node_name = (f'coyote_controller_{iid}' if iid == COYOTE_SLOT
                     else f'interceptor_controller_{iid}')
        nodes.append(
            Node(
                package='acsdg_c2',
                executable=executable,
                name=node_name,
                output='screen',
                emulate_tty=True,
                parameters=[{
                    'interceptor_id': iid,
                    'home_x': hx,
                    'home_y': hy,
                    'home_z': hz,
                }],
            )
        )

    return LaunchDescription(nodes)
```

- [ ] **Step 3: Build (launch files install via `share/`)**

```bash
wsl -d Ubuntu-22.04 -u mal -e bash -c "cd /home/mal/acsdg_ws && colcon build --symlink-install --packages-select acsdg_c2 2>&1 | tail -3"
```

Expected: build succeeds.

- [ ] **Step 4: Verify the launch description loads**

```bash
wsl -d Ubuntu-22.04 -u mal -e bash -c "source /opt/ros/humble/setup.bash && cd /home/mal/acsdg_ws && source install/setup.bash && python3 -c \"
import sys
sys.path.insert(0, '/home/mal/acsdg_ws/src/acsdg_c2/launch')
import c2 as m
print('FLEET:', m.FLEET)
print('COYOTE_SLOT:', m.COYOTE_SLOT)
ld = m.generate_launch_description()
print('node entities:', len(ld.entities))
\""
```

Expected: prints FLEET dict, `COYOTE_SLOT: 1`, and `node entities: 7` (3 Python C2 nodes + 4 controllers).

- [ ] **Step 5: Commit**

```bash
wsl -d Ubuntu-22.04 -u mal -e bash -c "cd /home/mal/acsdg_ws && git add src/acsdg_c2/launch/c2.launch.py && git commit -m 'Phase 2 Task 6: launch coyote_controller_node at slot 1, anvil controllers at 2-4' -m 'Slot 1 (NE) is now the Coyote launch site, in line with the world SDF coyote_1 spawn (Task 3) and the Coyote weapon registration in c2_engine (Task 7). Slots 2/3/4 keep their interceptor_controller_node Anvils.' -m 'Co-Authored-By: Claude Opus 4.7 (1M context) <noreply@anthropic.com>'"
```

---

## Task 7: C2 engine — register Coyote in slot 0 of the weapon registry

The c2_engine currently builds `[Anvil, Anvil, Anvil, Anvil]`. Replace the slot-0 entry with a `Coyote`. The Coyote takes the Anvil-vacated NE corner; weapon-id naming convention is preserved (`anvil_<N>` for Anvils, `coyote_<N>` for Coyotes), so the Dispatcher's numeric-tail mapping continues to work without changes.

**Files:**
- Modify: `src/acsdg_c2/acsdg_c2/c2_engine_node.py`

- [ ] **Step 1: Locate the weapon-registry construction**

In `src/acsdg_c2/acsdg_c2/c2_engine_node.py`, find the existing block that constructs `self._weapons`:

```python
        # ── Weapons (Phase 1: 4 Anvils) ──────────────────────────────────
        self._weapons: List[Anvil] = [
            Anvil(weapon_id=f"anvil_{i}", home_position=_DEFAULT_HOMES[i])
            for i in range(NUM_INTERCEPTORS)
        ]
```

- [ ] **Step 2: Update the imports**

Find the existing `from acsdg_c2.weapons import Anvil, Track` line. Replace it with:

```python
from acsdg_c2.weapons import Anvil, Coyote, Track, WeaponSystem
```

(The `WeaponSystem` import is needed because `self._weapons` now holds heterogeneous concrete types and the type hint changes from `List[Anvil]` to `List[WeaponSystem]`.)

- [ ] **Step 3: Align `_DEFAULT_HOMES` with the launch FLEET (Phase 1 reviewer I-2)**

Find the existing `_DEFAULT_HOMES` constant near the top of the file:

```python
# Legacy interceptor home positions — controllers know their own homes; the
# C2 engine only needs an initial guess until InterceptorState updates land.
_DEFAULT_HOMES = [
    (-100.0, -100.0, 20.0),
    ( 100.0, -100.0, 20.0),
    ( 100.0,  100.0, 20.0),
    (-100.0,  100.0, 20.0),
]
```

Replace with:

```python
# Slot-indexed initial home positions, kept in sync with c2.launch.py
# FLEET so the cost-matrix's first evaluation (before any InterceptorState
# odometry arrives) uses correct starting positions. Phase 1 reviewer I-2
# flagged the previous (±100) values as a 277 m mismatch with the
# controllers' real home parameters at (±200). Fixed alongside the
# Coyote registration so the Phase-2 cost matrix is correct from tick 0.
_DEFAULT_HOMES = [
    ( 200.0,  200.0, 20.0),    # slot 1 — NE (Coyote in Phase 2, Anvil in Phase 1)
    (-200.0,  200.0, 20.0),    # slot 2 — NW (Anvil)
    ( 200.0, -200.0, 20.0),    # slot 3 — SE (Anvil)
    (-200.0, -200.0, 20.0),    # slot 4 — SW (Anvil)
]
```

- [ ] **Step 4: Replace the weapon-registry construction**

Replace the Phase-1 block in Step 1 with:

```python
        # ── Weapons (Phase 2: 1 Coyote + 3 Anvils) ──────────────────────
        # Slot 0 (NE corner) is a Raytheon Coyote Block 2 frag-jet. Slots 1-3
        # are Anduril Anvil quadcopters. Both classes implement the same
        # WeaponSystem ABC so the dispatch loop is uniform.
        # weapon_id naming: "coyote_0" maps via Dispatcher's numeric-tail
        # logic to interceptor_id 1 (the legacy NE slot). Anvil 1-3 keep
        # their previous mappings (anvil_1 → 2, anvil_2 → 3, anvil_3 → 4).
        self._weapons: List[WeaponSystem] = [
            Coyote(weapon_id="coyote_0", home_position=_DEFAULT_HOMES[0]),
            Anvil(weapon_id="anvil_1",   home_position=_DEFAULT_HOMES[1]),
            Anvil(weapon_id="anvil_2",   home_position=_DEFAULT_HOMES[2]),
            Anvil(weapon_id="anvil_3",   home_position=_DEFAULT_HOMES[3]),
        ]
```

- [ ] **Step 5: Update the ready-log to reflect the new inventory**

Find the line in `__init__` near the timer setup:

```python
        self.get_logger().info(
            f"C2EngineNode ready (10 Hz, {NUM_INTERCEPTORS} Anvils)")
```

Replace with:

```python
        # Build a human-readable inventory summary so the launch log makes
        # it obvious what classes the C2 is composing this run.
        inventory = ", ".join(f"{w.__class__.__name__}({w.weapon_id})"
                              for w in self._weapons)
        self.get_logger().info(
            f"C2EngineNode ready (10 Hz, {len(self._weapons)} weapons: {inventory})")
```

- [ ] **Step 6: Run all unit tests — must pass**

```bash
wsl -d Ubuntu-22.04 -u mal -e bash -c "source /opt/ros/humble/setup.bash && cd /home/mal/acsdg_ws && colcon build --symlink-install --packages-select acsdg_c2 && source install/setup.bash && cd src/acsdg_c2 && python3 -m pytest test/ 2>&1 | tail -3"
```

Expected: 59 tests pass (unchanged from Task 1; the integration test in `test_c2_engine_integration.py` constructs `C2EngineNode` and verifies orders flow — it should still pass with the heterogeneous registry because the Coyote and Anvils share the same dispatch interface).

- [ ] **Step 7: Commit**

```bash
wsl -d Ubuntu-22.04 -u mal -e bash -c "cd /home/mal/acsdg_ws && git add src/acsdg_c2/acsdg_c2/c2_engine_node.py && git commit -m 'Phase 2 Task 7: c2_engine registers Coyote in slot 0, 3 Anvils in slots 1-3' -m 'Replaces the Phase-1 4-Anvil registry with 1 Coyote + 3 Anvils. The list is typed List[WeaponSystem] now to admit the heterogeneous concrete types. Aligns _DEFAULT_HOMES with the launch FLEET (Phase-1 reviewer I-2 fix). Ready-log updated to show inventory composition.' -m 'Co-Authored-By: Claude Opus 4.7 (1M context) <noreply@anthropic.com>'"
```

---

## Task 8: End-to-end regression — live demo with Coyote + 3 Anvils

This is the Phase 2 acceptance test. The demo must produce 4 NEUTRALISED kills (1 Coyote, 3 Anvils) on a single wave, with Coyote logging the FRAG-FUZE kill format.

- [ ] **Step 1: Full clean rebuild**

```bash
wsl -d Ubuntu-22.04 -u mal -e bash -c "source /opt/ros/humble/setup.bash && cd /home/mal/acsdg_ws && colcon build --symlink-install 2>&1 | tail -5"
```

Expected: all 8 packages build clean.

- [ ] **Step 2: Kill any stale sim processes**

```bash
wsl -d Ubuntu-22.04 -u mal -e bash -c "for pid in \$(pgrep -f 'ros2 launch'); do kill -9 \$pid; done; for pid in \$(pgrep -f ruby); do kill -9 \$pid; done; sleep 2; pgrep -af ruby || echo CLEAN"
```

Expected: `CLEAN`.

- [ ] **Step 3: Launch the sim**

In one shell (or background):

```bash
wsl -d Ubuntu-22.04 -u mal -e bash -c "source /opt/ros/humble/setup.bash && cd /home/mal/acsdg_ws && source install/setup.bash && LIBGL_ALWAYS_SOFTWARE=1 ros2 launch acsdg_bringup acsdg_full.launch.py headless:=false"
```

Wait until the launch log shows `C2EngineNode ready (10 Hz, 4 weapons: Coyote(coyote_0), Anvil(anvil_1), Anvil(anvil_2), Anvil(anvil_3))` and `CoyoteController #1 home=(200, 200, 20) max_speed=160m/s`.

- [ ] **Step 4: Fire the wave (5-publish DDS-race-safe pattern)**

In a second shell:

```bash
wsl -d Ubuntu-22.04 -u mal -e bash -c "source /opt/ros/humble/setup.bash && cd /home/mal/acsdg_ws && source install/setup.bash && ros2 topic pub --times 5 -r 0.5 /mission/wave_trigger std_msgs/Bool '{data: true}'"
```

- [ ] **Step 5: Wait ~30 s for engagements to complete; verify the launch log**

In the launch log, expect to see:

- `Mission active -- engagement enabled`
- 4 `Order: <weapon_id> -> threat <N>` lines
- `CoyoteController #1: Coyote #1 assigned to target #X`
- 3 × `Interceptor #N assigned to target #M` (the Anvils)
- 4 × `NEUTRALISED target #M at range=R.RRm` lines, including **at least one with `[FRAG-FUZE]`** (from the Coyote)

```bash
wsl -d Ubuntu-22.04 -u mal -e bash -c "grep -E 'NEUTRALISED|FRAG-FUZE' /tmp/acsdg_sim.log | tail -10 || true"
```

(If the launch redirects logs differently in your setup, `tail -300 ~/.ros/log/latest_log/launch.log | grep NEUTRALISED` works.)

- [ ] **Step 6: Confirm 4 enemies despawned**

```bash
wsl -d Ubuntu-22.04 -u mal -e bash -c "source /opt/ros/humble/setup.bash && gz model --list | grep enemy"
```

Expected: empty output (all 4 enemies removed by `enemy_driver_node` on NEUTRALIZED ack).

- [ ] **Step 7: Stop the sim**

Ctrl-C the launch terminal. Verify Gazebo and all ROS nodes exit.

- [ ] **Step 8: Final commit + HANDOFF.md update**

```bash
wsl -d Ubuntu-22.04 -u mal -e bash -c "cd /home/mal/acsdg_ws && cat >> HANDOFF.md << 'EOF'

## What changed in session 2026-05-XX (Phase 2)

Phase 2 of the AI-orchestrated heterogeneous-defense upgrade. Adds the Raytheon Coyote Block 2 frag-warhead jet interceptor as the second concrete WeaponSystem.

1. **Coyote weapon class** \`acsdg_c2/weapons/coyote.py\` — implements WeaponSystem with spec §6.2 values (160 m/s, 5 km sim range, 0.10 resource cost, frag Pkill table favoring Group-3 / Shahed-class targets).
2. **Coyote SDF model** \`acsdg_gazebo/models/coyote_b2/\` — jet form, no rotors, mass 7 kg, kinematic velocity-controlled body.
3. **Coyote controller** \`acsdg_c2/src/coyote_controller_node.cpp\` — high-speed 3D pursuit, 5 m frag-fuze proximity kill, 100 m fuze arm distance.
4. **World SDF** swaps interceptor_1 → coyote_1 at the NE post (177, 177, 20).
5. **Bridge shim** generalizes to a (kind, count) tuple table; coyote_1 wired alongside enemies and interceptors.
6. **Launch file** spawns coyote_controller_node at slot 1 instead of interceptor_controller_node.
7. **C2 engine** weapon registry: 1 Coyote + 3 Anvils. Same dispatch interface; Hungarian assignment now picks between heterogeneous airframes.

End-of-session live verification: 4 enemies neutralised, Coyote shows FRAG-FUZE kill log, 3 Anvils show collision-kill log.
EOF
git add HANDOFF.md && git commit -m 'docs(handoff): Phase 2 changelog (Coyote Block 2 added)' -m 'Co-Authored-By: Claude Opus 4.7 (1M context) <noreply@anthropic.com>'"
```

---

## Phase 2 done — exit criteria checklist

- [ ] Tasks 1–8 completed
- [ ] All ~59 unit tests pass (48 prior + 11 Coyote)
- [ ] Live demo neutralises 4 enemies in a single wave (1 Coyote `[FRAG-FUZE]` + 3 Anvils collision)
- [ ] Coyote engages a target ≥3 km away from its NE post (commanded speeds approach 160 m/s)
- [ ] No new errors or warnings in the launch log beyond the pre-existing cosmetic SystemLoader complaints
- [ ] HANDOFF.md "What changed in session 2026-05-XX (Phase 2)" section reflects the new state

When all boxes are ticked, the sim is ready for **Phase 3** (DroneHunter F700 — net-capture octocopter, replaces another Anvil). Phase 3's plan will be written when Phase 2 closes — informed by what was actually learned here.
