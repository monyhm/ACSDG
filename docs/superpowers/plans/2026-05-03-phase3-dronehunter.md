# Phase 3 — DroneHunter F700 Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Add the Fortem DroneHunter F700 net-capture octocopter as the third weapon class in the heterogeneous fleet, with a 180 s multi-shot cooldown that the AI dispatcher observes via `is_available()`.

**Architecture:** Direct extension of the Phase 3 prep templates — one Python `DroneHunter(WeaponSystem)` class with `is_available()` overriding for the cooldown clock, one C++ `DroneHunterControllerNode` subclass of `WeaponControllerBase` for binary-kill net-capture pursuit, one SDF model, one FLEET slot swap. Cooldown is Python-side only (no message-schema or C++ changes). Inventory after Phase 3: 1 Coyote (NE) + 2 Anvil (NW + SW) + 1 DroneHunter (SE).

**Tech Stack:** ROS 2 Humble (rclpy + rclcpp), Gazebo Harmonic, Python 3.10 (`@dataclass(frozen=True)`, pytest, `monkeypatch`), C++17 (virtual functions), colcon build system.

**Spec:** [docs/superpowers/specs/2026-05-03-phase3-dronehunter-design.md](docs/superpowers/specs/2026-05-03-phase3-dronehunter-design.md)

**Workspace:** `~/acsdg_ws` on WSL Ubuntu-22.04. From a Windows host:
- File access: `\\wsl.localhost\Ubuntu-22.04\home\mal\acsdg_ws\…`
- Commands: `wsl -d Ubuntu-22.04 -u mal -e bash -c "cd ~/acsdg_ws && <cmd>"`
- Build (sourced): `source /opt/ros/humble/setup.bash && colcon build --symlink-install --packages-select acsdg_c2`
- Python tests: `source /opt/ros/humble/setup.bash && source ~/acsdg_ws/install/setup.bash && python3 -m pytest src/acsdg_c2/test/ -v`
- Full test: `colcon test --packages-select acsdg_c2 acsdg_gazebo && colcon test-result --verbose`

**Baseline.** HEAD at `4ad34f3b` after the spec commit. Phase 3 prep complete and pushed to origin/main. Live demo from prep verified 4/0 NEUTRALISED. Existing test count: 79 (77 deterministic + 2 documented flaky integration tests).

---

## Task 1: Dispatcher routing verification + fix

**Files:**
- Read: `src/acsdg_c2/acsdg_c2/dispatcher/dispatcher.py`
- Possibly modify: `src/acsdg_c2/acsdg_c2/dispatcher/dispatcher.py`
- Possibly add: `src/acsdg_c2/test/test_dispatcher.py` (if a regression test is needed)

This task gates the integration test (Task 6). The dispatcher's numeric-tail logic maps `weapon_id` → `interceptor_id` for engagement orders. Phase 1 had Anvils only (`anvil_1 → 2`, `anvil_2 → 3`, `anvil_3 → 4`). Phase 2 added `coyote_0 → 1`. Phase 3 adds `dronehunter_0 → 3`. The mapping must be FLEET-driven, not class-name-encoded.

- [ ] **Step 1: Read the existing dispatcher**

```bash
wsl -d Ubuntu-22.04 -u mal -e bash -c "cd ~/acsdg_ws && cat src/acsdg_c2/acsdg_c2/dispatcher/dispatcher.py"
```

Note especially the `translate(weapon, track)` method body. Identify how `interceptor_id` is derived from `weapon.weapon_id`. There are two possible patterns:

**Pattern A (numeric-tail, fragile):** Extracts the trailing digit from `weapon_id` and adds an offset. E.g., `coyote_0 → 0+1 = 1`, `anvil_1 → 1+1 = 2`, `anvil_2 → 2+1 = 3`, `anvil_3 → 3+1 = 4`. With this pattern, `dronehunter_0` would map to `0+1 = 1` — WRONG (collides with Coyote on slot 1).

**Pattern B (FLEET lookup):** `interceptor_id = next(s.interceptor_id for s in FLEET if s.weapon_id == weapon.weapon_id)`. Robust against any `weapon_id` naming.

If pattern A: proceed to Step 2 to fix.
If pattern B: dispatcher already correct — skip to Step 5 (no commit needed; just confirm).

- [ ] **Step 2: Refactor `translate` to FLEET-driven lookup (only if Pattern A)**

Edit `src/acsdg_c2/acsdg_c2/dispatcher/dispatcher.py`. Wherever the numeric-tail extraction happens, replace with:

```python
from acsdg_c2.fleet import FLEET

def translate(self, weapon: WeaponSystem, track: Track) -> dict:
    """Build EngagementOrder payload from weapon + track decision.

    interceptor_id is sourced from FLEET — single source of truth for the
    weapon_id → slot mapping. Replaces the Phase 1 numeric-tail heuristic
    that broke when Phase 3 added a non-Anvil weapon at slot 3.
    """
    try:
        interceptor_id = next(
            s.interceptor_id for s in FLEET if s.weapon_id == weapon.weapon_id)
    except StopIteration:
        raise ValueError(
            f"Dispatcher: weapon_id={weapon.weapon_id!r} not found in FLEET. "
            f"Add it to acsdg_c2/fleet.py FLEET tuple.")
    # ... rest of translate body unchanged (priority calc, target_id, etc.)
```

Preserve the existing priority calculation and any other fields the order payload contains. Only change how `interceptor_id` is computed.

- [ ] **Step 3: Run existing dispatcher tests**

```bash
wsl -d Ubuntu-22.04 -u mal -e bash -c "source /opt/ros/humble/setup.bash && source ~/acsdg_ws/install/setup.bash && cd ~/acsdg_ws && python3 -m pytest src/acsdg_c2/test/ -k dispatcher -v"
```

Expected: existing dispatcher tests pass — they used `coyote_0` and `anvil_1..3` weapon_ids which are still in FLEET, so the FLEET lookup gives the same answer as the old numeric-tail logic for those weapons.

If a test regresses, debug by adding `print(f"FLEET[*].weapon_id = {[s.weapon_id for s in FLEET]}")` inside `translate` to confirm FLEET has the expected entries.

- [ ] **Step 4: Add regression test for dronehunter_0 routing**

Append to `src/acsdg_c2/test/test_dispatcher.py` (or create if absent):

```python
def test_dispatcher_routes_dronehunter_to_slot_3():
    """Phase 3 prep regression: weapon_id='dronehunter_0' must route to
    interceptor_id=3 (the SE post per FLEET), not interceptor_id=1 (the old
    numeric-tail logic would have given 0+1=1, colliding with Coyote)."""
    # This test exists today as a guard; the actual routing happens once
    # DroneHunter is added to FLEET in Task 5. Until then, the test is a
    # canary — it'll skip if 'dronehunter_0' isn't in FLEET yet.
    from acsdg_c2.fleet import FLEET
    matches = [s for s in FLEET if s.weapon_id == 'dronehunter_0']
    if not matches:
        pytest.skip("dronehunter_0 not in FLEET yet (added in Task 5)")
    assert matches[0].interceptor_id == 3
```

The test starts as `skip` and becomes a real check after Task 5 adds DroneHunter to FLEET. This is the cheapest way to lock in the routing contract early.

- [ ] **Step 5: Run, expect pass (or skip)**

```bash
wsl -d Ubuntu-22.04 -u mal -e bash -c "source /opt/ros/humble/setup.bash && source ~/acsdg_ws/install/setup.bash && cd ~/acsdg_ws && python3 -m pytest src/acsdg_c2/test/test_dispatcher.py -v"
```

Expected: all dispatcher tests pass; the new test is `skipped` (FLEET doesn't have DroneHunter yet — Task 5 adds it).

- [ ] **Step 6: Commit**

If Pattern A was found and fixed:
```bash
wsl -d Ubuntu-22.04 -u mal -e bash -c "cd ~/acsdg_ws && git add src/acsdg_c2/acsdg_c2/dispatcher/dispatcher.py src/acsdg_c2/test/test_dispatcher.py && git commit -m 'Phase 3 Task 1: dispatcher uses FLEET-driven weapon_id->interceptor_id lookup (closes deferred registry item)'"
```

If Pattern B was already in place:
```bash
wsl -d Ubuntu-22.04 -u mal -e bash -c "cd ~/acsdg_ws && git add src/acsdg_c2/test/test_dispatcher.py && git commit -m 'Phase 3 Task 1: add dronehunter_0 routing regression test (dispatcher already FLEET-driven)'"
```

---

## Task 2: DroneHunter Python class + unit tests

**Files:**
- Create: `src/acsdg_c2/acsdg_c2/weapons/dronehunter.py`
- Modify: `src/acsdg_c2/acsdg_c2/weapons/__init__.py` (export DroneHunter)
- Create: `src/acsdg_c2/test/test_weapon_dronehunter.py`

- [ ] **Step 1: Read coyote.py for the time_to_intercept idiom**

```bash
wsl -d Ubuntu-22.04 -u mal -e bash -c "cd ~/acsdg_ws && cat src/acsdg_c2/acsdg_c2/weapons/coyote.py"
```

Note the exact body of `time_to_intercept`: how range is computed, how `v_proj` (closing rate) is derived, and how `_clamp_eff_speed` is called. The DroneHunter class will mirror this idiom — only the constants change.

- [ ] **Step 2: Write the failing tests**

Create `src/acsdg_c2/test/test_weapon_dronehunter.py`:

```python
"""Tests for DroneHunter F700 weapon class."""
import pytest

from acsdg_c2.weapons import DroneHunter, Track
from acsdg_c2.weapons.types import TargetClass


def test_dronehunter_envelope_matches_spec():
    dh = DroneHunter(weapon_id="dh_test", home_position=(0.0, 0.0, 20.0))
    env = dh.engagement_envelope()
    assert env.min_range == 50.0
    assert env.max_range == 2000.0
    assert env.max_altitude == 4000.0


def test_dronehunter_pkill_table_matches_spec():
    dh = DroneHunter(weapon_id="dh_test", home_position=(0.0, 0.0, 20.0))
    assert dh.pkill(TargetClass.SMALL_QUAD) == 0.85
    assert dh.pkill(TargetClass.GROUP_1)    == 0.70
    assert dh.pkill(TargetClass.GROUP_3)    == 0.50
    assert dh.pkill(TargetClass.SHAHED)     == 0.40


def test_dronehunter_resource_cost():
    dh = DroneHunter(weapon_id="dh_test", home_position=(0.0, 0.0, 20.0))
    assert dh.resource_cost() == 0.20


def test_dronehunter_time_to_intercept_credits_inbound_closing_rate():
    """Closing target at 30 m/s should yield ToI smaller than the static
    range/max_speed estimate. Mirrors the Coyote/Anvil closing-rate test."""
    dh = DroneHunter(weapon_id="dh_test", home_position=(0.0, 0.0, 20.0))
    closing = Track(track_id=1, position=(500.0, 0.0, 30.0),
                    velocity=(-30.0, 0.0, 0.0),  # closing at 30 m/s
                    threat_score=0.0, state="DETECTED")
    toi = dh.time_to_intercept(closing)
    static_toi = 500.0 / 31.0  # ≈ 16.1 s
    assert toi < static_toi


def test_dronehunter_time_to_intercept_clamps_eff_speed_for_fleeing_shahed():
    """Fleeing SHAHED at 185 m/s vs DroneHunter's 31 m/s — eff_speed clamps
    at 0.5*max_speed = 15.5 m/s. ToI = 1000/15.5 ≈ 64.5s. Regression guard."""
    dh = DroneHunter(weapon_id="dh_test", home_position=(0.0, 0.0, 20.0))
    fleeing = Track(track_id=1, position=(1000.0, 0.0, 50.0),
                    velocity=(185.0, 0.0, 0.0),
                    threat_score=0.0, state="DETECTED")
    toi = dh.time_to_intercept(fleeing)
    assert toi == pytest.approx(64.5, rel=0.05)
    assert toi < 200.0   # regression guard against eff_speed=1.0 floor


def test_dronehunter_is_available_returns_false_during_cooldown():
    """After mark_idle on a recently-engaged DroneHunter, is_available is
    False until 180 s elapse — Phase-3 multi-shot relaunch_time."""
    dh = DroneHunter(weapon_id="dh_test", home_position=(0.0, 0.0, 20.0))
    assert dh.is_available()  # idle, no prior engagement
    dh.mark_engaged(target_id=42)
    assert not dh.is_available()
    dh.mark_idle()  # arrival home → cooldown starts
    assert not dh.is_available()  # cooldown active


def test_dronehunter_cooldown_clears_after_relaunch_time(monkeypatch):
    """is_available() returns True once 180 s have elapsed since mark_idle.
    Use monkeypatch on time.time to advance the clock without sleeping."""
    import acsdg_c2.weapons.dronehunter as dh_mod

    fake_now = [1000.0]
    monkeypatch.setattr(dh_mod, "time", type("T", (), {"time": lambda: fake_now[0]}))

    dh = DroneHunter(weapon_id="dh_test", home_position=(0.0, 0.0, 20.0))
    dh.mark_engaged(target_id=42)
    dh.mark_idle()
    assert not dh.is_available()  # cooldown started at fake_now=1000

    fake_now[0] = 1000.0 + 179.99   # one tick before expiry
    assert not dh.is_available()

    fake_now[0] = 1000.0 + 180.01   # cooldown expired
    assert dh.is_available()


def test_dronehunter_mark_idle_without_prior_engagement_does_not_start_cooldown():
    """The was_engaged guard: mark_idle ticks while idle (10 Hz) must not
    reset the cooldown clock. Otherwise the cooldown would never expire."""
    dh = DroneHunter(weapon_id="dh_test", home_position=(0.0, 0.0, 20.0))
    dh.mark_idle()  # was already idle
    dh.mark_idle()  # another tick — must not start cooldown
    assert dh.is_available()
```

- [ ] **Step 3: Run tests to verify they fail**

```bash
wsl -d Ubuntu-22.04 -u mal -e bash -c "source /opt/ros/humble/setup.bash && source ~/acsdg_ws/install/setup.bash && cd ~/acsdg_ws && python3 -m pytest src/acsdg_c2/test/test_weapon_dronehunter.py -v"
```

Expected: collection error / `ImportError: cannot import name 'DroneHunter' from 'acsdg_c2.weapons'`.

- [ ] **Step 4: Create dronehunter.py**

Create `src/acsdg_c2/acsdg_c2/weapons/dronehunter.py`:

```python
"""Fortem DroneHunter F700 — net-capture octocopter (spec §6.3).

Multi-shot weapon with a 180 s relaunch_time cooldown after each engagement.
The cooldown is enforced via is_available() override — c2_engine reads
is_available() to decide which weapons enter the cost matrix, so a
DroneHunter that just captured a target stays out of dispatch for 3 minutes.
"""

import math
import time
from typing import Tuple

from acsdg_c2.weapons.base import WeaponSystem
from acsdg_c2.weapons.types import EngagementEnvelope, TargetClass, Track


_DRONEHUNTER_MAX_SPEED       = 31.0      # m/s — post-2024 doubled-speed update
_DRONEHUNTER_MAX_RANGE       = 2000.0    # m — kill range
_DRONEHUNTER_MIN_RANGE       = 50.0      # m — safe-deploy distance from launcher
_DRONEHUNTER_MAX_ALT         = 4000.0    # m — class-typical for 18 kg multirotor
_DRONEHUNTER_KILL_RADIUS     = 15.0      # m — net-deploy range
_DRONEHUNTER_RESOURCE_COST   = 0.20      # 0.20 first shot per spec
_DRONEHUNTER_RELAUNCH_TIME_S = 180.0     # multi-shot cooldown


_PKILL = {
    TargetClass.SMALL_QUAD: 0.85,
    TargetClass.GROUP_1:    0.70,
    TargetClass.GROUP_3:    0.50,
    TargetClass.SHAHED:     0.40,
}


class DroneHunter(WeaponSystem):
    weapon_type = "net_octo"

    def __init__(self, weapon_id: str, home_position: Tuple[float, float, float]) -> None:
        super().__init__(weapon_id=weapon_id, home_position=home_position)
        self._cooldown_until: float = 0.0   # epoch seconds; 0 = no cooldown active

    def engagement_envelope(self) -> EngagementEnvelope:
        return EngagementEnvelope(
            min_range=_DRONEHUNTER_MIN_RANGE,
            max_range=_DRONEHUNTER_MAX_RANGE,
            max_altitude=_DRONEHUNTER_MAX_ALT,
        )

    def pkill(self, target_class: TargetClass) -> float:
        return _PKILL.get(target_class, 0.0)

    def resource_cost(self) -> float:
        return _DRONEHUNTER_RESOURCE_COST

    def time_to_intercept(self, track: Track) -> float:
        rx = track.position[0] - self._home_position[0]
        ry = track.position[1] - self._home_position[1]
        rz = track.position[2] - self._home_position[2]
        range_m = math.sqrt(rx*rx + ry*ry + rz*rz)
        if range_m < 1e-6:
            return 0.0
        # Closing rate: -(unit_range . target_velocity).
        # Positive when target is moving toward the launcher.
        ux, uy, uz = rx / range_m, ry / range_m, rz / range_m
        v_proj = -(ux * track.velocity[0]
                   + uy * track.velocity[1]
                   + uz * track.velocity[2])
        eff_speed = self._clamp_eff_speed(_DRONEHUNTER_MAX_SPEED, v_proj)
        return range_m / eff_speed

    def is_available(self) -> bool:
        """Override: cooldown blocks dispatch even when no target is engaged."""
        if self._engaged_target_id is not None:
            return False
        return time.time() >= self._cooldown_until

    def mark_idle(self) -> None:
        """Override: arrival-home transitions to cooldown, not immediate availability.

        DroneHunter's spec mandates 180 s relaunch_time after recovery — model this
        as a wall-clock cooldown that gates is_available() until elapsed.
        """
        was_engaged = self._engaged_target_id is not None
        super().mark_idle()
        if was_engaged:
            self._cooldown_until = time.time() + _DRONEHUNTER_RELAUNCH_TIME_S
```

If `time_to_intercept`'s body shape diverges from coyote.py (e.g., Coyote uses `math.hypot(*tuple)` instead of explicit `sqrt`), prefer Coyote's idiom for consistency. The semantics are identical.

- [ ] **Step 5: Export DroneHunter from the weapons module**

Edit `src/acsdg_c2/acsdg_c2/weapons/__init__.py`. Add `DroneHunter` to the imports/exports. The file likely already has lines like:

```python
from acsdg_c2.weapons.anvil import Anvil
from acsdg_c2.weapons.coyote import Coyote
from acsdg_c2.weapons.base import WeaponSystem
from acsdg_c2.weapons.types import Track, TargetClass, EngagementEnvelope
```

Add:
```python
from acsdg_c2.weapons.dronehunter import DroneHunter
```

Update the module's `__all__` if present:
```python
__all__ = ['Anvil', 'Coyote', 'DroneHunter', 'WeaponSystem', 'Track', 'TargetClass', 'EngagementEnvelope']
```

- [ ] **Step 6: Run tests, verify pass**

```bash
wsl -d Ubuntu-22.04 -u mal -e bash -c "source /opt/ros/humble/setup.bash && cd ~/acsdg_ws && colcon build --symlink-install --packages-select acsdg_c2 2>&1 | tail -3 && source install/setup.bash && python3 -m pytest src/acsdg_c2/test/test_weapon_dronehunter.py -v"
```

Expected: 8 tests pass.

- [ ] **Step 7: Commit**

```bash
wsl -d Ubuntu-22.04 -u mal -e bash -c "cd ~/acsdg_ws && git add src/acsdg_c2/acsdg_c2/weapons/dronehunter.py src/acsdg_c2/acsdg_c2/weapons/__init__.py src/acsdg_c2/test/test_weapon_dronehunter.py && git commit -m 'Phase 3 Task 2: DroneHunter F700 weapon class with 180s multi-shot cooldown'"
```

---

## Task 3: DroneHunter SDF model + model.config

**Files:**
- Create: `src/acsdg_gazebo/models/dronehunter_f700/model.sdf`
- Create: `src/acsdg_gazebo/models/dronehunter_f700/model.config`

This task creates the Gazebo model files. The model isn't used by anything until Task 5 wires it into the world spawn, but creating it first lets us validate the SDF parses cleanly via `gz sdf --print`.

- [ ] **Step 1: Create the model directory**

```bash
wsl -d Ubuntu-22.04 -u mal -e bash -c "mkdir -p ~/acsdg_ws/src/acsdg_gazebo/models/dronehunter_f700"
```

- [ ] **Step 2: Create model.sdf**

Create `src/acsdg_gazebo/models/dronehunter_f700/model.sdf`:

```xml
<?xml version="1.0"?>
<sdf version="1.9">
  <model name="dronehunter_f700">
    <pose>0 0 0 0 0 0</pose>

    <!-- Velocity-controlled body, gz-sim-velocity-control-system applies cmd_vel -->
    <plugin filename="gz-sim-velocity-control-system" name="gz::sim::systems::VelocityControl"/>

    <!-- 3D odometry publisher with explicit dimensions=3 -->
    <plugin filename="gz-sim-odometry-publisher-system"
            name="gz::sim::systems::OdometryPublisher">
      <odom_publish_frequency>50</odom_publish_frequency>
      <dimensions>3</dimensions>
    </plugin>

    <link name="body">
      <inertial>
        <mass>18.0</mass>
        <!-- Disc shape: r=0.5, h=0.15, m=18:
             ixx = iyy = (1/4)·m·r² + (1/12)·m·h² = 1.16
             izz = (1/2)·m·r² = 2.25 -->
        <inertia>
          <ixx>1.16</ixx><iyy>1.16</iyy><izz>2.25</izz>
          <ixy>0.0</ixy><ixz>0.0</ixz><iyz>0.0</iyz>
        </inertia>
      </inertial>
      <collision name="body_col">
        <geometry><cylinder><radius>0.5</radius><length>0.15</length></cylinder></geometry>
      </collision>
      <visual name="body_vis">
        <geometry><cylinder><radius>0.5</radius><length>0.15</length></cylinder></geometry>
        <material><ambient>0.2 0.2 0.2 1</ambient><diffuse>0.3 0.3 0.3 1</diffuse></material>
      </visual>
      <!-- 8 rotor stubs at 45° intervals, cosmetic (no joints, no animation) -->
      <visual name="rotor_e">
        <pose>0.45 0 0.1 0 0 0</pose>
        <geometry><cylinder><radius>0.15</radius><length>0.02</length></cylinder></geometry>
        <material><diffuse>0.4 0.4 0.4 1</diffuse></material>
      </visual>
      <visual name="rotor_ne">
        <pose>0.318 0.318 0.1 0 0 0</pose>
        <geometry><cylinder><radius>0.15</radius><length>0.02</length></cylinder></geometry>
        <material><diffuse>0.4 0.4 0.4 1</diffuse></material>
      </visual>
      <visual name="rotor_n">
        <pose>0 0.45 0.1 0 0 0</pose>
        <geometry><cylinder><radius>0.15</radius><length>0.02</length></cylinder></geometry>
        <material><diffuse>0.4 0.4 0.4 1</diffuse></material>
      </visual>
      <visual name="rotor_nw">
        <pose>-0.318 0.318 0.1 0 0 0</pose>
        <geometry><cylinder><radius>0.15</radius><length>0.02</length></cylinder></geometry>
        <material><diffuse>0.4 0.4 0.4 1</diffuse></material>
      </visual>
      <visual name="rotor_w">
        <pose>-0.45 0 0.1 0 0 0</pose>
        <geometry><cylinder><radius>0.15</radius><length>0.02</length></cylinder></geometry>
        <material><diffuse>0.4 0.4 0.4 1</diffuse></material>
      </visual>
      <visual name="rotor_sw">
        <pose>-0.318 -0.318 0.1 0 0 0</pose>
        <geometry><cylinder><radius>0.15</radius><length>0.02</length></cylinder></geometry>
        <material><diffuse>0.4 0.4 0.4 1</diffuse></material>
      </visual>
      <visual name="rotor_s">
        <pose>0 -0.45 0.1 0 0 0</pose>
        <geometry><cylinder><radius>0.15</radius><length>0.02</length></cylinder></geometry>
        <material><diffuse>0.4 0.4 0.4 1</diffuse></material>
      </visual>
      <visual name="rotor_se">
        <pose>0.318 -0.318 0.1 0 0 0</pose>
        <geometry><cylinder><radius>0.15</radius><length>0.02</length></cylinder></geometry>
        <material><diffuse>0.4 0.4 0.4 1</diffuse></material>
      </visual>
    </link>
  </model>
</sdf>
```

- [ ] **Step 3: Create model.config**

Create `src/acsdg_gazebo/models/dronehunter_f700/model.config`:

```xml
<?xml version="1.0"?>
<model>
  <name>DroneHunter F700</name>
  <version>1.0</version>
  <sdf version="1.9">model.sdf</sdf>
  <author>
    <name>ACSDG</name>
  </author>
  <description>
    Fortem DroneHunter F700 net-capture octocopter. Phase 3 weapon platform.
    18 kg, 8 rotors (cosmetic), 31 m/s max speed, 15 m net-deploy range.
  </description>
</model>
```

- [ ] **Step 4: Validate SDF parses cleanly**

```bash
wsl -d Ubuntu-22.04 -u mal -e bash -c "gz sdf --print ~/acsdg_ws/src/acsdg_gazebo/models/dronehunter_f700/model.sdf 2>&1 | head -20"
```

Expected: SDF echoes back with no warnings/errors. If `gz sdf` reports a parse error, fix the SDF before continuing.

- [ ] **Step 5: Verify the install rules ship the new model**

```bash
wsl -d Ubuntu-22.04 -u mal -e bash -c "cd ~/acsdg_ws && grep -n 'install.*models' src/acsdg_gazebo/CMakeLists.txt"
```

Expected: `install(DIRECTORY models DESTINATION share/${PROJECT_NAME})` (or similar). The new directory ships automatically — no CMakeLists change needed if the install rule is directory-wide. If it's per-model (unlikely), add `dronehunter_f700` to the list.

- [ ] **Step 6: Build acsdg_gazebo to confirm install**

```bash
wsl -d Ubuntu-22.04 -u mal -e bash -c "source /opt/ros/humble/setup.bash && cd ~/acsdg_ws && colcon build --symlink-install --packages-select acsdg_gazebo 2>&1 | tail -3 && ls install/acsdg_gazebo/share/acsdg_gazebo/models/dronehunter_f700/"
```

Expected: `model.config  model.sdf` listed.

- [ ] **Step 7: Commit**

```bash
wsl -d Ubuntu-22.04 -u mal -e bash -c "cd ~/acsdg_ws && git add src/acsdg_gazebo/models/dronehunter_f700/ && git commit -m 'Phase 3 Task 3: DroneHunter F700 SDF model (octocopter, 18 kg, 8 rotor stubs)'"
```

---

## Task 4: DroneHunter C++ controller + CMakeLists

**Files:**
- Create: `src/acsdg_c2/src/dronehunter_controller_node.cpp`
- Modify: `src/acsdg_c2/CMakeLists.txt`

This task creates the C++ controller subclass. No new tests — behavioral parity is verified via the live demo in Task 7. The controller links against the existing `weapon_controller_base` library from Phase 3 prep.

- [ ] **Step 1: Read coyote_controller_node.cpp for the subclass shape**

```bash
wsl -d Ubuntu-22.04 -u mal -e bash -c "cd ~/acsdg_ws && cat src/acsdg_c2/src/coyote_controller_node.cpp"
```

Note the constructor delegating to `WeaponControllerBase`, the override bodies, and the `main()` function.

- [ ] **Step 2: Read interceptor_controller_node.cpp for the 2D + alt-hold pursuit pattern**

```bash
wsl -d Ubuntu-22.04 -u mal -e bash -c "cd ~/acsdg_ws && cat src/acsdg_c2/src/interceptor_controller_node.cpp"
```

DroneHunter uses Anvil's pursuit pattern (2D + alt-hold) but with `kMaxSpeed=31` and `kKillRadius=15`. Note the exact `computePursuitCmd` body to mirror.

- [ ] **Step 3: Create dronehunter_controller_node.cpp**

Create `src/acsdg_c2/src/dronehunter_controller_node.cpp`:

```cpp
//============================================================================
// dronehunter_controller_node.cpp -- Fortem DroneHunter F700 net-capture
// octocopter controller.
//
// Extends acsdg_c2::WeaponControllerBase. DroneHunter-specific behaviour:
//   - 2D lead pursuit + altitude hold at cruise z (same pattern as Anvil —
//     31 m/s closes typical engagements in ~30s, long enough that 3D pursuit
//     would amplify radar-noisy tgt_vz over t_go)
//   - Binary kill at 15 m net-deploy range (no probabilistic ring; net
//     deployment is deterministic at close range)
//   - [NET-CAPTURE] log tag distinguishes this weapon class in demo logs
//
// The 180 s multi-shot cooldown is enforced Python-side in DroneHunter's
// is_available() override — this controller has no cooldown logic.
//============================================================================

#include "acsdg_c2/weapon_controller_base.hpp"

#include <algorithm>
#include <cmath>
#include <memory>
#include <string>

class DroneHunterControllerNode : public acsdg_c2::WeaponControllerBase
{
  // DroneHunter spec values (spec §6.3)
  static constexpr double kMaxSpeed   = 31.0;   // m/s — post-2024 doubled-speed
  static constexpr double kKillRadius = 15.0;   // m — net-deploy range
  static constexpr double kCruiseZ    = 50.0;   // m — fixed cruise altitude
  static constexpr double kAltKp      = 0.5;    // altitude-hold P gain
  static constexpr double kMaxVz      = 3.0;    // m/s — vertical clamp

public:
  DroneHunterControllerNode()
    : WeaponControllerBase("dronehunter_controller_node", "/dronehunter_",
                           kMaxSpeed, kKillRadius,
                           /*kill_radius_outer_m=*/0.0,
                           /*gz_model_kind=*/"dronehunter")
  {
    RCLCPP_INFO(get_logger(),
      "DroneHunterController #%d  home=(%.0f, %.0f, %.0f)  max_speed=%.0fm/s",
      id_, home_x_, home_y_, home_z_, kMaxSpeed);
  }

protected:
  void computePursuitCmd(double & vx, double & vy, double & vz) override
  {
    // 2D lead pursuit toward predicted intercept point at cruise altitude.
    // Same pattern as Anvil — see interceptor_controller_node.cpp for the
    // rationale (3D pursuit was rejected; radar-noisy tgt_vz over a long
    // t_go blew interceptors hundreds of metres above their targets).
    const double rx = tgt_x_ - pos_x_;
    const double ry = tgt_y_ - pos_y_;
    const double range_xy = std::sqrt(rx*rx + ry*ry);
    const double t_go = range_xy / kMaxSpeed;
    const double lead_x = tgt_x_ + tgt_vx_ * t_go;
    const double lead_y = tgt_y_ + tgt_vy_ * t_go;

    const double dx = lead_x - pos_x_;
    const double dy = lead_y - pos_y_;
    const double dmag = std::sqrt(dx*dx + dy*dy);
    const double s = (dmag > 1e-6) ? (kMaxSpeed / dmag) : 0.0;
    vx = dx * s;
    vy = dy * s;

    // Altitude-hold P-loop independent of xy pursuit
    const double dz = kCruiseZ - pos_z_;
    vz = std::max(-kMaxVz, std::min(kMaxVz, kAltKp * dz));
  }

  std::string onKill(double range_m) override
  {
    RCLCPP_INFO(get_logger(),
      "DroneHunter #%d: NEUTRALISED target #%d at range=%.2fm "
      "[NET-CAPTURE] dh(%.1f,%.1f,%.1f) tgt(%.1f,%.1f,%.1f)",
      id_, target_id_, range_m,
      pos_x_, pos_y_, pos_z_, tgt_x_, tgt_y_, tgt_z_);
    return "NEUTRALIZED";
  }
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<DroneHunterControllerNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
```

- [ ] **Step 4: Add the executable to CMakeLists.txt**

Edit `src/acsdg_c2/CMakeLists.txt`. Locate the existing `add_executable(coyote_controller_node ...)` block. Below it, add:

```cmake
add_executable(dronehunter_controller_node src/dronehunter_controller_node.cpp)
target_link_libraries(dronehunter_controller_node weapon_controller_base)
ament_target_dependencies(dronehunter_controller_node
  rclcpp acsdg_msgs geometry_msgs nav_msgs std_msgs builtin_interfaces
)
```

Update the install(TARGETS ...) line to include the new executable:

```cmake
install(TARGETS interceptor_controller_node coyote_controller_node dronehunter_controller_node
  DESTINATION lib/${PROJECT_NAME})
```

(If the install rule is split across multiple `install()` calls, find the one for the controller executables and add `dronehunter_controller_node` to it.)

- [ ] **Step 5: Build clean**

```bash
wsl -d Ubuntu-22.04 -u mal -e bash -c "source /opt/ros/humble/setup.bash && cd ~/acsdg_ws && colcon build --symlink-install --packages-select acsdg_c2 2>&1 | tail -10"
```

Expected: build succeeds; new `dronehunter_controller_node` binary appears at `install/acsdg_c2/lib/acsdg_c2/dronehunter_controller_node`.

Verify:
```bash
wsl -d Ubuntu-22.04 -u mal -e bash -c "ls -la ~/acsdg_ws/install/acsdg_c2/lib/acsdg_c2/dronehunter_controller_node"
```

- [ ] **Step 6: Smoke test — launch the node alone**

```bash
wsl -d Ubuntu-22.04 -u mal -e bash -c "source /opt/ros/humble/setup.bash && source ~/acsdg_ws/install/setup.bash && timeout 3 ros2 run acsdg_c2 dronehunter_controller_node --ros-args -p interceptor_id:=99 -p home_x:=0.0 -p home_y:=0.0 -p home_z:=20.0 2>&1 | head -3 || echo TIMEOUT_EXPECTED"
```

Expected stdout: `DroneHunterController #99  home=(0, 0, 20)  max_speed=31m/s`. The `TIMEOUT_EXPECTED` is fine — we just want the startup line.

- [ ] **Step 7: Commit**

```bash
wsl -d Ubuntu-22.04 -u mal -e bash -c "cd ~/acsdg_ws && git add src/acsdg_c2/src/dronehunter_controller_node.cpp src/acsdg_c2/CMakeLists.txt && git commit -m 'Phase 3 Task 4: DroneHunter F700 C++ controller (subclass of WeaponControllerBase)'"
```

---

## Task 5: FLEET update + world spawn + bridge fallback + test_fleet update

**Files:**
- Modify: `src/acsdg_c2/acsdg_c2/fleet.py`
- Modify: `src/acsdg_gazebo/worlds/military_base.sdf`
- Modify: `src/acsdg_gazebo/scripts/gz_bridge_shim.py`
- Modify: `src/acsdg_c2/test/test_fleet.py`

These four files MUST land in one commit. The FLEET-vs-SDF agreement test (`test_fleet_matches_sdf_spawn_poses`) parses both files at test-collection time — committing FLEET without the SDF (or vice versa) breaks the test on every commit until both sides land.

- [ ] **Step 1: Update fleet.py**

Edit `src/acsdg_c2/acsdg_c2/fleet.py`. Add `DroneHunter` to the imports:

```python
from acsdg_c2.weapons import Anvil, Coyote, DroneHunter, WeaponSystem
```

Update the `FLEET` tuple. Replace the slot-3 line (currently `Slot(3, Anvil, "anvil_2", ...)`) with the DroneHunter entry:

```python
FLEET: Tuple[Slot, ...] = (
    Slot(1, Coyote,      "coyote_0",      "coyote_controller_node",       "coyote",       1, ( 177.0,  177.0, 20.0)),
    Slot(2, Anvil,       "anvil_1",       "interceptor_controller_node",  "interceptor",  2, (-177.0,  177.0, 20.0)),
    Slot(3, DroneHunter, "dronehunter_0", "dronehunter_controller_node",  "dronehunter",  1, ( 177.0, -177.0, 20.0)),
    Slot(4, Anvil,       "anvil_3",       "interceptor_controller_node",  "interceptor",  4, (-177.0, -177.0, 20.0)),
)
```

Note: `anvil_2` (the old slot-3 weapon_id) is gone; `anvil_3` keeps its position on slot 4 for stability with existing test expectations.

- [ ] **Step 2: Update military_base.sdf**

Edit `src/acsdg_gazebo/worlds/military_base.sdf`. Find the block:

```xml
<include>
  <name>interceptor_3</name>
  <pose>177 -177 20 0 0 0</pose>
  <uri>model://interceptor_drone</uri>
</include>
```

Replace with:

```xml
<include>
  <name>dronehunter_1</name>
  <pose>177 -177 20 0 0 0</pose>
  <uri>model://dronehunter_f700</uri>
</include>
```

The other three interceptor blocks (`interceptor_2`, `interceptor_4`) and the Coyote (`coyote_1`) stay unchanged.

- [ ] **Step 3: Update gz_bridge_shim.py fallback list**

Edit `src/acsdg_gazebo/scripts/gz_bridge_shim.py`. Find the `except ImportError:` branch in the FLEET-import block (added in Phase 3 prep Task 5). Update the fallback tuple from:

```python
_INTERCEPTOR_MODELS = (('coyote', 1), ('interceptor', 4))
```

to:

```python
_INTERCEPTOR_MODELS = (('coyote', 1), ('dronehunter', 1), ('interceptor', 4))
```

The canonical (non-fallback) path auto-derives via `bridged_models()` and needs no change.

- [ ] **Step 4: Update test_fleet.py**

Edit `src/acsdg_c2/test/test_fleet.py`. Find the test:

```python
def test_bridged_models_for_phase2_inventory():
    """Today's FLEET = 1 Coyote at instance 1 + 3 Anvils at instance 2..4
    → bridged_models = (('coyote', 1), ('interceptor', 4))."""
    assert bridged_models() == (('coyote', 1), ('interceptor', 4))
```

Rename and update:

```python
def test_bridged_models_for_phase3_inventory():
    """Phase 3 FLEET = 1 Coyote (gz instance 1) + 1 DroneHunter (gz instance 1)
    + 2 Anvils (gz instances 2 and 4) → bridged_models returns the max instance
    index per kind, sorted alphabetically."""
    assert bridged_models() == (('coyote', 1), ('dronehunter', 1), ('interceptor', 4))
```

- [ ] **Step 5: Build, run full test suite**

```bash
wsl -d Ubuntu-22.04 -u mal -e bash -c "source /opt/ros/humble/setup.bash && cd ~/acsdg_ws && colcon build --symlink-install --packages-select acsdg_c2 acsdg_gazebo 2>&1 | tail -3 && source install/setup.bash && python3 -m pytest src/acsdg_c2/test/ 2>&1 | tail -10"
```

Expected: tests pass (modulo the documented integration-test flake — 77 deterministic + 2 flaky integration). The dispatcher-routing skip from Task 1 should now turn into a `passed` because FLEET has `dronehunter_0`.

If `test_fleet_matches_sdf_spawn_poses` fails: SDF spawn pose for `dronehunter_1` doesn't match FLEET's `(177, -177, 20)`. Re-check Step 2 — pose XML is `<pose>177 -177 20 0 0 0</pose>`.

- [ ] **Step 6: Smoke-test the bridge shim still imports**

```bash
wsl -d Ubuntu-22.04 -u mal -e bash -c "source /opt/ros/humble/setup.bash && source ~/acsdg_ws/install/setup.bash && python3 -c 'import sys; sys.path.insert(0, \"src/acsdg_gazebo/scripts\"); import gz_bridge_shim; print(gz_bridge_shim._BRIDGED_MODELS)'"
```

Expected: `(('enemy', 4), ('coyote', 1), ('dronehunter', 1), ('interceptor', 4))`.

- [ ] **Step 7: Commit**

```bash
wsl -d Ubuntu-22.04 -u mal -e bash -c "cd ~/acsdg_ws && git add src/acsdg_c2/acsdg_c2/fleet.py src/acsdg_gazebo/worlds/military_base.sdf src/acsdg_gazebo/scripts/gz_bridge_shim.py src/acsdg_c2/test/test_fleet.py && git commit -m 'Phase 3 Task 5: FLEET registers DroneHunter at SE slot 3; world spawn + bridge fallback updated'"
```

---

## Task 6: DroneHunter integration test

**Files:**
- Create: `src/acsdg_c2/test/test_dronehunter_integration.py`

This test reuses the SpyNode harness pattern from `test_coyote_integration.py`. Two scenarios: dispatch routing to interceptor_id=3, and cooldown behavior.

- [ ] **Step 1: Read test_coyote_integration.py for the SpyNode helper structure**

```bash
wsl -d Ubuntu-22.04 -u mal -e bash -c "cd ~/acsdg_ws && cat src/acsdg_c2/test/test_coyote_integration.py"
```

Note especially: the `harness` fixture, the SpyNode class, the `publish_target` / `publish_idle_state` / `publish_ack` / `publish_wave` helpers, and `wait_for_order`.

The DroneHunter integration test will share the SAME SpyNode shape — copy the relevant helpers verbatim. We don't lift to a shared `conftest.py` because it would refactor Phase 2's existing test, and the spec says "reuse SpyNode harness pattern" not "share via fixture."

- [ ] **Step 2: Create test_dronehunter_integration.py**

Create `src/acsdg_c2/test/test_dronehunter_integration.py`:

```python
"""DroneHunter F700 engagement integration test (in-process rclpy harness).

Runs C2EngineNode + a SpyNode in a MultiThreadedExecutor, without Gazebo or
launch_testing. Mirrors test_coyote_integration.py with two DroneHunter-specific
scenarios:
  1. Dispatch routing: SE-incoming target → interceptor_id=3 (the DroneHunter slot)
  2. Cooldown gate: after capture + IDLE, DroneHunter is unavailable for 180 s
"""
import json
import threading
import time

import pytest
import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from std_msgs.msg import Bool, String

from acsdg_msgs.msg import EngagementOrder, FusedTarget, InterceptorState

from acsdg_c2.c2_engine_node import C2EngineNode


class SpyNode(Node):
    """Test driver — publishes synthetic targets and watches orders/acks."""

    def __init__(self) -> None:
        super().__init__('integration_spy')
        self.last_order: EngagementOrder | None = None
        self.last_ack: dict | None = None
        self._wave_pub = self.create_publisher(Bool, '/mission/wave_trigger', 10)
        self._fused_pub = self.create_publisher(String, '/sensors/fusion/targets', 10)
        self._target_pubs: dict[int, "rclpy.publisher.Publisher"] = {}
        self._state_pubs: dict[int, "rclpy.publisher.Publisher"] = {}
        self._ack_pub_for_test = self.create_publisher(String, '/mission/engagement_ack', 10)
        self.create_subscription(EngagementOrder, '/c2/engagement_orders', self._on_order, 10)
        self.create_subscription(String, '/mission/engagement_ack', self._on_ack, 10)

    def _on_order(self, msg: EngagementOrder) -> None:
        self.last_order = msg

    def _on_ack(self, msg: String) -> None:
        try:
            self.last_ack = json.loads(msg.data)
        except json.JSONDecodeError:
            pass

    def publish_wave(self) -> None:
        self._wave_pub.publish(Bool(data=True))

    def publish_target(self, *, track_id: int, x: float, y: float, z: float,
                       vx: float = 0.0, vy: float = 0.0, vz: float = 0.0,
                       state: str = "DETECTED") -> None:
        payload = json.dumps([{
            "id": track_id, "state": state,
            "position": {"x": x, "y": y, "z": z},
            "velocity": {"x": vx, "y": vy, "z": vz},
        }])
        self._fused_pub.publish(String(data=payload))
        topic = f"/threats/target_{track_id}/fused_target"
        if track_id not in self._target_pubs:
            self._target_pubs[track_id] = self.create_publisher(FusedTarget, topic, 10)
        ft = FusedTarget()
        ft.id = track_id
        ft.position.x = x; ft.position.y = y; ft.position.z = z
        ft.velocity.x = vx; ft.velocity.y = vy; ft.velocity.z = vz
        self._target_pubs[track_id].publish(ft)

    def publish_state(self, *, interceptor_id: int, status: str,
                      target_id: int = 0,
                      x: float = 0.0, y: float = 0.0, z: float = 20.0) -> None:
        """Publish an InterceptorState for a given slot. Used to mark slots
        as PURSUING (so the cost matrix excludes them) without spawning real
        controllers."""
        if interceptor_id not in self._state_pubs:
            self._state_pubs[interceptor_id] = self.create_publisher(
                InterceptorState, f'/interceptors/unit_{interceptor_id}/state', 10)
        msg = InterceptorState()
        msg.id = interceptor_id
        msg.status = status
        msg.position.x = x; msg.position.y = y; msg.position.z = z
        msg.target_id = target_id
        self._state_pubs[interceptor_id].publish(msg)

    def publish_idle_state(self, x: float, y: float, z: float, *,
                           interceptor_id: int = 1) -> None:
        self.publish_state(interceptor_id=interceptor_id, status="IDLE",
                           x=x, y=y, z=z)

    def publish_ack(self, target_id: int, interceptor_id: int, outcome: str) -> None:
        ack = String()
        ack.data = json.dumps({
            "target_id": target_id,
            "interceptor_id": interceptor_id,
            "outcome": outcome,
        })
        self._ack_pub_for_test.publish(ack)

    def wait_for_order(self, timeout_s: float = 5.0) -> EngagementOrder:
        deadline = time.time() + timeout_s
        while time.time() < deadline:
            if self.last_order is not None:
                return self.last_order
            time.sleep(0.05)
        raise AssertionError(f"No EngagementOrder within {timeout_s}s")


@pytest.fixture
def harness():
    rclpy.init()
    executor = MultiThreadedExecutor(num_threads=4)
    c2 = C2EngineNode()
    spy = SpyNode()
    for n in (c2, spy):
        executor.add_node(n)
    thread = threading.Thread(target=executor.spin, daemon=True)
    thread.start()
    time.sleep(0.5)
    yield spy
    executor.shutdown()
    thread.join(timeout=2.0)
    rclpy.shutdown()


# Target placement: SE-of-origin lands closer to DroneHunter (177, -177, 20)
# than to any other slot. Coyote at (177, 177) and Anvils at (-177, ±177) are
# all 400-700m away vs DroneHunter's 174m. DroneHunter's max_speed of 31 m/s
# wins ToI at this geometry.
_SE_TARGET_POS = (300.0, -300.0, 20.0)


def _mark_others_busy(spy: SpyNode) -> None:
    """Mark Coyote (slot 1) + both Anvils (slots 2, 4) as PURSUING phantom
    targets, so DroneHunter (slot 3) is the only available weapon for the
    cost matrix.

    Why this contrivance: at any plausible target geometry, Coyote's 160 m/s
    dominates the ToI cost matrix (e.g., target at (300,-300): Coyote 493m/160
    = 3.08s vs DroneHunter 174m/31 = 5.61s — Coyote wins). To isolate the
    routing logic for DroneHunter, the test forces all other slots out of the
    cost matrix. Real cost-matrix tie-breaking is exercised by the live demo
    in Task 7.
    """
    spy.publish_state(interceptor_id=1, status="PURSUING", target_id=99,
                      x=177.0, y=177.0, z=20.0)
    spy.publish_state(interceptor_id=2, status="PURSUING", target_id=98,
                      x=-177.0, y=177.0, z=20.0)
    spy.publish_state(interceptor_id=4, status="PURSUING", target_id=97,
                      x=-177.0, y=-177.0, z=20.0)


def test_c2_dispatches_dronehunter_to_se_target(harness):
    """End-to-end: arm mission → mark others busy → publish SE target →
    assert engagement order routes to interceptor_id=3 (the DroneHunter slot)."""
    spy = harness
    _mark_others_busy(spy)
    spy.publish_idle_state(177.0, -177.0, 20.0, interceptor_id=3)  # DroneHunter idle
    spy.publish_wave()
    time.sleep(0.5)
    spy.publish_target(track_id=1,
                       x=_SE_TARGET_POS[0], y=_SE_TARGET_POS[1], z=_SE_TARGET_POS[2])
    order = spy.wait_for_order(timeout_s=3.0)
    assert order.target_id == 1
    assert order.interceptor_id == 3, (
        f"Expected DroneHunter (interceptor_id=3), got {order.interceptor_id}")


def test_dronehunter_unavailable_during_cooldown_after_engagement(harness):
    """After capture + IDLE, DroneHunter's is_available() returns False for
    180 s. Confirm a second target spawned shortly after routes to a different
    slot — observable evidence of the cooldown."""
    spy = harness
    _mark_others_busy(spy)
    spy.publish_idle_state(177.0, -177.0, 20.0, interceptor_id=3)
    spy.publish_wave()
    time.sleep(0.3)
    spy.publish_target(track_id=1,
                       x=_SE_TARGET_POS[0], y=_SE_TARGET_POS[1], z=_SE_TARGET_POS[2])
    first = spy.wait_for_order(timeout_s=3.0)
    assert first.interceptor_id == 3   # DroneHunter took it

    # Simulate capture + return-home: ack the kill, then publish IDLE state
    # to trigger DroneHunter.mark_idle() → cooldown starts.
    spy.publish_ack(target_id=1, interceptor_id=3, outcome="NEUTRALIZED")
    spy.publish_idle_state(177.0, -177.0, 20.0, interceptor_id=3)

    # Free up the Anvils (Coyote stays PURSUING for stable cost-matrix). After
    # cooldown gates DroneHunter, an Anvil should win.
    spy.publish_state(interceptor_id=2, status="IDLE",
                      x=-177.0, y=177.0, z=20.0)
    spy.publish_state(interceptor_id=4, status="IDLE",
                      x=-177.0, y=-177.0, z=20.0)

    time.sleep(0.5)
    spy.last_order = None

    # Retire track 1 + send a new SE target. With DroneHunter on cooldown
    # and Coyote PURSUING, the only candidates are Anvils 2 and 4.
    spy.publish_target(track_id=1,
                       x=_SE_TARGET_POS[0], y=_SE_TARGET_POS[1], z=_SE_TARGET_POS[2],
                       state="TERMINATED")
    spy.publish_target(track_id=2,
                       x=_SE_TARGET_POS[0], y=_SE_TARGET_POS[1], z=_SE_TARGET_POS[2])

    second = spy.wait_for_order(timeout_s=3.0)
    assert second.interceptor_id != 3, (
        f"Expected non-DroneHunter slot (cooldown active), "
        f"got interceptor_id={second.interceptor_id}")
    assert second.interceptor_id in (2, 4), (
        f"Expected one of the freed Anvils (slot 2 or 4), "
        f"got interceptor_id={second.interceptor_id}")
```

- [ ] **Step 3: Run the new integration tests**

```bash
wsl -d Ubuntu-22.04 -u mal -e bash -c "source /opt/ros/humble/setup.bash && source ~/acsdg_ws/install/setup.bash && cd ~/acsdg_ws && python3 -m pytest src/acsdg_c2/test/test_dronehunter_integration.py -v"
```

Expected: 2 passed (subject to the documented DDS-discovery flake; one re-run if it fails).

If both tests fail repeatedly: check that DroneHunter is correctly registered in FLEET (Task 5 step 1) and the dispatcher routes `dronehunter_0 → 3` (Task 1 step 2).

- [ ] **Step 4: Run full suite — confirm no regressions**

```bash
wsl -d Ubuntu-22.04 -u mal -e bash -c "cd ~/acsdg_ws && python3 -m pytest src/acsdg_c2/test/ 2>&1 | tail -10"
```

Expected: previous 79 tests + 8 new DroneHunter weapon tests + 1 dispatcher routing test (was skipped, now passes) + 2 new integration tests = ~89 tests. Modulo the 2 documented Coyote integration flakes, expect 87 deterministic.

- [ ] **Step 5: Commit**

```bash
wsl -d Ubuntu-22.04 -u mal -e bash -c "cd ~/acsdg_ws && git add src/acsdg_c2/test/test_dronehunter_integration.py && git commit -m 'Phase 3 Task 6: DroneHunter integration test (dispatch routing + cooldown gate)'"
```

---

## Task 7: HANDOFF.md changelog + final live demo + final code review

**Files:**
- Modify: `HANDOFF.md`

This task is the final acceptance gate. Live demo must reproduce 4/0 NEUTRALISED with `[NET-CAPTURE]` log present and DroneHunter sitting out subsequent waves.

- [ ] **Step 1: Append Phase 3 section to HANDOFF.md**

Open `HANDOFF.md`. Find the existing "## What changed in session 2026-05-03 — Phase 3 prep" section. Insert a new section AFTER it (before "## Auto-memory checkpoint" if present):

```markdown
## What changed in session 2026-05-03 (continued) — Phase 3 (DroneHunter F700)

**Phase 3 proper** lands the third weapon class — the Fortem DroneHunter F700 net-capture octocopter. Spec at `docs/superpowers/specs/2026-05-03-phase3-dronehunter-design.md`, plan at `docs/superpowers/plans/2026-05-03-phase3-dronehunter.md`.

1. **DroneHunter Python class** (`src/acsdg_c2/acsdg_c2/weapons/dronehunter.py`). Concrete `WeaponSystem` subclass — max_speed=31 m/s, kill_radius=15 m (net-deploy range), max_range=2 km, resource_cost=0.20, relaunch_time=180 s. Pkill table: SMALL_QUAD=0.85, GROUP_1=0.70, GROUP_3=0.50, SHAHED=0.40 (net entanglement, no recovery). The 180 s multi-shot cooldown is enforced via `is_available()` and `mark_idle()` overrides — Python-only, no message-schema or C++ changes.

2. **DroneHunter C++ controller** (`src/acsdg_c2/src/dronehunter_controller_node.cpp`). ~85-line subclass of `WeaponControllerBase`. 2D pursuit + altitude hold (Anvil pattern) since DroneHunter's 31 m/s closes typical engagements in ~30 s — long enough that 3D pursuit would amplify radar-noisy `tgt_vz` over t_go. Binary kill at 15 m with `[NET-CAPTURE]` log tag.

3. **DroneHunter SDF model** (`src/acsdg_gazebo/models/dronehunter_f700/`). Octocopter form, 18 kg cylindrical body, 8 cosmetic rotor stubs at 45° intervals. Uses `gz-sim-velocity-control-system` + 3D `OdometryPublisher` (same plugin set as Anvil/Coyote).

4. **FLEET update.** Slot 3 (SE post at 177, -177, 20) now hosts DroneHunter (`weapon_id="dronehunter_0"`, `gz_model_kind="dronehunter"`). Slot 4 keeps `anvil_3` for naming stability. Inventory: 1 Coyote (NE) + 2 Anvil (NW + SW) + 1 DroneHunter (SE).

5. **Dispatcher refactor.** Phase 1's numeric-tail `weapon_id → interceptor_id` heuristic was replaced with a FLEET-driven lookup. This closes the deferred "explicit weapon_id→interceptor_id registry" item from the prep — `dispatcher.translate(weapon, track)` now reads `slot.interceptor_id` from FLEET by matching `slot.weapon_id == weapon.weapon_id`. Robust against any future weapon naming.

6. **Tests added.** `test_weapon_dronehunter.py` (8 tests: envelope, Pkill table, resource_cost, ToI closing-rate, ToI clamp, cooldown-during-engagement, cooldown-clears-after-180s with `monkeypatch`, idle-tick-doesn't-reset-cooldown). `test_dronehunter_integration.py` (2 tests: dispatch routing to interceptor_id=3, cooldown gate after engagement). `test_fleet.py` updated for Phase 3 inventory. Suite grew from 79 to ~89 tests.

**Live verification.** Phase 2 demo procedure produces 4 NEUTRALISED, 0 BREACHED with one `[NET-CAPTURE]` log line from DroneHunter alongside Coyote's `[FRAG-FUZE]` and three standard Anvil kills. With wave-stacked engagements, DroneHunter visibly sits out subsequent waves for 180 s while Anvils + Coyote handle later threats — observable evidence of the multi-shot cooldown driving adaptive AI dispatch.

**Open items.** Phase 4 (Skyranger 30 + Bayesian classifier hookup) and Phase 5 (expected-utility cost function + showcase demo) still pending. The pre-existing Python/C++ Anvil max_speed mismatch (Python 45 m/s vs C++ 15 m/s) is unaddressed and now joined by a similar question for DroneHunter (Python 31 m/s used in cost matrix; C++ 31 m/s used in pursuit — these match by design, but any future refactor must keep them in sync).
```

- [ ] **Step 2: Run full test suite — final clean pass**

```bash
wsl -d Ubuntu-22.04 -u mal -e bash -c "source /opt/ros/humble/setup.bash && cd ~/acsdg_ws && colcon build --symlink-install --packages-select acsdg_c2 acsdg_gazebo 2>&1 | tail -3 && source install/setup.bash && python3 -m pytest src/acsdg_c2/test/ 2>&1 | tail -10"
```

Expected: ~89 tests pass (modulo the documented 2-test integration flake).

- [ ] **Step 3: Build clean, no new warnings**

```bash
wsl -d Ubuntu-22.04 -u mal -e bash -c "source /opt/ros/humble/setup.bash && cd ~/acsdg_ws && colcon build --symlink-install --cmake-clean-cache --packages-select acsdg_c2 acsdg_gazebo 2>&1 | grep -iE 'warning|error' | grep -v 'CMP0148' | head -20 || echo NO_NEW_WARNINGS"
```

Expected: `NO_NEW_WARNINGS` or only known cosmetic ones.

- [ ] **Step 4: Live demo — final verification**

Kill any stale processes:
```bash
wsl -d Ubuntu-22.04 -u mal -e bash -c "for pid in \$(ps -eo pid,comm | awk '/ruby|ros2|controller_node|c2_engine|gz_bridge|enemy_driver/ {print \$1}'); do kill -9 \$pid 2>/dev/null; done; sleep 1; ps -eo pid,comm | grep -iE 'ruby|controller|c2_engine|gz_bridge' | grep -v grep || echo CLEAN"
```

Launch (headless, log to file):
```bash
wsl -d Ubuntu-22.04 -u mal -e bash -c "source /opt/ros/humble/setup.bash && source ~/acsdg_ws/install/setup.bash && cd ~/acsdg_ws && rm -f /tmp/phase3_dronehunter_demo.log && LIBGL_ALWAYS_SOFTWARE=1 ros2 launch acsdg_bringup acsdg_full.launch.py headless:=true > /tmp/phase3_dronehunter_demo.log 2>&1 &"
```

Wait for sim to come up:
```bash
wsl -d Ubuntu-22.04 -u mal -e bash -c "until grep -q 'C2EngineNode ready' /tmp/phase3_dronehunter_demo.log 2>/dev/null; do sleep 2; done; echo SIM_UP; grep 'DroneHunterController' /tmp/phase3_dronehunter_demo.log"
```

Expected: SIM_UP plus a `DroneHunterController #3 home=(177, -177, 20) max_speed=31m/s` startup line.

Trigger waves:
```bash
wsl -d Ubuntu-22.04 -u mal -e bash -c "source /opt/ros/humble/setup.bash && source ~/acsdg_ws/install/setup.bash && ros2 topic pub --times 5 -r 0.5 /mission/wave_trigger std_msgs/Bool '{data: true}' 2>&1 | tail -2"
```

Wait for kills:
```bash
wsl -d Ubuntu-22.04 -u mal -e bash -c "for i in {1..30}; do if grep -qE 'NEUTRALISED.*target #4|BREACHED' /tmp/phase3_dronehunter_demo.log 2>/dev/null; then echo DEMO_DONE; break; fi; sleep 2; done"
```

Read the demo summary:
```bash
wsl -d Ubuntu-22.04 -u mal -e bash -c "grep -E 'NEUTRALISED|BREACHED|FRAG-FUZE|NET-CAPTURE' /tmp/phase3_dronehunter_demo.log | head -20"
```

Expected log evidence:
- 4 `NEUTRALISED` lines, 0 `BREACHED`
- 1 `[NET-CAPTURE]` log line from DroneHunter (`DroneHunter #3: NEUTRALISED ...`)
- 1 `[FRAG-FUZE p50]` or `[FRAG-FUZE p30]` line from Coyote
- 2 plain `NEUTRALISED` lines from Anvils at ranges 6-8 m

Kill sim:
```bash
wsl -d Ubuntu-22.04 -u mal -e bash -c "for pid in \$(ps -eo pid,comm | awk '/ruby|ros2|controller_node|c2_engine|gz_bridge|enemy_driver/ {print \$1}'); do kill -9 \$pid 2>/dev/null; done"
```

If 4/0 doesn't reproduce: do NOT commit HANDOFF. Investigate the regression. Common failure modes:
- DroneHunter not in FLEET (Task 5 step 1) → C2 doesn't know about it → only 3 weapons in cost matrix → 3 NEUTRALISED + 1 BREACHED
- DroneHunter SDF not installed (Task 3 step 6) → spawn fails → Gazebo logs error → check `/tmp/phase3_dronehunter_demo.log` for "Could not load model"
- Topic prefix mismatch (`/dronehunter_3/cmd_vel` vs Gazebo's `/model/dronehunter_1/cmd_vel`) → controller spins but bridge silently drops cmd_vel → DroneHunter never moves → enemy reaches origin → BREACHED. Check the bridge shim's `_BRIDGED_MODELS`.

- [ ] **Step 5: Commit HANDOFF**

```bash
wsl -d Ubuntu-22.04 -u mal -e bash -c "cd ~/acsdg_ws && git add HANDOFF.md && git commit -m 'docs(handoff): Phase 3 DroneHunter F700 changelog (third weapon class, multi-shot cooldown)'"
```

- [ ] **Step 6: Final code review**

After committing HANDOFF, the controller (you, the agent dispatching subagents) should dispatch a final `superpowers:code-reviewer` subagent across the entire Phase 3 delta:

```
Range: 4ad34f3b..HEAD (the spec commit through the HANDOFF commit)
Focus: spec compliance against docs/superpowers/specs/2026-05-03-phase3-dronehunter-design.md;
       cohesion of the DroneHunter integration with the Phase 3 prep templates
       (FLEET / WeaponControllerBase / launch_parameters / _clamp_eff_speed);
       Phase-4-readiness (will Skyranger 30 slot in cleanly?).
```

Address Critical / Important findings before declaring Phase 3 complete. Nits go onto a deferred-list followup.

---

## Task summary

| # | Task | Closes / Delivers | New tests | Files touched |
|---|---|---|---|---|
| 1 | Dispatcher routing verification + fix | Phase-3-prep-deferred "explicit registry" | 1 | 1 mod, 1 mod (test) |
| 2 | DroneHunter Python class + unit tests | Spec §6.3 Python side | 8 | 1 new, 1 mod, 1 new (test) |
| 3 | DroneHunter SDF model | Spec §6.3 Gazebo side | 0 | 2 new |
| 4 | DroneHunter C++ controller + CMake | Spec §6.3 controller side | 0 | 1 new, 1 mod |
| 5 | FLEET + world + bridge + test_fleet | Phase 3 fleet integration | 0 (existing test repurposed) | 4 mod |
| 6 | DroneHunter integration test | M-1 equivalent for DroneHunter | 2 | 1 new |
| 7 | HANDOFF + final demo + final review | (close) | 0 | 1 mod |

**Total:** 11 new tests, 6 new files, 8 modified files, ~+700 lines net.

**Estimated effort:** 3-5 days experienced; longer at graduation-project pace.
