# Phase 1: C2 Refactor + Anvil Port — Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Decompose the monolithic `c2_engine_node.py` into modular components (`weapons/`, `classifier/`, `cost_function/`, `assignment/`, `dispatcher/`), port the existing kinetic interceptor behavior into a new `Anvil` weapon class, and verify the existing 4-vs-4 demo runs identically with no behavior change.

**Architecture:** Module-based decomposition with clean interfaces. Create `acsdg_c2/{weapons,classifier,cost_function,assignment,dispatcher}/` packages. `WeaponSystem` ABC defines the per-weapon contract; `Anvil` is the first concrete implementation. `c2_engine_node.py` becomes a thin orchestrator that composes the modules.

**Tech Stack:** ROS 2 Humble, Python 3.10, pytest, colcon (`ament_cmake`+`ament_cmake_python`), Gazebo Harmonic.

**Reference spec:** `docs/superpowers/specs/2026-05-02-ai-orchestrated-heterogeneous-defense-design.md`

**Phase exit criterion:** `ros2 launch acsdg_bringup acsdg_full.launch.py headless:=false` followed by the 5-publish wave trigger reproduces the existing demo: 4 Anvils intercept 4 enemies at the previously-recorded 6.81–8.0 m kill ranges, with no console errors and no SystemLoader complaints beyond the existing cosmetic ones.

---

## File Structure

### Files to create

| Path | Responsibility |
|---|---|
| `src/acsdg_c2/acsdg_c2/weapons/__init__.py` | Package marker; re-export `WeaponSystem`, `Anvil` |
| `src/acsdg_c2/acsdg_c2/weapons/types.py` | `TargetClass` enum, `EngagementEnvelope`, `Track`, `WeaponState` dataclasses |
| `src/acsdg_c2/acsdg_c2/weapons/base.py` | `WeaponSystem` ABC |
| `src/acsdg_c2/acsdg_c2/weapons/anvil.py` | `Anvil(WeaponSystem)` concrete class |
| `src/acsdg_c2/acsdg_c2/classifier/__init__.py` | Re-export `Classifier` |
| `src/acsdg_c2/acsdg_c2/classifier/bayesian.py` | Phase-1 stub: returns deterministic single-class posterior |
| `src/acsdg_c2/acsdg_c2/cost_function/__init__.py` | Re-export `build_cost_matrix`, `score_threat` |
| `src/acsdg_c2/acsdg_c2/cost_function/threat_priority.py` | `score_threat(track)` — extracted from `_score` in c2_engine |
| `src/acsdg_c2/acsdg_c2/cost_function/expected_utility.py` | `build_cost_matrix(weapons, tracks, classifier)` — Phase-1 returns time-to-intercept geometry; later phases enrich it |
| `src/acsdg_c2/acsdg_c2/assignment/__init__.py` | Re-export `assign` |
| `src/acsdg_c2/acsdg_c2/assignment/solver.py` | `assign(cost_matrix)` — wraps existing `acsdg_c2.hungarian.hungarian` |
| `src/acsdg_c2/acsdg_c2/dispatcher/__init__.py` | Re-export `Dispatcher` |
| `src/acsdg_c2/acsdg_c2/dispatcher/dispatcher.py` | `Dispatcher` — converts assignment to `EngagementOrder` messages |
| `src/acsdg_c2/test/__init__.py` | (empty) Python test discovery marker |
| `src/acsdg_c2/test/test_weapons_types.py` | Unit tests for `EngagementEnvelope`, `TargetClass`, `Track` |
| `src/acsdg_c2/test/test_weapon_anvil.py` | Unit tests for `Anvil` envelope, Pkill, `can_engage`, `dispatch` |
| `src/acsdg_c2/test/test_assignment.py` | Unit tests for `assign()` wrapper on toy 4×4 matrices |
| `src/acsdg_c2/test/test_cost_function.py` | Unit tests for `build_cost_matrix` time-to-intercept math |
| `src/acsdg_c2/test/test_threat_priority.py` | Unit tests for `score_threat` (matches old `_score` for fixed inputs) |
| `src/acsdg_c2/test/test_classifier_stub.py` | Unit tests for Phase-1 stub posterior |
| `src/acsdg_c2/test/test_dispatcher.py` | Unit tests for `EngagementOrder` construction |
| `src/acsdg_c2/test/test_c2_integration.py` | Smoke test: full pipeline produces same orders as legacy code on a fixed scenario |

### Files to modify

| Path | What changes |
|---|---|
| `src/acsdg_c2/setup.py` | Replace `packages=[package_name]` with `packages=find_packages(exclude=['test'])` so the new sub-packages are installed |
| `src/acsdg_c2/CMakeLists.txt` | Verify `ament_python_install_package(${PROJECT_NAME})` already installs sub-packages; if not, adjust |
| `src/acsdg_c2/acsdg_c2/c2_engine_node.py` | Replace inline scoring + Hungarian with calls to the new modules; instantiate 4 `Anvil` weapons; behavior unchanged |

### Files NOT touched

- `src/acsdg_c2/src/interceptor_controller_node.cpp` — unchanged. Anvil's `dispatch()` produces the same `EngagementOrder` messages this controller already consumes.
- `src/acsdg_c2/acsdg_c2/interceptor_manager_node.py` — unchanged.
- `src/acsdg_c2/acsdg_c2/mission_manager_node.py` — unchanged.
- `src/acsdg_c2/acsdg_c2/hungarian.py` — kept as-is; `assignment/solver.py` wraps it.
- All sensor / Gazebo / bringup packages — unchanged.

---

## Pre-Task: Verify baseline runs

Before any refactor, confirm the existing demo works on this machine. This is the regression target everything else compares against.

- [ ] **Step 1: Build the workspace fresh**

```bash
cd ~/acsdg_ws && colcon build --symlink-install
```

Expected: build succeeds for all packages with no errors.

- [ ] **Step 2: Run the existing demo and trigger one wave**

In one terminal:

```bash
cd ~/acsdg_ws && source install/setup.bash
LIBGL_ALWAYS_SOFTWARE=1 ros2 launch acsdg_bringup acsdg_full.launch.py headless:=false
```

In a second terminal (after sim is up and all four enemies are hovering):

```bash
cd ~/acsdg_ws && source install/setup.bash
ros2 topic pub --times 5 -r 0.5 /mission/wave_trigger std_msgs/Bool '{data: true}'
```

Expected: console shows `Mission active — engagement enabled`, four `EngagementOrder` log lines from c2_engine, four interceptors fly to targets, four `Interceptor #N: NEUTRALISED target #M at range=R.RRm` messages with R between 6.81 and 8.0.

- [ ] **Step 3: Record the baseline kill ranges**

```bash
echo "Baseline kill ranges (Phase 1 regression target):" > /tmp/phase1_baseline.txt
grep "NEUTRALISED" /tmp/acsdg_sim.log | tail -4 >> /tmp/phase1_baseline.txt
cat /tmp/phase1_baseline.txt
```

Expected: 4 lines, kill ranges all in [6.81, 8.0]. Save this file — Task 9 compares against it.

- [ ] **Step 4: Stop the sim cleanly**

Ctrl-C the launch terminal. Verify Gazebo and all ROS nodes exit.

---

## Task 1: Domain types — `TargetClass`, `EngagementEnvelope`, `Track`, `WeaponState`

**Why this comes first:** every other module imports these. Define them once, in one place.

**Files:**
- Create: `src/acsdg_c2/acsdg_c2/weapons/__init__.py`
- Create: `src/acsdg_c2/acsdg_c2/weapons/types.py`
- Create: `src/acsdg_c2/test/__init__.py` (empty)
- Create: `src/acsdg_c2/test/test_weapons_types.py`

- [ ] **Step 1: Write the failing test**

Create `src/acsdg_c2/test/test_weapons_types.py`:

```python
"""Tests for domain types used by the weapons / cost-function modules."""

import math
import pytest

from acsdg_c2.weapons.types import (
    TargetClass,
    EngagementEnvelope,
    Track,
    WeaponState,
)


def test_target_class_enum_has_four_phase_taxonomy_classes():
    assert {c.name for c in TargetClass} == {
        "SMALL_QUAD",
        "GROUP_1_FIXED_WING",
        "GROUP_3_LOITERING",
        "SHAHED_CLASS",
    }


def test_engagement_envelope_in_range_within_bounds():
    env = EngagementEnvelope(
        min_range=100.0, max_range=1500.0,
        min_alt=0.0, max_alt=1000.0,
        max_closing_speed=80.0,
    )
    assert env.contains(range_m=500.0, alt_m=200.0) is True


def test_engagement_envelope_out_of_range_too_far():
    env = EngagementEnvelope(100.0, 1500.0, 0.0, 1000.0, 80.0)
    assert env.contains(range_m=2000.0, alt_m=200.0) is False


def test_engagement_envelope_out_of_range_too_low_alt():
    env = EngagementEnvelope(100.0, 1500.0, 50.0, 1000.0, 80.0)
    assert env.contains(range_m=500.0, alt_m=10.0) is False


def test_track_distance_to_origin():
    t = Track(
        track_id=7,
        position=(30.0, 40.0, 0.0),
        velocity=(0.0, 0.0, 0.0),
        threat_score=0.5,
        state="DETECTED",
    )
    assert math.isclose(t.range_from(0.0, 0.0, 0.0), 50.0)


def test_weapon_state_marks_unavailable_when_not_idle():
    s = WeaponState(
        weapon_id="anvil_1",
        weapon_type="kinetic_quad",
        position=(0.0, 0.0, 0.0),
        available=False,
        ammo_remaining=None,
    )
    assert s.available is False
    assert s.ammo_remaining is None
```

- [ ] **Step 2: Create the empty test marker**

```bash
touch ~/acsdg_ws/src/acsdg_c2/test/__init__.py
```

- [ ] **Step 3: Run the test to verify it fails**

```bash
cd ~/acsdg_ws && colcon build --symlink-install --packages-select acsdg_c2 \
  && source install/setup.bash \
  && cd src/acsdg_c2 && python3 -m pytest test/test_weapons_types.py -v
```

Expected: import error — module `acsdg_c2.weapons.types` doesn't exist yet.

- [ ] **Step 4: Implement `weapons/types.py`**

Create `src/acsdg_c2/acsdg_c2/weapons/types.py`:

```python
"""Domain types shared by weapons, classifier, cost_function, dispatcher.

These are pure data structures with no ROS or Gazebo dependencies, so they can
be unit-tested without a running ROS graph.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from enum import Enum, auto
from typing import Optional, Tuple


class TargetClass(Enum):
    """The 4 threat classes from the design spec, §7."""
    SMALL_QUAD = auto()
    GROUP_1_FIXED_WING = auto()
    GROUP_3_LOITERING = auto()
    SHAHED_CLASS = auto()


@dataclass(frozen=True)
class EngagementEnvelope:
    """Per-weapon spatial envelope — within these bounds, can_engage may return True."""

    min_range: float       # m, slant range from weapon to target
    max_range: float       # m
    min_alt: float         # m, target altitude AGL
    max_alt: float         # m
    max_closing_speed: float  # m/s, weapon's own max airspeed (or muzzle velocity for guns)

    def contains(self, *, range_m: float, alt_m: float) -> bool:
        return (self.min_range <= range_m <= self.max_range
                and self.min_alt <= alt_m <= self.max_alt)


@dataclass(frozen=True)
class Track:
    """Snapshot of a fused target track at one cost-function evaluation tick."""

    track_id: int
    position: Tuple[float, float, float]   # (x, y, z) in world frame
    velocity: Tuple[float, float, float]   # (vx, vy, vz) in world frame
    threat_score: float                    # [0, 1] from threat_priority module
    state: str                             # FusedTarget.state ("DETECTED", "TARGETED", ...)

    def range_from(self, x: float, y: float, z: float) -> float:
        dx = self.position[0] - x
        dy = self.position[1] - y
        dz = self.position[2] - z
        return math.sqrt(dx * dx + dy * dy + dz * dz)


@dataclass
class WeaponState:
    """Snapshot of a weapon's runtime state — used for visualization and AI decisions."""

    weapon_id: str
    weapon_type: str           # "kinetic_quad" / "frag_jet" / "net_octo" / "gun_turret"
    position: Tuple[float, float, float]
    available: bool
    ammo_remaining: Optional[int]   # None for one-shot weapons, count for multi-shot
```

- [ ] **Step 5: Create the package init**

Create `src/acsdg_c2/acsdg_c2/weapons/__init__.py`:

```python
"""Weapon abstractions for the ACSDG C2 layer.

Phase 1 only exports the domain types and the abstract base class;
later phases add concrete WeaponSystem subclasses (Anvil, Coyote, etc.).
"""

from acsdg_c2.weapons.types import (
    TargetClass,
    EngagementEnvelope,
    Track,
    WeaponState,
)

__all__ = ["TargetClass", "EngagementEnvelope", "Track", "WeaponState"]
```

- [ ] **Step 6: Update `setup.py` to install sub-packages**

Modify `src/acsdg_c2/setup.py` from:

```python
from setuptools import setup
package_name = 'acsdg_c2'
setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    ...
)
```

to:

```python
from setuptools import setup, find_packages
package_name = 'acsdg_c2'
setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test', 'test.*']),
    ...
)
```

- [ ] **Step 7: Rebuild and run the test to verify it passes**

```bash
cd ~/acsdg_ws && colcon build --symlink-install --packages-select acsdg_c2 \
  && source install/setup.bash \
  && cd src/acsdg_c2 && python3 -m pytest test/test_weapons_types.py -v
```

Expected: 6 tests pass.

- [ ] **Step 8: Commit**

```bash
cd ~/acsdg_ws && git -C src/acsdg_c2 status   # shows new files (workspace not under git, so this may say "not a repo")
```

Workspace is not under git per the spec. Skip the commit; instead, append a one-line note to `~/acsdg_ws/HANDOFF.md` under a new "Phase 1 progress" section:

```bash
echo "" >> ~/acsdg_ws/HANDOFF.md
echo "## Phase 1 progress (2026-05-02)" >> ~/acsdg_ws/HANDOFF.md
echo "" >> ~/acsdg_ws/HANDOFF.md
echo "- Task 1: domain types in \`acsdg_c2/weapons/types.py\` — TargetClass, EngagementEnvelope, Track, WeaponState. 6 unit tests passing." >> ~/acsdg_ws/HANDOFF.md
```

(All subsequent task commits become HANDOFF.md append-only entries until the user runs `git init`.)

---

## Task 2: `WeaponSystem` ABC and `Anvil` concrete class

**Files:**
- Create: `src/acsdg_c2/acsdg_c2/weapons/base.py`
- Create: `src/acsdg_c2/acsdg_c2/weapons/anvil.py`
- Modify: `src/acsdg_c2/acsdg_c2/weapons/__init__.py` (add re-exports)
- Create: `src/acsdg_c2/test/test_weapon_anvil.py`

- [ ] **Step 1: Write the failing test**

Create `src/acsdg_c2/test/test_weapon_anvil.py`:

```python
"""Tests for Anvil — the Phase-1 kinetic-quad weapon system."""

import pytest

from acsdg_c2.weapons.types import TargetClass, Track
from acsdg_c2.weapons.anvil import Anvil


def make_anvil(weapon_id: str = "anvil_0", home: tuple = (0.0, 0.0, 20.0)) -> Anvil:
    return Anvil(weapon_id=weapon_id, home_position=home)


def make_track(track_id: int = 1, pos: tuple = (100.0, 0.0, 50.0),
               vel: tuple = (-5.0, 0.0, 0.0)) -> Track:
    return Track(track_id=track_id, position=pos, velocity=vel,
                 threat_score=0.7, state="DETECTED")


def test_anvil_weapon_type_is_kinetic_quad():
    a = make_anvil()
    assert a.weapon_type == "kinetic_quad"


def test_anvil_pkill_table_matches_spec():
    a = make_anvil()
    assert a.pkill(TargetClass.SMALL_QUAD) == pytest.approx(0.95)
    assert a.pkill(TargetClass.GROUP_1_FIXED_WING) == pytest.approx(0.85)
    assert a.pkill(TargetClass.GROUP_3_LOITERING) == pytest.approx(0.70)
    assert a.pkill(TargetClass.SHAHED_CLASS) == pytest.approx(0.40)


def test_anvil_engagement_envelope_matches_spec():
    a = make_anvil()
    env = a.engagement_envelope()
    assert env.max_range == pytest.approx(1500.0)   # 1.5 km from spec §6.1
    assert env.max_closing_speed == pytest.approx(45.0)   # m/s from spec §6.1


def test_anvil_can_engage_within_envelope_when_available():
    a = make_anvil(home=(0.0, 0.0, 20.0))
    track = make_track(pos=(500.0, 0.0, 50.0))   # 500 m horizontal, 30 m up
    assert a.can_engage(track) is True


def test_anvil_cannot_engage_out_of_range():
    a = make_anvil(home=(0.0, 0.0, 20.0))
    track = make_track(pos=(2000.0, 0.0, 50.0))  # > 1.5 km
    assert a.can_engage(track) is False


def test_anvil_cannot_engage_when_unavailable():
    a = make_anvil()
    a.mark_engaged(target_id=42)
    track = make_track(pos=(500.0, 0.0, 50.0))
    assert a.is_available() is False
    assert a.can_engage(track) is False


def test_anvil_dispatch_records_engagement_and_returns_order_payload():
    a = make_anvil()
    track = make_track(track_id=99, pos=(500.0, 0.0, 50.0))
    payload = a.dispatch(track)
    assert payload["target_id"] == 99
    assert payload["weapon_id"] == "anvil_0"
    assert a.is_available() is False
    assert a.engaged_target_id() == 99


def test_anvil_engaged_target_id_resets_on_mark_idle():
    a = make_anvil()
    a.mark_engaged(target_id=7)
    assert a.engaged_target_id() == 7
    a.mark_idle()
    assert a.engaged_target_id() is None


def test_anvil_resource_cost_matches_spec():
    a = make_anvil()
    assert a.resource_cost() == pytest.approx(0.05)


def test_anvil_time_to_intercept_uses_max_speed_and_closing_rate():
    a = make_anvil(home=(0.0, 0.0, 20.0))
    # Target 500 m away, closing at 5 m/s along the line of sight
    track = make_track(pos=(500.0, 0.0, 20.0), vel=(-5.0, 0.0, 0.0))
    # eff_speed = max_speed (45) + closing (5) = 50; t = 500 / 50 = 10 s
    assert a.time_to_intercept(track) == pytest.approx(10.0, rel=0.01)
```

- [ ] **Step 2: Run the test to verify it fails**

```bash
cd ~/acsdg_ws && colcon build --symlink-install --packages-select acsdg_c2 \
  && source install/setup.bash \
  && cd src/acsdg_c2 && python3 -m pytest test/test_weapon_anvil.py -v
```

Expected: import error — `acsdg_c2.weapons.anvil` doesn't exist.

- [ ] **Step 3: Implement the `WeaponSystem` ABC**

Create `src/acsdg_c2/acsdg_c2/weapons/base.py`:

```python
"""WeaponSystem — the abstract base class every weapon implements.

The C2 engine treats the weapon registry uniformly through this interface;
each concrete weapon (Anvil, Coyote, DroneHunter, Skyranger) supplies its own
engagement envelope, Pkill curve, resource model, and dispatch behavior.
"""

from __future__ import annotations

from abc import ABC, abstractmethod
from typing import Any, Dict

from acsdg_c2.weapons.types import (
    TargetClass,
    EngagementEnvelope,
    Track,
    WeaponState,
)


class WeaponSystem(ABC):

    weapon_id: str       # unique e.g. "anvil_0", "coyote_0"
    weapon_type: str     # "kinetic_quad" / "frag_jet" / "net_octo" / "gun_turret"

    # ── Capability queries ───────────────────────────────────────────────

    @abstractmethod
    def is_available(self) -> bool:
        """True if this weapon has resources and is not currently engaged."""

    @abstractmethod
    def engagement_envelope(self) -> EngagementEnvelope:
        """Spatial bounds within which can_engage may return True."""

    @abstractmethod
    def pkill(self, target_class: TargetClass) -> float:
        """Per-class kill probability (0.0–1.0). Lookup from spec table."""

    @abstractmethod
    def time_to_intercept(self, track: Track) -> float:
        """Geometric ETA in seconds. Drone: lead-pursuit time. Gun: ballistic ToF."""

    @abstractmethod
    def resource_cost(self) -> float:
        """Normalized $cost or unit-deplete penalty per dispatch."""

    @abstractmethod
    def can_engage(self, track: Track) -> bool:
        """In envelope AND available AND has ammo (for guns)."""

    # ── Dispatch and state ────────────────────────────────────────────────

    @abstractmethod
    def dispatch(self, track: Track) -> Dict[str, Any]:
        """Issue the engagement command. Decrements resources / sets engaged flag.

        Returns a dict with keys {target_id, weapon_id, priority, ...} that the
        Dispatcher converts into an EngagementOrder ROS message.
        """

    @abstractmethod
    def state(self) -> WeaponState:
        """Snapshot for visualization and AI decisions."""
```

- [ ] **Step 4: Implement `Anvil`**

Create `src/acsdg_c2/acsdg_c2/weapons/anvil.py`:

```python
"""Anduril Anvil — kinetic-quadcopter kamikaze interceptor (DoD Group 1/2).

Spec values from §6.1 of the design doc:
  max_speed = 45 m/s, max_range = 1.5 km (LOS-tethered),
  resource_cost = 0.05, one-shot, collision kill at <8 m.
Pkill table:
  small-quad 0.95, group-1-fixed-wing 0.85, group-3-loitering 0.70, shahed 0.40.
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


_ANVIL_PKILL = {
    TargetClass.SMALL_QUAD: 0.95,
    TargetClass.GROUP_1_FIXED_WING: 0.85,
    TargetClass.GROUP_3_LOITERING: 0.70,
    TargetClass.SHAHED_CLASS: 0.40,
}

_ANVIL_MAX_SPEED = 45.0       # m/s — spec §6.1
_ANVIL_MAX_RANGE = 1500.0     # m — Phase-1 sim value, spec §6.1
_ANVIL_MIN_RANGE = 5.0        # m — collision-kill close-in
_ANVIL_RESOURCE_COST = 0.05


class Anvil(WeaponSystem):

    weapon_type = "kinetic_quad"

    def __init__(
        self,
        weapon_id: str,
        home_position: Tuple[float, float, float],
    ) -> None:
        self.weapon_id = weapon_id
        self._home = home_position
        self._current_position = home_position
        self._engaged_target_id: Optional[int] = None

    # ── External state hooks (called by c2_engine on InterceptorState updates) ─

    def update_position(self, position: Tuple[float, float, float]) -> None:
        self._current_position = position

    def mark_engaged(self, target_id: int) -> None:
        self._engaged_target_id = target_id

    def mark_idle(self) -> None:
        self._engaged_target_id = None

    def engaged_target_id(self):  # Optional[int]
        """The track id this weapon is currently engaging, or None if available."""
        return self._engaged_target_id

    # ── WeaponSystem interface ────────────────────────────────────────────

    def is_available(self) -> bool:
        return self._engaged_target_id is None

    def engagement_envelope(self) -> EngagementEnvelope:
        return EngagementEnvelope(
            min_range=_ANVIL_MIN_RANGE,
            max_range=_ANVIL_MAX_RANGE,
            min_alt=0.0,
            max_alt=1066.0,        # DoD Group 2 ceiling (spec §6.1)
            max_closing_speed=_ANVIL_MAX_SPEED,
        )

    def pkill(self, target_class: TargetClass) -> float:
        return _ANVIL_PKILL[target_class]

    def time_to_intercept(self, track: Track) -> float:
        d = track.range_from(*self._current_position)
        if d <= 0.0:
            return 0.0
        # closing rate along line of sight (positive = inbound)
        ux = (track.position[0] - self._current_position[0]) / d
        uy = (track.position[1] - self._current_position[1]) / d
        uz = (track.position[2] - self._current_position[2]) / d
        # target velocity projected toward the weapon (inbound = negative)
        v_proj = (track.velocity[0] * (-ux)
                  + track.velocity[1] * (-uy)
                  + track.velocity[2] * (-uz))
        eff_speed = max(1.0, _ANVIL_MAX_SPEED + v_proj)
        return d / eff_speed

    def resource_cost(self) -> float:
        return _ANVIL_RESOURCE_COST

    def can_engage(self, track: Track) -> bool:
        if not self.is_available():
            return False
        d = track.range_from(*self._current_position)
        return self.engagement_envelope().contains(
            range_m=d, alt_m=track.position[2])

    def dispatch(self, track: Track) -> Dict[str, Any]:
        self.mark_engaged(track.track_id)
        return {
            "target_id": track.track_id,
            "weapon_id": self.weapon_id,
            "priority": int(min(255, max(0, round(track.threat_score * 255)))),
        }

    def state(self) -> WeaponState:
        return WeaponState(
            weapon_id=self.weapon_id,
            weapon_type=self.weapon_type,
            position=self._current_position,
            available=self.is_available(),
            ammo_remaining=None,    # Anvil is one-shot
        )
```

- [ ] **Step 5: Update `weapons/__init__.py` to re-export the new symbols**

Modify `src/acsdg_c2/acsdg_c2/weapons/__init__.py`:

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

__all__ = [
    "TargetClass", "EngagementEnvelope", "Track", "WeaponState",
    "WeaponSystem", "Anvil",
]
```

- [ ] **Step 6: Run the tests to verify they pass**

```bash
cd ~/acsdg_ws && colcon build --symlink-install --packages-select acsdg_c2 \
  && source install/setup.bash \
  && cd src/acsdg_c2 && python3 -m pytest test/test_weapon_anvil.py -v
```

Expected: 10 tests pass.

- [ ] **Step 7: Note the progress**

```bash
echo "- Task 2: \`Anvil\` weapon class with full Pkill table, engagement envelope, and engagement-state hooks. 10 unit tests passing." >> ~/acsdg_ws/HANDOFF.md
```

---

## Task 3: Assignment module — `assign(cost_matrix)`

**Files:**
- Create: `src/acsdg_c2/acsdg_c2/assignment/__init__.py`
- Create: `src/acsdg_c2/acsdg_c2/assignment/solver.py`
- Create: `src/acsdg_c2/test/test_assignment.py`

This is a thin wrapper around the existing `acsdg_c2.hungarian.hungarian`. We wrap (rather than rename) so future phases can swap in a different solver without touching call sites.

- [ ] **Step 1: Write the failing test**

Create `src/acsdg_c2/test/test_assignment.py`:

```python
"""Tests for the assignment module — currently a thin wrapper on hungarian()."""

import math

from acsdg_c2.assignment import assign


def test_assign_3x3_minimum_cost_matching():
    cost = [
        [4.0, 1.0, 3.0],
        [2.0, 0.0, 5.0],
        [3.0, 2.0, 2.0],
    ]
    result = assign(cost)
    # Minimum cost matching is (0,1)+(1,0)+(2,2) = 1+2+2 = 5
    assert sorted(result) == [(0, 1), (1, 0), (2, 2)]


def test_assign_handles_rectangular_matrix_more_agents_than_jobs():
    cost = [
        [1.0, 5.0],
        [3.0, 2.0],
        [4.0, 4.0],
    ]
    result = assign(cost)
    # Two jobs, three agents — exactly two pairs, both within bounds
    assert len(result) == 2
    job_indices = sorted(j for _, j in result)
    assert job_indices == [0, 1]


def test_assign_empty_matrix_returns_empty():
    assert assign([]) == []


def test_assign_single_cell():
    assert assign([[1.5]]) == [(0, 0)]
```

- [ ] **Step 2: Run the test to verify it fails**

```bash
cd ~/acsdg_ws && colcon build --symlink-install --packages-select acsdg_c2 \
  && source install/setup.bash \
  && cd src/acsdg_c2 && python3 -m pytest test/test_assignment.py -v
```

Expected: import error — `acsdg_c2.assignment` doesn't exist.

- [ ] **Step 3: Implement the wrapper**

Create `src/acsdg_c2/acsdg_c2/assignment/solver.py`:

```python
"""Assignment solver wrapper.

Phase 1 delegates to the existing pure-Python Hungarian implementation in
acsdg_c2.hungarian. Later phases may swap in scipy.optimize.linear_sum_assignment
or a custom solver without callers having to know.
"""

from __future__ import annotations

from typing import List, Sequence, Tuple

from acsdg_c2.hungarian import hungarian


def assign(cost_matrix: Sequence[Sequence[float]]) -> List[Tuple[int, int]]:
    """Return the min-cost (agent, job) assignment.

    Length is min(rows, cols). Indices refer to the original matrix dimensions.
    Empty input returns an empty list.
    """
    return hungarian([list(row) for row in cost_matrix])
```

Create `src/acsdg_c2/acsdg_c2/assignment/__init__.py`:

```python
"""Assignment-solver module for the C2 engine."""

from acsdg_c2.assignment.solver import assign

__all__ = ["assign"]
```

- [ ] **Step 4: Run the tests to verify they pass**

```bash
cd ~/acsdg_ws && colcon build --symlink-install --packages-select acsdg_c2 \
  && source install/setup.bash \
  && cd src/acsdg_c2 && python3 -m pytest test/test_assignment.py -v
```

Expected: 4 tests pass.

- [ ] **Step 5: Note the progress**

```bash
echo "- Task 3: \`assignment.assign()\` wrapper around existing Hungarian. 4 unit tests passing." >> ~/acsdg_ws/HANDOFF.md
```

---

## Task 4: Threat-priority extraction — `score_threat(track)`

**Why a separate module:** the current `_score()` function inside c2_engine_node.py is pure (no ROS deps), so it lifts out cleanly and is unit-testable. Locking down its behavior with tests now makes the c2_engine refactor (Task 8) safe.

**Files:**
- Create: `src/acsdg_c2/acsdg_c2/cost_function/__init__.py`
- Create: `src/acsdg_c2/acsdg_c2/cost_function/threat_priority.py`
- Create: `src/acsdg_c2/test/test_threat_priority.py`

- [ ] **Step 1: Write the failing test**

Create `src/acsdg_c2/test/test_threat_priority.py`:

```python
"""Tests for threat priority scoring — must match the legacy _score()."""

import math
import pytest

from acsdg_c2.cost_function.threat_priority import score_threat
from acsdg_c2.weapons.types import Track


def make_track(pos, vel, threat_score=0.0, state="DETECTED", track_id=1):
    return Track(track_id=track_id, position=pos, velocity=vel,
                 threat_score=threat_score, state=state)


def test_score_at_origin_with_no_velocity_is_proximity_only():
    """A target at the origin gets full proximity score and no heading/speed."""
    t = make_track(pos=(0.0, 0.0, 0.0), vel=(0.0, 0.0, 0.0))
    # proximity = 1.0, heading = 0, speed = 0 → 0.5*1 + 0.3*0 + 0.2*0 = 0.5
    assert score_threat(t) == pytest.approx(0.5)


def test_score_far_target_zero_proximity():
    """Beyond MAX_RANGE the proximity term saturates at 0."""
    t = make_track(pos=(500.0, 0.0, 0.0), vel=(0.0, 0.0, 0.0))
    # MAX_RANGE = 400, so proximity = 0; heading = 0; speed = 0 → 0
    assert score_threat(t) == pytest.approx(0.0)


def test_score_inbound_target_credits_heading():
    """Target heading toward the origin gets full heading bonus."""
    # at (200, 0, 0) moving at (-10, 0, 0): heading vector exactly aligned
    t = make_track(pos=(200.0, 0.0, 0.0), vel=(-10.0, 0.0, 0.0))
    # proximity = 1 - 200/400 = 0.5
    # heading = 1.0 (perfect inbound)
    # speed_s = min(1, 10/20) = 0.5
    # → 0.5*0.5 + 0.3*1.0 + 0.2*0.5 = 0.25 + 0.30 + 0.10 = 0.65
    assert score_threat(t) == pytest.approx(0.65, rel=1e-3)


def test_score_outbound_target_zero_heading():
    """Target moving away contributes no heading score."""
    t = make_track(pos=(200.0, 0.0, 0.0), vel=(10.0, 0.0, 0.0))
    # proximity = 0.5, heading = 0 (max(0, ...) clips), speed_s = 0.5
    # → 0.25 + 0 + 0.10 = 0.35
    assert score_threat(t) == pytest.approx(0.35, rel=1e-3)


def test_score_speed_saturates_at_max_speed():
    """Speed term clamps at 1.0 once track speed exceeds MAX_SPEED."""
    t = make_track(pos=(200.0, 0.0, 0.0), vel=(-100.0, 0.0, 0.0))
    # proximity = 0.5, heading = 1, speed_s = 1
    # → 0.25 + 0.30 + 0.20 = 0.75
    assert score_threat(t) == pytest.approx(0.75, rel=1e-3)
```

- [ ] **Step 2: Run the test to verify it fails**

```bash
cd ~/acsdg_ws && colcon build --symlink-install --packages-select acsdg_c2 \
  && source install/setup.bash \
  && cd src/acsdg_c2 && python3 -m pytest test/test_threat_priority.py -v
```

Expected: import error — module doesn't exist.

- [ ] **Step 3: Extract `_score` into the new module**

Create `src/acsdg_c2/acsdg_c2/cost_function/threat_priority.py`:

```python
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
```

Create `src/acsdg_c2/acsdg_c2/cost_function/__init__.py`:

```python
"""Cost-function module for C2 weapon-target pairing."""

from acsdg_c2.cost_function.threat_priority import score_threat

__all__ = ["score_threat"]
```

- [ ] **Step 4: Run the tests to verify they pass**

```bash
cd ~/acsdg_ws && colcon build --symlink-install --packages-select acsdg_c2 \
  && source install/setup.bash \
  && cd src/acsdg_c2 && python3 -m pytest test/test_threat_priority.py -v
```

Expected: 5 tests pass.

- [ ] **Step 5: Note the progress**

```bash
echo "- Task 4: \`cost_function.score_threat()\` extracted from legacy c2_engine. 5 unit tests passing." >> ~/acsdg_ws/HANDOFF.md
```

---

## Task 5: Cost-matrix builder — `build_cost_matrix(weapons, tracks)`

**Files:**
- Create: `src/acsdg_c2/acsdg_c2/cost_function/expected_utility.py`
- Modify: `src/acsdg_c2/acsdg_c2/cost_function/__init__.py` (add re-export)
- Create: `src/acsdg_c2/test/test_cost_function.py`

**Phase 1 contract:** the cost matrix entry for (weapon_i, track_j) is the time-to-intercept (seconds) — the same quantity the legacy c2_engine put into its inline matrix. Phase 5 will replace this body with the full expected-utility math from spec §11.

- [ ] **Step 1: Write the failing test**

Create `src/acsdg_c2/test/test_cost_function.py`:

```python
"""Tests for the Phase-1 cost-matrix builder (time-to-intercept geometry)."""

import math
import pytest

from acsdg_c2.cost_function.expected_utility import build_cost_matrix
from acsdg_c2.weapons.anvil import Anvil
from acsdg_c2.weapons.types import Track


def make_track(track_id, pos, vel=(0.0, 0.0, 0.0), threat=0.5):
    return Track(track_id=track_id, position=pos, velocity=vel,
                 threat_score=threat, state="DETECTED")


def test_cost_matrix_shape_matches_inputs():
    weapons = [Anvil("anvil_0", (0.0, 0.0, 20.0)),
               Anvil("anvil_1", (1000.0, 0.0, 20.0))]
    tracks = [make_track(1, (500.0, 0.0, 50.0)),
              make_track(2, (1500.0, 0.0, 50.0)),
              make_track(3, (300.0, 0.0, 50.0))]
    matrix = build_cost_matrix(weapons, tracks)
    assert len(matrix) == 2
    assert all(len(row) == 3 for row in matrix)


def test_cost_matrix_uses_time_to_intercept():
    a = Anvil("anvil_0", (0.0, 0.0, 0.0))
    track = make_track(1, (450.0, 0.0, 0.0), vel=(0.0, 0.0, 0.0))
    matrix = build_cost_matrix([a], [track])
    # eff_speed = 45 m/s (max_speed) + 0 closing = 45; ToI = 450/45 = 10 s
    assert matrix[0][0] == pytest.approx(10.0, rel=0.01)


def test_cost_matrix_marks_unavailable_weapon_as_infinite():
    a = Anvil("anvil_0", (0.0, 0.0, 0.0))
    a.mark_engaged(target_id=99)   # weapon already busy
    track = make_track(1, (500.0, 0.0, 0.0))
    matrix = build_cost_matrix([a], [track])
    assert math.isinf(matrix[0][0])


def test_cost_matrix_marks_out_of_envelope_as_infinite():
    a = Anvil("anvil_0", (0.0, 0.0, 0.0))
    track = make_track(1, (5000.0, 0.0, 0.0))   # > 1.5 km Anvil max range
    matrix = build_cost_matrix([a], [track])
    assert math.isinf(matrix[0][0])


def test_empty_inputs_return_empty_matrix():
    assert build_cost_matrix([], []) == []
    assert build_cost_matrix([Anvil("a", (0, 0, 0))], []) == [[]]
    assert build_cost_matrix([], [make_track(1, (0, 0, 0))]) == []
```

- [ ] **Step 2: Run the test to verify it fails**

```bash
cd ~/acsdg_ws && colcon build --symlink-install --packages-select acsdg_c2 \
  && source install/setup.bash \
  && cd src/acsdg_c2 && python3 -m pytest test/test_cost_function.py -v
```

Expected: import error.

- [ ] **Step 3: Implement the builder**

Create `src/acsdg_c2/acsdg_c2/cost_function/expected_utility.py`:

```python
"""Cost-matrix builder for C2 weapon-target pairing.

Phase 1 entry: cost = time_to_intercept(weapon, track), with +inf for
out-of-envelope or unavailable pairs. Phase 5 will replace this body with
the full expected-utility math (Pkill × class posterior, resource cost,
time-to-intercept, threat priority) per spec §11.
"""

from __future__ import annotations

import math
from typing import List, Sequence

from acsdg_c2.weapons.base import WeaponSystem
from acsdg_c2.weapons.types import Track


def build_cost_matrix(
    weapons: Sequence[WeaponSystem],
    tracks: Sequence[Track],
) -> List[List[float]]:
    """Return an m×n cost matrix where m=len(weapons), n=len(tracks).

    Cost is approximate time-to-intercept in seconds; +inf flags forbidden
    assignments (weapon unavailable or target outside engagement envelope).
    """
    matrix: List[List[float]] = []
    for w in weapons:
        row: List[float] = []
        for t in tracks:
            if not w.can_engage(t):
                row.append(math.inf)
            else:
                row.append(w.time_to_intercept(t))
        matrix.append(row)
    return matrix
```

- [ ] **Step 4: Update `cost_function/__init__.py`**

Modify `src/acsdg_c2/acsdg_c2/cost_function/__init__.py`:

```python
"""Cost-function module for C2 weapon-target pairing."""

from acsdg_c2.cost_function.threat_priority import score_threat
from acsdg_c2.cost_function.expected_utility import build_cost_matrix

__all__ = ["score_threat", "build_cost_matrix"]
```

- [ ] **Step 5: Run the tests to verify they pass**

```bash
cd ~/acsdg_ws && colcon build --symlink-install --packages-select acsdg_c2 \
  && source install/setup.bash \
  && cd src/acsdg_c2 && python3 -m pytest test/test_cost_function.py -v
```

Expected: 5 tests pass.

- [ ] **Step 6: Note the progress**

```bash
echo "- Task 5: \`cost_function.build_cost_matrix()\` (Phase-1 time-to-intercept). 5 unit tests passing." >> ~/acsdg_ws/HANDOFF.md
```

---

## Task 6: Classifier stub — `Classifier.posterior(track)`

**Phase 1 behavior:** every track returns a posterior with all probability mass on `SMALL_QUAD`. This satisfies the interface for downstream modules without doing real classification work — that's Phase 4.

**Files:**
- Create: `src/acsdg_c2/acsdg_c2/classifier/__init__.py`
- Create: `src/acsdg_c2/acsdg_c2/classifier/bayesian.py`
- Create: `src/acsdg_c2/test/test_classifier_stub.py`

- [ ] **Step 1: Write the failing test**

Create `src/acsdg_c2/test/test_classifier_stub.py`:

```python
"""Tests for the Phase-1 classifier stub."""

import pytest

from acsdg_c2.classifier import Classifier
from acsdg_c2.weapons.types import TargetClass, Track


def make_track(track_id=1, pos=(0.0, 0.0, 0.0), vel=(0.0, 0.0, 0.0)):
    return Track(track_id=track_id, position=pos, velocity=vel,
                 threat_score=0.0, state="DETECTED")


def test_phase1_posterior_returns_full_mass_on_small_quad():
    c = Classifier()
    posterior = c.posterior(make_track())
    assert posterior[TargetClass.SMALL_QUAD] == pytest.approx(1.0)
    for cls in TargetClass:
        if cls is not TargetClass.SMALL_QUAD:
            assert posterior[cls] == pytest.approx(0.0)


def test_posterior_sums_to_one_for_any_track():
    c = Classifier()
    p = c.posterior(make_track(track_id=42, pos=(100.0, 200.0, 50.0)))
    assert sum(p.values()) == pytest.approx(1.0)


def test_posterior_keys_are_all_four_target_classes():
    c = Classifier()
    p = c.posterior(make_track())
    assert set(p.keys()) == set(TargetClass)
```

- [ ] **Step 2: Run the test to verify it fails**

```bash
cd ~/acsdg_ws && colcon build --symlink-install --packages-select acsdg_c2 \
  && source install/setup.bash \
  && cd src/acsdg_c2 && python3 -m pytest test/test_classifier_stub.py -v
```

Expected: import error.

- [ ] **Step 3: Implement the stub**

Create `src/acsdg_c2/acsdg_c2/classifier/bayesian.py`:

```python
"""Bayesian classifier — Phase 1 stub.

The Phase-1 stub returns a degenerate posterior with all mass on SMALL_QUAD.
Phase 4 replaces the body with a real naive-Bayes Gaussian classifier driven
by per-track size/speed/RCS/IR observations. The interface stays the same.
"""

from __future__ import annotations

from typing import Dict

from acsdg_c2.weapons.types import TargetClass, Track


class Classifier:

    def posterior(self, track: Track) -> Dict[TargetClass, float]:
        """Return P(class | observations) for each TargetClass.

        Phase 1: deterministic — full mass on SMALL_QUAD.
        """
        return {
            TargetClass.SMALL_QUAD: 1.0,
            TargetClass.GROUP_1_FIXED_WING: 0.0,
            TargetClass.GROUP_3_LOITERING: 0.0,
            TargetClass.SHAHED_CLASS: 0.0,
        }
```

Create `src/acsdg_c2/acsdg_c2/classifier/__init__.py`:

```python
"""Classifier module for C2."""

from acsdg_c2.classifier.bayesian import Classifier

__all__ = ["Classifier"]
```

- [ ] **Step 4: Run the tests to verify they pass**

```bash
cd ~/acsdg_ws && colcon build --symlink-install --packages-select acsdg_c2 \
  && source install/setup.bash \
  && cd src/acsdg_c2 && python3 -m pytest test/test_classifier_stub.py -v
```

Expected: 3 tests pass.

- [ ] **Step 5: Note the progress**

```bash
echo "- Task 6: \`classifier.Classifier\` Phase-1 stub. 3 unit tests passing." >> ~/acsdg_ws/HANDOFF.md
```

---

## Task 7: Dispatcher — `Dispatcher.translate(weapon, track)`

**Files:**
- Create: `src/acsdg_c2/acsdg_c2/dispatcher/__init__.py`
- Create: `src/acsdg_c2/acsdg_c2/dispatcher/dispatcher.py`
- Create: `src/acsdg_c2/test/test_dispatcher.py`

**Phase 1 contract:** convert a (weapon, track) pair into a fully-populated `EngagementOrder` ROS message. Caller (c2_engine) is responsible for publishing it.

The legacy c2_engine published `EngagementOrder` with fields {target_id (uint32), interceptor_id (uint32), priority (uint8), issued_at (Time)}. Anvil's id is `"anvil_0"` etc.; the legacy interceptor controllers expect a numeric id 1–4. The dispatcher extracts the numeric tail of the weapon_id (`"anvil_0"` → 1, `"anvil_1"` → 2, …) so existing controllers keep working in Phase 1.

- [ ] **Step 1: Write the failing test**

Create `src/acsdg_c2/test/test_dispatcher.py`:

```python
"""Tests for the Dispatcher translator (weapon + track → EngagementOrder)."""

import pytest

from acsdg_c2.dispatcher import Dispatcher
from acsdg_c2.weapons.anvil import Anvil
from acsdg_c2.weapons.types import Track


def make_track(track_id=7, pos=(500.0, 0.0, 50.0), threat=0.6):
    return Track(track_id=track_id, position=pos, velocity=(0.0, 0.0, 0.0),
                 threat_score=threat, state="DETECTED")


def test_dispatcher_translates_anvil_dispatch_to_engagement_order_payload():
    d = Dispatcher()
    a = Anvil("anvil_0", (0.0, 0.0, 20.0))
    order = d.translate(a, make_track(track_id=7, threat=0.5))
    assert order["target_id"] == 7
    assert order["interceptor_id"] == 1     # "anvil_0" → 1 (numeric tail + 1)
    assert order["priority"] == 127         # round(0.5 * 255) = 127 or 128


def test_dispatcher_handles_anvil_3_index():
    d = Dispatcher()
    a = Anvil("anvil_2", (0.0, 0.0, 20.0))
    order = d.translate(a, make_track())
    assert order["interceptor_id"] == 3


def test_dispatcher_priority_clamps_to_uint8_range():
    d = Dispatcher()
    a = Anvil("anvil_0", (0.0, 0.0, 20.0))
    order = d.translate(a, make_track(threat=2.0))    # > 1.0 corner case
    assert 0 <= order["priority"] <= 255


def test_dispatcher_marks_weapon_engaged():
    d = Dispatcher()
    a = Anvil("anvil_0", (0.0, 0.0, 20.0))
    assert a.is_available() is True
    d.translate(a, make_track(track_id=11))
    assert a.is_available() is False
```

- [ ] **Step 2: Run the test to verify it fails**

```bash
cd ~/acsdg_ws && colcon build --symlink-install --packages-select acsdg_c2 \
  && source install/setup.bash \
  && cd src/acsdg_c2 && python3 -m pytest test/test_dispatcher.py -v
```

Expected: import error.

- [ ] **Step 3: Implement the Dispatcher**

Create `src/acsdg_c2/acsdg_c2/dispatcher/dispatcher.py`:

```python
"""Dispatcher — converts a (weapon, track) decision into an EngagementOrder payload.

Phase 1: every weapon is an Anvil, identified `anvil_0`..`anvil_3`. The legacy
interceptor controller subscribes to /interceptors/unit_{1..4}/state and expects
EngagementOrder.interceptor_id ∈ {1, 2, 3, 4}. We map "anvil_<N>" → N+1 so the
existing C++ controller keeps working without modification.

In Phase 4, when heterogeneous weapons appear, this mapping becomes
weapon-id-aware (each weapon controller gets its own namespace).
"""

from __future__ import annotations

from typing import Any, Dict

from acsdg_c2.weapons.base import WeaponSystem
from acsdg_c2.weapons.types import Track


class Dispatcher:

    def translate(self, weapon: WeaponSystem, track: Track) -> Dict[str, Any]:
        """Issue weapon.dispatch() and return the payload for EngagementOrder."""
        payload = weapon.dispatch(track)
        # Phase 1: extract numeric tail of "anvil_N" and add 1 → unit_{1..4}
        wid = payload.get("weapon_id", "")
        try:
            numeric_tail = int(wid.split("_")[-1])
            interceptor_id = numeric_tail + 1
        except (ValueError, IndexError):
            interceptor_id = 0   # caller treats 0 as "unknown / drop"
        priority = int(min(255, max(0, payload.get("priority", 0))))
        return {
            "target_id": int(payload["target_id"]),
            "interceptor_id": interceptor_id,
            "priority": priority,
        }
```

Create `src/acsdg_c2/acsdg_c2/dispatcher/__init__.py`:

```python
"""Dispatcher module for C2."""

from acsdg_c2.dispatcher.dispatcher import Dispatcher

__all__ = ["Dispatcher"]
```

- [ ] **Step 4: Run the tests to verify they pass**

```bash
cd ~/acsdg_ws && colcon build --symlink-install --packages-select acsdg_c2 \
  && source install/setup.bash \
  && cd src/acsdg_c2 && python3 -m pytest test/test_dispatcher.py -v
```

Expected: 4 tests pass.

- [ ] **Step 5: Note the progress**

```bash
echo "- Task 7: \`dispatcher.Dispatcher.translate()\` — weapon→EngagementOrder. 4 unit tests passing." >> ~/acsdg_ws/HANDOFF.md
```

---

## Task 8: Refactor `c2_engine_node.py` to compose the new modules

**Files:**
- Modify: `src/acsdg_c2/acsdg_c2/c2_engine_node.py`

This is the load-bearing refactor. The strategy is to replace the inline scoring + cost-matrix + Hungarian + EngagementOrder construction with calls into the new modules, while keeping the ROS topic shapes and timing identical. No new behavior.

The refactored file holds:
1. A registry of 4 `Anvil` instances (one per legacy interceptor)
2. The same subscriptions and publishers as before
3. A 10 Hz timer that builds Tracks from FusedTarget JSON, builds the cost matrix via `build_cost_matrix`, runs `assign`, and emits `EngagementOrder` messages via `Dispatcher`

- [ ] **Step 1: Read the current c2_engine_node.py end to end**

```bash
cat ~/acsdg_ws/src/acsdg_c2/acsdg_c2/c2_engine_node.py
```

Confirm you understand the four legacy responsibilities the refactor preserves: (1) score threats, (2) build cost matrix, (3) run Hungarian, (4) publish EngagementOrder + threat scores. The diff in this task is *purely structural* — the produced messages must match.

- [ ] **Step 2: Replace the file with the refactored version**

Overwrite `src/acsdg_c2/acsdg_c2/c2_engine_node.py`:

```python
#!/usr/bin/env python3
"""
c2_engine_node.py — Modular threat scoring + weapon-target assignment.

Phase 1 of the AI-orchestrated heterogeneous-defense upgrade. This node
composes per-module abstractions (classifier, cost_function, assignment,
dispatcher) and a registry of WeaponSystem instances. Phase 1 inventory is
4 Anvil quadcopters, reproducing the pre-refactor demo behavior.

Subscribes
----------
/sensors/fusion/targets    std_msgs/String   JSON array of FusedTargets
/interceptors/{1-4}/state  acsdg_msgs/InterceptorState
/mission/wave_trigger      std_msgs/Bool
/mission/engagement_ack    std_msgs/String   JSON {target_id, outcome}

Publishes
---------
/c2/threat_scores          std_msgs/String   JSON [{id, score, state}, ...]
/c2/engagement_orders      acsdg_msgs/EngagementOrder
"""

import json
from typing import Dict, List

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, String
from acsdg_msgs.msg import EngagementOrder, InterceptorState

from acsdg_c2.assignment import assign
from acsdg_c2.classifier import Classifier
from acsdg_c2.cost_function import build_cost_matrix, score_threat
from acsdg_c2.dispatcher import Dispatcher
from acsdg_c2.weapons import Anvil, Track

NUM_INTERCEPTORS = 4

# Legacy interceptor home positions — controllers know their own homes; the
# C2 engine only needs an initial guess until InterceptorState updates land.
_DEFAULT_HOMES = [
    (-100.0, -100.0, 20.0),
    ( 100.0, -100.0, 20.0),
    ( 100.0,  100.0, 20.0),
    (-100.0,  100.0, 20.0),
]


class C2EngineNode(Node):

    def __init__(self) -> None:
        super().__init__("c2_engine_node")

        # ── Weapons (Phase 1: 4 Anvils) ──────────────────────────────────
        self._weapons: List[Anvil] = [
            Anvil(weapon_id=f"anvil_{i}", home_position=_DEFAULT_HOMES[i])
            for i in range(NUM_INTERCEPTORS)
        ]

        # ── Modules ──────────────────────────────────────────────────────
        self._classifier = Classifier()
        self._dispatcher = Dispatcher()

        # ── State ────────────────────────────────────────────────────────
        self._fused: Dict[int, dict] = {}     # fusion-node JSON dicts keyed by id
        self._mission_active: bool = False

        # ── Subscriptions ────────────────────────────────────────────────
        self.create_subscription(
            String, "/sensors/fusion/targets", self._on_targets, 10)
        self.create_subscription(
            Bool, "/mission/wave_trigger", self._on_wave, 10)
        self.create_subscription(
            String, "/mission/engagement_ack", self._on_engagement_ack, 10)
        for iid in range(1, NUM_INTERCEPTORS + 1):
            self.create_subscription(
                InterceptorState,
                f"/interceptors/unit_{iid}/state",
                self._on_interceptor_state, 10)

        # ── Publishers ───────────────────────────────────────────────────
        self._scores_pub = self.create_publisher(
            String, "/c2/threat_scores", 10)
        self._engage_pub = self.create_publisher(
            EngagementOrder, "/c2/engagement_orders", 10)

        # ── 10 Hz control loop ───────────────────────────────────────────
        self.create_timer(0.1, self._on_timer)
        self.get_logger().info(
            f"C2EngineNode ready (10 Hz, {NUM_INTERCEPTORS} Anvils)")

    # ── Callbacks ────────────────────────────────────────────────────────

    def _on_targets(self, msg: String) -> None:
        try:
            targets = json.loads(msg.data)
            self._fused = {int(t["id"]): t for t in targets}
        except Exception as exc:
            self.get_logger().warn(f"Target parse error: {exc}")

    def _on_interceptor_state(self, msg: InterceptorState) -> None:
        idx = int(msg.id) - 1
        if 0 <= idx < len(self._weapons):
            self._weapons[idx].update_position(
                (msg.position.x, msg.position.y, msg.position.z))
            if msg.status == "IDLE":
                self._weapons[idx].mark_idle()
            else:
                # Either PURSUING or RETURNING — both mean unavailable.
                self._weapons[idx].mark_engaged(target_id=int(msg.target_id))

    def _on_wave(self, msg: Bool) -> None:
        if msg.data and not self._mission_active:
            self._mission_active = True
            self.get_logger().info("Mission active — engagement enabled")

    def _on_engagement_ack(self, msg: String) -> None:
        try:
            data = json.loads(msg.data)
            tid = int(data.get("target_id", -1))
        except Exception:
            return
        # Free any weapon that was engaging this target.
        for w in self._weapons:
            if w.is_available():
                continue
            # Anvil tracks engaged target via mark_engaged; the InterceptorState
            # callback will mark it idle once status flips to IDLE upstream.
            # We just consume the ack here; no extra state needed in Phase 1.
            pass

    # ── 10 Hz timer ──────────────────────────────────────────────────────

    def _on_timer(self) -> None:
        if not self._mission_active:
            return
        if not self._fused:
            return

        # Build Tracks from fusion JSON, scoring as we go
        active_states = ("DETECTED", "TARGETED")
        tracks: List[Track] = []
        for fid, t in self._fused.items():
            if t.get("state") not in active_states:
                continue
            track = Track(
                track_id=int(fid),
                position=(float(t["position"]["x"]),
                          float(t["position"]["y"]),
                          float(t["position"]["z"])),
                velocity=(float(t["velocity"]["x"]),
                          float(t["velocity"]["y"]),
                          float(t["velocity"]["z"])),
                threat_score=0.0,
                state=t.get("state", "UNKNOWN"),
            )
            score = score_threat(track)
            track = Track(
                track_id=track.track_id,
                position=track.position,
                velocity=track.velocity,
                threat_score=score,
                state=track.state,
            )
            tracks.append(track)

        # Publish scores for the dashboard (matches legacy schema)
        score_list = [
            {"id": tr.track_id,
             "score": round(tr.threat_score, 3),
             "state": tr.state}
            for tr in tracks
        ]
        score_msg = String()
        score_msg.data = json.dumps(score_list)
        self._scores_pub.publish(score_msg)

        # Only consider DETECTED, unassigned tracks for assignment.
        # Phase 1: a track is "assigned" iff some weapon is engaging it.
        engaged_target_ids = {
            w.engaged_target_id() for w in self._weapons
            if w.engaged_target_id() is not None
        }
        unassigned = [tr for tr in tracks
                      if tr.state == "DETECTED"
                      and tr.track_id not in engaged_target_ids]
        idle_weapons = [w for w in self._weapons if w.is_available()]
        if not idle_weapons or not unassigned:
            return

        # Build cost matrix and solve
        cost = build_cost_matrix(idle_weapons, unassigned)
        pairs = assign(cost)

        for w_idx, t_idx in pairs:
            weapon = idle_weapons[w_idx]
            track = unassigned[t_idx]
            order_payload = self._dispatcher.translate(weapon, track)

            order = EngagementOrder()
            order.target_id = order_payload["target_id"]
            order.interceptor_id = order_payload["interceptor_id"]
            order.priority = order_payload["priority"]
            order.issued_at = self.get_clock().now().to_msg()
            self._engage_pub.publish(order)

            self.get_logger().info(
                f"Order: {weapon.weapon_id} → threat {track.track_id}  "
                f"score={track.threat_score:.2f}  "
                f"priority={order.priority}")


def main(args=None) -> None:
    rclpy.init(args=args)
    node = C2EngineNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
```

- [ ] **Step 3: Build and check for syntax errors**

```bash
cd ~/acsdg_ws && colcon build --symlink-install --packages-select acsdg_c2
```

Expected: build succeeds, no Python import errors.

- [ ] **Step 4: Run all unit tests for the package**

```bash
cd ~/acsdg_ws && source install/setup.bash \
  && cd src/acsdg_c2 && python3 -m pytest test/ -v
```

Expected: all unit tests from Tasks 1–7 still pass (~37 tests).

- [ ] **Step 5: Note the progress**

```bash
echo "- Task 8: \`c2_engine_node.py\` refactored to compose weapons/classifier/cost_function/assignment/dispatcher modules. All unit tests passing." >> ~/acsdg_ws/HANDOFF.md
```

---

## Task 9: End-to-end regression test

**Files:**
- (No new files — this is a manual-then-automated smoke test that compares Phase 1 behavior against the baseline captured in Pre-Task Step 3.)

- [ ] **Step 1: Launch the refactored sim**

In one terminal:

```bash
cd ~/acsdg_ws && source install/setup.bash
LIBGL_ALWAYS_SOFTWARE=1 ros2 launch acsdg_bringup acsdg_full.launch.py headless:=false
```

Wait until all 4 enemies hover at perimeter and all 4 interceptors hover at their corner posts. Confirm the C2EngineNode log line: `C2EngineNode ready (10 Hz, 4 Anvils)`.

- [ ] **Step 2: Trigger a wave**

In a second terminal:

```bash
cd ~/acsdg_ws && source install/setup.bash
ros2 topic pub --times 5 -r 0.5 /mission/wave_trigger std_msgs/Bool '{data: true}'
```

- [ ] **Step 3: Observe the engagement and capture fresh kill ranges**

Watch the launch terminal for `Mission active — engagement enabled`, four `Order: anvil_<N> → threat <M> ...` lines, four interceptors converging to targets, and four `Interceptor #N: NEUTRALISED target #M at range=R.RRm` messages.

- [ ] **Step 4: Capture the Phase 1 kill ranges and diff against baseline**

```bash
echo "Phase 1 refactor kill ranges:" > /tmp/phase1_after.txt
grep "NEUTRALISED" /tmp/acsdg_sim.log | tail -4 >> /tmp/phase1_after.txt
diff /tmp/phase1_baseline.txt /tmp/phase1_after.txt | head -20
```

Expected: kill ranges still in [6.81, 8.0] m, all 4 enemies despawned. Exact ranges may differ slightly due to startup-timing nondeterminism, but every range must be ≤ 8.0 m (kill radius). If any range exceeds 8 m, the refactor introduced a real geometry change — investigate before proceeding.

- [ ] **Step 5: Verify all 4 enemies were despawned**

```bash
gz model --list | grep enemy
```

Expected: empty output (all 4 enemies removed by enemy_driver_node on NEUTRALIZED ack).

- [ ] **Step 6: Stop the sim cleanly and finalize**

Ctrl-C the launch terminal.

```bash
echo "" >> ~/acsdg_ws/HANDOFF.md
echo "### Phase 1 complete (2026-05-02)" >> ~/acsdg_ws/HANDOFF.md
echo "" >> ~/acsdg_ws/HANDOFF.md
echo "C2 engine decomposed into modular weapons/classifier/cost_function/assignment/dispatcher packages. 4 \`Anvil\` instances replicate the existing demo end-to-end; kill ranges still in [6.81, 8.0] m. Ready for Phase 2 (Coyote Block 2)." >> ~/acsdg_ws/HANDOFF.md
```

- [ ] **Step 7: Final verification — run the full unit test suite one more time**

```bash
cd ~/acsdg_ws && source install/setup.bash \
  && cd src/acsdg_c2 && python3 -m pytest test/ -v
```

Expected: ~37 tests pass, 0 fail.

---

## Phase 1 done — exit criteria checklist

- [ ] All Tasks 1–9 marked complete
- [ ] All ~37 unit tests pass
- [ ] End-to-end demo reproduces 4 successful intercepts in [6.81, 8.0] m kill range
- [ ] No new errors or warnings in `/tmp/acsdg_sim.log` beyond the pre-existing cosmetic SystemLoader complaints
- [ ] `HANDOFF.md` reflects "Phase 1 complete" and lists the new module structure
- [ ] No regression in launch time or runtime FPS

When all boxes ticked, the sim is ready for Phase 2 (add the Raytheon Coyote Block 2 frag interceptor). Phase 2's plan will be written when Phase 1 closes — that way it can be informed by what was actually learned here.
