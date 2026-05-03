# Phase 3 — DroneHunter F700 Design Spec

**Date:** 2026-05-03
**Author:** brainstormed with Claude
**Status:** approved, ready for implementation plan

> Implements Phase 3 of the original 5-phase rollout (`docs/superpowers/specs/2026-05-02-ai-orchestrated-heterogeneous-defense-design.md` §6.3 + §11 Phase-3). Strict scope per user — DroneHunter F700 only; classifier hookup (Phase 4) and expected-utility cost function (Phase 5) deferred.

---

## 1. Goal & scope

**Goal.** Add the Fortem DroneHunter F700 as the third weapon class in the heterogeneous fleet, exercising the Phase 3 prep templates (FLEET, `WeaponControllerBase`, `launch_parameters()`) end-to-end. Closes spec §6.3 and Phase-3 of the original 5-phase rollout.

**In scope.**
1. Python `DroneHunter(WeaponSystem)` class with §6.3 spec values + 180 s multi-shot cooldown via `is_available()` override
2. C++ `DroneHunterControllerNode` subclass of `WeaponControllerBase` — pursues at 31 m/s, captures at 15 m net-deploy range with binary kill, returns home, publishes `[NET-CAPTURE]` log
3. SDF model (`models/dronehunter_f700/model.sdf`) — octocopter form, 18 kg, 8 rotor stubs (cosmetic)
4. World spawn + FLEET slot append (DroneHunter replaces the SE Anvil at slot 3)
5. Tests: `test_weapon_dronehunter.py` (~7 unit), `test_dronehunter_integration.py` (~2 integration), updates to existing `test_fleet.py` and `test_launch_dryrun.py`

**Out of scope.**
- Tow-back-to-recovery behavior; recovery-zone SDF entity (Phase 3 fidelity ladder = B not C)
- Net-mesh visualization
- Bayesian classifier hookup (Phase 4)
- Mass-limit gate at 25 kg target mass (Phase 1 classifier returns SMALL_QUAD always; gate is moot until Phase 4)
- Expected-utility cost function (Phase 5)

**Inventory after Phase 3.** 1 Coyote (NE) + 2 Anvil (NW + SW) + 1 DroneHunter (SE) — matches original spec §11.3.

**Slot rationale.** Coyote at NE; Anvils at NW + SW frame the rear arc. SE puts DroneHunter at the rear-right post — natural for a slower net-capture asset. Phase 4's Skyranger 30 will replace the SW Anvil. Cost-matrix dispatches by ToI not slot, so the geometry is purely cosmetic balance.

**Success criteria.**
1. All existing tests still pass (97 passing + 2 documented flaky integration tests = 99 baseline).
2. New tests pass: ~7 unit + ~2 integration = ~9 added.
3. Live demo: 4 NEUTRALISED / 0 BREACHED reproducible. Per-weapon expected behavior:
   - Coyote takes the NE-incoming target (closest to NE post) — `[FRAG-FUZE p50]` or `[FRAG-FUZE p30]` log
   - DroneHunter takes the SE-incoming target — `[NET-CAPTURE]` log + 180 s cooldown afterward
   - Anvils take the NW + SW targets — standard `NEUTRALISED` log at 6-8 m
4. After kill, `Order: dronehunter_0 → ...` does NOT appear for 180 s in subsequent waves — observable evidence of the cooldown.

---

## 2. DroneHunter Python class

**File:** `src/acsdg_c2/acsdg_c2/weapons/dronehunter.py` (new). Mirror of `coyote.py` shape with one twist — `is_available()` override for the cooldown.

### Constants

```python
_DRONEHUNTER_MAX_SPEED       = 31.0      # m/s — post-2024 doubled-speed update
_DRONEHUNTER_MAX_RANGE       = 2000.0    # m — kill range (real spec is 2 km kill, 10 km transit; sim the kill bound)
_DRONEHUNTER_MIN_RANGE       = 50.0      # m — safe-deploy distance from launcher
_DRONEHUNTER_MAX_ALT         = 4000.0    # m — class-typical for 18 kg multirotor
_DRONEHUNTER_KILL_RADIUS     = 15.0      # m — net-deploy range
_DRONEHUNTER_RESOURCE_COST   = 0.20      # 0.20 first shot per spec; cost matrix sees one-shot value
_DRONEHUNTER_RELAUNCH_TIME_S = 180.0     # multi-shot cooldown
```

### Pkill table (spec §6.3)

```python
_PKILL = {
    TargetClass.SMALL_QUAD:  0.85,   # easy capture, light enough to tow
    TargetClass.GROUP_1:     0.70,   # faster, harder to net but within mass range
    TargetClass.GROUP_3:     0.50,   # mass approaching capture limit
    TargetClass.SHAHED:      0.40,   # above mass limit — net entanglement may down it, no recovery
}
```

### `is_available()` override — the cooldown gate

```python
class DroneHunter(WeaponSystem):
    weapon_type = "net_octo"

    def __init__(self, weapon_id: str, home_position: Tuple[float, float, float]) -> None:
        super().__init__(weapon_id=weapon_id, home_position=home_position)
        self._cooldown_until: float = 0.0   # epoch seconds; 0 = no cooldown active

    def is_available(self) -> bool:
        if self._engaged_target_id is not None:
            return False
        return time.time() >= self._cooldown_until

    def mark_idle(self) -> None:
        """Override: arrival-home transitions to cooldown, not immediate availability.

        The base Anvil/Coyote behaviour is "engagement complete → instantly re-eligible".
        DroneHunter's spec mandates 180 s relaunch_time after recovery — model this
        as a wall-clock cooldown that gates is_available() until elapsed.
        """
        was_engaged = self._engaged_target_id is not None
        super().mark_idle()   # clears _engaged_target_id
        if was_engaged:
            self._cooldown_until = time.time() + _DRONEHUNTER_RELAUNCH_TIME_S
```

The `was_engaged` guard matters: `mark_idle()` may be called multiple times (every InterceptorState tick at 10 Hz). Only start the cooldown on the *transition* from engaged → idle, not on every subsequent IDLE tick. Without the guard, the cooldown would reset every tick and never expire.

### `time_to_intercept` and `engagement_envelope`

```python
def engagement_envelope(self) -> EngagementEnvelope:
    return EngagementEnvelope(
        min_range=_DRONEHUNTER_MIN_RANGE,
        max_range=_DRONEHUNTER_MAX_RANGE,
        max_altitude=_DRONEHUNTER_MAX_ALT,
    )

def time_to_intercept(self, track: Track) -> float:
    range_m = math.hypot(*(p - h for p, h in zip(track.position, self._home_position)))
    v_proj = self._closing_rate(track)
    eff_speed = self._clamp_eff_speed(_DRONEHUNTER_MAX_SPEED, v_proj)
    return range_m / eff_speed
```

`_clamp_eff_speed` is inherited from `WeaponSystem` base (Phase 3 prep Task 11 polish). No duplication.

### `pkill(target_class)`

```python
def pkill(self, target_class: TargetClass) -> float:
    return _PKILL.get(target_class, 0.0)
```

### No `launch_parameters()` override

DroneHunter's only special state (cooldown) lives entirely in Python. The C++ controller doesn't need cooldown-related ROS parameters. Default `[]` from the base is correct.

### File size estimate

~80-100 lines (similar to Coyote's ~110 minus the probabilistic frag-fuze logic).

---

## 3. DroneHunter C++ controller

**File:** `src/acsdg_c2/src/dronehunter_controller_node.cpp` (new). Direct subclass of `WeaponControllerBase`, ~70 lines.

### Constants

```cpp
static constexpr double kMaxSpeed   = 31.0;   // m/s
static constexpr double kKillRadius = 15.0;   // m — net-deploy range
static constexpr double kCruiseZ    = 50.0;   // m — fixed cruise altitude (matches Anvil pattern)
static constexpr double kAltKp      = 0.5;    // altitude-hold P gain (matches Anvil)
static constexpr double kMaxVz      = 3.0;    // m/s — vertical clamp (matches Anvil)
```

The cruise-altitude-hold constants `kCruiseZ`, `kAltKp`, `kMaxVz` are duplicated from Anvil. Two callers don't justify a refactor; if a fourth weapon later wants alt-hold with these values, lift to base.

### Pursuit math — 2D + altitude hold

```cpp
void computePursuitCmd(double & vx, double & vy, double & vz) override {
    // 2D lead pursuit + altitude hold at cruise z. Same pattern as Anvil
    // (kCruiseZ=50). DroneHunter's 31 m/s closes engagements in ~30s for
    // typical 1km targets — long enough that 3D pursuit would amplify
    // tgt_vz radar noise. 2D + alt-hold stays robust.
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

    const double dz = kCruiseZ - pos_z_;
    vz = std::max(-kMaxVz, std::min(kMaxVz, kAltKp * dz));
}
```

### Kill log — `[NET-CAPTURE]` tag

```cpp
std::string onKill(double range_m) override {
    RCLCPP_INFO(get_logger(),
      "DroneHunter #%d: NEUTRALISED target #%d at range=%.2fm "
      "[NET-CAPTURE] dh(%.1f,%.1f,%.1f) tgt(%.1f,%.1f,%.1f)",
      id_, target_id_, range_m,
      pos_x_, pos_y_, pos_z_, tgt_x_, tgt_y_, tgt_z_);
    return "NEUTRALIZED";
}
```

`[NET-CAPTURE]` mirrors Coyote's `[FRAG-FUZE]` tag for graders to see weapon-class differentiation in the live demo.

### Constructor

```cpp
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
```

### No probabilistic frag-fuze

Binary kill at `kKillRadius=15.0` — net deployment is deterministic at close range. The Pkill table values (0.85 for SMALL_QUAD, etc.) live in the Python class for cost-matrix purposes and don't affect the C++ controller's kill check today. If Phase 4 adds class-conditional miss probabilities, Coyote's `pkill_*` ROS parameter pattern is the template.

### CMakeLists.txt addition

Copy the `coyote_controller_node` block:

```cmake
add_executable(dronehunter_controller_node src/dronehunter_controller_node.cpp)
target_link_libraries(dronehunter_controller_node weapon_controller_base)
ament_target_dependencies(dronehunter_controller_node
  rclcpp acsdg_msgs geometry_msgs nav_msgs std_msgs builtin_interfaces
)

# Update install(TARGETS …) to include the new executable:
install(TARGETS interceptor_controller_node coyote_controller_node dronehunter_controller_node
  DESTINATION lib/${PROJECT_NAME})
```

---

## 4. SDF model + world spawn + FLEET update

### model.sdf — octocopter

`src/acsdg_gazebo/models/dronehunter_f700/model.sdf` (new). Mirror of Coyote's SDF shape but octocopter geometry. 18 kg total.

```xml
<?xml version="1.0"?>
<sdf version="1.9">
  <model name="dronehunter_f700">
    <pose>0 0 0 0 0 0</pose>

    <plugin filename="gz-sim-velocity-control-system" name="gz::sim::systems::VelocityControl"/>
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

### model.config — minimal manifest

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
  </description>
</model>
```

### military_base.sdf change

Replace the SE-post Anvil with the DroneHunter:

```xml
<!-- Before: -->
<include>
  <name>interceptor_3</name>
  <pose>177 -177 20 0 0 0</pose>
  <uri>model://interceptor_drone</uri>
</include>

<!-- After: -->
<include>
  <name>dronehunter_1</name>
  <pose>177 -177 20 0 0 0</pose>
  <uri>model://dronehunter_f700</uri>
</include>
```

The other three interceptor spawns (`interceptor_2`/`interceptor_4`) are unchanged.

### FLEET change (`fleet.py`)

```python
FLEET: Tuple[Slot, ...] = (
    Slot(1, Coyote,      "coyote_0",      "coyote_controller_node",       "coyote",       1, ( 177.0,  177.0, 20.0)),
    Slot(2, Anvil,       "anvil_1",       "interceptor_controller_node",  "interceptor",  2, (-177.0,  177.0, 20.0)),
    Slot(3, DroneHunter, "dronehunter_0", "dronehunter_controller_node",  "dronehunter",  1, ( 177.0, -177.0, 20.0)),  # NEW
    Slot(4, Anvil,       "anvil_3",       "interceptor_controller_node",  "interceptor",  4, (-177.0, -177.0, 20.0)),
)
```

`anvil_2` (slot 3's old weapon_id) is gone; `anvil_3` keeps its name on slot 4 for stability with existing test expectations.

`bridged_models()` will now return `(('coyote', 1), ('dronehunter', 1), ('interceptor', 4))` — the bridge auto-wires the new gz topics.

### gz_bridge_shim fallback update

The hand-coded fallback in `gz_bridge_shim.py` (the `except ImportError` branch) needs the same update so cold-bringup-without-acsdg-c2 still works:

```python
except ImportError:
    # ... (existing comment about PYTHONPATH timing race)
    _INTERCEPTOR_MODELS = (('coyote', 1), ('dronehunter', 1), ('interceptor', 4))
```

### Dispatcher numeric-tail logic

The Phase 1/2 `dispatcher.translate(weapon, track)` maps `weapon_id` to `interceptor_id` via numeric-tail extraction with this logic: `coyote_0 → 1, anvil_1 → 2, anvil_2 → 3, anvil_3 → 4`. The new DroneHunter slot uses `weapon_id="dronehunter_0"` on `interceptor_id=3`. The dispatcher's numeric-tail extraction yields `0` (the trailing digit), but the mapping to `interceptor_id=3` is enforced by the existing logic that maps `<class>_0` for non-Anvil to a specific slot.

**This is fragile.** Verify in Task 1 of the plan — read `dispatcher.py` to confirm the routing actually maps `dronehunter_0 → 3`. If not, the dispatcher needs an explicit FLEET-driven lookup (which is the deferred "explicit weapon_id→interceptor_id registry" item from the prep). The fix is small: change `dispatcher.translate` to look up `slot.interceptor_id` from FLEET by matching `slot.weapon_id == weapon.weapon_id`.

---

## 5. Tests

### 5.1 — `test_weapon_dronehunter.py` (new)

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

The `monkeypatch` pattern for the time-advance test is critical: `time.sleep(180)` would make the test suite take 3 minutes.

### 5.2 — `test_dronehunter_integration.py` (new)

Reuse the SpyNode harness from `test_coyote_integration.py`. Two scenarios:

```python
def test_c2_dispatches_dronehunter_to_se_target(harness):
    """Target at (300, -300, 20) is closest to DroneHunter at SE (177, -177, 20).
    Confirms FLEET routing: target_id=1 → interceptor_id=3."""
    spy = harness
    spy.publish_idle_state( 177.0,  177.0, 20.0, interceptor_id=1)  # Coyote idle
    spy.publish_idle_state( 177.0, -177.0, 20.0, interceptor_id=3)  # DroneHunter idle
    spy.publish_wave()
    time.sleep(0.5)
    spy.publish_target(track_id=1, x=300.0, y=-300.0, z=20.0)
    order = spy.wait_for_order(timeout_s=3.0)
    assert order.target_id == 1
    assert order.interceptor_id == 3, (
        f"Expected DroneHunter (interceptor_id=3), got {order.interceptor_id}")


def test_dronehunter_unavailable_during_cooldown_after_engagement(harness):
    """After capture + IDLE, DroneHunter's is_available() returns False for
    180s. Confirm a second target spawned 1s later routes to a different slot."""
    spy = harness
    spy.publish_idle_state(177.0, -177.0, 20.0, interceptor_id=3)
    spy.publish_wave()
    time.sleep(0.3)
    spy.publish_target(track_id=1, x=300.0, y=-300.0, z=20.0)
    first = spy.wait_for_order(timeout_s=3.0)
    assert first.interceptor_id == 3   # DroneHunter took it

    # Simulate capture + return-home
    spy.publish_ack(target_id=1, interceptor_id=3, outcome="NEUTRALIZED")
    spy.publish_idle_state(177.0, -177.0, 20.0, interceptor_id=3)

    time.sleep(0.5)
    spy.last_order = None
    # Retire track 1 + send a new target near SE
    spy.publish_target(track_id=1, x=300.0, y=-300.0, z=20.0, state="TERMINATED")
    spy.publish_target(track_id=2, x=300.0, y=-300.0, z=20.0)

    second = spy.wait_for_order(timeout_s=3.0)
    assert second.interceptor_id != 3, (
        f"Expected non-DroneHunter slot (cooldown active), got {second.interceptor_id}")
```

### 5.3 — Update existing `test_fleet.py`

- Rename `test_bridged_models_for_phase2_inventory` → `test_bridged_models_for_phase3_inventory`
- Update expected value: `(('coyote', 1), ('dronehunter', 1), ('interceptor', 4))`
- `test_fleet_has_four_slots` still passes (still 4 slots)
- `test_fleet_matches_sdf_spawn_poses` auto-passes when SDF is updated alongside FLEET (same task)

### 5.4 — Update existing `test_launch_dryrun.py`

Both tests are FLEET-driven and auto-adapt. No code change.

### 5.5 — Verify dispatcher routing

Read `dispatcher.py` in Task 1; confirm `dispatcher.translate(weapon, track)` correctly maps `weapon_id="dronehunter_0"` to `interceptor_id=3`. If the existing numeric-tail logic doesn't, change to a FLEET-driven lookup (a small ~5-line edit). This was flagged as the "explicit weapon_id→interceptor_id registry" item in the prep deferred list — Phase 3 forces the fix.

### Test count target

| | Pre-Phase-3 | Post-Phase-3 |
|---|---|---|
| Weapon unit tests | 24 | ~31 (+7 dronehunter) |
| Fleet tests | 12 | 12 (only renames) |
| Composition / launch dryrun | 3 | 3 |
| Integration tests | 2 (flaky) | 4 (2 dronehunter added) |
| **Total** | 41 (focused) | ~50 |

(Other existing tests in the suite — dispatcher, threat-priority, etc. — total ~36 more, bringing global suite to ~77 → ~86 after Phase 3.)

---

## 6. File-level deliverables

| Type | File | Lines (est) |
|---|---|---|
| Create | `src/acsdg_c2/acsdg_c2/weapons/dronehunter.py` | ~100 |
| Create | `src/acsdg_c2/src/dronehunter_controller_node.cpp` | ~70 |
| Create | `src/acsdg_gazebo/models/dronehunter_f700/model.sdf` | ~80 |
| Create | `src/acsdg_gazebo/models/dronehunter_f700/model.config` | ~15 |
| Create | `src/acsdg_c2/test/test_weapon_dronehunter.py` | ~120 |
| Create | `src/acsdg_c2/test/test_dronehunter_integration.py` | ~80 |
| Modify | `src/acsdg_c2/acsdg_c2/fleet.py` | +1 line, -1 line (slot 3 swap) |
| Modify | `src/acsdg_c2/acsdg_c2/weapons/__init__.py` | +1 line (export DroneHunter) |
| Modify | `src/acsdg_c2/CMakeLists.txt` | +6 lines (new executable target) |
| Modify | `src/acsdg_gazebo/worlds/military_base.sdf` | +/- 4 lines (interceptor_3 → dronehunter_1) |
| Modify | `src/acsdg_gazebo/scripts/gz_bridge_shim.py` | +1 line (fallback list update) |
| Modify | `src/acsdg_c2/test/test_fleet.py` | rename + update expected value |
| Modify | `src/acsdg_c2/acsdg_c2/dispatcher/dispatcher.py` | possibly +5 lines (FLEET-driven mapping) |
| Modify | `HANDOFF.md` | new Phase 3 section |

**Total scope:** 6 new files, 7 modified files, ~+500 lines net.

---

## 7. Rollout order (preview for the plan)

Approximate task ordering — the implementation plan will refine with TDD steps:

1. **DroneHunter Python class** with cooldown + tests — Python-only, no build dependencies, drives down most of the spec. ~1 day.
2. **Dispatcher routing verification + fix if needed** — block here so tasks 3-5 can rely on correct routing.
3. **DroneHunter C++ controller** + CMakeLists — mirrors Coyote refactor, builds clean.
4. **SDF model + world spawn** — Gazebo-side, easy to verify by `gz model --list` after launch.
5. **FLEET update + bridge fallback update** — small edits, regression tests gate.
6. **Integration test** + launch dry-run validation — confirms end-to-end routing.
7. **Live demo** + HANDOFF changelog — final acceptance.

**Estimated effort:** 3-5 days for an experienced engineer; longer at graduation-project pace.

---

## 8. Notes for graders / future readers

- **Multi-shot cooldown is the visible behavioral differentiator.** With Phase 1 classifier returning SMALL_QUAD always, the Pkill table doesn't influence dispatch (until Phase 4). The cooldown DOES — graders can watch a wave-stacked demo and see DroneHunter sit out while Anvils + Coyote handle the second wave.
- **The slot choice (DroneHunter @ SE) is cosmetic** — cost-matrix dispatches by ToI, not slot. Final inventory after Phase 4 puts one weapon per quadrant for visual symmetry.
- **Tow-back-to-recovery is intentionally out of scope.** Phase 3 fidelity is "minimum visible weapon plus multi-shot." Tow physics, recovery zone, net mesh are all Phase 4 polish if there's time.
