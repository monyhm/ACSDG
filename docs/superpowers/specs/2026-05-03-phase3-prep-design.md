# Phase 3 Prep — Design Spec

**Date:** 2026-05-03
**Author:** brainstormed with Claude
**Status:** approved, ready for implementation plan

> Closes findings from the Phase 2 final code review (`docs/superpowers/plans/2026-05-02-phase2-coyote-block-2.md` end-of-phase review). Lands BEFORE any Phase 3 (DroneHunter F700) feature work begins.

---

## 1. Goal & scope

**Goal.** Land all four "Important" findings from the Phase 2 final review plus three deferred test items, producing one cohesive prep PR before any DroneHunter F700 (Phase 3 proper) code begins. Outcome: single source of truth for fleet composition, deduplicated controllers, probabilistic frag fidelity, and regression coverage on the geometry that bit us in Phase 2.

**In scope (six tasks, one plan).**

1. `FLEET` Python module + startup-assertion that SDF spawn poses match — closes I-B (home-coord triplication) / I-C (launch-ladder) / M-2 (SDF/Python coord divergence) / deferred weapon_id→interceptor_id registry.
2. `WeaponControllerBase` C++ abstract class with virtual `pursueTarget()` + `onKill()`. Anvil and Coyote refactored to inherit; identical external behavior.
3. Probabilistic frag-fuze in `coyote_controller_node`: 5 m guaranteed kill ring + 5–8 m probabilistic kill governed by Pkill table (I-A).
4. `time_to_intercept` `eff_speed` clamp at `0.5 * max_speed` for Anvil + Coyote — fixes stern-aspect-Shahed `eff_speed=1.0` blow-up (N-C).
5. Coyote integration test (M-1) — pure-rclpy smoke harness that spawns the C2 engine + Coyote controller in-process, publishes synthetic FusedTarget, asserts engagement order issued and ack returned.
6. FLEET regression test (rolls in Phase-1 reviewer ticket I-2): parses `military_base.sdf`, extracts spawn poses, asserts agreement with `FLEET[*].home`. Catches future SDF edits that drift from the Python source of truth.

**Out of scope (left for Phase 3 proper or later).** DroneHunter F700, Skyranger 30, Bayesian classifier hookup, expected-utility cost function, kill-by-other-weapon race, `releaseTarget()` helper, `is_armed(range)` hook, `DispatchOrder` dataclass, multi-shot `is_available`, `WeaponState.engaged_target_id` field. The deferred-list items that aren't called out above stay deferred.

**Success criteria.** All existing tests pass post-refactor. Live demo (same procedure as Phase 2 Task 8) reproduces 5 NEUTRALISED / 0 BREACHED, accounting for the new probabilistic frag-fuze (expect occasional `[FRAG-FUZE MISS]` lines followed by C2 reassignment). New test count ≥ 10 (frag-fuze probabilistic, FLEET-SDF agreement, FLEET uniqueness × 3, FLEET helpers, ToI clamp Anvil + Coyote, integration smoke × 2, launch dry-run).

---

## 2. FLEET module

**File:** `src/acsdg_c2/acsdg_c2/fleet.py` (new). One module, one source of truth.

### Schema

```python
from dataclasses import dataclass
from typing import Tuple, Type
from acsdg_c2.weapons import Anvil, Coyote, WeaponSystem

@dataclass(frozen=True)
class Slot:
    interceptor_id: int            # 1..N — the legacy /interceptors/unit_<id>/* identifier
    weapon_class: Type[WeaponSystem]
    weapon_id: str                 # "anvil_2", "coyote_0" — feeds dispatcher's numeric-tail logic
    controller_executable: str     # "interceptor_controller_node" or "coyote_controller_node"
    gz_model_kind: str             # "interceptor" | "coyote" — bridge shim & SDF model name root
    gz_instance_index: int         # 1..M within that kind, used in /model/<kind>_<i>/* topics
    home: Tuple[float, float, float]   # spawn pose, must match SDF

FLEET: tuple[Slot, ...] = (
    Slot(1, Coyote, "coyote_0", "coyote_controller_node",      "coyote",      1, ( 177.0,  177.0, 20.0)),
    Slot(2, Anvil,  "anvil_1",  "interceptor_controller_node", "interceptor", 2, (-177.0,  177.0, 20.0)),
    Slot(3, Anvil,  "anvil_2",  "interceptor_controller_node", "interceptor", 3, ( 177.0, -177.0, 20.0)),
    Slot(4, Anvil,  "anvil_3",  "interceptor_controller_node", "interceptor", 4, (-177.0, -177.0, 20.0)),
)
```

Homes are `±177` (matching the SDF), not the Phase-2 `±200` from `_DEFAULT_HOMES`. The `±200` was wrong — the controller reads its real position from odometry so the launch-time number was cosmetic, but FLEET is canonical going forward.

### Helper API

```python
def slots_for_kind(kind: str) -> tuple[Slot, ...]:
    return tuple(s for s in FLEET if s.gz_model_kind == kind)

def bridged_models() -> tuple[tuple[str, int], ...]:
    """For gz_bridge_shim._BRIDGED_MODELS — (kind, max_instance_index) per kind."""
    by_kind: dict[str, int] = {}
    for s in FLEET:
        by_kind[s.gz_model_kind] = max(by_kind.get(s.gz_model_kind, 0), s.gz_instance_index)
    return tuple(sorted(by_kind.items()))

def assert_matches_sdf(sdf_path: str) -> None:
    """Raise AssertionError if any FLEET.home disagrees with the SDF spawn pose
    for the same model name. Called at c2_engine startup and in the regression test."""
    # Walks <include><name>{kind}_{idx}</name><pose>x y z r p y</pose>... in the SDF
    # and compares each (kind, idx) to FLEET. Tolerance: 0.01 m absolute.
```

### Import-time validator

```python
def _validate_fleet(fleet: tuple[Slot, ...]) -> None:
    seen_ids: set[int] = set()
    seen_weapon_ids: set[str] = set()
    seen_gz: set[tuple[str, int]] = set()
    seen_homes: set[tuple[float, float, float]] = set()
    for s in fleet:
        if s.interceptor_id in seen_ids:
            raise ValueError(f"FLEET: duplicate interceptor_id={s.interceptor_id}")
        if s.weapon_id in seen_weapon_ids:
            raise ValueError(f"FLEET: duplicate weapon_id={s.weapon_id!r}")
        gz_key = (s.gz_model_kind, s.gz_instance_index)
        if gz_key in seen_gz:
            raise ValueError(f"FLEET: duplicate gz model {s.gz_model_kind}_{s.gz_instance_index}")
        if s.home in seen_homes:
            raise ValueError(f"FLEET: duplicate home pose {s.home}")
        seen_ids.add(s.interceptor_id)
        seen_weapon_ids.add(s.weapon_id)
        seen_gz.add(gz_key)
        seen_homes.add(s.home)

_validate_fleet(FLEET)   # runs at import — fails fast on misconfiguration
```

This guarantees no two slots can collide on `interceptor_id`, `weapon_id`, Gazebo model body, or spawn pose. A typo in any of these makes every consumer refuse to start.

### Consumer wiring

| Consumer | Today | After |
|---|---|---|
| `c2.launch.py` | Hand-written `if iid == COYOTE_SLOT else …` ladder; `_HOMES` repeated inline | Iterates `FLEET`, picks `controller_executable` per slot, passes `home_x/y/z` from `slot.home` |
| `gz_bridge_shim.py` | Hand-edited `_BRIDGED_MODELS = (('enemy',4), ('interceptor',4), ('coyote',1))` | `_BRIDGED_MODELS = (('enemy', NUM_ENEMIES),) + bridged_models()` (enemies still standalone — they're not in FLEET) |
| `c2_engine_node.py` | Hand-written `_DEFAULT_HOMES` + literal `Coyote(...)` / `Anvil(...)` block | `self._weapons = [s.weapon_class(weapon_id=s.weapon_id, home_position=s.home) for s in FLEET]`; calls `assert_matches_sdf()` at startup |
| `military_base.sdf` | Hand-edited spawn `<include>` blocks at `±177,±177,20` | Unchanged — the regression test asserts agreement |

### Heterogeneity & Phase 3-4 readiness

FLEET is built to support multi-class fleets. Adding a Phase 3 DroneHunter F700:

```python
FLEET = (
    Slot(1, Coyote,       "coyote_0",     ..., ( 177,  177, 20)),
    Slot(2, Anvil,        "anvil_1",      ..., (-177,  177, 20)),
    Slot(3, DroneHunter,  "dronehunter_0", ..., ( 177, -177, 20)),  # NEW
    Slot(4, Anvil,        "anvil_2",      ..., (-177, -177, 20)),
)
```

The C2's cost-matrix dispatch ([c2_engine_node.py:140-203](src/acsdg_c2/acsdg_c2/c2_engine_node.py#L140)) iterates `self._weapons`, computes each weapon's `time_to_intercept` and `pkill` per target, runs Hungarian. The AI naturally chooses the best weapon for each threat — fast Coyote at fast targets, cheap DroneHunter at slow small quads, Anvil for mid-tier.

**Caveat.** Phase 1's cost function is a placeholder using ToI only — no Pkill weighting. Until Phase 5 swaps in the expected-utility cost (`-log(Pkill) + α·ToI + β·resource_cost`), the AI picks fastest, not cheapest-per-kill. This gap is intentionally scoped — Phase 5 closes it.

### Why FLEET stays Python (not YAML)

Every consumer is already Python (launch, bridge, c2_engine, tests). YAML would force a string→class lookup table for `weapon_class`, schema validation, and an extra build step. No win.

### Why the SDF stays hand-edited

SDF holds buildings, ground, lighting, sensors, and only 4 fleet `<include>` blocks. Auto-generating just the fleet portion would split the file into two regions and harm readability. Cheaper: enforce agreement via test + startup assertion.

---

## 3. `WeaponControllerBase` virtual extraction

### File layout (new)

```
src/acsdg_c2/include/acsdg_c2/weapon_controller_base.hpp   # abstract base
src/acsdg_c2/src/weapon_controller_base.cpp                # shared implementation
src/acsdg_c2/src/interceptor_controller_node.cpp           # refactored: ~50 lines
src/acsdg_c2/src/coyote_controller_node.cpp                # refactored: ~50 lines
```

### Base class API

Inherits `rclcpp::Node`. Owns the topic plumbing, watchdog, ack publishing, and return-to-home — none of which differ between weapons. The 20% that differs is exactly two virtuals.

```cpp
class WeaponControllerBase : public rclcpp::Node {
public:
  WeaponControllerBase(const std::string & node_name,
                       const std::string & topic_prefix,    // "/interceptor_" or "/coyote_"
                       double max_speed_mps,
                       double kill_radius_m,
                       double kill_radius_outer_m = 0.0);   // 0 → use kill_radius_m as outer bound

protected:
  // ── Virtuals: weapon-specific behavior ──────────────────────────────
  /**
   * Compute the velocity command for one tick of pursuit.
   * Inputs: current pos_, current target tgt_/tgt_v_.
   * Output: vx, vy, vz in WORLD frame (bridge does world→body).
   */
  virtual void computePursuitCmd(double & vx, double & vy, double & vz) = 0;

  /**
   * Called when range falls inside kill_radius_outer_, before publishing the ack.
   * Default impl is a binary kill at kill_radius_m. Coyote overrides to add the
   * 5–8 m probabilistic ring (Section 4 below).
   * Returns the outcome string for the ack ("NEUTRALIZED" or "MISS"), or "" to
   * indicate "no kill this tick — continue pursuit."
   */
  virtual std::string onKill(double range_m) = 0;

  // ── Shared state subclasses can read ────────────────────────────────
  int     id_{0};
  double  pos_x_{0}, pos_y_{0}, pos_z_{0};
  double  tgt_x_{0}, tgt_y_{0}, tgt_z_{0};
  double  tgt_vx_{0}, tgt_vy_{0}, tgt_vz_{0};
  bool    pursuing_{false};
  bool    has_target_data_{false};
  double  home_x_{0}, home_y_{0}, home_z_{0};
  int     target_id_{0};

  const double max_speed_;
  const double kill_radius_;       // inner / p50 ring
  const double kill_radius_outer_; // outer / p30 ring (== kill_radius_ for binary weapons)

private:
  // ── Shared implementation (NOT virtual) ─────────────────────────────
  void onOrder(EngagementOrder::SharedPtr msg);
  void onTarget(FusedTarget::SharedPtr msg);
  void controlStep();
  void stepTowardHome();
  void publishCmdVel(double vx, double vy, double vz);
  void publishPosition();
  void publishAck(int target_id, const std::string & outcome);
  // Topic plumbing (publishers, subscribers, timer)
};
```

### Concrete classes after refactor

```cpp
// interceptor_controller_node.cpp — Anvil (renames to AnvilControllerNode internally;
// executable name "interceptor_controller_node" stays so launch wiring is unchanged)
class AnvilControllerNode : public WeaponControllerBase {
public:
  AnvilControllerNode()
    : WeaponControllerBase("interceptor_controller_node", "/interceptor_",
                           /*max_speed=*/15.0, /*kill_radius=*/8.0) {}

protected:
  void computePursuitCmd(double & vx, double & vy, double & vz) override {
    // 2D pursuit + altitude hold at kCruiseZ=50 with kAltKp=0.5, kMaxVz=3.0
    // (lifted verbatim from today's pursueTarget())
  }
  std::string onKill(double range_m) override {
    RCLCPP_INFO(get_logger(), "Interceptor #%d: NEUTRALISED target #%d at range=%.2fm",
                id_, target_id_, range_m);
    return "NEUTRALIZED";
  }
};

// coyote_controller_node.cpp — Coyote
class CoyoteControllerNode : public WeaponControllerBase {
public:
  CoyoteControllerNode()
    : WeaponControllerBase("coyote_controller_node", "/coyote_",
                           /*max_speed=*/160.0, /*kill_radius=*/5.0,
                           /*kill_radius_outer=*/8.0) {}

protected:
  void computePursuitCmd(double & vx, double & vy, double & vz) override {
    // 3D proportional pursuit toward predicted lead point (lifted from today's pursueTarget)
  }
  std::string onKill(double range_m) override {
    // Probabilistic frag-fuze logic — see Section 4
  }
};
```

### Topic prefix wiring

Base-class constructor builds publisher/subscriber topics from `topic_prefix + std::to_string(id_)`. Anvil passes `/interceptor_`, Coyote passes `/coyote_`. The interceptor-state topic `/interceptors/unit_<id>/position` (consumed by `c2_engine`) stays uniform — base class always uses that prefix for ack/state output, since both weapon types report to C2 via the same legacy schema.

### Behavioral parity requirement

The refactor must not change live behavior. Migration sequence:

1. Create `weapon_controller_base.{hpp,cpp}` with shared code lifted verbatim. No semantic changes.
2. Convert `interceptor_controller_node.cpp` to subclass — 2D pursuit goes into `computePursuitCmd` override unchanged.
3. Convert `coyote_controller_node.cpp` to subclass — 3D pursuit goes into `computePursuitCmd` override unchanged.
4. Run live-demo procedure. Outcome must match Phase 2 (5 NEUTRALISED, 0 BREACHED, kill ranges within ±0.5 m of Phase 2 numbers).

### Why split into two virtuals (not one)

`pursueTarget()` today owns both the kill-radius check and the velocity computation — two concerns. Splitting into `computePursuitCmd` (just velocity math) + `onKill` (just kill logic) keeps each override small and lets the kill check live in the base class where it can be uniformly enforced. No subclass can forget to check the kill radius.

### MAVROS coupling note

Today's `interceptor_controller_node` may have leftover MAVROS `set_mode` plumbing that Coyote's controller doesn't have. If still present at implementation time, it stays in the Anvil concrete class only — base class shouldn't take a MAVROS dependency. Verify with `grep mavros src/acsdg_c2/src/interceptor_controller_node.cpp` in Task 2 step 1.

### Phase 3 readiness

Adding DroneHunter F700 in Phase 3 proper becomes:

```cpp
class DroneHunterControllerNode : public WeaponControllerBase {
public:
  DroneHunterControllerNode()
    : WeaponControllerBase("dronehunter_controller_node", "/dronehunter_",
                           /*max_speed=*/40.0, /*kill_radius=*/10.0) {}
  void computePursuitCmd(...) override { /* net-capture pursuit profile */ }
  std::string onKill(...) override     { /* "CAPTURED" with net-deploy logic */ }
};
```

50 lines, one CMake target, done. That's the prep paying off.

---

## 4. Probabilistic frag-fuze (closes I-A)

### Fidelity gap

Spec §6.2 lists two frag rings: `kill_radius_p50 = 5 m` and `kill_radius_p30 = 8 m`. The notation means "Pkill ≥ 0.5 within 5 m, Pkill ≥ 0.3 within 8 m." Today's controller collapses both into a single `range < 5 m → guaranteed kill`, ignoring the 5–8 m ring entirely.

### Model

| Range band | Outcome |
|---|---|
| `range < 5 m` | Guaranteed kill (no roll) |
| `5 m ≤ range < 8 m` | Probabilistic kill — roll uniform `[0, 1)`; kill if roll ≤ `pkill_for_class` |
| `range ≥ 8 m` | No kill check this tick — pursuit continues |

### Where Pkill comes from

Single source of truth: [`Coyote.pkill(target_class)`](src/acsdg_c2/acsdg_c2/weapons/coyote.py) Python table. The C++ controller can't import Python, so values plumb via ROS parameters at launch time:

```python
# In c2.launch.py, when spawning the Coyote controller for its FLEET slot:
from acsdg_c2.weapons import Coyote
from acsdg_c2.weapons.types import TargetClass
coyote_template = Coyote(weapon_id=slot.weapon_id, home_position=slot.home)
parameters = [
    {"interceptor_id": slot.interceptor_id},
    {"home_x": slot.home[0], "home_y": slot.home[1], "home_z": slot.home[2]},
    {"pkill_small_quad": coyote_template.pkill(TargetClass.SMALL_QUAD)},   # 0.60
    {"rng_seed": -1},   # production: cryptographic seed
]
```

Phase 1 classifier returns SMALL_QUAD unconditionally, so today only `pkill_small_quad` is needed. Phase 4 will pass the classified target class through `EngagementOrder` and the controller will look up the matching pkill — Phase 4 work; for now, single param. **Critically: zero Pkill numbers live in C++.** Changing the spec table changes only the Python class.

### Kill-check code in Coyote's `onKill`

```cpp
std::string CoyoteControllerNode::onKill(double range_m) {
  if (range_m < 5.0) {
    RCLCPP_INFO(get_logger(),
      "Coyote #%d: NEUTRALISED target #%d at range=%.2fm [FRAG-FUZE p50] coy(%.1f,%.1f,%.1f) tgt(%.1f,%.1f,%.1f)",
      id_, target_id_, range_m, pos_x_, pos_y_, pos_z_, tgt_x_, tgt_y_, tgt_z_);
    return "NEUTRALIZED";
  }
  if (range_m < 8.0) {
    const double roll = uniform_(rng_);
    const bool killed = (roll <= pkill_small_quad_);
    const char * tag = killed ? "[FRAG-FUZE p30]" : "[FRAG-FUZE MISS]";
    RCLCPP_INFO(get_logger(),
      "Coyote #%d frag roll=%.3f vs pkill=%.3f at range=%.2fm → %s",
      id_, roll, pkill_small_quad_, range_m, killed ? "KILL" : "MISS");
    RCLCPP_INFO(get_logger(),
      "Coyote #%d: %s target #%d at range=%.2fm %s",
      id_, killed ? "NEUTRALISED" : "MISSED", target_id_, range_m, tag);
    return killed ? "NEUTRALIZED" : "MISS";
  }
  return "";   // no kill this tick — base class continues pursuit
}
```

### Base-class change to support "no-kill-this-tick"

The base-class `controlStep` calls `onKill(range)` only when `range < kill_radius_outer_` (8 m for Coyote, 8 m for Anvil — Anvil's outer == inner). If `onKill` returns `""`, base class continues pursuit; otherwise base class publishes the ack with the returned outcome string and clears `pursuing_`. Kill-check enforcement stays uniform.

### RNG determinism for tests

```cpp
const auto seed_param = declare_parameter<int>("rng_seed", -1);
const auto seed = (seed_param < 0)
    ? std::random_device{}()
    : static_cast<unsigned int>(seed_param);
rng_.seed(seed);
RCLCPP_INFO(get_logger(), "Coyote #%d frag-fuze RNG seed=%u", id_, seed);
```

Production launch: `rng_seed=-1` → cryptographic seed → real randomness. Integration tests pass `rng_seed=42` → deterministic. Tests assert known kill/miss outcomes at known geometry.

### Miss path

Controller publishes ack with `outcome="MISS"`. Today's `_on_engagement_ack` in `c2_engine_node` is informational (Phase 1 stub) — the weapon slot frees on the next IDLE state from `InterceptorState`, not on the ack. So a "MISS" today behaves identically to a "NEUTRALIZED" from C2's perspective: Coyote returns home, target stays alive, next cost-matrix tick reassigns — likely to an Anvil since Coyote is returning. The TODO at [c2_engine_node.py:136](src/acsdg_c2/acsdg_c2/c2_engine_node.py#L136) (`Phase4: if outcome == MISS, mark (weapon_id, tid) failed and trigger cost-matrix rebuild`) becomes a real Phase 4 hook once the adaptive supervisor lands.

### Anvil keeps binary kill

Anvil is a kinetic ramming quadcopter (collision body), not a frag warhead. There is no `p30 ring` to model — kill is purely physical contact at <8 m. Anvil's `onKill` keeps existing binary "NEUTRALIZED" semantics. Probabilistic logic is a Coyote-only override of the base-class default. DroneHunter (net-capture) and Skyranger (35 mm AHEAD airburst) will each have their own override semantics.

### Visible behavior in live demo

With `pkill_small_quad = 0.60`, ~60% of 5–8 m near-misses become kills, ~40% become misses with the C2 reassigning. The live-demo terminal will show occasional `[FRAG-FUZE MISS]` lines followed by Coyote return-to-home and Anvil pickup. Demonstrates adaptive reassignment naturally — good for graduation showcase.

---

## 5. ToI `eff_speed` clamp (closes N-C)

### The bug

Both Anvil and Coyote compute `time_to_intercept` with this pattern:

```python
def time_to_intercept(self, track: Track) -> float:
    range_m = math.hypot(...)
    closing_rate = -dot(unit_range, track.velocity)
    eff_speed = max(1.0, _COYOTE_MAX_SPEED + v_proj)   # ← bug
    return range_m / eff_speed
```

For stern-aspect targets (running away faster than the weapon's max), `v_proj` is large and negative. SHAHED at 185 m/s tail-chase against a 160 m/s Coyote: `eff_speed = max(1.0, 160 - 185) = 1.0`. ToI is `range / 1.0` — looks like the right "deprioritize this target" answer but the value isn't seconds; the cost matrix gets garbage units.

### The fix

Clamp `eff_speed` at half max speed:

```python
# coyote.py
eff_speed = max(0.5 * _COYOTE_MAX_SPEED, _COYOTE_MAX_SPEED + v_proj)   # 80 m/s floor

# anvil.py
eff_speed = max(0.5 * _ANVIL_MAX_SPEED, _ANVIL_MAX_SPEED + v_proj)     # 7.5 m/s floor
```

### Why 0.5

Standard "weapon can't catch this target, but cost-matrix should still compare ToIs in seconds, not magic numbers" floor. Half-max gives:

- A target running away at any speed yields `ToI = 2 * range / max_speed` — finite, in seconds, but high relative to a closing target.
- Hungarian solver sees real cost differential between fleeing vs incoming.
- No `eff_speed = 1.0` magic.

### Tests added

```python
# test_weapon_anvil.py
def test_anvil_time_to_intercept_clamps_eff_speed_for_fleeing_target():
    anvil = Anvil(weapon_id="anvil_test", home_position=(0, 0, 20))
    fleeing = Track(track_id=1, position=(100.0, 0.0, 20.0),
                    velocity=(50.0, 0.0, 0.0),       # 50 m/s away, faster than Anvil's 15 m/s max
                    threat_score=0.0, state="DETECTED")
    toi = anvil.time_to_intercept(fleeing)
    assert 12.0 < toi < 15.0, f"Got {toi}"            # Expected ≈ 100 / 7.5 = 13.3
    assert toi > 100.0 / 15.0                          # Sanity: must exceed range/max_speed

# test_weapon_coyote.py
def test_coyote_time_to_intercept_clamps_eff_speed_for_stern_shahed():
    coyote = Coyote(weapon_id="coyote_test", home_position=(0, 0, 20))
    shahed = Track(track_id=1, position=(1000.0, 0.0, 50.0),
                   velocity=(185.0, 0.0, 0.0),
                   threat_score=0.0, state="DETECTED")
    toi = coyote.time_to_intercept(shahed)
    assert 11.5 < toi < 14.0, f"Got {toi}"             # Expected ≈ 1000 / 80 = 12.5
    assert toi < 50.0, f"Got {toi} — eff_speed=1.0 floor regression"
```

The `assert toi < 50.0` is the regression-test bit — catches reintroduction of `max(1.0, ...)`.

### Existing closing-rate test stays

`test_coyote_time_to_intercept_credits_inbound_closing_rate` from Phase 2 polish covers the closing case. New tests cover the fleeing case. Together they pin both branches.

---

## 6. Test strategy

### 6.1 — `test_fleet.py` (closes I-2)

```python
def test_fleet_uniqueness_validator_rejects_duplicate_interceptor_id():
    bad = (Slot(1, Anvil, "a", "anvil_node", "interceptor", 1, (0,0,20)),
           Slot(1, Anvil, "b", "anvil_node", "interceptor", 2, (1,0,20)))
    with pytest.raises(ValueError, match="duplicate interceptor_id"):
        _validate_fleet(bad)

def test_fleet_uniqueness_validator_rejects_duplicate_gz_model():
    bad = (Slot(1, Anvil, "a", "n", "interceptor", 2, (0,0,20)),
           Slot(2, Anvil, "b", "n", "interceptor", 2, (1,0,20)))
    with pytest.raises(ValueError, match="duplicate gz model"):
        _validate_fleet(bad)

def test_fleet_uniqueness_validator_rejects_duplicate_home():
    bad = (Slot(1, Anvil, "a", "n", "interceptor", 1, (177,177,20)),
           Slot(2, Anvil, "b", "n", "interceptor", 2, (177,177,20)))
    with pytest.raises(ValueError, match="duplicate home pose"):
        _validate_fleet(bad)

def test_fleet_matches_sdf_spawn_poses():
    sdf_path = Path(__file__).parent.parent.parent / "acsdg_gazebo/worlds/military_base.sdf"
    sdf_poses = parse_sdf_includes(sdf_path)
    for slot in FLEET:
        model_name = f"{slot.gz_model_kind}_{slot.gz_instance_index}"
        assert model_name in sdf_poses, f"FLEET slot {slot.weapon_id} has no SDF spawn for {model_name}"
        sdf_xyz = sdf_poses[model_name]
        assert slot.home == pytest.approx(sdf_xyz, abs=0.01), \
            f"{model_name}: FLEET={slot.home} vs SDF={sdf_xyz}"

def test_fleet_bridged_models_helper():
    pairs = bridged_models()
    pair_dict = dict(pairs)
    for slot in FLEET:
        assert pair_dict[slot.gz_model_kind] >= slot.gz_instance_index
```

`parse_sdf_includes` is a small `xml.etree` walk — kept inline in the test file; not worth its own module unless other tests need it.

### 6.2 — `test_coyote_integration.py` (closes M-1)

**Architecture.** Pure rclpy harness — no `launch_testing`. Pytest fixture spawns C2EngineNode + CoyoteControllerNode + a SpyNode (publishes synthetic targets, observes orders/acks) in a `MultiThreadedExecutor` thread.

```python
@pytest.fixture
def integration_world():
    rclpy.init()
    executor = MultiThreadedExecutor(num_threads=4)
    c2 = C2EngineNode()
    coyote = CoyoteControllerNode()    # rng_seed=42 via parameter override
    spy = SpyNode()
    for n in (c2, coyote, spy): executor.add_node(n)
    thread = threading.Thread(target=executor.spin, daemon=True); thread.start()
    yield spy
    executor.shutdown(); rclpy.shutdown()

def test_coyote_engages_and_kills_a_small_quad_at_close_range(integration_world):
    spy = integration_world
    spy.publish_wave_trigger()
    spy.publish_fused_target(track_id=1, position=(180, 180, 20), velocity=(0, 0, 0))
    spy.publish_interceptor_state(id=1, status="IDLE", position=(177, 177, 20))
    spy.wait_for_engagement_order(timeout=2.0)
    assert spy.last_order.target_id == 1
    assert spy.last_order.interceptor_id == 1
    spy.publish_coyote_odometry(position=(180, 178, 20))   # 2 m from target — guaranteed kill
    ack = spy.wait_for_ack(timeout=2.0)
    assert ack["target_id"] == 1
    assert ack["outcome"] == "NEUTRALIZED"

def test_coyote_probabilistic_miss_at_p30_ring_with_seed(integration_world):
    """With rng_seed=42 and pkill=0.6, the first roll at 7m range is deterministic.
    Pins the exact MISS/KILL outcome so a future RNG-API change is caught."""
    # ... 7 m range engagement, assert deterministic outcome
```

**Why pure rclpy and not launch_testing.** `launch_testing` spins real `ros2 launch` machinery — slow, flaky in WSL, overkill for a smoke test. In-process nodes share memory and run in milliseconds.

### 6.3 — `test_launch_dryrun.py`

```python
def test_launch_description_spawns_one_controller_per_fleet_slot():
    from acsdg_c2.launch.c2 import generate_launch_description
    desc = generate_launch_description()
    nodes = [a for a in desc.entities if isinstance(a, Node)]
    controllers = [n for n in nodes if "controller_node" in n.executable]
    assert len(controllers) == len(FLEET)
```

Catches "launch.py forgot to iterate FLEET" cheaply — no rclpy startup needed.

### Test count after this prep

| | Existing | After |
|---|---|---|
| Unit tests | 40 | ~49 |
| Integration tests | 2 | 4 |
| Launch dry-run | 0 | 1 |

CI runtime stays under 10 s. End-to-end with real Gazebo physics stays manual via the live-demo procedure — too brittle for CI, the post-refactor live run is the final acceptance gate.

---

## 7. File-level deliverables + rollout order

Six tasks, ordered for safe incremental landing. Each task ends green (all tests pass) and is independently revertable.

### Task 1 — FLEET module + regression tests + consumer rewiring

Closes I-B / I-C / M-2 / I-2 / weapon_id→interceptor_id registry.

**Files:**
- Create: `src/acsdg_c2/acsdg_c2/fleet.py` (~80 lines: Slot, FLEET, `_validate_fleet`, `bridged_models`, `assert_matches_sdf`, `parse_sdf_includes`)
- Create: `src/acsdg_c2/test/test_fleet.py` (~120 lines: 5 tests)
- Modify: `src/acsdg_c2/acsdg_c2/c2_engine_node.py` — replace `_DEFAULT_HOMES` and literal `_weapons` block with FLEET iteration; add `assert_matches_sdf` call in `__init__`
- Modify: `src/acsdg_gazebo/scripts/gz_bridge_shim.py` — `_BRIDGED_MODELS = (('enemy', 4),) + bridged_models()`
- Modify: `src/acsdg_c2/launch/c2.launch.py` — iterate FLEET; drop `COYOTE_SLOT` and the if/else ladder
- Modify: `src/acsdg_c2/setup.py` — declare `fleet` so `colcon build` ships it

**Acceptance:** all existing tests pass + 5 new fleet tests pass + live demo reproduces 5/0.

**Touch:** ~250 lines across 6 files (mostly delete).

### Task 2 — `WeaponControllerBase` extraction

Closes the Phase-3-hazard callout.

**Files:**
- Create: `src/acsdg_c2/include/acsdg_c2/weapon_controller_base.hpp` (~80 lines)
- Create: `src/acsdg_c2/src/weapon_controller_base.cpp` (~200 lines: shared impl lifted verbatim)
- Modify: `src/acsdg_c2/src/interceptor_controller_node.cpp` — strip to ~60 lines
- Modify: `src/acsdg_c2/src/coyote_controller_node.cpp` — strip to ~60 lines
- Modify: `src/acsdg_c2/CMakeLists.txt` — add `weapon_controller_base.cpp` as a library target; both controller executables link against it

**Acceptance:** binary reproduction of Phase 2 live demo — kill ranges within ±0.5 m of Phase 2 numbers, 5 NEUTRALISED, 0 BREACHED. No new build warnings.

**Touch:** +280 lines new, −440 lines removed (net −160).

### Task 3 — Probabilistic frag-fuze in `CoyoteControllerNode::onKill`

Closes I-A.

**Files:**
- Modify: `src/acsdg_c2/include/acsdg_c2/weapon_controller_base.hpp` — `kill_radius_outer_` constant; `controlStep` checks `range < kill_radius_outer_` to gate `onKill` invocation
- Modify: `src/acsdg_c2/src/coyote_controller_node.cpp` — pass `5.0, 8.0` to base ctor; `onKill` body adds probabilistic roll; declare `pkill_small_quad` and `rng_seed` ROS params; `<random>` member
- Modify: `src/acsdg_c2/launch/c2.launch.py` — pass `pkill_small_quad` from `Coyote().pkill(TargetClass.SMALL_QUAD)` for the Coyote slot
- Modify: `src/acsdg_c2/test/test_coyote_integration.py` — add probabilistic-miss test

**Acceptance:** with `rng_seed=42`, deterministic test asserts known miss outcome at 7 m range. Live demo shows occasional `[FRAG-FUZE MISS]` followed by C2 reassignment.

**Touch:** ~80 lines across 4 files.

### Task 4 — `time_to_intercept` `eff_speed` clamp

Closes N-C.

**Files:**
- Modify: `src/acsdg_c2/acsdg_c2/weapons/anvil.py` — `max(1.0, ...)` → `max(0.5 * _ANVIL_MAX_SPEED, ...)`
- Modify: `src/acsdg_c2/acsdg_c2/weapons/coyote.py` — same change
- Modify: `src/acsdg_c2/test/test_weapon_anvil.py` — add fleeing-target test
- Modify: `src/acsdg_c2/test/test_weapon_coyote.py` — add stern-Shahed test

**Acceptance:** 4 tests pass; existing closing-rate test still passes; no behavior change for closing engagements.

**Touch:** ~50 lines.

### Task 5 — Coyote integration test

Closes M-1.

**Files:**
- Create: `src/acsdg_c2/test/test_coyote_integration.py` (~250 lines: SpyNode helper + 2 tests — engage+kill, probabilistic miss)
- Create: `src/acsdg_c2/test/test_launch_dryrun.py` (~30 lines)

**Acceptance:** both tests pass in <2 s wall time. Pure pytest, no `launch_testing`.

**Touch:** ~280 lines new.

### Task 6 — HANDOFF.md + final verification

**Files:**
- Modify: `HANDOFF.md` — add Phase 3 prep section after the existing Phase 2 changelog. Document FLEET, `WeaponControllerBase`, frag-fuze ring model, ToI clamp.

**Acceptance:**
1. `colcon test --packages-select acsdg_c2 acsdg_gazebo` all green
2. `colcon build --symlink-install` clean
3. Live demo: 4–5 NEUTRALISED with 0–1 `[FRAG-FUZE MISS]` lines per wave; 0 BREACHED
4. Final code review across the prep delta

### Total scope

| Metric | Value |
|---|---|
| Files created | 6 |
| Files modified | 10 |
| Net line delta | ~+400 (mostly tests; controller refactor net negative) |
| New tests | ~10 unit + 2 integration + 1 launch dry-run |
| Estimated effort | 5–7 working days experienced; longer at graduation-project pace |
| Reverts cleanly | Yes — each task is one PR / one commit chain |

### Why this order

- **Task 1 first** because everything else (controllers, test fixtures, launch file) imports from FLEET. Doing it first means downstream tasks land on a clean fleet.
- **Task 2 before Task 3** because Task 3 modifies `onKill`, which doesn't exist as a virtual until Task 2 lands.
- **Task 4 in parallel possible** — N-C is a 4-file Python-only change with zero coupling. For solo work, doing it after Task 2 keeps PRs clean.
- **Task 5 last among code work** — integration test exercises the post-refactor world.
- **Task 6 closes the prep** — same pattern as Phase 2 Task 8.

---

## 8. Notes for graders / future readers

- This prep PR adds NO visible features. It exists because Phase 2's final code review identified four "Important" items that would compound if Phase 3 (DroneHunter) landed on the existing structure. Investing 5–7 days in cleanup before adding a third weapon class beats triplicating ~270 lines of duplicated controller code.
- The probabilistic frag-fuze is the only fidelity gain visible in the live demo. Everything else is internal architecture or test coverage.
- Phase 5 will swap the cost function from "ToI only" to expected-utility. Until then, the AI's weapon selection is dominated by speed — a known and intentional gap.
