# AI-Orchestrated Heterogeneous Counter-Drone Defense — Design Spec

**Date:** 2026-05-02
**Project:** ACSDG (Anti-Counter-Swarm Drone Defense Gazebo Sim)
**Workspace:** `~/acsdg_ws`
**Status:** Draft — pending implementation plan

---

## 1. Goal

Upgrade ACSDG from a homogeneous force of 4 identical kinetic interceptors to a **heterogeneous 4-system inventory** (Anduril Anvil / Raytheon Coyote Block 2 / Fortem DroneHunter F700 / Rheinmetall Skyranger 30) controlled by an **AI command-and-control layer** that performs probabilistic expected-utility weapon-target assignment under threat-classification uncertainty.

The deliverable is a polished, demoable simulation in which AI is *visibly* doing real work at each layer of the defense pipeline — perception, prediction, classification, orchestration, and decision-making. The graduation-project goal is to **showcase how AI can be an integral part of a real defense system**: identifying, predicting, planning, dispatching, and adapting against threats with unknown characteristics.

This is a demonstrative project, not a research study. There is no formal hypothesis test, no statistical-significance comparison. Success is measured by whether the AI's role at each layer is *visible and explicable* in a live demo.

---

## 2. Showcased AI Layers

The five layers of AI behavior the demo must surface visibly:

1. **Perception / classification.** Sensor fusion ingests noisy radar / EO observations and produces P(class | obs) over a finite taxonomy of target classes via Bayesian update. AI identifies *what* an unknown drone is.
2. **Prediction.** Per-track Kalman state estimation plus short-horizon trajectory prediction conditioned on inferred class (Shahed-class flies straight; small-quad jinks; Group-1 cruises smoothly). AI predicts *where* unknown drones are going.
3. **Orchestration.** Cost-matrix builder converts (weapon × track × class-posterior) tuples into expected-utility scores using each weapon's published kill-probability curves and resource model. AI reasons about *which weapon fits which target*.
4. **Decision-making.** Hungarian solver commits an assignment under resource constraints (Skyranger ammo, one-shot expendables). AI *acts*.
5. **Adaptive re-tasking.** Engagement supervisor monitors mid-flight outcomes; if a kill fails or aborts, re-runs the planner with the failed pair excluded. AI *adapts* under uncertainty.

Every one of these layers must have a *visible* diagnostic in the showcase scenario (see §14).

---

## 3. Scope

**In scope:**
- 4 weapon platforms (Anvil, Coyote Block 2, DroneHunter F700, Skyranger 30) with real-world spec anchors and full citation provenance
- Multi-class threat spawn (4 classes) with class-conditional sensor signatures
- Bayesian classifier producing P(class | obs) per track
- Class-conditioned trajectory predictor
- Expected-utility cost function with α/β/γ/δ weights, exposed as ROS params
- Hungarian assignment on enriched cost matrix
- Adaptive re-tasking on engagement failure
- Per-layer visualization (RViz overlays, cost-matrix heatmap, decision logs, resource panels, predicted-trajectory cones)
- Multi-wave showcase scenario as final demo deliverable

**Out of scope (explicit future-work):**
- Robust-optimization variant of the cost function (worst-case Pkill across the posterior)
- Reinforcement-learned assignment policy
- Soft-kill weapons (RF jamming, GPS spoofing, HPM)
- Hardware integration
- Statistical comparison study with hypothesis testing
- Networked multi-C2 coordination

---

## 4. System Architecture

### 4.1 Module map

```
sensors → fusion → tracks ──┬───────────────────────────────────────────────┐
                            │                                                │
                            ├──→ classifier ──P(class|obs)──┐               │
                            │                                │               │
                            ├──→ predictor ──future_traj──┐  │               │
                            │                              ↓  ↓               │
                            │                    cost_function (Module C)    │
                            │                              │                 │
                weapon_registry (Module W) ────────────────┤                 │
                  ├─ Anvil (kinetic_quad)                  │                 │
                  ├─ Coyote B2 (frag_jet)                  │                 │
                  ├─ DroneHunter (net_octo)                ↓                 │
                  └─ Skyranger (gun_turret)        cost_matrix                │
                                                          │                  │
                                                          ↓                  │
                                          assignment (Module A: Hungarian)   │
                                                          │                  │
                                                          ↓                  │
                                          dispatcher (Module D)               │
                                                          │                  │
                                                          ↓                  │
                                          EngagementOrder per weapon          │
                                                          │                  │
                                                          ↓                  │
                                          per-weapon controllers ─────────────┘
                                                          │
                                                          ↓
                                          Gazebo cmd_vel / fire commands
                                                          │
                                                          ↓
                                          supervisor monitors outcomes ──→ re-plan trigger
```

### 4.2 Existing modules (preserved, extended)

- `acsdg_gazebo/scripts/gz_bridge_shim.py` — ROS↔GZ bridge. **No changes.**
- `acsdg_gazebo/scripts/enemy_driver_node.py` — enemy waypoint follower + despawn. **Extended in Phase 4 for class-aware multi-spawn.**
- `acsdg_sensors/src/radar_node.cpp` — Gazebo-truth radar. **Extended in Phase 4 to publish per-track size/RCS/IR observations.**
- `acsdg_sensors/src/sensor_fusion_node.cpp` — NN-associated tracker. **Extended in Phase 4 with classifier.**
- `acsdg_c2/acsdg_c2/c2_engine_node.py` — current scoring + Hungarian. **Decomposed in Phase 1 into the modules below.**
- `acsdg_c2/acsdg_c2/mission_manager_node.py` — KPIs + wave logic. **Extended for per-weapon resource accounting.**
- `acsdg_c2/acsdg_c2/interceptor_manager_node.py` — IDLE/PURSUING/RETURNING state machine. **Generalized for non-drone weapons in Phase 4.**

### 4.3 New modules

**Python (acsdg_c2 package):**
- `acsdg_c2/weapons/base.py` — `WeaponSystem` ABC
- `acsdg_c2/weapons/anvil.py` — Anduril Anvil (kinetic_quad)
- `acsdg_c2/weapons/coyote_b2.py` — Raytheon Coyote Block 2 (frag_jet)
- `acsdg_c2/weapons/dronehunter.py` — Fortem DroneHunter F700 (net_octo)
- `acsdg_c2/weapons/skyranger.py` — Rheinmetall Skyranger 30 (gun_turret)
- `acsdg_c2/classifier/bayesian.py` — naive-Bayes Gaussian classifier
- `acsdg_c2/predictor/cv_predictor.py` — class-conditioned constant-velocity predictor with class-specific process noise
- `acsdg_c2/cost_function/expected_utility.py` — cost-matrix builder
- `acsdg_c2/assignment/hungarian.py` — assignment solver wrapper (scipy.optimize.linear_sum_assignment)
- `acsdg_c2/dispatcher/dispatcher.py` — translates assignment to per-weapon EngagementOrders
- `acsdg_c2/supervisor/supervisor.py` — outcome monitor and re-plan trigger

**C++ (acsdg_c2 package):**
- `acsdg_c2/src/coyote_controller_node.cpp` — high-speed jet pursuit + proximity fuze
- `acsdg_c2/src/dronehunter_controller_node.cpp` — net deployment + tow recovery
- `acsdg_c2/src/skyranger_controller_node.cpp` — slew + AHEAD burst + ammo tracking

**Messages (acsdg_msgs package):**
- `acsdg_msgs/msg/ClassPosterior.msg` — track_id + class_probs[4]
- `acsdg_msgs/msg/WeaponState.msg` — weapon_id + type + available + ammo + pose
- `acsdg_msgs/msg/CostMatrix.msg` — flat + dims for visualization
- `acsdg_msgs/msg/EngagementOrder.msg` — extended with weapon_id + class_posterior + expected_pkill + cost_breakdown

**SDF models (acsdg_gazebo package):**
- `acsdg_gazebo/models/coyote_b2/model.sdf` — jet missile form (no rotors), kinematic + velocity-control plugin
- `acsdg_gazebo/models/dronehunter_f700/model.sdf` — octocopter form
- `acsdg_gazebo/models/skyranger_30/model.sdf` — fixed turret on ground base, with slewable turret link

**Visualization (new package):**
- `acsdg_viz/` — RViz overlay nodes + rqt cost-matrix dashboard plugin

### 4.4 Data flow

A track update fires the chain:
1. Radar/EO observation arrives at `sensor_fusion_node`.
2. Tracker associates and updates Kalman state.
3. Classifier likelihoods over (size, speed, RCS, IR) updated; posterior published as `ClassPosterior`.
4. Predictor generates a 5–10 s trajectory cone conditioned on argmax-class.
5. `cost_function` rebuilds the cost matrix from current weapon registry × active tracks.
6. `assignment` runs Hungarian; emits the new assignment.
7. `dispatcher` diffs the new vs current assignment; sends `EngagementOrder` for changed pairs.
8. Per-weapon controllers execute the order.
9. `supervisor` watches `/engagement/<weapon>/outcome`; on miss, requests cost-function rebuild excluding the failed pair.

Re-planning throttled to 5 Hz max.

---

## 5. WeaponSystem Abstraction

Defined as a Python ABC in `acsdg_c2/weapons/base.py`:

```python
class WeaponSystem(ABC):
    weapon_id: str             # "anvil_0", "coyote_0", etc.
    weapon_type: WeaponType    # Enum: KINETIC_QUAD | FRAG_JET | NET_OCTO | GUN_TURRET

    @abstractmethod
    def is_available(self) -> bool:
        """Resources available, not currently engaged, in working state."""

    @abstractmethod
    def engagement_envelope(self) -> EngagementEnvelope:
        """min/max range, min/max altitude, max closing speed."""

    @abstractmethod
    def pkill(self, target_class: TargetClass) -> float:
        """Per-class kill probability lookup (from §6 spec table)."""

    @abstractmethod
    def time_to_intercept(self, track: Track) -> float:
        """Geometric ETA. Drone: lead-pursuit time. Gun: ballistic time-of-flight."""

    @abstractmethod
    def resource_cost(self) -> float:
        """Normalized $cost or unit-deplete penalty per dispatch."""

    @abstractmethod
    def can_engage(self, track: Track) -> bool:
        """In envelope AND available AND ammo > 0."""

    @abstractmethod
    def dispatch(self, track: Track) -> EngagementOrder:
        """Issue the command and decrement resource."""

    @abstractmethod
    def state(self) -> WeaponState:
        """Current pose, ammo, availability — for visualization."""
```

---

## 6. Weapon Spec Parameters

All values traceable to public sources. Estimated values are flagged.

### 6.1 Anduril Anvil — Type: `KINETIC_QUAD`

**Sources:**
- [Anduril Anvil page](https://www.anduril.com/anvil)
- [The Defense Post — Falcon Peak demo (2025-10)](https://thedefensepost.com/2025/10/20/anduril-demos-cuas-falcon-peak/)
- [C4ISRNET — Anvil-M unveil (2023-10)](https://www.c4isrnet.com/unmanned/uas/2023/10/05/anduril-unveils-anvil-m-counter-drone-kit-that-can-defeat-smaller-uas/)
- [Anduril Anvil-M launch announcement](https://www.anduril.com/article/anvil-m-launch/)

**Sim values:**

| Parameter | Value | Confidence |
|---|---|---|
| Configuration | Electric quadcopter, 4 rotors, autonomous | Manufacturer-published |
| `mass` | 5.3 kg (11.6 lb) | Defense-press-published |
| `max_speed` | 45 m/s (100 mph; recent reports up to 89 m/s) | Defense-press-published |
| `cruise_speed` | 25 m/s | **Estimated** (class-typical) |
| `max_range` | 1.5 km | **Estimated** — Anvil is LOS-tethered to its launch box; intentionally short-range so the AI must choose Coyote/Skyranger for distant targets (preserves decision-space differentiation) |
| `endurance` | 25 min loiter | **Estimated** |
| `kill_radius` | 8 m collision (matches existing sim behavior) | Manufacturer-qualitative |
| `one_shot` | true | Manufacturer-published |
| `resource_cost` | 0.05 (~$50k normalized) | **Estimated** |
| `dimensions` (rotor span × body × height) | 1.0 × 0.6 × 0.25 m | **Estimated** from press photos |

**Pkill table per target class:**

| Class | Pkill | Reasoning |
|---|---|---|
| small-quad | 0.95 | Designed exactly for this — DoD Group 1 |
| group-1-fixed-wing | 0.85 | DoD Group 2 — within design envelope |
| group-3-loitering | 0.70 | Heavier, faster — collision still works but margins thinner |
| shahed-class | 0.40 | Mass exceeds Group 2 — interceptor outclassed |

### 6.2 Raytheon Coyote Block 2 — Type: `FRAG_JET`

**Sources:**
- [Raytheon Coyote — Wikipedia](https://en.wikipedia.org/wiki/Raytheon_Coyote)
- [RTX Coyote product page](https://www.rtx.com/raytheon/what-we-do/integrated-air-and-missile-defense/coyote)
- [TWZ — Army Coyote purchase plans](https://www.twz.com/drastic-increase-in-army-coyote-drone-interceptor-purchase-plans)
- [Army Recognition — Block 2C in CENTCOM (2025)](https://www.armyrecognition.com/news/army-news/2025/us-army-relies-on-coyote-block-2c-to-protect-bases-from-air-threats-in-the-middle-east)
- [designation-systems.net — Raytheon Coyote](https://www.designation-systems.net/dusrm/app4/coyote.html)

**Sim values:**

| Parameter | Value | Confidence |
|---|---|---|
| Configuration | Tube-launched, rocket-boosted, turbojet-sustained missile | Manufacturer-published |
| `max_speed` | 160 m/s (~Mach 0.45) | Defense-press-published |
| `mass` | 7 kg launch weight | **Estimated** |
| `max_range` | **5 km in sim** (real 10–15 km — see §16 sim-vs-real) | Defense-press-published, scaled |
| `endurance` | 4 min | Manufacturer-published |
| `frag_kill_radius_p50` | 5 m | **Estimated** (typical for ~2 kg tungsten frag class) |
| `frag_kill_radius_p30` | 8 m | **Estimated** |
| `proximity_fuze_arm_distance` | 100 m from target | **Estimated** (typical) |
| `one_shot` | true | Manufacturer-published |
| `resource_cost` | 0.10 (~$100k FY24 unit) | Defense-press-published |
| `dimensions` | 1.0 × 0.5 × 0.15 m | **Estimated** (canister-launched form factor) |
| `altitude_ceiling` | 4500 m (sim) | **Estimated** — matches §7 Group-3 loitering operational ceiling, the dominant target class for Coyote |

**Pkill table per target class:**

| Class | Pkill | Reasoning |
|---|---|---|
| small-quad | 0.60 | Frag pattern wasted on tiny target; quads can squeeze through gaps |
| group-1-fixed-wing | 0.85 | In-design-envelope target |
| group-3-loitering | 0.95 | Optimal target — slow, large, mid-altitude |
| shahed-class | 0.95 | Main intended threat — straight-line, predictable |

### 6.3 Fortem DroneHunter F700 — Type: `NET_OCTO`

**Sources:**
- [Fortem F700 product page](https://fortemtech.com/products/dronehunter-f700/)
- [Fortem F700 datasheet PDF](https://cdn.prod.website-files.com/648c3eaae99a3460f13becec/64a7deadd6b09be15aded1c7_MKT_DH%20F700-Data%20Sheet_20230119.pdf)
- [Avionics Today (2020)](https://www.aviationtoday.com/2020/04/02/fortems-f700-dronehunter-open-architecture-autonomous-drone-drone-combat/)
- [Lockheed Martin / Fortem investment (2026-04)](https://news.lockheedmartin.com/2026-04-22-Lockheed-Martin-Invests-25M-in-Fortem-Technologies)

**Sim values:**

| Parameter | Value | Confidence |
|---|---|---|
| Configuration | Octocopter (8 rotors), AI-piloted | Trade-press-published |
| `mass_with_payload` | 18 kg | Trade-press-published |
| `max_payload` | 2.27 kg | Trade-press-published |
| `max_towed_target` | 5.9 kg | Manufacturer-published |
| `max_speed` | 31 m/s | **Estimated** (Fortem reports "doubled" post-2024 to defeat Shahed tail-chase) |
| `cruise_speed` | 18 m/s | **Estimated** |
| `endurance` | 18 min | **Estimated** |
| `max_range` | 2 km kill / 10 km transit | Trade-press-published |
| `net_deploy_range` | 15 m | **Estimated** (NetGun standard) |
| `net_diameter` | 3 m | **Estimated** |
| `target_max_mass` | 25 kg (above this, capture fails) | Manufacturer-published (≤Group 3 capture; tow ≤5.9 kg) |
| `relaunch_time` | 180 s | Manufacturer-published |
| `one_shot` | false (relaunch <3 min after recovery) | Manufacturer-published |
| `resource_cost_per_engagement` | 0.20 first shot, 0.05 per relaunch | **Estimated** |

**Pkill table per target class:**

| Class | Pkill | Reasoning |
|---|---|---|
| small-quad | 0.85 | Easy capture, light enough to tow |
| group-1-fixed-wing | 0.70 | Faster, harder to net but within mass range |
| group-3-loitering | 0.50 | Mass approaching capture limit |
| shahed-class | 0.40 | Above mass limit — net entanglement may down it but no recovery |

### 6.4 Rheinmetall Skyranger 30 — Type: `GUN_TURRET`

**Sources:**
- [Skyranger 30 — Wikipedia](https://en.wikipedia.org/wiki/Skyranger_30)
- [Rheinmetall Skyranger 30 brochure PDF](https://www.rheinmetall.com/Rheinmetall%20Group/brochure-download/Air-Defence/D994e0222-Oerlikon-Skyranger-30.pdf)

**Sim values:**

| Parameter | Value | Confidence |
|---|---|---|
| Configuration | Fixed turret, 30 × 173 mm KCE revolver cannon | Manufacturer-published |
| `caliber` | 30 mm | Manufacturer-published |
| `muzzle_velocity` | 1075 m/s | Manufacturer-published |
| `rate_of_fire_sustained` | 200 rpm | Manufacturer-published |
| `rate_of_fire_cyclic` | 1200 rpm | Manufacturer-published |
| `max_engagement_range` | 3 km | Manufacturer-published |
| `min_engagement_range` | 0.1 km (AHEAD fuze arming) | **Estimated** |
| `elevation_envelope` | -10° to +85° | Manufacturer-published |
| `traverse` | 360° | Manufacturer-published |
| `slew_rate_az` | 120°/s | **Estimated** (class-typical) |
| `slew_rate_el` | 60°/s | **Estimated** |
| `ammo_capacity` | 300 rounds ready | Manufacturer-published |
| `burst_size` | 20 rounds per engagement | Doctrine-typical |
| `ahead_lethal_radius` | 10 m forward cone at burst | Defense literature |
| `ahead_subprojectiles` | 162 tungsten cylinders per round (PMC308) | Manufacturer-published |
| `one_shot` | false (multi-shot until ammo exhausted) | Manufacturer-published |
| `resource_cost_per_burst` | 0.001 (~$1k for 20 rounds) | **Estimated** |
| `dimensions` | 5.175 × 2.568 × 1.444 m turret | Manufacturer-published |

**Pkill table per target class:**

| Class | Pkill | Reasoning |
|---|---|---|
| small-quad | 0.90 | AHEAD frag pattern excels at small fast targets |
| group-1-fixed-wing | 0.85 | Designed-target for AHEAD |
| group-3-loitering | 0.70 | Larger target, more rounds needed; mostly in envelope |
| shahed-class | 0.40 | Mostly outside 3 km envelope at engagement geometry |

---

## 7. Threat Class Taxonomy

Four classes with distinguishing observable signatures:

| Class | Mass (kg) | Speed (m/s) | RCS (m²) | IR signature | Maneuver prior |
|---|---|---|---|---|---|
| `small-quad` | 0.5–5 | 5–20 | 0.01–0.05 | low (electric) | jinking, low altitude |
| `group-1-fixed-wing` | 5–25 | 25–50 | 0.05–0.3 | medium | smooth cruise, mid altitude |
| `group-3-loitering` | 25–150 | 30–80 | 0.3–1.5 | medium-high | slow turn, mid altitude |
| `shahed-class` | 150–600 | 50–120 | 1.5–4.0 | high (jet/IC engine) | straight-line, mid-low altitude |

Enemy SDF spawn picks a class per drone (configurable per launch); class determines mass, max_speed, visual model, and observable signature parameters.

---

## 8. Sensor Observation Model

Per-track observations at radar/EO update rate (typically 10 Hz):

| Observation | Distribution | Justification |
|---|---|---|
| `apparent_size` | N(class_size, σ_size²) | Visual extent in radians |
| `apparent_speed` | N(class_speed, σ_speed²) | Instantaneous track speed |
| `apparent_RCS` | N(class_rcs, σ_rcs²) | Radar cross-section |
| `apparent_IR` | N(class_ir, σ_ir²) | IR signature intensity |

Class-conditional Gaussians with means at class center and variances tuned (Phase 4 task) so:
- First 1–2 s of track: posterior diffuse (classifier uncertain)
- After ~3 s: posterior concentrates on true class

This makes the AI's classification non-trivial early but high-confidence by engagement decision time — exactly the showcase narrative.

---

## 9. Bayesian Classifier

Per-track posterior update on each new observation:

$$P(c \mid o_{1:t}) \propto P(c \mid o_{1:t-1}) \cdot \prod_d \mathcal{N}(o_{d,t};\, \mu_{c,d},\, \sigma_{c,d}^2)$$

Naive-Bayes assumption (feature independence given class) — justified for sim simplicity.

Prior: uniform over the 4 classes. Configurable per scenario.

Output: `ClassPosterior` message published per track, at observation rate.

---

## 10. Trajectory Predictor

Class-conditioned constant-velocity-with-noise. argmax-class selects:

| Class | Process noise | Resulting cone |
|---|---|---|
| `small-quad` | High σ_a (jinking) | Wide cone |
| `group-1-fixed-wing` | Low σ_a (smooth) | Narrow cone |
| `group-3-loitering` | Medium σ_a + slight turn-rate prior | Medium cone |
| `shahed-class` | Very low σ_a | Narrow straight-line cone |

Horizon: 5–10 s. Output: polygon vertices for RViz visualization.

---

## 11. Cost Function

For each (weapon $w_i$, track $t_j$) pair:

$$\mathbb{E}[P_\text{kill}](w_i, t_j) = \mathbb{1}[\text{inEnvelope}] \cdot \mathbb{1}[\text{isAvailable}] \cdot \sum_{c=1}^{4} P(c \mid o_{1:t_j}) \cdot P_\text{kill}(w_i, c)$$

$$C(w_i, t_j) = -\alpha \cdot \mathbb{E}[P_\text{kill}] + \beta \cdot \text{ResourceCost}(w_i) + \gamma \cdot \widetilde{\text{TimeToIntercept}}(w_i, t_j) + \delta \cdot \text{ThreatPriority}(t_j)$$

- $\widetilde{\text{TimeToIntercept}}$ is normalized to [0, 1] using max plausible engagement time (~30 s).
- $\text{ThreatPriority}$ is the existing per-track threat score from `c2_engine` (range [0, 1]).
- Out-of-envelope or unavailable: $C = +\infty$ (forbidden assignment).

**Default weights** (ROS params, tunable at runtime):

| Param | Default | Meaning |
|---|---|---|
| α | 1.0 | Kill probability — primary objective |
| β | 0.1 | Resource preservation — secondary |
| γ | 0.05 | Time pressure — light |
| δ | 0.5 | Threat priority — moderate |

---

## 12. Assignment

Hungarian solver via `scipy.optimize.linear_sum_assignment` on the cost matrix.

Non-square matrices (more weapons than threats or vice versa) padded with +∞ rows/cols.

Re-run triggers:
- New track creation
- Track loss (lost or despawned)
- Engagement outcome (success → free reusable weapon; failure → exclude failed pair)
- Resource depletion event (Skyranger ammo drops below threshold)

Throttle: 5 Hz max.

---

## 13. Adaptive Re-Tasking (Layer 5)

Engagement supervisor subscribes to `/engagement/<weapon>/outcome` topic.

On `MISS` or `ABORT`:
1. Mark the (weapon, track) pair as failed in cost-function blacklist.
2. Free the weapon for re-assignment if reusable; remove from registry if expended.
3. Trigger cost-matrix rebuild and re-assignment.
4. Publish replan event to `/c2/replan_events` with reason code (for visualization flash).

On `SUCCESS`:
1. Free weapon if reusable (DroneHunter post-recovery, Skyranger if ammo > 0).
2. Mark target as neutralized; despawn enemy (preserves existing despawn flow).

Edge case: if all weapons fail against a target, publish `un_engageable` flag for that track. Do not block other engagements; alert in log.

---

## 14. Visualization Requirements

Every showcased AI layer must have a visible diagnostic:

| Layer | Visualization |
|---|---|
| Perception (classification) | RViz `MarkerArray` with class-posterior bars over each track. Color-coded by dominant class. Updated at fusion rate. |
| Prediction | RViz polygon showing 5–10 s predicted trajectory cone per track. |
| Orchestration (cost matrix) | Cost-matrix heatmap published as `Image` topic, viewable in rqt. Annotated weapon/track labels. Current min-cost assignment highlighted. |
| Decision (dispatch) | Per-engagement INFO log: `"Coyote_0 → Track_3 \| Pkill_exp=0.95 \| cost=0.42 \| reason: out-of-envelope for all alternatives"`. Plus structured `/c2/decisions` topic. |
| Adaptive | Replan flash on cost-matrix heatmap; `/c2/replan_events` topic with reason codes. |
| Resource state | Per-weapon resource panels in rqt: Skyranger ammo countdown bar, one-shot drones marked spent (greyed out). |

---

## 15. Phasing

Each phase leaves the sim runnable end-to-end. Each phase produces an automated test suite increment.

### Phase 1 — Refactor + Anvil port

**Goal.** Establish target architecture without behavior change. Existing 4 kinetic interceptors become 4 Anvil `WeaponSystem` instances. Sim runs the existing demo identically.

**Tasks.**
- Create `WeaponSystem` ABC and `Anvil` subclass.
- Decompose `c2_engine_node.py` into stubs for `classifier` (returns argmax over 1 class), `cost_function` (target priority only), `assignment` (Hungarian), `dispatcher` (current behavior).
- Move existing controller into Anvil's dispatch path.
- Update launch files.

**Smoke test.** Wave trigger → 4 Anvils intercept 4 enemies at 6.81–8.0 m kill ranges (matching pre-refactor baseline).

**New automated tests.**
- `test_weapon_anvil.py` — engagement envelope and Pkill lookup.
- `test_assignment.py` — Hungarian on 4×4 toy cost matrix returns expected assignment.

### Phase 2 — Coyote Block 2

**Goal.** Add a fundamentally different airframe (jet, frag-warhead, missile-form). Replace 1 of 4 Anvils.

**Tasks.**
- New SDF `models/coyote_b2/model.sdf` with jet form (no rotors).
- New controller `coyote_controller_node.cpp` — high-speed pursuit, proximity fuze (kill on approach within 5 m, frag damage out to 8 m).
- New `Coyote` weapon class with frag Pkill curve.
- Cost function still simple (geometry-driven); Hungarian picks Coyote for far/fast targets.

**Smoke test.** Coyote engages target launched from far (>3 km), proximity-fuzes at 5 m, kills.

**New automated tests.**
- `test_weapon_coyote.py` — envelope, Pkill, proximity fuze logic.
- Integration: Coyote intercepts a Group-3 target spawn.

### Phase 3 — DroneHunter F700

**Goal.** Add a non-destructive, recovery-capable interceptor.

**Tasks.**
- New SDF `models/dronehunter_f700/model.sdf` (octocopter).
- New controller `dronehunter_controller_node.cpp` — pursuit, net deploy (visual: spawn net mesh under target), tow (carry to recovery zone or drop with parachute).
- DroneHunter weapon class with capture/tow Pkill curve.
- Inventory: 2 Anvil + 1 Coyote + 1 DroneHunter.

**Smoke test.** DroneHunter captures small target with net, tows to recovery zone (a defined SDF zone), returns to home pad.

**New automated tests.**
- `test_weapon_dronehunter.py` — envelope, mass-limit logic, recovery state machine.

### Phase 4 — Skyranger 30 + Multi-class Threats + Classifier

**Goal.** Add the fixed turret. Introduce threat-class diversity. Activate the Bayesian classifier and trajectory predictor.

**Tasks.**
- New SDF `models/skyranger_30/model.sdf` — fixed turret on ground base, slewable turret link.
- New controller `skyranger_controller_node.cpp` — slew to target, fire AHEAD burst (visual: muzzle flash + ray from turret to predicted intercept point + air-burst frag effect), ammo tracking, ammo depleted state.
- Skyranger weapon class.
- Multi-class enemy spawn: each enemy SDF receives class-specific properties.
- Radar node extended to publish `apparent_size`, `apparent_speed`, `apparent_RCS`, `apparent_IR` per track with class-conditional noise.
- Sensor fusion node extended to call classifier on each observation; publish `ClassPosterior`.
- Trajectory predictor module added; publishes prediction cones.
- Final inventory: 1 Anvil + 1 Coyote + 1 DroneHunter + 1 Skyranger.

**Smoke test.** Multi-wave with mixed threats (small-quad, group-1, group-3, shahed-class). Classifier converges to correct class within 3 s of track. Skyranger engages within range; others assigned per envelope.

**New automated tests.**
- `test_classifier.py` — posterior converges on each class given representative observation streams.
- `test_predictor.py` — trajectory cone covers true future position over 5 s horizon.
- `test_weapon_skyranger.py` — envelope, ammo decrement, AHEAD burst pattern.

### Phase 5 — Expected-Utility Cost Function + Adaptive Supervisor + Showcase Demo

**Goal.** Wire up the full AI orchestration loop with all 5 layers visible. Produce a showcase video.

**Tasks.**
- Implement full cost-function math (expected Pkill, resource cost, time-to-intercept, threat priority weighted).
- Wire α/β/γ/δ as ROS params with sensible defaults.
- Implement engagement supervisor with adaptive re-tasking on miss/abort.
- Build visualization layer (`acsdg_viz` package) — RViz overlays + cost-matrix heatmap + decision-log subscriber + resource panels.
- Define the showcase scenario: 3 waves of mixed threats arriving at staggered intervals, totaling 12 enemies across 4 classes.
- Run the showcase scenario; record video; verify all 5 AI layers visible in playback.

**Smoke test.** Full showcase scenario runs end-to-end; AI orchestrates assignments; all 12 threats handled (some engaged, some intercepted, possibly some surviving — acceptable as long as decisions are visible and reasoned).

**New automated tests.**
- `test_cost_function.py` — cost matrix matches hand-computed values for toy scenarios.
- `test_supervisor.py` — re-plan triggered on miss; failed pair excluded.
- Integration: showcase scenario completes without crashes.

**Deliverables.**
- Working sim: `ros2 launch acsdg_bringup acsdg_full.launch.py`.
- 2-minute showcase video.
- Updated `HANDOFF.md` describing final state.
- (Stretch) side-by-side homogeneous-Anvil run as illustrative comparison.

---

## 16. Sim-vs-Real Fidelity Tradeoffs

Documented honestly so the project defense can answer "where are you simplifying and why?":

- **Coyote 12 km range scaled to 5 km in sim.** Current world is small. Either expand world boundaries or document scaling. **Decision: scale to 5 km, document as "scaled but proportional to engagement geometry".**
- **Skyranger ballistics are ray-cast, not ballistic.** At 1075 m/s, ToF at 3 km is ~3 s — not negligible. We model AHEAD as ray-cast probability check at the burst point with geometric ToF. Caveat: "engagement-envelope simulation, not terminal ballistics."
- **Sensor noise is Gaussian and feature-independent given class.** Real sensors have correlated noise and richer features. Justification: simplification for sim tractability; can be relaxed in future work.
- **No comms / cyber attack modeling.** All channels assumed nominal. This is a kinetic-only defense layer.
- **No human-in-the-loop authorization.** Real systems generally require operator confirmation. Sim runs in auto-engage mode. Documented.
- **Pkill values are estimates calibrated qualitatively.** Real Pkill curves are classified. Spec sources every value to publication where possible; estimates are flagged.
- **Coyote propulsion is kinematic velocity-controlled, not jet thrust dynamics.** Simulates control envelope, not engine physics. Justified as engagement-level fidelity.
- **Engagement-geometry tuning.** Sim ranges: Anvil 1.5 km, DroneHunter 2 km kill, Skyranger 3 km, Coyote 5 km. World should be sized so each weapon dominates a clearly-different region (close-in / mid / far / very-far). World boundaries and weapon emplacement positions are tunable parameters and will be calibrated in Phase 5 to make the AI's decision space visible. Real-world ranges differ — see each weapon's row in §6 for the published value.

---

## 17. Open Questions / Risks

1. **Pkill calibration.** Initial values picked qualitatively to make weapon-class fit non-trivial. Monte Carlo calibration is stretch goal. Project falls back to "values defensible from public spec coverage."
2. **Sensor-noise model tuning.** Need σ values such that classification is unreliable for first 1–2 s and reliable after 3+ s. Iterative tuning during Phase 4.
3. **Skyranger model in Gazebo.** Fixed turret with 30 mm gunfire visualization is non-trivial. Plan: cone-ray + timed muzzle-flash + simple particle effect at burst point.
4. **Workspace size.** Coyote real range exceeds current world. Plan: scale to 5 km in sim; document.
5. **Enemy spawn variety.** SDF authoring time per class (4 distinct visual models). Plan: reuse open-source models (PX4 quad, generic fixed-wing, Shahed-style mesh).
6. **Adaptive re-tasking edge cases.** What if all weapons fail against a target? Track marked `un_engageable`; alert in log; do not block other engagements.
7. **Visualization scope creep.** RViz overlays + rqt heatmap + decision logs — risk of over-polish. Plan: minimum viable visualizations first; polish only if time permits.
8. **Workspace not under git.** Spec written to `docs/superpowers/specs/` but not committed (no git repo). Optional: `git init` to gain versioning.
9. **World sizing for showcase scenario.** The 4-weapon engagement geometry (Anvil 1.5 km / DroneHunter 2 km / Skyranger 3 km / Coyote 5 km) requires a world large enough that each weapon has its own dominance zone but small enough to render in real time. Plan: size at ~5 km diameter with weapons emplaced to create overlapping-but-differentiated coverage. Tune in Phase 5.

---

## 18. Success Criteria Summary

The project succeeds when:

1. **All 5 AI layers run live** in the showcase scenario (perception, prediction, classification, orchestration, decision-making, adaptive re-tasking).
2. **Each layer has a visible diagnostic** (RViz overlay, cost-matrix heatmap, decision log, predicted-trajectory cones, replan-event flash, resource panels).
3. **The 4-weapon heterogeneous inventory operates per published spec** — every parameter traceable to a citation.
4. **The showcase scenario engages 3 mixed-class waves end-to-end** without crashes; the AI's decisions are explicable from the published cost-function math.
5. **A 2-minute showcase video exists** demonstrating the AI's role at each layer for graduation defense.

---

## 19. Implementation Plan Reference

Implementation plan to be written by `superpowers:writing-plans` after this design is approved by the user. Plan will decompose each phase into TDD-style tasks with explicit test gates.
