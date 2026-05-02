# ACSDG Handoff — drop this into a fresh Claude chat to continue

You are picking up an in-progress simulation project. **Read the cited files** rather than trusting this summary blindly — files may have moved on.

## Project at a glance

- **What**: Anti-Counter-Swarm Drone Defense Gazebo sim. 4 enemy drones spawn at the perimeter and fly toward the origin; 4 interceptor drones at corner posts pursue, physically intercept (real <8 m proximity), and despawn the enemy bodies.
- **Where**: `~/acsdg_ws` (ROS 2 Humble + Gazebo Harmonic). Git-tracked on `main`, remote `github.com/monyhm/ACSDG`.
- **Build**: `cd ~/acsdg_ws && colcon build --symlink-install --allow-overriding <pkg>` (most edits land live via symlinks; `acsdg_c2` is C++ and needs a real rebuild on each change).
- **Run**: `LIBGL_ALWAYS_SOFTWARE=1 ros2 launch acsdg_bringup acsdg_full.launch.py headless:=false`
- **Trigger a wave**: `ros2 topic pub --times 5 -r 0.5 /mission/wave_trigger std_msgs/Bool '{data: true}'`. **Do not use `-1`** — DDS discovery sometimes drops the first publish before c2_engine's subscription is matched. Five publishes over 8 s reliably hits everyone. Each publish increments the wave counter; speeds default to 5 m/s for waves > 3, so it's safe.

## Demo flow that currently works end-to-end

1. Launch — sim comes up, all four enemies hover at their spawn perimeter (z=50), four interceptors hover at their corner homes (z=20). C2 is gated, so nothing happens yet.
2. Publish `/mission/wave_trigger` → enemy_driver activates enemies, c2_engine logs "Mission active — engagement enabled" and issues four `EngagementOrder`s (Hungarian assignment).
3. Interceptors climb to cruise z=50, fly out to lead-point of their assigned target, close to <8 m.
4. On contact, controller publishes `NEUTRALIZED` ack with kill-time int/tgt positions logged. enemy_driver receives the ack, stops commanding the enemy, fires a `gz service` call to remove the model.
5. The red drone vanishes from the GUI. Interceptor returns to home corner.

Verified kill ranges: 6.81 m, 7.42 m, 7.84 m, 8.00 m (kill radius is 8 m).

## Packages and load-bearing files

```
src/
  acsdg_bringup/launch/acsdg_full.launch.py     # full-stack launcher; passes use_gazebo_truth='true'
  acsdg_gazebo/
    worlds/military_base.sdf                    # world; <gravity>0 0 0</gravity> at line 15
    models/{enemy,interceptor}_drone/model.sdf  # kinematic, gz-sim-velocity-control-system, OdometryPublisher with <dimensions>3</dimensions>
    scripts/gz_bridge_shim.py                   # ROS↔GZ data-plane bridge; full inverse-quaternion rotation on cmd_vel — see below
    scripts/enemy_driver_node.py                # drives enemies on /mission/wave_trigger; despawns on NEUTRALIZED ack
    launch/sensors.launch.py                    # rf_node skipped in gazebo-truth mode
  acsdg_sensors/src/
    radar_node.cpp                              # use_gazebo_truth=true → reads /model/enemy_N/odometry
    rf_node.cpp                                 # hardcoded spiral sim — disabled in gazebo-truth
    sensor_fusion_node.cpp                      # NN-associated, gate=15m, drop=2s
  acsdg_c2/
    src/interceptor_controller_node.cpp         # per-interceptor C++ — 2D pursuit + altitude hold; subscribes to /model/interceptor_{id}/odometry for real pos_
    acsdg_c2/c2_engine_node.py                  # threat scoring + Hungarian assignment, GATED on /mission/wave_trigger
    acsdg_c2/interceptor_manager_node.py        # IDLE/PURSUING/RETURNING state machine
    acsdg_c2/mission_manager_node.py            # KPIs, wave logic
```

## Critical architecture details (don't relitigate these)

**Bridge** (`gz_bridge_shim.py`). ROS 2 Humble's stock `ros_gz_bridge` targets Ignition Fortress (`ign.msgs.*`); Gazebo Harmonic uses `gz.msgs.*` — same wire format, different type strings, stock bridge silently drops everything. The shim uses `gz.transport13` + `gz.msgs10` Python bindings directly. **`gz-sim-velocity-control-system` in Harmonic applies `cmd_vel` in BODY frame** (verified against gz-sim8 source — no world-frame SDF option, plugin writes `LinearVelocityCmd` unrotated). Controllers and `enemy_driver` publish in WORLD frame, so `_forward_twist` applies the **full inverse-quaternion rotation** R(q)^T·v_world using the body's orientation cached from `/model/<name>/odometry`. Yaw-only rotation was tried and rejected: simple `2*atan2(qz, qw)` extraction breaks when bodies pick up roll/pitch from numerical noise. The full quaternion form is robust to any orientation. Forwarded angular is also forced to (0,0,0) as defense-in-depth — drones never need angular cmd in this project. The earliest commit (51897bb3) had cmd_vel passthrough, which silently broke for enemies (yaw=π) — they flew opposite of commanded direction. The 2026-04-27 session removed yaw rotation under the false belief that the plugin used world frame, masking the bug as "interceptors mostly work, enemies broken." The 2026-05-02 fix (commit 5fa0955c) is the correct treatment.

**Gravity** (`worlds/military_base.sdf` line 15). `<gravity>0 0 0</gravity>` at world level. Drones are kinematic velocity-controlled bodies; without gravity off, the velocity-control plugin can't hold altitude between command ticks. Link-level `<gravity>false</gravity>` is silently ignored by Harmonic — don't trust it. World-level zero is the real fix; safe because every other include in the world is `<static>true</static>`.

**3D odometry** (both model SDFs). `gz-sim-odometry-publisher-system` defaults to **2D** in Harmonic; pose.z is reported as 0. The plugin needs `<dimensions>3</dimensions>` to publish real z. Without this, every downstream consumer reads z=0 and the controller hold-altitude loop runaways.

**Sensor mode** (`sensors.launch.py` + `acsdg_full.launch.py`). `rf_node` publishes detections from a **hardcoded spiral trajectory** unrelated to Gazebo. Combining radar (Gazebo-truth) + rf (hardcoded sim) creates ghost tracks that look real to fusion. The launch passes `use_gazebo_truth='true'` and `sensors.launch.py` skips rf_node under that condition. Don't reintroduce rf_node without giving it a Gazebo-truth path.

**Controller geometry** (`interceptor_controller_node.cpp`). Two non-obvious pieces:
- `pos_x_/y_/z_` is overwritten from `/model/interceptor_{id}/odometry` on every odom msg. Don't reintroduce the dead-reckoning integration that was here originally — it diverges from the real Gazebo body and the kill-radius check becomes fictional.
- Pursuit is **2D** (xy lead toward `tgt + tgt_v * t_go`); altitude is held by a separate outer P-loop at the constants `kCruiseZ=50`, `kAltKp=0.5`, `kMaxVz=3.0`. 3D lead pursuit was unstable — radar-noisy `tgt_vz` extrapolated over a 10+ second `t_go` blew interceptors hundreds of metres above their targets.

## What changed in the most recent session (2026-04-27)

1. Wave gate in `c2_engine_node.py`: `_mission_active` defaults False, set True on first `/mission/wave_trigger`. Imports `Bool` from `std_msgs.msg`.
2. Despawn in `enemy_driver_node.py`: `_ack_cb` calls `_despawn_enemy(tid)` on any NEUTRALIZED ack with `tid in SPAWN`. Fire-and-forget `subprocess.Popen(['gz', 'service', '-s', '/world/military_base/remove', ...])`. Don't block the rclpy callback.
3. Controller geometry: 2D lead + cruise altitude hold. Uses real interceptor odometry. See above.
4. Bridge shim: yaw-rotation removed.

All four user-facing behaviors that were broken at the start of the session now work: altitude is real, interceptors physically reach their targets, intercepts despawn the enemy, and engagement only happens after the wave is armed.

## Known issues (left in place — verify before relying on them)

- `echoguard_radar_plugin` and `rf360_plugin` shared libs aren't built — gz sim logs SystemLoader errors at startup. Sensor pipeline works without them. Cosmetic.
- `enemy_driver_node._control_loop` divides by `dist_xy` without a guard. Multi-wave stacking near origin can fling enemies out of the world boundary. Cosmetic for clean single-wave runs.
- DDS discovery race on `/mission/wave_trigger` — first publish often misses c2. Mitigated by always using `--times 5 -r 0.5`. A proper fix would set `transient_local` durability on the publisher and matching subscribers.
- Interceptor `pos_` defaults to `home_x/y/z` at construction; controller uses `has_odom_` flag to suppress cmd_vel until real odom arrives. If odometry stops mid-flight the controller will hold position (zero velocity), not return-to-home blindly.

## Useful diagnostic snippets

Real positions (bypasses the bridge shim — ground truth):
```bash
gz topic -e -n 1 -t /world/military_base/dynamic_pose/info
```

Live model list (post-despawn):
```bash
gz model --list
```

Kill-time geometry from controller:
```
Interceptor #N: NEUTRALISED target #M at range=R.RRm int(...) tgt(...)
```
If `range > 8m` you're back to phantom tracks — check that `rf_node` is off (`pgrep rf_node` should be empty) and that fusion's seeing radar from Gazebo truth (`/tmp/acsdg_sim.log` should show "RadarNode [GAZEBO-TRUTH]").

## What changed in session 2026-05-02

**Phase 1 of the AI-orchestrated heterogeneous-defense upgrade** + a load-bearing physics fix. Spec at `docs/superpowers/specs/2026-05-02-ai-orchestrated-heterogeneous-defense-design.md`, plan at `docs/superpowers/plans/2026-05-02-phase1-c2-refactor-anvil-port.md`.

1. **C2 modular refactor.** `c2_engine_node.py` is no longer monolithic. The threat-scoring + cost-matrix + Hungarian + EngagementOrder construction has been decomposed into composable modules under `acsdg_c2/`:
   - `weapons/` — `WeaponSystem` ABC + `Anvil` concrete class (Phase 1 has 4 Anvils; Phases 2–4 will add Coyote Block 2, DroneHunter F700, Skyranger 30).
   - `classifier/bayesian.py` — Phase-1 stub; full Bayesian classifier wired up in Phase 4.
   - `cost_function/threat_priority.py` — `score_threat(track)` extracted verbatim from legacy `_score`.
   - `cost_function/expected_utility.py` — `build_cost_matrix(weapons, tracks)` returns time-to-intercept geometry; Phase 5 swaps in the full expected-utility math.
   - `assignment/solver.py` — `assign(cost_matrix)` thin wrapper over the existing pure-Python Hungarian.
   - `dispatcher/dispatcher.py` — translates `(weapon, track)` decisions into `EngagementOrder` payloads. Phase 1 maps `weapon_id "anvil_<N>"` → `interceptor_id (N+1)` so the legacy C++ controller subscribing to `/interceptors/unit_{1..4}/state` keeps working unchanged.
   - 40 unit tests + 2 integration tests cover the refactor. ROS topic shapes preserved.
2. **Bridge shim full-quaternion rotation** (commit `5fa0955c`). `_forward_twist` now applies `R(q)^T · v_world` to convert world-frame ROS Twist into body-frame for Gazebo's `gz-sim-velocity-control-system`. The plugin applies cmd_vel in BODY frame (verified against gz-sim8 source), not WORLD as the prior session believed. See the "Bridge" architecture detail above.
3. **Interceptor spawn z bumped from 5 to 20** in `military_base.sdf` to clear ground-collision impulses that were tipping bodies on startup. With z=20 (matching `interceptor_controller_node` `home_z`), bodies stay at identity orientation `(0,0,0,1)`.
4. **Wave-trigger DDS race**: `--once` publishes can still miss `enemy_driver` due to discovery race. `--times 5 -r 0.5` remains the reliable pattern (per HANDOFF). Each publish increments wave count; speeds 5/8/12/5/5 m/s for waves 1–5.

End-of-session live verification: 4 interceptors NEUTRALISED 4 enemies at ranges 7.82–7.95 m (within HANDOFF baseline 6.81–8.0 m), zero breaches.

## Auto-memory checkpoint

The latest checkpoint lives at `~/.claude/projects/-home-mal/memory/project_acsdg_status.md` and reflects this end-of-session state. Future Claude Code sessions in `~/` will load it automatically.
