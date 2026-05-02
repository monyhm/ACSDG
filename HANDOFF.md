# ACSDG Handoff — drop this into a fresh Claude chat to continue

You are picking up an in-progress simulation project. **Read the cited files** rather than trusting this summary blindly — files may have moved on.

## Project at a glance

- **What**: Anti-Counter-Swarm Drone Defense Gazebo sim. 4 enemy drones spawn at the perimeter and fly toward the origin; 4 interceptor drones at corner posts pursue, physically intercept (real <8 m proximity), and despawn the enemy bodies.
- **Where**: `~/acsdg_ws` (ROS 2 Humble + Gazebo Harmonic). Workspace not under git.
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
    scripts/gz_bridge_shim.py                   # ROS↔GZ data-plane bridge (NO yaw rotation — see below)
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

**Bridge** (`gz_bridge_shim.py`). ROS 2 Humble's stock `ros_gz_bridge` targets Ignition Fortress (`ign.msgs.*`); Gazebo Harmonic uses `gz.msgs.*` — same wire format, different type strings, stock bridge silently drops everything. The shim uses `gz.transport13` + `gz.msgs10` Python bindings directly. **It does NOT rotate cmd_vel.** `gz-sim-velocity-control-system` in Harmonic applies `cmd_vel` in WORLD frame. An earlier version of the shim rotated by cached yaw assuming body-frame; once bodies started tumbling (no rotational damping with gravity off) that rotation became actively wrong and interceptors drifted to z=2 km. Don't reintroduce the rotation.

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

## Auto-memory checkpoint

The latest checkpoint lives at `~/.claude/projects/-home-mal/memory/project_acsdg_status.md` and reflects this end-of-session state. Future Claude Code sessions in `~/` will load it automatically.
