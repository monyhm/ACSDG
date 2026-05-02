#!/usr/bin/env python3
"""
enemy_driver_node.py — Drives enemy drones toward the defended origin in Gazebo.

Each enemy drone is driven by publishing geometry_msgs/Twist on /enemy_N/cmd_vel,
which ros_gz_bridge forwards to /model/enemy_N/cmd_vel consumed by the
gz-sim-velocity-control-system plugin.

Behaviour
---------
- Drones fly toward origin (0, 0) at 50m AGL by default.
- Speed increases with wave number and difficulty.
- When a drone reaches kill_radius (30m from origin), it is "reset" —
  velocity set to zero and position logged. (Actual respawn requires gz service;
  we simply pause then re-engage from a new computed position.)
- Subscribes to /mission/wave_trigger (std_msgs/Bool) to increment wave.
- Subscribes to /mission/difficulty (std_msgs/Float32) to scale speed.

Publishes: /enemy_N/cmd_vel  geometry_msgs/Twist  @ 10 Hz
"""

import json
import math
import subprocess
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool, Float32, String


# ── Drone configuration ───────────────────────────────────────────────────────

# Spawn positions match military_base.sdf include poses
SPAWN = {
    1: (300.0,   0.0, 30.0),
    2: (  0.0, 300.0, 30.0),
    3: (-300.0,  0.0, 30.0),
    4: (  0.0, -300.0, 30.0),
}

# Base speed per wave (m/s). Wave 0 = idle (drones not yet launched).
WAVE_SPEEDS = {
    1: 5.0,   # Wave 1: slow spread
    2: 8.0,   # Wave 2: pincer
    3: 12.0,  # Wave 3: saturation
}

# Which drones participate per wave
WAVE_DRONES = {
    1: [1, 2, 3],
    2: [1, 2, 3, 4, 5],   # only 4 exist; clamp
    3: [1, 2, 3, 4],
}

TARGET_ALTITUDE = 50.0   # metres AGL
KILL_RADIUS     = 30.0   # metres from origin — drone "hit" or "passed"
HOLD_Z_KP       = 0.5    # altitude hold proportional gain
MAX_DRONES      = 4


class EnemyDriverNode(Node):

    def __init__(self):
        super().__init__('enemy_driver_node')

        self._wave       = 0        # current wave (0 = not started)
        self._difficulty = 0.5
        self._active     = set()    # set of drone indices currently flying

        # Per-drone state
        self._pos = {i: list(SPAWN[i]) for i in range(1, MAX_DRONES + 1)}
        self._has_odom = {i: False for i in range(1, MAX_DRONES + 1)}

        # Publishers — one per drone
        self._cmd_pubs = {}
        for i in range(1, MAX_DRONES + 1):
            self._cmd_pubs[i] = self.create_publisher(
                Twist, f'/enemy_{i}/cmd_vel', 10)

        # Shared engagement-outcome channel. When a drone reaches the base
        # perimeter we emit a BREACHED ack so mission_manager / learning_node
        # get a real signal instead of depending on proximity heuristics.
        # NB: target_id here is the enemy model index (1..4); the fusion node
        # assigns its own auto-incrementing track ids, so cross-referencing
        # BREACHED acks back to a specific fused target is approximate.
        self._ack_pub = self.create_publisher(
            String, '/mission/engagement_ack', 10)

        # Odometry subscribers (position feedback from Gazebo)
        for i in range(1, MAX_DRONES + 1):
            self.create_subscription(
                Odometry,
                f'/model/enemy_{i}/odometry',
                lambda msg, idx=i: self._odom_cb(idx, msg),
                10,
            )

        # Mission control
        self.create_subscription(Bool,    '/mission/wave_trigger', self._wave_cb,  10)
        self.create_subscription(Float32, '/mission/difficulty',   self._diff_cb,  10)
        # Engagement outcomes: when one of OUR drones is NEUTRALIZED we stop
        # commanding it so it doesn't keep flying toward origin (and re-
        # triggering kills until fusion decides to drop the track).
        self.create_subscription(
            String, '/mission/engagement_ack', self._ack_cb, 10)

        # 10 Hz control loop
        self.create_timer(0.1, self._control_loop)

        self.get_logger().info(
            'EnemyDriverNode ready. Waiting for /mission/wave_trigger ...')

    # ── Callbacks ─────────────────────────────────────────────────────────────

    def _odom_cb(self, idx: int, msg: Odometry):
        p = msg.pose.pose.position
        self._pos[idx] = [p.x, p.y, p.z]
        self._has_odom[idx] = True

    def _wave_cb(self, msg: Bool):
        if msg.data:
            self._wave += 1
            drones = [d for d in WAVE_DRONES.get(self._wave, [1,2,3,4])
                      if d <= MAX_DRONES]
            self._active = set(drones)
            speed = self._wave_speed()
            self.get_logger().info(
                f'Wave {self._wave} triggered — drones {sorted(self._active)}, '
                f'speed={speed:.1f} m/s (difficulty={self._difficulty:.2f})')

    def _diff_cb(self, msg: Float32):
        self._difficulty = float(msg.data)
        self.get_logger().info(f'Difficulty updated to {self._difficulty:.2f}')

    def _ack_cb(self, msg: String):
        try:
            data = json.loads(msg.data)
            tid  = int(data.get('target_id', 0))
            outc = str(data.get('outcome', ''))
        except Exception:
            return
        if outc != 'NEUTRALIZED' or tid not in SPAWN:
            return
        # Stop commanding the drone and hand it a zero twist once so the
        # velocity-control plugin bleeds off any inertia.
        if tid in self._active:
            self._active.discard(tid)
            self._cmd_pubs[tid].publish(Twist())
            self.get_logger().info(
                f'Enemy {tid} NEUTRALIZED — stopping commands.')
        # Despawn unconditionally so duplicate ACKs and pre-wave kills are
        # both handled. The Gazebo service is idempotent (already-removed
        # model returns false harmlessly).
        self._despawn_enemy(tid)

    def _despawn_enemy(self, tid: int) -> None:
        """Fire-and-forget Gazebo entity removal so the red drone disappears
        from the scene the moment the kill ack lands. Popen, not run — the
        rclpy callback thread must not block on a 1-second service timeout."""
        try:
            subprocess.Popen([
                'gz', 'service',
                '-s', '/world/military_base/remove',
                '--reqtype', 'gz.msgs.Entity',
                '--reptype', 'gz.msgs.Boolean',
                '--timeout', '1000',
                '--req', f'name: "enemy_{tid}", type: MODEL',
            ], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
            self.get_logger().info(f'Enemy {tid} despawn requested.')
        except Exception as exc:
            self.get_logger().warn(f'Despawn enemy_{tid} failed: {exc}')

    # ── Speed calculation ─────────────────────────────────────────────────────

    def _wave_speed(self) -> float:
        base = WAVE_SPEEDS.get(self._wave, 5.0)
        # Difficulty scales speed 0.5×..1.5×
        return base * (0.5 + self._difficulty)

    # ── Control loop ──────────────────────────────────────────────────────────

    def _control_loop(self):
        if self._wave == 0:
            return  # No wave triggered yet — drones hold position

        speed = self._wave_speed()

        for i in range(1, MAX_DRONES + 1):
            twist = Twist()

            if i not in self._active:
                # Drone not in this wave — hold still
                self._cmd_pubs[i].publish(twist)
                continue

            x, y, z = self._pos[i]
            dist_xy = math.sqrt(x * x + y * y)

            if dist_xy < KILL_RADIUS:
                # Drone reached the base — stop, log, and emit BREACHED ack
                self._cmd_pubs[i].publish(twist)
                if self._has_odom[i]:
                    self.get_logger().warn(
                        f'Enemy {i} BREACHED base (dist={dist_xy:.1f}m)')
                    ack = String()
                    ack.data = json.dumps({
                        'target_id':      i,
                        'interceptor_id': 0,
                        'outcome':        'BREACHED',
                    })
                    self._ack_pub.publish(ack)
                    self._active.discard(i)
                continue

            # Compute unit vector toward origin in XY plane
            ux = -x / dist_xy
            uy = -y / dist_xy

            # Apply speed
            twist.linear.x = ux * speed
            twist.linear.y = uy * speed

            # Altitude hold: drive toward TARGET_ALTITUDE
            alt_err = TARGET_ALTITUDE - z
            twist.linear.z = max(-3.0, min(3.0, HOLD_Z_KP * alt_err))

            self._cmd_pubs[i].publish(twist)


# ── Entry point ───────────────────────────────────────────────────────────────

def main(args=None):
    rclpy.init(args=args)
    node = EnemyDriverNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
