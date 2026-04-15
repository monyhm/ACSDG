#!/usr/bin/env python3
"""
mission_manager_node.py — Escalating wave spawner and mission scorekeeper.

Generates threat waves of increasing difficulty and tracks mission KPIs
(drones detected/neutralized/breached, cost saved, response time).

Publishes
---------
/mission/wave_status   std_msgs/String         JSON wave description
/mission/status        acsdg_msgs/MissionStatus
/mission/difficulty    std_msgs/Float32

Subscribes
----------
/mission/wave_trigger  std_msgs/Bool     True → spawn next wave
/sensors/fusion/targets std_msgs/String  JSON — tracks detection events
"""

import json
import random

import rclpy
from rclpy.node import Node
from rclpy.time import Time
from std_msgs.msg import Bool, Float32, String
from acsdg_msgs.msg import MissionStatus

# ── Wave definitions ──────────────────────────────────────────────────────
# Wave 4+ uses the last entry as a template and scales with difficulty.
WAVE_SPECS: list = [
    {'wave': 1, 'drones':  3, 'speed':  5.0, 'tactic': 'SPREAD'},
    {'wave': 2, 'drones':  5, 'speed':  8.0, 'tactic': 'PINCER'},
    {'wave': 3, 'drones':  8, 'speed': 12.0, 'tactic': 'SATURATION'},
    {'wave': 4, 'drones': 10, 'speed': 15.0, 'tactic': 'SATURATION'},
]

# Economics
INTERCEPT_COST_SAVED = 50_000    # USD: interceptor vs. missile ($1M)
BREACH_COST          = 0         # penalty could be added here


class MissionManagerNode(Node):

    def __init__(self) -> None:
        super().__init__('mission_manager_node')

        # ── Parameters ────────────────────────────────────────────────────
        self.declare_parameter('difficulty', 0.5)
        self._difficulty: float = float(
            self.get_parameter('difficulty').value)

        # ── Mission state ─────────────────────────────────────────────────
        self._wave_number       = 0
        self._drones_detected   = 0
        self._drones_neutralized = 0
        self._drones_breached   = 0
        self._cost_saved        = 0.0
        self._response_times_ms: list = []

        # Detection-time registry: target_id → rclpy.Time
        self._detect_times: dict = {}
        # Targets already counted
        self._counted_ids: set = set()

        # ── Publishers ────────────────────────────────────────────────────
        self._status_pub     = self.create_publisher(MissionStatus, '/mission/status', 10)
        self._wave_pub       = self.create_publisher(String, '/mission/wave_status', 10)
        self._difficulty_pub = self.create_publisher(Float32, '/mission/difficulty', 10)

        # ── Subscriptions ─────────────────────────────────────────────────
        self.create_subscription(Bool, '/mission/wave_trigger',
                                 self._on_wave_trigger, 10)
        self.create_subscription(String, '/sensors/fusion/targets',
                                 self._on_fusion_targets, 10)
        # Acknowledgements from controllers (optional integration point)
        self.create_subscription(String, '/mission/engagement_ack',
                                 self._on_engagement_ack, 10)

        # ── Timers ────────────────────────────────────────────────────────
        self.create_timer(1.0, self._publish_status)   # 1 Hz status
        self.create_timer(1.0, self._publish_difficulty)

        self.get_logger().info(
            f'MissionManagerNode: difficulty={self._difficulty:.1f}')

    # ── Wave spawning ──────────────────────────────────────────────────────

    def _on_wave_trigger(self, msg: Bool) -> None:
        if msg.data:
            self._spawn_next_wave()

    def _spawn_next_wave(self) -> None:
        self._wave_number += 1
        idx   = min(self._wave_number - 1, len(WAVE_SPECS) - 1)
        spec  = WAVE_SPECS[idx]

        if self._wave_number > len(WAVE_SPECS):
            # Escalating random beyond predefined waves
            n = random.randint(8, 12)
            s = 12.0 + 4.0 * self._difficulty
            tactic = random.choice(['SATURATION', 'PINCER', 'FEINT'])
            spec = {'wave': self._wave_number, 'drones': n, 'speed': s, 'tactic': tactic}

        # Scale by difficulty  (0.0 = easy: half speed/count; 1.0 = full)
        scale  = 0.5 + 0.5 * self._difficulty
        drones = max(1, round(spec['drones'] * scale))
        speed  = spec['speed'] * scale

        wave_info = {
            'wave':   self._wave_number,
            'drones': drones,
            'speed':  round(speed, 1),
            'tactic': spec['tactic'],
        }
        msg = String()
        msg.data = json.dumps(wave_info)
        self._wave_pub.publish(msg)

        self.get_logger().info(
            f"Wave {self._wave_number}: {drones} drones @ {speed:.1f} m/s "
            f"[{spec['tactic']}]")

    # ── Detection tracking ─────────────────────────────────────────────────

    def _on_fusion_targets(self, msg: String) -> None:
        try:
            targets = json.loads(msg.data)
        except Exception:
            return
        now = self.get_clock().now()
        for tgt in targets:
            tid   = tgt.get('id')
            state = tgt.get('state', '')
            if state == 'DETECTED' and tid not in self._counted_ids:
                self._drones_detected += 1
                self._detect_times[tid] = now
                self._counted_ids.add(tid)

    def _on_engagement_ack(self, msg: String) -> None:
        """Optional: controllers publish ack when a target is neutralized."""
        try:
            data    = json.loads(msg.data)
            tid     = data.get('target_id')
            outcome = data.get('outcome', 'NEUTRALIZED')
        except Exception:
            return

        if outcome == 'NEUTRALIZED':
            self._drones_neutralized += 1
            self._cost_saved += INTERCEPT_COST_SAVED
            if tid in self._detect_times:
                elapsed_ns = (self.get_clock().now() - self._detect_times.pop(tid)).nanoseconds
                self._response_times_ms.append(elapsed_ns / 1e6)
        elif outcome == 'BREACHED':
            self._drones_breached += 1

    # ── Periodic publishing ────────────────────────────────────────────────

    def _publish_status(self) -> None:
        msg = MissionStatus()
        msg.wave_number        = self._wave_number
        msg.drones_detected    = self._drones_detected
        msg.drones_neutralized = self._drones_neutralized
        msg.drones_breached    = self._drones_breached
        msg.cost_saved         = self._cost_saved
        msg.avg_response_time_ms = (
            sum(self._response_times_ms) / len(self._response_times_ms)
            if self._response_times_ms else 0.0
        )
        self._status_pub.publish(msg)

    def _publish_difficulty(self) -> None:
        dmsg = Float32()
        dmsg.data = self._difficulty
        self._difficulty_pub.publish(dmsg)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = MissionManagerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
