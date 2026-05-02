#!/usr/bin/env python3
"""
c2_engine_node.py — Threat scoring and optimal interceptor assignment.

Subscribes
----------
/sensors/fusion/targets   std_msgs/String   JSON array of FusedTargets
/interceptors/{1-4}/state acsdg_msgs/InterceptorState

Publishes
---------
/c2/threat_scores         std_msgs/String   JSON [{"id", "score", "state"}, ...]
/c2/engagement_orders     acsdg_msgs/EngagementOrder

Algorithm
---------
Threat score = 0.5·proximity + 0.3·heading + 0.2·speed
Optimal assignment via Hungarian algorithm (minimize total distance cost).
Only IDLE interceptors and DETECTED/unassigned threats participate.
Runs at 10 Hz.
"""

import json
import math

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, String
from acsdg_msgs.msg import InterceptorState, EngagementOrder

from acsdg_c2.hungarian import hungarian

# ── Scoring constants ─────────────────────────────────────────────────────
MAX_RANGE = 400.0   # m — normalisation range for proximity score
MAX_SPEED = 20.0    # m/s — normalisation speed

# Interceptor top speed — must match kMaxSpeed in interceptor_controller_node.cpp.
# Used in the assignment-cost denominator so closing geometry affects pairing.
INTERCEPTOR_MAX_SPEED = 15.0

# Number of interceptors managed by the fleet
NUM_INTERCEPTORS = 4


class C2EngineNode(Node):

    def __init__(self) -> None:
        super().__init__('c2_engine_node')

        # Keyed by uint32 id
        self._targets: dict       = {}   # id → JSON dict from fusion node
        self._interceptors: dict  = {}   # id → InterceptorState msg
        # Track which threats already have an engagement order issued this cycle
        self._assigned_threats: set = set()
        # Engagement is gated on the mission being active. Without this gate,
        # radar sees the stationary enemies at spawn before any wave is
        # triggered, fusion mints tracks, and interceptors fly out and kill
        # them before the operator has a chance to start the demo.
        self._mission_active: bool = False

        # ── Subscriptions ─────────────────────────────────────────────────
        self.create_subscription(
            String, '/sensors/fusion/targets',
            self._on_targets, 10)

        self.create_subscription(
            Bool, '/mission/wave_trigger',
            self._on_wave, 10)

        # Engagement outcomes from interceptor controllers — clears the
        # assignment so C2 can retask if the target survives (LOST) or
        # is gone (NEUTRALIZED).
        self.create_subscription(
            String, '/mission/engagement_ack',
            self._on_engagement_ack, 10)

        for iid in range(1, NUM_INTERCEPTORS + 1):
            self.create_subscription(
                InterceptorState,
                f'/interceptors/unit_{iid}/state',
                lambda msg: self._on_interceptor_state(msg),
                10)

        # ── Publishers ────────────────────────────────────────────────────
        self._scores_pub   = self.create_publisher(String, '/c2/threat_scores', 10)
        self._engage_pub   = self.create_publisher(EngagementOrder, '/c2/engagement_orders', 10)

        # ── Timer: 10 Hz ──────────────────────────────────────────────────
        self.create_timer(0.1, self._on_timer)
        self.get_logger().info('C2EngineNode ready (10 Hz)')

    # ── Callbacks ─────────────────────────────────────────────────────────

    def _on_targets(self, msg: String) -> None:
        try:
            targets = json.loads(msg.data)
            self._targets = {int(t['id']): t for t in targets}
        except Exception as exc:
            self.get_logger().warn(f'Target parse error: {exc}')

    def _on_interceptor_state(self, msg: InterceptorState) -> None:
        self._interceptors[int(msg.id)] = msg

    def _on_wave(self, msg: Bool) -> None:
        if msg.data and not self._mission_active:
            self._mission_active = True
            self.get_logger().info('Mission active — engagement enabled')

    def _on_engagement_ack(self, msg: String) -> None:
        try:
            data = json.loads(msg.data)
            tid  = int(data.get('target_id', -1))
        except Exception:
            return
        # Any outcome releases the assignment slot.
        self._assigned_threats.discard(tid)

    # ── 10 Hz control loop ────────────────────────────────────────────────

    def _on_timer(self) -> None:
        if not self._mission_active:
            return
        if not self._targets:
            return

        # ── Score every active threat ─────────────────────────────────────
        scores: dict = {}
        for tid, tgt in self._targets.items():
            state = tgt.get('state', 'UNKNOWN')
            if state in ('DETECTED', 'TARGETED'):
                scores[tid] = self._score(tgt)

        # Publish threat scores for dashboard
        score_list = [
            {'id': tid, 'score': round(s, 3),
             'state': self._targets[tid].get('state', 'UNKNOWN')}
            for tid, s in scores.items()
        ]
        score_msg = String()
        score_msg.data = json.dumps(score_list)
        self._scores_pub.publish(score_msg)

        # ── Assignment: IDLE interceptors ↔ DETECTED threats ─────────────
        idle_ids = sorted(
            iid for iid, ist in self._interceptors.items()
            if ist.status == 'IDLE'
        )
        unassigned_ids = sorted(
            tid for tid, tgt in self._targets.items()
            if tgt.get('state') == 'DETECTED'
               and tid not in self._assigned_threats
        )

        if not idle_ids or not unassigned_ids:
            return

        # Cost matrix: rows = idle interceptors, cols = unassigned threats.
        # Cost is an approximate time-to-intercept rather than raw distance —
        # so a far-but-inbound threat is preferred over a close-but-crossing
        # one (its closing rate credits the denominator).
        cost: list = []
        for iid in idle_ids:
            ist = self._interceptors[iid]
            row = []
            for tid in unassigned_ids:
                tgt = self._targets[tid]
                dx = tgt['position']['x'] - ist.position.x
                dy = tgt['position']['y'] - ist.position.y
                dz = tgt['position']['z'] - ist.position.z
                d  = math.sqrt(dx*dx + dy*dy + dz*dz)
                if d < 1e-6:
                    row.append(0.0)
                    continue
                # Target velocity component pointing AT the interceptor
                # (positive means target is closing in on it).
                urx, ury, urz = -dx / d, -dy / d, -dz / d
                vx = tgt['velocity']['x']
                vy = tgt['velocity']['y']
                vz = tgt['velocity']['z']
                closing = vx * urx + vy * ury + vz * urz
                eff = max(1.0, INTERCEPTOR_MAX_SPEED + closing)
                row.append(d / eff)
            cost.append(row)

        assignments = hungarian(cost)

        for agent_idx, task_idx in assignments:
            iid = idle_ids[agent_idx]
            tid = unassigned_ids[task_idx]
            score = scores.get(tid, 0.0)

            order = EngagementOrder()
            order.target_id      = tid
            order.interceptor_id = iid
            order.priority       = min(255, max(0, int(score * 255)))
            order.issued_at      = self.get_clock().now().to_msg()

            self._engage_pub.publish(order)
            self._assigned_threats.add(tid)

            self.get_logger().info(
                f'Order: interceptor {iid} → threat {tid}  '
                f'score={score:.2f}  priority={order.priority}')

        # Forget assignments once a full cycle of scoring runs without the
        # threat reappearing (it was dropped from the fusion node)
        self._assigned_threats &= set(self._targets.keys())

    # ── Threat scoring ────────────────────────────────────────────────────

    def _score(self, tgt: dict) -> float:
        px = tgt['position']['x']
        py = tgt['position']['y']
        pz = tgt['position']['z']
        vx = tgt['velocity']['x']
        vy = tgt['velocity']['y']
        vz = tgt['velocity']['z']

        dist  = math.sqrt(px*px + py*py + pz*pz)
        speed = math.sqrt(vx*vx + vy*vy + vz*vz)

        proximity = max(0.0, 1.0 - dist / MAX_RANGE)
        speed_s   = min(1.0, speed / MAX_SPEED)

        if dist > 1e-6 and speed > 1e-6:
            toward_x = -px / dist
            toward_y = -py / dist
            toward_z = -pz / dist
            v_unit_x = vx / speed
            v_unit_y = vy / speed
            v_unit_z = vz / speed
            heading = max(0.0,
                toward_x * v_unit_x + toward_y * v_unit_y + toward_z * v_unit_z)
        else:
            heading = 0.0

        return 0.5 * proximity + 0.3 * heading + 0.2 * speed_s


def main(args=None) -> None:
    rclpy.init(args=args)
    node = C2EngineNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
