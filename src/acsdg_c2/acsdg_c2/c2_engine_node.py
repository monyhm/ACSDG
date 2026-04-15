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
from std_msgs.msg import String
from acsdg_msgs.msg import InterceptorState, EngagementOrder

from acsdg_c2.hungarian import hungarian

# ── Scoring constants ─────────────────────────────────────────────────────
MAX_RANGE = 400.0   # m — normalisation range for proximity score
MAX_SPEED = 20.0    # m/s — normalisation speed

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

        # ── Subscriptions ─────────────────────────────────────────────────
        self.create_subscription(
            String, '/sensors/fusion/targets',
            self._on_targets, 10)

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

    # ── 10 Hz control loop ────────────────────────────────────────────────

    def _on_timer(self) -> None:
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

        # Cost matrix: rows = idle interceptors, cols = unassigned threats
        cost: list = []
        for iid in idle_ids:
            ist = self._interceptors[iid]
            row = []
            for tid in unassigned_ids:
                tgt = self._targets[tid]
                dx = ist.position.x - tgt['position']['x']
                dy = ist.position.y - tgt['position']['y']
                dz = ist.position.z - tgt['position']['z']
                row.append(math.sqrt(dx*dx + dy*dy + dz*dz))
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
