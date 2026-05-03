#!/usr/bin/env python3
"""
c2_engine_node.py -- Modular threat scoring + weapon-target assignment.

Phase 2 of the AI-orchestrated heterogeneous-defense upgrade. This node
composes per-module abstractions (classifier, cost_function, assignment,
dispatcher) and a registry of WeaponSystem instances. Phase 2 inventory is
1 Raytheon Coyote Block 2 (NE post) + 3 Anduril Anvil quadcopters
(NW/SE/SW posts). Phase 1 was 4 Anvils; Phase 3 will add a DroneHunter F700
and Phase 4 a Skyranger 30 — both via the same WeaponSystem ABC.

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
from acsdg_c2.fleet import FLEET, assert_matches_sdf
from acsdg_c2.weapons import Track, WeaponSystem

NUM_INTERCEPTORS = len(FLEET)


class C2EngineNode(Node):

    def __init__(self) -> None:
        super().__init__("c2_engine_node")

        # ── Weapons (composed from FLEET — see acsdg_c2/fleet.py) ───────
        self._weapons: List[WeaponSystem] = [
            slot.weapon_class(weapon_id=slot.weapon_id, home_position=slot.home)
            for slot in FLEET
        ]

        # Defense in depth: catch FLEET/SDF coord drift at startup, before
        # any controller can spawn at the wrong post. In test mode (acsdg_gazebo
        # not installed), downgrade to a warning since the test_fleet regression
        # test covers the same agreement check via the source-tree SDF.
        try:
            from ament_index_python.packages import (
                get_package_share_directory, PackageNotFoundError)
            sdf_path = (
                f"{get_package_share_directory('acsdg_gazebo')}/worlds/military_base.sdf")
            assert_matches_sdf(sdf_path)
        except PackageNotFoundError:
            self.get_logger().warn(
                "acsdg_gazebo not installed — skipping FLEET/SDF agreement check "
                "(likely test mode; CI covers this via test_fleet.py)")
        except (AssertionError, FileNotFoundError) as exc:
            self.get_logger().error(
                f"FLEET/SDF agreement check failed: {exc}. C2 refusing to start.")
            raise

        # -- Modules --
        self._classifier = Classifier()
        self._dispatcher = Dispatcher()

        # -- State --
        self._fused: Dict[int, dict] = {}
        self._mission_active: bool = False

        # -- Subscriptions --
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

        # -- Publishers --
        self._scores_pub = self.create_publisher(
            String, "/c2/threat_scores", 10)
        self._engage_pub = self.create_publisher(
            EngagementOrder, "/c2/engagement_orders", 10)

        # -- 10 Hz control loop --
        self.create_timer(0.1, self._on_timer)
        # Build a human-readable inventory summary so the launch log makes
        # it obvious what classes the C2 is composing this run.
        inventory = ", ".join(f"{w.__class__.__name__}({w.weapon_id})"
                              for w in self._weapons)
        self.get_logger().info(
            f"C2EngineNode ready (10 Hz, {len(self._weapons)} weapons: {inventory})")

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
                # Either PURSUING or RETURNING -- both mean unavailable.
                self._weapons[idx].mark_engaged(target_id=int(msg.target_id))

    def _on_wave(self, msg: Bool) -> None:
        if msg.data and not self._mission_active:
            self._mission_active = True
            self.get_logger().info("Mission active -- engagement enabled")

    def _on_engagement_ack(self, msg: String) -> None:
        try:
            data = json.loads(msg.data)
            tid = int(data.get("target_id", -1))
        except Exception:
            return
        # Phase 1: ack is informational. Weapon slot frees when InterceptorState
        # arrives with status=IDLE on the next 10 Hz tick. Phase 4's adaptive
        # supervisor will use this hook for re-tasking on miss/abort.
        # TODO(phase4): if outcome == MISS, mark (weapon_id, tid) failed and
        # trigger cost-matrix rebuild excluding the failed pair.
        del tid  # silence unused-variable warning

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
            tracks.append(track.with_score(score_threat(track)))

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
                f"Order: {weapon.weapon_id} -> threat {track.track_id}  "
                f"score={track.threat_score:.2f}  "
                f"priority={order.priority}")


def main(args=None) -> None:
    rclpy.init(args=args)
    node = C2EngineNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
