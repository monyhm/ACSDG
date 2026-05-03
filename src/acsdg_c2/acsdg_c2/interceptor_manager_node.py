#!/usr/bin/env python3
"""
interceptor_manager_node.py — Fleet state manager for 4 interceptors.

Maintains authoritative state (position, status, target) for the 4-unit
interceptor fleet.  Bridges engagement orders from C2 engine to per-unit
state topics consumed by the dashboard and flight controllers.

Subscribes
----------
/c2/engagement_orders          acsdg_msgs/EngagementOrder
/interceptors/{id}/position    geometry_msgs/Point   (from controller)

Publishes
---------
/interceptors/{id}/state       acsdg_msgs/InterceptorState  @ 10 Hz
/interceptors/fleet_status     std_msgs/String              @ 10 Hz (JSON)
"""

import json

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point
from std_msgs.msg import String
from acsdg_msgs.msg import EngagementOrder, InterceptorState

from acsdg_c2.fleet import FLEET

# ── Fleet configuration ───────────────────────────────────────────────────
# Home positions sourced from FLEET — single source of truth across launch,
# bridge, c2_engine, and this manager. See acsdg_c2/fleet.py.
HOME: dict = {slot.interceptor_id: slot.home for slot in FLEET}


class InterceptorManagerNode(Node):

    def __init__(self) -> None:
        super().__init__('interceptor_manager_node')

        # Internal state per interceptor
        self._state: dict = {}
        for iid, (hx, hy, hz) in HOME.items():
            self._state[iid] = {
                'id':        iid,
                'position':  [hx, hy, hz],
                'status':    'IDLE',
                'target_id': 0,
            }

        # ── Publishers ────────────────────────────────────────────────────
        self._state_pubs: dict = {}
        for iid in HOME:
            self._state_pubs[iid] = self.create_publisher(
                InterceptorState, f'/interceptors/unit_{iid}/state', 10)

        self._fleet_pub = self.create_publisher(String, '/interceptors/fleet_status', 10)

        # ── Subscriptions ─────────────────────────────────────────────────
        self.create_subscription(
            EngagementOrder, '/c2/engagement_orders',
            self._on_order, 10)

        # Engagement outcomes: flip PURSUING → RETURNING so the position
        # check below can eventually flip back to IDLE. Without this the
        # fleet gets stuck PURSUING forever after its first assignment.
        self.create_subscription(
            String, '/mission/engagement_ack',
            self._on_engagement_ack, 10)

        for iid in HOME:
            self.create_subscription(
                Point, f'/interceptors/unit_{iid}/position',
                lambda msg, i=iid: self._on_position(msg, i),
                10)

        # ── Timer: 10 Hz ──────────────────────────────────────────────────
        self.create_timer(0.1, self._on_timer)
        self.get_logger().info(
            f'InterceptorManagerNode: {len(HOME)} interceptors at '
            + ', '.join(f'#{i}={HOME[i]}' for i in sorted(HOME)))

    # ── Callbacks ─────────────────────────────────────────────────────────

    def _on_order(self, msg: EngagementOrder) -> None:
        iid = int(msg.interceptor_id)
        if iid not in self._state:
            self.get_logger().warn(f'Unknown interceptor id {iid} in order')
            return
        prev = self._state[iid]['status']
        self._state[iid]['status']    = 'PURSUING'
        self._state[iid]['target_id'] = int(msg.target_id)
        self.get_logger().info(
            f'Interceptor #{iid}: {prev} → PURSUING target #{msg.target_id}')

    def _on_engagement_ack(self, msg: String) -> None:
        try:
            data = json.loads(msg.data)
            iid  = int(data.get('interceptor_id', 0))
        except Exception:
            return
        if iid in self._state and self._state[iid]['status'] == 'PURSUING':
            self._state[iid]['status']    = 'RETURNING'
            self._state[iid]['target_id'] = 0
            self.get_logger().info(f'Interceptor #{iid}: PURSUING → RETURNING')

    def _on_position(self, msg: Point, iid: int) -> None:
        """Update interceptor position from the flight controller."""
        self._state[iid]['position'] = [msg.x, msg.y, msg.z]

        # Auto-transition: if interceptor is very close to home → IDLE
        hx, hy, hz = HOME[iid]
        dx = msg.x - hx; dy = msg.y - hy; dz = msg.z - hz
        dist = (dx*dx + dy*dy + dz*dz) ** 0.5
        if self._state[iid]['status'] == 'RETURNING' and dist < 5.0:
            self._state[iid]['status']    = 'IDLE'
            self._state[iid]['target_id'] = 0
            self.get_logger().info(f'Interceptor #{iid}: RETURNING → IDLE')

    # ── 10 Hz publish ─────────────────────────────────────────────────────

    def _on_timer(self) -> None:
        for iid, st in self._state.items():
            msg = InterceptorState()
            msg.id           = iid
            msg.position.x   = st['position'][0]
            msg.position.y   = st['position'][1]
            msg.position.z   = st['position'][2]
            msg.status       = st['status']
            msg.target_id    = st['target_id']
            self._state_pubs[iid].publish(msg)

        fleet_data = {
            'interceptors': [
                {
                    'id':        iid,
                    'status':    st['status'],
                    'target_id': st['target_id'],
                    'position':  {
                        'x': round(st['position'][0], 1),
                        'y': round(st['position'][1], 1),
                        'z': round(st['position'][2], 1),
                    },
                }
                for iid, st in sorted(self._state.items())
            ]
        }
        fleet_msg = String()
        fleet_msg.data = json.dumps(fleet_data)
        self._fleet_pub.publish(fleet_msg)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = InterceptorManagerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
