"""Coyote engagement integration test (in-process rclpy harness).

Runs C2EngineNode + a SpyNode in a MultiThreadedExecutor, without Gazebo or
launch_testing. Closes the M-1 reviewer item from the Phase-2 final code review
(Coyote integration test).

Two scenarios:
  1. Close-range engagement: assert C2 routes the order to interceptor_id=1
     (the Coyote slot per FLEET).
  2. MISS-ack handling: confirm C2 doesn't crash when a controller publishes
     outcome=MISS, and that subsequent dispatch still works.
"""
import json
import threading
import time

import pytest
import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from std_msgs.msg import Bool, String

from acsdg_msgs.msg import EngagementOrder, FusedTarget, InterceptorState

from acsdg_c2.c2_engine_node import C2EngineNode


class SpyNode(Node):
    """Test driver — publishes synthetic targets and watches orders/acks."""

    def __init__(self) -> None:
        super().__init__('integration_spy')
        self.last_order: EngagementOrder | None = None
        self.last_ack: dict | None = None
        self._wave_pub = self.create_publisher(Bool, '/mission/wave_trigger', 10)
        self._fused_pub = self.create_publisher(String, '/sensors/fusion/targets', 10)
        self._target_pubs: dict[int, "rclpy.publisher.Publisher"] = {}
        self._state_pubs: dict[int, "rclpy.publisher.Publisher"] = {}
        self._ack_pub_for_test = self.create_publisher(String, '/mission/engagement_ack', 10)
        self.create_subscription(EngagementOrder, '/c2/engagement_orders', self._on_order, 10)
        self.create_subscription(String, '/mission/engagement_ack', self._on_ack, 10)

    def _on_order(self, msg: EngagementOrder) -> None:
        self.last_order = msg

    def _on_ack(self, msg: String) -> None:
        try:
            self.last_ack = json.loads(msg.data)
        except json.JSONDecodeError:
            pass

    def publish_wave(self) -> None:
        self._wave_pub.publish(Bool(data=True))

    def publish_target(self, *, track_id: int, x: float, y: float, z: float,
                       vx: float = 0.0, vy: float = 0.0, vz: float = 0.0,
                       state: str = "DETECTED") -> None:
        # Fused-targets feed (drives c2 cost-matrix dispatch)
        payload = json.dumps([{
            "id": track_id, "state": state,
            "position": {"x": x, "y": y, "z": z},
            "velocity": {"x": vx, "y": vy, "z": vz},
        }])
        self._fused_pub.publish(String(data=payload))
        # Per-target FusedTarget topic (real controllers subscribe here, but
        # we're not running any in this in-process test; published for
        # symmetry with the live system).
        topic = f"/threats/target_{track_id}/fused_target"
        if track_id not in self._target_pubs:
            self._target_pubs[track_id] = self.create_publisher(FusedTarget, topic, 10)
        ft = FusedTarget()
        ft.id = track_id
        ft.position.x = x; ft.position.y = y; ft.position.z = z
        ft.velocity.x = vx; ft.velocity.y = vy; ft.velocity.z = vz
        self._target_pubs[track_id].publish(ft)

    def publish_idle_state(self, x: float, y: float, z: float, *,
                           interceptor_id: int = 1) -> None:
        if interceptor_id not in self._state_pubs:
            self._state_pubs[interceptor_id] = self.create_publisher(
                InterceptorState, f'/interceptors/unit_{interceptor_id}/state', 10)
        msg = InterceptorState()
        msg.id = interceptor_id
        msg.status = "IDLE"
        msg.position.x = x; msg.position.y = y; msg.position.z = z
        msg.target_id = 0
        self._state_pubs[interceptor_id].publish(msg)

    def publish_ack(self, target_id: int, interceptor_id: int, outcome: str) -> None:
        """Publish a synthetic ack as the controller would after an engagement."""
        ack = String()
        ack.data = json.dumps({
            "target_id": target_id,
            "interceptor_id": interceptor_id,
            "outcome": outcome,
        })
        self._ack_pub_for_test.publish(ack)

    def wait_for_order(self, timeout_s: float = 5.0) -> EngagementOrder:
        deadline = time.time() + timeout_s
        while time.time() < deadline:
            if self.last_order is not None:
                return self.last_order
            time.sleep(0.05)
        raise AssertionError(f"No EngagementOrder within {timeout_s}s")

    def wait_for_ack(self, timeout_s: float = 5.0) -> dict:
        deadline = time.time() + timeout_s
        while time.time() < deadline:
            if self.last_ack is not None:
                return self.last_ack
            time.sleep(0.05)
        raise AssertionError(f"No engagement_ack within {timeout_s}s")


@pytest.fixture
def harness():
    """Spin C2 + spy in a MultiThreadedExecutor thread.

    The CoyoteControllerNode is NOT spawned in-process — it's a C++ node and
    can't be imported from Python. Instead, the spy publishes the ack itself
    after observing the order; this tests the C2 engine's order-issuance
    pipeline end-to-end without requiring a C++ in-process spin.
    """
    rclpy.init()
    executor = MultiThreadedExecutor(num_threads=4)
    c2 = C2EngineNode()
    spy = SpyNode()
    for n in (c2, spy):
        executor.add_node(n)
    thread = threading.Thread(target=executor.spin, daemon=True)
    thread.start()
    # Let discovery settle
    time.sleep(0.5)
    yield spy
    executor.shutdown()
    thread.join(timeout=2.0)
    rclpy.shutdown()


# Target placement: Coyote home is (177, 177, 20) with a 100 m booster-clear
# minimum range. Anvils sit at (±177, ±177, 20) with min_range=5 m and
# max_range=1500 m. A target at (300, 300, 20) is ~174 m from the Coyote
# (in-envelope) and ~493 m / ~675 m from the Anvils — Coyote's max_speed=160 m/s
# vs Anvil's 45 m/s makes Coyote dominate the time-to-intercept cost matrix.
# This routes the engagement order to interceptor_id=1 (Coyote) end-to-end.
_TARGET_POS = (300.0, 300.0, 20.0)


def test_c2_dispatches_coyote_to_close_range_target(harness):
    """End-to-end: arm mission → publish target → assert engagement order routes
    to interceptor_id=1 (the Coyote slot). This confirms the FLEET refactor
    didn't break the cost-matrix → dispatcher → engagement order pipeline."""
    spy = harness
    spy.publish_idle_state(177.0, 177.0, 20.0)   # Coyote at home, idle
    spy.publish_wave()
    time.sleep(0.5)
    spy.publish_target(track_id=1, x=_TARGET_POS[0], y=_TARGET_POS[1], z=_TARGET_POS[2])
    order = spy.wait_for_order(timeout_s=3.0)
    assert order.target_id == 1
    assert order.interceptor_id == 1, (
        f"Expected Coyote (interceptor_id=1), got {order.interceptor_id}")


def test_c2_handles_miss_ack_without_crashing(harness):
    """Phase-1 stub behavior: a MISS ack is informational; weapon slot frees
    on the next IDLE InterceptorState. Verify C2 doesn't crash and accepts
    the ack quietly. This is the Phase-3-prep contract."""
    spy = harness
    spy.publish_idle_state(177.0, 177.0, 20.0)
    spy.publish_wave()
    time.sleep(0.3)
    spy.publish_target(track_id=2, x=_TARGET_POS[0], y=_TARGET_POS[1], z=_TARGET_POS[2])
    spy.wait_for_order(timeout_s=3.0)
    # Manually publish a MISS ack as the controller would
    spy.publish_ack(target_id=2, interceptor_id=1, outcome="MISS")
    # Phase-1 contract: the slot frees on the next IDLE state from that unit.
    # Publish IDLE so C2 marks unit_1 available before the second target arrives.
    spy.publish_idle_state(177.0, 177.0, 20.0, interceptor_id=1)
    # Retire track 2 (missed) so the cost matrix only sees the new threat.
    # Without this, C2 would re-assign the now-available Coyote to track 2
    # again (lowest cost) and give track 3 to an Anvil instead.
    spy.publish_target(track_id=2, x=_TARGET_POS[0], y=_TARGET_POS[1], z=_TARGET_POS[2],
                       state="TERMINATED")
    # If C2 crashes on MISS handling, the executor thread dies and subsequent
    # publishes get no response. Verify by issuing a second engagement.
    time.sleep(0.5)
    spy.last_order = None
    spy.publish_target(track_id=3, x=_TARGET_POS[0], y=_TARGET_POS[1], z=_TARGET_POS[2])
    second = spy.wait_for_order(timeout_s=3.0)
    assert second.target_id == 3
    assert second.interceptor_id == 1, (
        f"Expected slot 1 (Coyote) reused after MISS+IDLE, got {second.interceptor_id}")
