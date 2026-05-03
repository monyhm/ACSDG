"""DroneHunter F700 engagement integration test (in-process rclpy harness).

Runs C2EngineNode + a SpyNode in a MultiThreadedExecutor, without Gazebo or
launch_testing. Mirrors test_coyote_integration.py with two DroneHunter-specific
scenarios:
  1. Dispatch routing: SE-incoming target → interceptor_id=3 (the DroneHunter slot)
  2. Cooldown gate: after capture + IDLE, DroneHunter is unavailable for 180 s
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
        payload = json.dumps([{
            "id": track_id, "state": state,
            "position": {"x": x, "y": y, "z": z},
            "velocity": {"x": vx, "y": vy, "z": vz},
        }])
        self._fused_pub.publish(String(data=payload))
        topic = f"/threats/target_{track_id}/fused_target"
        if track_id not in self._target_pubs:
            self._target_pubs[track_id] = self.create_publisher(FusedTarget, topic, 10)
        ft = FusedTarget()
        ft.id = track_id
        ft.position.x = x; ft.position.y = y; ft.position.z = z
        ft.velocity.x = vx; ft.velocity.y = vy; ft.velocity.z = vz
        self._target_pubs[track_id].publish(ft)

    def publish_state(self, *, interceptor_id: int, status: str,
                      target_id: int = 0,
                      x: float = 0.0, y: float = 0.0, z: float = 20.0) -> None:
        """Publish an InterceptorState for a given slot. Used to mark slots
        as PURSUING (so the cost matrix excludes them) without spawning real
        controllers."""
        if interceptor_id not in self._state_pubs:
            self._state_pubs[interceptor_id] = self.create_publisher(
                InterceptorState, f'/interceptors/unit_{interceptor_id}/state', 10)
        msg = InterceptorState()
        msg.id = interceptor_id
        msg.status = status
        msg.position.x = x; msg.position.y = y; msg.position.z = z
        msg.target_id = target_id
        self._state_pubs[interceptor_id].publish(msg)

    def publish_idle_state(self, x: float, y: float, z: float, *,
                           interceptor_id: int = 1) -> None:
        self.publish_state(interceptor_id=interceptor_id, status="IDLE",
                           x=x, y=y, z=z)

    def publish_ack(self, target_id: int, interceptor_id: int, outcome: str) -> None:
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


@pytest.fixture
def harness():
    rclpy.init()
    executor = MultiThreadedExecutor(num_threads=4)
    c2 = C2EngineNode()
    spy = SpyNode()
    for n in (c2, spy):
        executor.add_node(n)
    thread = threading.Thread(target=executor.spin, daemon=True)
    thread.start()
    # Pre-create all 4 per-unit state publishers and broadcast IDLE so the C2
    # engine sees a known-good baseline regardless of any stale DDS messages
    # left over from a prior test run.  This also gives DDS sufficient time to
    # match the publishers against C2's subscriptions before the 0.5 s sleep.
    # Also flush the fused-target dict so stale tracks from prior runs don't
    # cause spurious dispatches when the wave arms the engine.
    _home = {1: (177.0, 177.0), 2: (-177.0, 177.0), 3: (177.0, -177.0), 4: (-177.0, -177.0)}
    for iid, (hx, hy) in _home.items():
        spy.publish_state(interceptor_id=iid, status="IDLE", x=hx, y=hy, z=20.0)
    spy._fused_pub.publish(String(data='[]'))  # flush stale fused targets
    time.sleep(0.5)
    yield spy
    executor.shutdown()
    thread.join(timeout=2.0)
    rclpy.shutdown()


# Target placement: SE-of-origin lands closer to DroneHunter (177, -177, 20)
# than to any other slot. Coyote at (177, 177) and Anvils at (-177, ±177) are
# all 400-700m away vs DroneHunter's 174m. DroneHunter's max_speed of 31 m/s
# wins ToI at this geometry.
_SE_TARGET_POS = (300.0, -300.0, 20.0)


def _mark_others_busy(spy: SpyNode) -> None:
    """Mark Coyote (slot 1) + both Anvils (slots 2, 4) as PURSUING phantom
    targets, so DroneHunter (slot 3) is the only available weapon for the
    cost matrix.

    Why this contrivance: at any plausible target geometry, Coyote's 160 m/s
    dominates the ToI cost matrix (e.g., target at (300,-300): Coyote 493m/160
    = 3.08s vs DroneHunter 174m/31 = 5.61s — Coyote wins). To isolate the
    routing logic for DroneHunter, the test forces all other slots out of the
    cost matrix. Real cost-matrix tie-breaking is exercised by the live demo
    in Task 7.
    """
    spy.publish_state(interceptor_id=1, status="PURSUING", target_id=99,
                      x=177.0, y=177.0, z=20.0)
    spy.publish_state(interceptor_id=2, status="PURSUING", target_id=98,
                      x=-177.0, y=177.0, z=20.0)
    spy.publish_state(interceptor_id=4, status="PURSUING", target_id=97,
                      x=-177.0, y=-177.0, z=20.0)


def test_c2_dispatches_dronehunter_to_se_target(harness):
    """End-to-end: arm mission → mark others busy → publish SE target →
    assert engagement order routes to interceptor_id=3 (the DroneHunter slot)."""
    spy = harness
    _mark_others_busy(spy)
    spy.publish_idle_state(177.0, -177.0, 20.0, interceptor_id=3)  # DroneHunter idle
    # Second fused-target flush just before arming: any stale DDS tracks that
    # arrived during the state-publish window are cleared before the wave fires.
    spy._fused_pub.publish(String(data='[]'))
    spy.publish_wave()
    time.sleep(0.5)
    spy.publish_target(track_id=1,
                       x=_SE_TARGET_POS[0], y=_SE_TARGET_POS[1], z=_SE_TARGET_POS[2])
    order = spy.wait_for_order(timeout_s=3.0)
    assert order.target_id == 1
    assert order.interceptor_id == 3, (
        f"Expected DroneHunter (interceptor_id=3), got {order.interceptor_id}")


def test_dronehunter_unavailable_during_cooldown_after_engagement(harness):
    """After capture + IDLE, DroneHunter's is_available() returns False for
    180 s. Confirm a second target spawned shortly after routes to a different
    slot — observable evidence of the cooldown."""
    spy = harness
    _mark_others_busy(spy)
    spy.publish_idle_state(177.0, -177.0, 20.0, interceptor_id=3)
    # Second fused-target flush just before arming: any stale DDS tracks that
    # arrived during the state-publish window are cleared before the wave fires.
    spy._fused_pub.publish(String(data='[]'))
    spy.publish_wave()
    time.sleep(0.5)
    spy.publish_target(track_id=1,
                       x=_SE_TARGET_POS[0], y=_SE_TARGET_POS[1], z=_SE_TARGET_POS[2])
    first = spy.wait_for_order(timeout_s=5.0)
    assert first.interceptor_id == 3   # DroneHunter took it

    # Simulate capture + return-home: ack the kill, then publish IDLE state
    # to trigger DroneHunter.mark_idle() → cooldown starts.
    spy.publish_ack(target_id=1, interceptor_id=3, outcome="NEUTRALIZED")
    spy.publish_idle_state(177.0, -177.0, 20.0, interceptor_id=3)

    # Retire track 1 and flush fused targets BEFORE freeing Anvils.  This
    # ensures no stale DDS tracks (from prior test runs) can be dispatched
    # to the newly freed Anvils during the settlement window.
    spy._fused_pub.publish(String(data='[]'))

    # Free up the Anvils (Coyote stays PURSUING for stable cost-matrix). After
    # cooldown gates DroneHunter, an Anvil should win.
    spy.publish_state(interceptor_id=2, status="IDLE",
                      x=-177.0, y=177.0, z=20.0)
    spy.publish_state(interceptor_id=4, status="IDLE",
                      x=-177.0, y=-177.0, z=20.0)

    time.sleep(0.5)
    spy.last_order = None

    # Retire track 1 + send a new SE target. With DroneHunter on cooldown
    # and Coyote PURSUING, the only candidates are Anvils 2 and 4.
    spy.publish_target(track_id=1,
                       x=_SE_TARGET_POS[0], y=_SE_TARGET_POS[1], z=_SE_TARGET_POS[2],
                       state="TERMINATED")
    spy.publish_target(track_id=2,
                       x=_SE_TARGET_POS[0], y=_SE_TARGET_POS[1], z=_SE_TARGET_POS[2])

    second = spy.wait_for_order(timeout_s=3.0)
    assert second.interceptor_id != 3, (
        f"Expected non-DroneHunter slot (cooldown active), "
        f"got interceptor_id={second.interceptor_id}")
    assert second.interceptor_id in (2, 4), (
        f"Expected one of the freed Anvils (slot 2 or 4), "
        f"got interceptor_id={second.interceptor_id}")
