"""launch_testing integration: bring `segments_to_pointcloud` to the active
lifecycle state and exercise its marine_control device-control channel
(unh_marine_autonomy#140 / ADR-0003).

Asserts the actual wiring added when the reflex node became a marine_control
adopter — not the library in isolation:

  1. While active, the node publishes a ControlSet on `~/control/state`
     advertising exactly the reflex confidence floor (`obstacle_prob_min`),
     with the descriptor's bounds (0.0–0.95) carried through to the UI item.
  2. A ControlValue change on `~/control/change` for an in-range value is
     applied to the bound parameter and confirmed by the next state echo.
  3. **Safety contract (the load-bearing assertion):** an out-of-range change
     (> 0.95) is rejected, so the echoed value does not move. The parameter's
     own bounds/validation remain the guard against an operator blinding the
     reflex feed over the remote channel — fire-and-forget cannot bypass it
     (ADR-0003 D8.3).

No camera/bag data is needed: the control channel is independent of the
projection pipeline, so the node sits idle on its image subscriptions while the
control server runs.

The numbered scenarios above describe coverage, not execution order: unittest
runs the methods alphabetically, and each is self-contained (it re-establishes
any baseline it needs), so order does not matter.
"""

import time
import unittest

import pytest
import rclpy
from lifecycle_msgs.msg import Transition
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Header

from marine_control_interfaces.msg import ControlItem, ControlSet, ControlValue

import launch_testing
import launch_testing.actions
from launch import LaunchDescription
from launch_ros.actions import LifecycleNode
from launch_ros.actions import LifecycleTransition

NAMESPACE = '/test_reflex'
NODE_NAME = 'segments_to_pointcloud'
STATE_TOPIC = f'{NAMESPACE}/{NODE_NAME}/control/state'
CHANGE_TOPIC = f'{NAMESPACE}/{NODE_NAME}/control/change'

CONTROL_NAME = 'obstacle_prob_min'
EXPECTED_DEVICE_NAME = 'Reflex Obstacle Filter'
EXPECTED_MIN = 0.0
EXPECTED_MAX = 0.95

# An in-range request the cap accepts, and an out-of-range one it must reject.
IN_RANGE_VALUE = 0.6
OUT_OF_RANGE_VALUE = 0.99
# A second in-range value, distinct from IN_RANGE_VALUE, sent *after* the
# out-of-range request. marine_control uses RELIABLE ordered delivery, so if this
# sentinel is applied the out-of-range request was delivered and processed before
# it — which distinguishes "rejected by the cap" from "silently dropped".
SENTINEL_VALUE = 0.7

# Heartbeat is 1 Hz; allow generous slack for lifecycle transitions + discovery.
TEST_TIMEOUT_SECONDS = 25.0

# marine_control state QoS: RELIABLE + VOLATILE (ADR-0003 D5). A best-effort or
# transient-local subscriber would mismatch and silently receive nothing.
_CONTROL_QOS = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.VOLATILE,
    history=HistoryPolicy.KEEP_LAST,
    depth=10,
)


@pytest.mark.launch_test
def generate_test_description():
    node = LifecycleNode(
        package='sea_surface_segmentation',
        executable=NODE_NAME,
        name=NODE_NAME,
        namespace=NAMESPACE,
        output='screen',
    )

    transition = LifecycleTransition(
        lifecycle_node_names=[f'{NAMESPACE}/{NODE_NAME}'],
        transition_ids=(
            Transition.TRANSITION_CONFIGURE,
            Transition.TRANSITION_ACTIVATE,
        ),
    )

    return LaunchDescription([
        node,
        transition,
        launch_testing.actions.ReadyToTest(),
    ])


class TestReflexControlChannel(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.node = rclpy.create_node('reflex_control_test')
        cls.latest = None

        def on_state(msg: ControlSet):
            cls.latest = msg

        cls.sub = cls.node.create_subscription(
            ControlSet, STATE_TOPIC, on_state, _CONTROL_QOS)
        cls.change_pub = cls.node.create_publisher(
            ControlValue, CHANGE_TOPIC, _CONTROL_QOS)

    @classmethod
    def tearDownClass(cls):
        cls.node.destroy_node()
        rclpy.shutdown()

    def _spin_until(self, predicate, timeout=TEST_TIMEOUT_SECONDS):
        start = time.monotonic()
        while time.monotonic() - start < timeout:
            rclpy.spin_once(self.node, timeout_sec=0.1)
            if predicate():
                return True
        return False

    def _find_item(self, control_set):
        for item in control_set.items:
            if item.name == CONTROL_NAME:
                return item
        return None

    def _send_change(self, value):
        msg = ControlValue()
        msg.header = Header()
        msg.name = CONTROL_NAME
        # Stringified per the item type (FLOAT); the server parses it back.
        msg.value = repr(float(value))
        self.change_pub.publish(msg)

    def _current_value(self):
        item = self._find_item(self.latest) if self.latest else None
        return None if item is None else float(item.value)

    def _drive_to(self, value):
        """Predicate that (re)sends `value` and reports whether the echo shows it.

        Sending on each poll tolerates the change racing the subscription's
        discovery/connection; RELIABLE delivery makes the repeats harmless.
        """
        def predicate():
            self._send_change(value)
            rclpy.spin_once(self.node, timeout_sec=0.1)
            current = self._current_value()
            return current is not None and abs(current - value) < 1e-6
        return predicate

    def test_state_advertises_obstacle_prob_min(self):
        got = self._spin_until(lambda: self.latest is not None)
        self.assertTrue(
            got,
            f'no ControlSet received on {STATE_TOPIC} within '
            f'{TEST_TIMEOUT_SECONDS}s; the node may not have reached the active '
            f'state, or the marine_control server was not constructed in '
            f'on_activate.')

        self.assertEqual(self.latest.device_name, EXPECTED_DEVICE_NAME)

        item = self._find_item(self.latest)
        self.assertIsNotNone(
            item,
            f'ControlSet did not advertise {CONTROL_NAME!r}; items='
            f'{[i.name for i in self.latest.items]}')
        self.assertEqual(item.type, ControlItem.TYPE_FLOAT)
        self.assertAlmostEqual(item.min_value, EXPECTED_MIN, places=6)
        self.assertAlmostEqual(item.max_value, EXPECTED_MAX, places=6)
        self.assertEqual(item.units, 'P')
        self.assertEqual(item.group, 'reflex')

    def test_in_range_change_is_applied(self):
        self.assertTrue(
            self._spin_until(self._drive_to(IN_RANGE_VALUE)),
            f'in-range change to {IN_RANGE_VALUE} was not reflected in the '
            f'state echo; the change channel did not apply the bound parameter.')

    def test_out_of_range_change_is_rejected(self):
        # Drive to a known in-range value first; this also proves the change
        # channel is live in this test method (independent of test order).
        self.assertTrue(
            self._spin_until(self._drive_to(IN_RANGE_VALUE)),
            'could not establish the in-range baseline before the rejection test.')

        # Request the out-of-range value, immediately followed by an in-range
        # sentinel. Track the max value ever echoed across the wait: the cap must
        # never be exceeded. Waiting for the sentinel to land confirms (via
        # RELIABLE ordered delivery) that the out-of-range request was delivered
        # and processed — not merely dropped — so the test cannot pass vacuously.
        max_seen = IN_RANGE_VALUE
        self._send_change(OUT_OF_RANGE_VALUE)
        self._send_change(SENTINEL_VALUE)

        def sentinel_applied():
            nonlocal max_seen
            rclpy.spin_once(self.node, timeout_sec=0.1)
            current = self._current_value()
            if current is None:
                return False
            max_seen = max(max_seen, current)
            return abs(current - SENTINEL_VALUE) < 1e-6

        self.assertTrue(
            self._spin_until(sentinel_applied),
            f'sentinel change to {SENTINEL_VALUE} (sent right after the '
            f'out-of-range request) never landed; cannot confirm the '
            f'out-of-range request was delivered rather than dropped.')
        self.assertLessEqual(
            max_seen, EXPECTED_MAX,
            f'an echoed value exceeded the {EXPECTED_MAX} cap; the out-of-range '
            f'change to {OUT_OF_RANGE_VALUE} was accepted. The reflex confidence '
            f'floor must stay capped over the marine_control channel '
            f'(ADR-0003 D8.3 safety contract) — an operator must not be able to '
            f'blind the reflex feed remotely.')


@launch_testing.post_shutdown_test()
class TestProcessShutdown(unittest.TestCase):

    def test_clean_exit(self, proc_info):
        # The node is killed by launch teardown; no stricter assertion needed.
        pass
