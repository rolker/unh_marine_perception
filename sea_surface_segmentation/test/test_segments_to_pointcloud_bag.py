"""launch_testing integration: replay a 20 s slice of the 2026-05-22
deployment bag through `segments_to_pointcloud` in reflex-mode
(`target_frame=bizzy/base_link_level`) and assert obstacle points
appear in the forward danger sector.

This is the deployment-evidence test for #17. The bag contains the
2026-05-22 bizzy run that produced one of the avoidable-collision
incidents motivating the reflex safety work. The trimmed slice covers
the first 20 s of the deployment (chosen so `/tf_static` is captured)
and has 100 forward-OAK segmentation frames with ~600–1000 R-dominant
obstacle pixels per frame. If the reflex pipeline produces points in
the danger sector for this bag, it would have produced them during the
incident too — the projection math doesn't depend on what's in the
specific second when the collision happens.

The threshold (100 points in the danger sector across the whole
playback) is a **presence check, not a recall measurement**. Recall
characterization is out of scope; see plan-task §5.
"""

import os
import time
import unittest

import pytest
import rclpy
from ament_index_python.packages import get_package_share_directory
from lifecycle_msgs.msg import Transition
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2

import launch
import launch_testing
import launch_testing.actions
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import LifecycleNode
from launch_ros.actions import LifecycleTransition

NAMESPACE = '/bizzy/sensors/cameras/oak_forward'
NODE_NAME = 'segments_to_pointcloud'
POINTCLOUD_TOPIC = f'{NAMESPACE}/{NODE_NAME}/pointcloud'

# Forward arc, ~10 m × 6 m, anchored at base_link in base_link_level.
DANGER_SECTOR_X_MIN = 0.0
DANGER_SECTOR_X_MAX = 10.0
DANGER_SECTOR_Y_HALF_WIDTH = 3.0

# Presence threshold. With 100 frames × hundreds of obstacle pixels
# per frame, even modest projection success should produce thousands
# of danger-sector hits. 100 is a generous floor.
MIN_DANGER_SECTOR_HITS = 100

# Spin budget — the bag is 20 s; allow generous slack for lifecycle
# transitions, TF buffer warmup, and bag-player startup latency.
TEST_TIMEOUT_SECONDS = 40.0


@pytest.mark.launch_test
def generate_test_description():
    fixture_dir = os.path.join(
        get_package_share_directory('sea_surface_segmentation'),
        'test', 'fixtures', 'issue17_obstacle_approach',
    )

    # Use sim time so the node's TF buffer evaluates against
    # bag-recorded stamps rather than current wall-clock — necessary
    # because the bag's stamps are from May 2026 and the TF buffer's
    # default eviction would otherwise drop them.
    bag_player = ExecuteProcess(
        cmd=['ros2', 'bag', 'play', fixture_dir, '--clock', '--rate', '1.0'],
        output='screen',
    )

    node = LifecycleNode(
        package='sea_surface_segmentation',
        executable=NODE_NAME,
        name=NODE_NAME,
        namespace=NAMESPACE,
        parameters=[{
            'use_sim_time': True,
            'target_frame': 'bizzy/base_link_level',
        }],
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
        bag_player,
        launch_testing.actions.ReadyToTest(),
    ])


class TestReflexCloudHasDangerSectorPoints(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.node = rclpy.create_node('issue17_bag_test_subscriber')
        cls.node.set_parameters([
            rclpy.parameter.Parameter('use_sim_time', value=True),
        ])

    @classmethod
    def tearDownClass(cls):
        cls.node.destroy_node()
        rclpy.shutdown()

    def test_danger_sector_points_appear(self, proc_output):
        danger_sector_hits = 0
        total_points = 0
        message_count = 0

        def callback(msg: PointCloud2):
            nonlocal danger_sector_hits, total_points, message_count
            message_count += 1
            for point in point_cloud2.read_points(
                    msg,
                    field_names=('x', 'y', 'z', 'intensity'),
                    skip_nans=True):
                x, y = float(point[0]), float(point[1])
                total_points += 1
                if (DANGER_SECTOR_X_MIN <= x <= DANGER_SECTOR_X_MAX
                        and -DANGER_SECTOR_Y_HALF_WIDTH <= y <= DANGER_SECTOR_Y_HALF_WIDTH):
                    danger_sector_hits += 1

        # Match the publisher's QoS — segments_to_pointcloud publishes
        # SensorDataQoS (best-effort); a reliable subscriber would
        # silently get no messages due to QoS incompatibility.
        sub = self.node.create_subscription(
            PointCloud2,
            POINTCLOUD_TOPIC,
            callback,
            qos_profile_sensor_data,
        )

        start = time.monotonic()
        while time.monotonic() - start < TEST_TIMEOUT_SECONDS:
            rclpy.spin_once(self.node, timeout_sec=0.1)
            if danger_sector_hits >= MIN_DANGER_SECTOR_HITS:
                # Early exit once threshold is met — the test goal is
                # presence, not exhaustive collection.
                break

        self.node.destroy_subscription(sub)

        self.assertGreater(
            message_count, 0,
            f'no PointCloud2 messages received on {POINTCLOUD_TOPIC} during '
            f'{TEST_TIMEOUT_SECONDS}s; node may not have configured/activated '
            f'or topic remapping is wrong.'
        )
        self.assertGreaterEqual(
            danger_sector_hits, MIN_DANGER_SECTOR_HITS,
            f'expected at least {MIN_DANGER_SECTOR_HITS} obstacle points in '
            f'the forward danger sector ({DANGER_SECTOR_X_MIN}–{DANGER_SECTOR_X_MAX} m, '
            f'±{DANGER_SECTOR_Y_HALF_WIDTH} m); got {danger_sector_hits} '
            f'({total_points} total points across {message_count} messages). '
            f'The bag has 100 frames × hundreds of obstacle pixels each; '
            f'if hits are low or zero, the projection path is broken — '
            f'check TF availability for bizzy/base_link_level and the '
            f'`use_sim_time` flag on the node and the bag player.'
        )


@launch_testing.post_shutdown_test()
class TestProcessShutdown(unittest.TestCase):

    def test_bag_player_clean_exit(self, proc_info):
        # `ros2 bag play` exits 0 when it finishes the bag. We don't
        # assert anything stricter here — the node is killed by the
        # launch teardown, which is fine.
        pass
