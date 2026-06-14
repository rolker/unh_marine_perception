from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.substitutions import PythonExpression
from launch_ros.actions import LifecycleNode
from launch_ros.actions import LifecycleTransition
from launch_ros.parameter_descriptions import ParameterValue

from lifecycle_msgs.msg import Transition


def generate_launch_description():
    # Node name. Two parallel instances of segments_to_pointcloud
    # (legacy map-frame for sea_surface_layer + new base_link_level
    # for nav2_collision_monitor) coexist under the same parent
    # namespace by using distinct names — combined with the node's
    # ~/pointcloud private-namespace publisher, this auto-isolates the
    # two output topics. Default preserves the historical name.
    name = LaunchConfiguration('name', default='segments_to_pointcloud')

    # Optional override of the projection target frame. Empty (default)
    # keeps the legacy map_frame-based projection — what every existing
    # includer of this launch file expects. Set to a heading-only frame
    # such as `<robot>/base_link_level` to produce a failure-stage-
    # independent feed for the Collision Monitor reflex layer.
    target_frame = LaunchConfiguration('target_frame', default='')

    # Reflex confidence floor (see segments_projection.hpp / #35). Default 0.0 =
    # off, so existing includers are unaffected; platforms pass a positive value
    # (BizzyBoat: 0.60) to reject low-confidence returns like calm-water
    # reflections.
    obstacle_prob_min = LaunchConfiguration('obstacle_prob_min', default='0.0')

    return LaunchDescription([
        DeclareLaunchArgument(
            'name',
            default_value='segments_to_pointcloud',
            description=(
                'Node name. Override when running two instances of this '
                'node in the same namespace (e.g. a base_link_level '
                'reflex-mode instance alongside the legacy map-frame one).'
            ),
        ),
        DeclareLaunchArgument(
            'target_frame',
            default_value='',
            description=(
                'When non-empty, project into and stamp output points in '
                'this frame instead of `map_frame` — used for the '
                'nav2_collision_monitor reflex feed.'
            ),
        ),
        DeclareLaunchArgument(
            'obstacle_prob_min',
            default_value='0.0',
            description=(
                'Reflex confidence floor: project an obstacle pixel only if '
                'P(obstacle)=R/(R+G+B) >= this. 0.0 (default) disables the gate; '
                'platforms raise it to reject low-confidence reflections.'
            ),
        ),
        LifecycleNode(
            package='sea_surface_segmentation',
            executable='segments_to_pointcloud',
            name=name,
            namespace='',
            parameters=[{
                'target_frame': target_frame,
                # Cast the string launch arg to double for the node's param.
                'obstacle_prob_min': ParameterValue(
                    obstacle_prob_min, value_type=float),
            }],
            respawn=True,
            respawn_delay=2,
            emulate_tty=True
        ),
        LifecycleTransition(
            lifecycle_node_names=(
                PythonExpression(
                    expression=[
                        '"',
                        LaunchConfiguration('ros_namespace', default=''),
                        '" + "/" + "',
                        name,
                        '"',
                    ],
                ),
            ),
            transition_ids=(
                Transition.TRANSITION_CONFIGURE,
                Transition.TRANSITION_ACTIVATE,
            )
        ),
    ])
