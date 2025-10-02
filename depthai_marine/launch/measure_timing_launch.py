from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    return LaunchDescription([
        Node(
            package="depthai_marine",
            executable="measure_timing",
            name="measure_timing",
            parameters=[{
                'find_text_neural_network': PathJoinSubstitution([
                    FindPackageShare("depthai_marine"),
                    "config",
                    "east_text_detection_256x256.blob"
                ]),
                'text_recognition_neural_network': PathJoinSubstitution([
                    FindPackageShare("depthai_marine"),
                    "config",
                    "text-recognition-0012.blob"
                ])
            }],
            #emulate_tty=True
            output="screen"
        )
    ])
