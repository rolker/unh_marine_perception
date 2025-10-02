from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package="detection_visualizer",
            executable="detection_visualizer",
            name="detection_visualizer",
            remappings=[
                ("/detection_visualizer/detections", "/camera_19443010E11A872D00/find_text"),
                ("/detection_visualizer/images", "/camera_19443010E11A872D00/image_raw")
            ]
        )
    ])
