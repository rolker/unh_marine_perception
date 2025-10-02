from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    return LaunchDescription([
        Node(
            package="depthai_marine",
            executable="wide_stereo",
            name="wide_stereo",
            parameters=[{
                'right_camera_id': "19443010D117872D00",
                'left_camera_id': "19443010E11A872D00"
            }],
            #emulate_tty=True
            output="screen"
        )
    ])
