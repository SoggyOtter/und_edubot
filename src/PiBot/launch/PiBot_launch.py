from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch.actions import IncludeLaunchDescription


def generate_launch_description():
    return LaunchDescription(
        [
            Node(
                package="PiBot",
                executable="encoder_data",
                name="encoder_data",
                # output='screen'
            ),
            Node(
                package="PiBot",
                executable="velocity_control",
                name="velocity_control",
                # output='screen'
            ),
            Node(
                package="PiBot",
                executable="wheel_odometry",
                name="wheel_odometry"
            )
        ]
    )
