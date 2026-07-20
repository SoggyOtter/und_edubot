from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration

from ament_index_python.packages import get_package_share_directory

# from launch_ros.substitutions import FindPackageShare
# from launch.actions import IncludeLaunchDescription


def generate_launch_description():

    launch_foxglove_bridge_arg = DeclareLaunchArgument(
        "launch_foxglove_bridge",
        default_value='False',
        description="Whether or not to launch the foxglove bridge",
    )

    foxglove_bridge = IncludeLaunchDescription(
        PathJoinSubstitution(
            [
                get_package_share_directory("foxglove_bridge"),
                "launch",
                "foxglove_bridge_launch.xml",
            ]
        ),
        condition=IfCondition(LaunchConfiguration("launch_foxglove_bridge")),
    )

    encoder_node = Node(
                package="PiBot",
                executable="encoder_data",
                name="encoder_data",
                # output='screen'
            )
    
    velocity_control_node = Node(
                package="PiBot",
                executable="velocity_control",
                name="velocity_control",
                # output='screen'
            )
    
    wheel_odometry_node = Node(
                package="PiBot",
                executable="wheel_odometry",
                name="wheel_odometry",
            )
    
    motion_control_node = Node(
                package="PiBot",
                executable="motion_controller",
                name="motion_controller",
            )

    # TODO(Alex), break apart to base launch description, then create other launch files to launch specific modules
    return LaunchDescription(
        [
            encoder_node,
            velocity_control_node,
            wheel_odometry_node,
            motion_control_node,
            launch_foxglove_bridge_arg,
            foxglove_bridge
        ]
    )
