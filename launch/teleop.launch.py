import os

import launch_ros.actions
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration


def _setup(context):
    robot_type_value = LaunchConfiguration('robot_type').perform(context)
    param = os.path.join(
        get_package_share_directory('unitree_bringup'),
        'config', robot_type_value, 'joy.yaml'
    )
    return [launch_ros.actions.Node(
        package='joy_teleop', executable='joy_teleop',
        parameters=[param])]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('robot_type', default_value='g1'),
        launch_ros.actions.Node(
            package='joy_linux',
            executable='joy_linux_node',
            output='both'),
        OpaqueFunction(function=_setup),
    ])
