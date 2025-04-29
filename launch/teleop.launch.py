import launch_ros.actions
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription


def generate_launch_description():
    param = os.path.join(
        get_package_share_directory('unitree_bringup'),
        'config', 'g1', 'joy.yaml'
    )
    joy_node = launch_ros.actions.Node(package='joy_linux',
                                       executable='joy_linux_node',
                                       output='both')

    ld = LaunchDescription([joy_node])

    ld.add_action(launch_ros.actions.Node(
        package='joy_teleop', executable='joy_teleop',
        parameters=[param]))

    return ld
