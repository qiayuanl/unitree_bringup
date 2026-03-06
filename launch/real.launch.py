import os

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    OpaqueFunction,
    SetLaunchConfiguration,
    IncludeLaunchDescription
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution, LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from legged_bringup.launch_utils import get_controller_names, generate_temp_config, resolve_policy_paths, \
    control_spawner


def setup_controllers(context):
    robot_type_value = LaunchConfiguration('robot_type').perform(context)
    policy_path_value = LaunchConfiguration('policy_path').perform(context)

    controllers_config_path = f'config/{robot_type_value}/controllers.yaml'

    kv_pairs = resolve_policy_paths(controllers_config_path, "unitree_bringup")
    if policy_path_value:
        abs_path = os.path.abspath(os.path.expanduser(os.path.expandvars(policy_path_value)))
        kv_pairs.append(('walking_controller.policy.path', abs_path))
    temp_controllers_config_path = generate_temp_config(
        controllers_config_path,
        "unitree_bringup",
        kv_pairs
    )

    set_controllers_yaml = SetLaunchConfiguration(
        name='controllers_yaml',
        value=temp_controllers_config_path
    )

    all_controllers = get_controller_names(controllers_config_path, "unitree_bringup")
    active_list = ["state_estimator", "standby_controller"]
    inactive_list = [c for c in all_controllers if c not in active_list]
    param_file = LaunchConfiguration('controllers_yaml')
    active_spawner = control_spawner(active_list, param_file=param_file)
    inactive_spawner = control_spawner(inactive_list, inactive=True, param_file=param_file)

    return [
        set_controllers_yaml,
        active_spawner,
        inactive_spawner,
    ]


def generate_launch_description():
    robot_type = LaunchConfiguration('robot_type')
    network_interface = LaunchConfiguration('network_interface')
    urdf_name = PythonExpression([
        "'g1' if '", robot_type, "' == 'g1' else ('go2' if '", robot_type, "' == 'go2' else 'sdk1')"
    ])

    robot_description_command = Command([
        PathJoinSubstitution([FindExecutable(name='xacro')]),
        " ",
        PathJoinSubstitution([
            FindPackageShare("unitree_description"),
            "urdf",
            urdf_name,
            "robot.xacro"
        ]),
        " ",
        "robot_type:=", robot_type,
        " ",
        "simulation:=", "false",
        " ",
        "network_interface:=", network_interface
    ])
    robot_description = {"robot_description": robot_description_command}

    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[robot_description, {
            'publish_frequency': 500.0,
        }],
    )

    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_description, LaunchConfiguration('controllers_yaml')],
        output="both",
        respawn=True,
    )

    controllers_opaque_func = OpaqueFunction(function=setup_controllers)

    teleop = PathJoinSubstitution([
        FindPackageShare('unitree_bringup'),
        'launch',
        'teleop.launch.py'
    ])

    return LaunchDescription([
        DeclareLaunchArgument('robot_type', default_value='g1'),
        DeclareLaunchArgument('network_interface', default_value=''),
        DeclareLaunchArgument(
            'policy_path',
            default_value='',
            description='Absolute or ~-expanded path for walking_controller.policy.path'
        ),
        controllers_opaque_func,
        control_node,
        node_robot_state_publisher,
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(teleop),
            launch_arguments={'robot_type': robot_type}.items()
        )
    ])
