import os

import yaml
from ament_index_python.packages import get_package_share_directory
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


def fill_policy_path(config_path, package_name):
    pkg_dir = get_package_share_directory(package_name)
    src_path = os.path.join(pkg_dir, config_path)
    dst_path = os.path.join('/tmp', package_name, 'temp_controllers.yaml')

    os.makedirs(os.path.dirname(dst_path), exist_ok=True)

    with open(src_path, 'r') as f:
        config = yaml.safe_load(f)

    for ns in list(config.keys()):
        params = config.get(ns, {}).setdefault('ros__parameters', {})
        if 'policy' in params and 'path' in params['policy']:
            params['policy']['path'] = os.path.join(pkg_dir, params['policy']['path'])

    with open(dst_path, 'w') as f:
        yaml.dump(config, f)
        print(f"Modified controllers.yaml saved to {dst_path}")

    return dst_path


def control_spawner(names, inactive=False):
    # Start building the arguments list with the controller names
    args = list(names)

    # If you want them to start inactive (rather than active), pass `--inactive`
    if inactive:
        args.append('--inactive')

    # Return the spawner node
    return Node(
        package='controller_manager',
        executable='spawner',
        arguments=args,
        output='screen'
    )


def setup_controllers(context):
    robot_type_value = LaunchConfiguration('robot_type').perform(context)

    controllers_config_path = 'config/' + robot_type_value + '/controllers.yaml'
    temp_controllers_config_path = fill_policy_path(
        controllers_config_path,
        "unitree_bringup"
    )

    set_controllers_yaml = SetLaunchConfiguration(
        name='controllers_yaml',
        value=temp_controllers_config_path
    )

    active_list = [
        "state_estimator",
        "standby_controller"
    ]

    inactive_list = [
        "walking_controller",
        # "getup_controller",
        # "handstand_controller",
    ]
    active_spawner = control_spawner(active_list)
    inactive_spawner = control_spawner(inactive_list, inactive=True)
    return [
        set_controllers_yaml,
        active_spawner,
        inactive_spawner
    ]


def generate_launch_description():
    robot_type = LaunchConfiguration('robot_type')

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
        "simulation:=", "mujoco"])
    robot_description = {"robot_description": robot_description_command}

    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[robot_description, {
            'publish_frequency': 500.0,
            'use_sim_time': True
        }],
    )

    mujoco_simulator = Node(
        package='mujoco_sim_ros2',
        executable='mujoco_sim',
        parameters=[
            {"model_package": "unitree_description",
             "model_file": PythonExpression(["'/mjcf/", robot_type, ".xml'"]),
             "physics_plugins": ["mujoco_ros2_control::MujocoRos2ControlPlugin"],
             'use_sim_time': True,
             },
            robot_description,
            LaunchConfiguration('controllers_yaml'),
        ],
        output='screen')

    controllers_opaque_func = OpaqueFunction(
        function=setup_controllers
    )

    teleop = PathJoinSubstitution([
        FindPackageShare('unitree_bringup'),
        'launch',
        'teleop.launch.py'
    ])

    return LaunchDescription([
        DeclareLaunchArgument('robot_type', default_value='g1'),
        controllers_opaque_func,
        mujoco_simulator,
        node_robot_state_publisher,
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(teleop)
        )
    ])
