#!/usr/bin/env python3
"""Launch Livox Driver and GlimROS as composable nodes using package-based config discovery.

This launch file replaces hard‑coded paths with automatic lookup via
`ament_index_python.get_package_share_directory`, so the configs are
resolved from their respective ROS 2 packages at launch‑time.
"""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode


def generate_launch_description() -> LaunchDescription:
    # ------------------------------------------------------------------
    # Compute default config paths via ament package index  ❱❱❱ find‑package
    # ------------------------------------------------------------------
    livox_default = os.path.join(
        get_package_share_directory("unitree_bringup"),
        "config", "g1", "MID360_config.json",
    )

    glim_default = os.path.join(
        get_package_share_directory("unitree_bringup"),
        "config", "g1", "glim"
    )

    # ------------------------------------------------------------------
    # Launch arguments (still override‑able from the CLI)
    # ------------------------------------------------------------------
    livox_config_arg = DeclareLaunchArgument(
        "livox_config",
        default_value=livox_default,
        description="Path to Livox driver JSON configuration file.",
    )

    glim_config_arg = DeclareLaunchArgument(
        "glim_config",
        default_value=glim_default,
        description="Path to GlimROS configuration directory/file.",
    )

    # ------------------------------------------------------------------
    # Composable nodes
    # ------------------------------------------------------------------
    livox_node = ComposableNode(
        package="livox_ros_driver2",
        plugin="livox_ros::DriverNode",
        name="livox_driver",
        parameters=[
            {"user_config_path": LaunchConfiguration("livox_config")},
        ],
    )

    glim_node = ComposableNode(
        package="glim_ros",
        plugin="glim::GlimROS",
        name="glim",
        parameters=[
            {"config_path": LaunchConfiguration("glim_config")},
        ],
    )

    # ------------------------------------------------------------------
    # Container
    # ------------------------------------------------------------------
    container = ComposableNodeContainer(
        name="ComponentManager",  # matches CLI tool default
        namespace="",
        package="rclcpp_components",
        executable="component_container",
        composable_node_descriptions=[livox_node, glim_node],
        output="screen",
        emulate_tty=True,
    )

    return LaunchDescription([
        livox_config_arg,
        glim_config_arg,
        container,
    ])
