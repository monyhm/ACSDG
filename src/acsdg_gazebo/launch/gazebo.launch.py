"""
gazebo.launch.py — Launch Gazebo Harmonic with the ACSDG military base world.

Arguments
---------
  enemy_count       int   Number of enemy drones (default 4)
  interceptor_count int   Number of interceptor drones (default 4)
  difficulty        float Mission difficulty 0.0–1.0 (default 0.5)
  headless          bool  Run Gazebo without GUI (default false)
  bridge            bool  Launch ros_gz_bridge (default true)
"""

import os
import math
import pathlib

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, ExecuteProcess, LogInfo,
    OpaqueFunction, SetEnvironmentVariable,
)
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_share = get_package_share_directory('acsdg_gazebo')
    world_file  = os.path.join(pkg_share, 'worlds', 'military_base.sdf')
    model_path  = os.path.join(pkg_share, 'models')
    bridge_cfg  = os.path.join(pkg_share, 'config', 'bridge_config.yaml')

    # Also include PX4 models and acsdg_sensors_hw models
    px4_model_path = os.path.expanduser('~/PX4-Autopilot/Tools/simulation/gz/models')
    sensors_hw_share = get_package_share_directory('acsdg_sensors_hw')
    sensors_model_path = os.path.join(sensors_hw_share, 'models')

    gz_model_path = f"{model_path}:{sensors_model_path}"
    if os.path.isdir(px4_model_path):
        gz_model_path = f"{gz_model_path}:{px4_model_path}"

    return LaunchDescription([
        # ── Arguments ─────────────────────────────────────────────────────
        DeclareLaunchArgument('enemy_count',       default_value='4'),
        DeclareLaunchArgument('interceptor_count', default_value='4'),
        DeclareLaunchArgument('difficulty',        default_value='0.5'),
        DeclareLaunchArgument('headless',          default_value='false'),
        DeclareLaunchArgument('bridge',            default_value='true'),

        # ── Environment: tell Gazebo where to find our models ─────────────
        SetEnvironmentVariable('GZ_SIM_RESOURCE_PATH', gz_model_path),

        # ── Gazebo Harmonic (gz sim) ───────────────────────────────────────
        ExecuteProcess(
            cmd=[
                'gz', 'sim', '-r',
                PythonExpression([
                    '"--headless-rendering -s " if "', LaunchConfiguration('headless'),
                    '" == "true" else ""',
                ]),
                world_file,
            ],
            output='screen',
            additional_env={'GZ_SIM_RESOURCE_PATH': gz_model_path},
            condition=IfCondition(
                PythonExpression(['"false" if "', LaunchConfiguration('headless'), '" == "true" else "true"'])
            ),
        ),

        # Headless path (server only, no GUI)
        ExecuteProcess(
            cmd=['gz', 'sim', '-r', '-s', '--headless-rendering', world_file],
            output='screen',
            additional_env={'GZ_SIM_RESOURCE_PATH': gz_model_path},
            condition=IfCondition(LaunchConfiguration('headless')),
        ),

        # ── ros_gz_bridge ─────────────────────────────────────────────────
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            name='ros_gz_bridge',
            output='screen',
            parameters=[{'config_file': bridge_cfg}],
            condition=IfCondition(LaunchConfiguration('bridge')),
        ),

        # ── Data-plane shim for cmd_vel + odometry ────────────────────────
        # The stock ros_gz_bridge in Humble ships as ign.msgs.*; Gazebo
        # Harmonic wants gz.msgs.*. This shim uses gz.transport13 directly.
        Node(
            package='acsdg_gazebo',
            executable='gz_bridge_shim',
            name='gz_bridge_shim',
            output='screen',
            emulate_tty=True,
            condition=IfCondition(LaunchConfiguration('bridge')),
        ),

        # ── Enemy drone driver ────────────────────────────────────────────
        # Waits for wave_trigger; publishes cmd_vel to drive drones toward origin
        Node(
            package='acsdg_gazebo',
            executable='enemy_driver_node',
            name='enemy_driver_node',
            output='screen',
            emulate_tty=True,
            condition=IfCondition(LaunchConfiguration('bridge')),
        ),

        # ── Info ──────────────────────────────────────────────────────────
        LogInfo(msg='─────────────────────────────────────────────────────'),
        LogInfo(msg='  ACSDG Gazebo: military_base world loading...'),
        LogInfo(msg='  Models path: ' + gz_model_path),
        LogInfo(msg='  Verify: gz topic -l | grep military_base'),
        LogInfo(msg='─────────────────────────────────────────────────────'),
    ])
