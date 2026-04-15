"""
acsdg_full.launch.py — Full ACSDG system bringup.

Launch order (with TimerAction delays):
  t=0s   acsdg_gazebo  — Gazebo world + drone models + ros_gz_bridge
  t=5s   acsdg_sensors — Radar (Gazebo truth mode), RF, Sensor Fusion
  t=8s   acsdg_c2      — C2 engine, interceptor manager, mission manager,
                         4× interceptor flight controllers
  t=10s  acsdg_ai      — LSTM threat predictor, GNN swarm classifier,
                         online learning node
  t=13s  acsdg_dashboard — rosbridge WebSocket + React HTTP server

Arguments
---------
  headless     bool   Run Gazebo without GUI (default: false)
  difficulty   float  Mission difficulty 0.0–1.0 (default: 0.5)
  enemy_count  int    Number of enemy drones (default: 4)
"""

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, GroupAction, IncludeLaunchDescription,
    LogInfo, TimerAction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def _include(pkg: str, launch_file: str, **kwargs):
    """Helper: include a launch file from a package with optional args."""
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([FindPackageShare(pkg), 'launch', launch_file])
        ]),
        launch_arguments=kwargs.items(),
    )


def generate_launch_description():
    return LaunchDescription([

        # ── Arguments ─────────────────────────────────────────────────────
        DeclareLaunchArgument('headless',     default_value='true',
                              description='Run Gazebo without GUI'),
        DeclareLaunchArgument('difficulty',   default_value='0.5',
                              description='Mission difficulty 0.0-1.0'),
        DeclareLaunchArgument('enemy_count',  default_value='4',
                              description='Number of enemy drones'),

        # ── t=0s: Gazebo world ────────────────────────────────────────────
        LogInfo(msg='[ACSDG] t=0s  Starting Gazebo (military_base world)...'),
        _include('acsdg_gazebo', 'gazebo.launch.py',
                 headless=LaunchConfiguration('headless'),
                 enemy_count=LaunchConfiguration('enemy_count'),
                 bridge='true'),

        # ── t=5s: Sensor layer ────────────────────────────────────────────
        TimerAction(
            period=5.0,
            actions=[
                LogInfo(msg='[ACSDG] t=5s  Starting sensors (Gazebo truth mode)...'),
                _include('acsdg_sensors', 'sensors.launch.py',
                         use_gazebo_truth='true'),
            ],
        ),

        # ── t=8s: C2 layer ───────────────────────────────────────────────
        TimerAction(
            period=8.0,
            actions=[
                LogInfo(msg='[ACSDG] t=8s  Starting C2 engine and flight controllers...'),
                _include('acsdg_c2', 'c2.launch.py'),
            ],
        ),

        # ── t=10s: AI layer ───────────────────────────────────────────────
        TimerAction(
            period=10.0,
            actions=[
                LogInfo(msg='[ACSDG] t=10s Starting AI inference nodes...'),
                _include('acsdg_ai', 'ai.launch.py'),
            ],
        ),

        # ── t=13s: Dashboard ──────────────────────────────────────────────
        TimerAction(
            period=13.0,
            actions=[
                LogInfo(msg='[ACSDG] t=13s Starting dashboard (rosbridge + HTTP)...'),
                _include('acsdg_dashboard', 'dashboard.launch.py'),
                LogInfo(msg='───────────────────────────────────────────────'),
                LogInfo(msg=' ACSDG FULL SYSTEM ONLINE'),
                LogInfo(msg=' Dashboard:  http://172.19.140.70:3000'),
                LogInfo(msg=' rosbridge:  ws://172.19.140.70:9090'),
                LogInfo(msg=' Health:     bash ~/acsdg_ws/scripts/health_check.sh'),
                LogInfo(msg=' Demo:       bash ~/acsdg_ws/scripts/demo_scenario.sh'),
                LogInfo(msg='───────────────────────────────────────────────'),
            ],
        ),
    ])
