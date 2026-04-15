"""
spawn_drone.launch.py — Spawn a single drone model into a running Gazebo world.

Arguments
---------
  model_name  str   Name to give the model in Gazebo (e.g. enemy_1)
  model_type  str   'enemy_drone' or 'interceptor_drone'
  x, y, z     float Spawn position in world frame
  yaw         float Heading in radians (default 0)
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_share  = get_package_share_directory('acsdg_gazebo')
    model_path = os.path.join(pkg_share, 'models')

    return LaunchDescription([
        DeclareLaunchArgument('model_name', default_value='drone_1'),
        DeclareLaunchArgument('model_type', default_value='enemy_drone'),
        DeclareLaunchArgument('x',   default_value='0.0'),
        DeclareLaunchArgument('y',   default_value='0.0'),
        DeclareLaunchArgument('z',   default_value='1.0'),
        DeclareLaunchArgument('yaw', default_value='0.0'),

        ExecuteProcess(
            cmd=[
                'gz', 'service',
                '-s', '/world/military_base/create',
                '--reqtype', 'gz.msgs.EntityFactory',
                '--reptype', 'gz.msgs.Boolean',
                '--timeout', '5000',
                '--req',
                [
                    'sdf_filename: "',
                    model_path, '/',
                    LaunchConfiguration('model_type'),
                    '/model.sdf", ',
                    'name: "', LaunchConfiguration('model_name'), '", ',
                    'pose: {position: {x: ', LaunchConfiguration('x'),
                    ', y: ', LaunchConfiguration('y'),
                    ', z: ', LaunchConfiguration('z'), '}}',
                ],
            ],
            output='screen',
        ),
    ])
