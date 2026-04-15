from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'use_gazebo_truth',
            default_value='false',
            description='When true, radar reads /model/enemy_N/odometry from Gazebo',
        ),

        Node(
            package='acsdg_sensors',
            executable='radar_node',
            name='radar_node',
            output='screen',
            emulate_tty=True,
            parameters=[{
                'use_gazebo_truth': LaunchConfiguration('use_gazebo_truth'),
            }],
        ),
        Node(
            package='acsdg_sensors',
            executable='rf_node',
            name='rf_node',
            output='screen',
            emulate_tty=True,
        ),
        Node(
            package='acsdg_sensors',
            executable='sensor_fusion_node',
            name='sensor_fusion_node',
            output='screen',
            emulate_tty=True,
        ),
    ])
