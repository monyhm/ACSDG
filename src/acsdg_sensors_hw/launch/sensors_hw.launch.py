"""
sensors_hw.launch.py — Launch all ACSDG hardware-abstracted sensor nodes.

Brings up:
  • 6× EchoGuardNode   (echoguard_1 … echoguard_6)
  • 4× RF360Node       (rf360_1 … rf360_4)
  • 4× BosonNode       (boson_1 … boson_4)
  • 1× SensorFusionNode (three-sensor fusion with weights 0.60 / 0.25 / 0.15)

All parameters are loaded from sensor_params.yaml.

To switch any sensor to real hardware, either:
  1. Edit sensor_params.yaml and set use_real_hardware: true, OR
  2. Pass a command-line override:
       ros2 launch acsdg_sensors_hw sensors_hw.launch.py echoguard_hw:=true

The downstream topic tree is identical in both modes:
  /sensors/echoguard_{1..6}/tracks
  /sensors/echoguard_{1..6}/status
  /sensors/rf360_{1..4}/detections
  /sensors/rf360_{1..4}/status
  /sensors/boson_{1..4}/thermal_image
  /sensors/boson_{1..4}/camera_info
  /sensors/boson_{1..4}/detections
  /sensors/boson_{1..4}/status
  /sensors/fusion/targets
  /threats/target_{N}/fused_target
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg = get_package_share_directory("acsdg_sensors_hw")
    params_file = os.path.join(pkg, "config", "sensor_params.yaml")

    # ── Optional HW override arguments ──────────────────────────────────────
    declared_args = [
        DeclareLaunchArgument(
            "echoguard_hw",
            default_value="false",
            description="Set true to use real EchoGuard hardware for ALL radars"),
        DeclareLaunchArgument(
            "rf360_hw",
            default_value="false",
            description="Set true to use real RF-360 hardware for ALL sensors"),
        DeclareLaunchArgument(
            "boson_hw",
            default_value="false",
            description="Set true to use real Boson+ hardware for ALL cameras"),
    ]

    # ── EchoGuard nodes (6 instances, hexagon r=150 m) ──────────────────────
    echoguard_nodes = []
    sensor_positions = [
        # (id, x, y, yaw_rad)  — inward-facing yaw = angle + π
        (1, 150.0,   0.0,    3.1416),
        (2,  75.0, 129.9,   -2.6180),
        (3, -75.0, 129.9,   -1.5708),
        (4, -150.0,  0.0,    0.0),
        (5, -75.0, -129.9,   0.5236),
        (6,  75.0, -129.9,   1.5708),
    ]
    for sid, sx, sy, yaw in sensor_positions:
        echoguard_nodes.append(
            Node(
                package="acsdg_sensors_hw",
                executable="echoguard_node",
                name=f"echoguard_{sid}",
                namespace="",
                parameters=[
                    params_file,
                    {
                        "sensor_id":  sid,
                        "frame_id":  f"echoguard_{sid}",
                        "sensor_x":   sx,
                        "sensor_y":   sy,
                        "sensor_z":   1.5,
                        "sensor_yaw": yaw,
                    },
                ],
                output="screen",
                emulate_tty=True,
            )
        )

    # ── RF-360 nodes (4 instances, defense post corners r≈250 m) ────────────
    rf360_nodes = []
    rf360_positions = [
        # (id, x, y)
        (1,  177.0,  177.0),
        (2, -177.0,  177.0),
        (3,  177.0, -177.0),
        (4, -177.0, -177.0),
    ]
    for sid, sx, sy in rf360_positions:
        rf360_nodes.append(
            Node(
                package="acsdg_sensors_hw",
                executable="rf360_node",
                name=f"rf360_{sid}",
                namespace="",
                parameters=[
                    params_file,
                    {
                        "sensor_id":  sid,
                        "frame_id":  f"rf360_{sid}",
                        "sensor_x":   sx,
                        "sensor_y":   sy,
                        "sensor_z":   3.0,
                    },
                ],
                output="screen",
                emulate_tty=True,
            )
        )

    # ── Boson+ nodes (4 instances, guard towers) ─────────────────────────────
    boson_nodes = []
    boson_positions = [
        # (id, x, y, v4l2_dev)
        (1,  190.0,  190.0, 0),
        (2, -190.0,  190.0, 1),
        (3,  190.0, -190.0, 2),
        (4, -190.0, -190.0, 3),
    ]
    for sid, sx, sy, dev in boson_positions:
        boson_nodes.append(
            Node(
                package="acsdg_sensors_hw",
                executable="boson_node",
                name=f"boson_{sid}",
                namespace="",
                parameters=[
                    params_file,
                    {
                        "sensor_id":     sid,
                        "frame_id":     f"boson_{sid}",
                        "sensor_x":      sx,
                        "sensor_y":      sy,
                        "sensor_z":      13.0,
                        "v4l2_device":   dev,
                    },
                ],
                output="screen",
                emulate_tty=True,
            )
        )

    # ── Sensor fusion node ───────────────────────────────────────────────────
    fusion_node = Node(
        package="acsdg_sensors_hw",
        executable="sensor_fusion_node",
        name="sensor_fusion_node",
        namespace="",
        parameters=[params_file],
        output="screen",
        emulate_tty=True,
    )

    return LaunchDescription(
        declared_args
        + echoguard_nodes
        + rf360_nodes
        + boson_nodes
        + [fusion_node]
    )
