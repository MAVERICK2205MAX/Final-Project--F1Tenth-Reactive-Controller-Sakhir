#!/usr/bin/env python3
"""
Final Project — Challenge 1: Static Obstacles Competition Launch File
Track: Sakhir (Group 2) | Student: Islam Qaj Mazarul (MAVERICK2205MAX)

Launches:
1. Official F1Tenth Gym Bridge with modified Sakhir_obs_map (5 static obstacles)
2. Reactive Follow-The-Gap Controller node (with Adaptive Gap Centering)
3. Live Telemetry HUD Overlay (Lap Counter, Stopwatch, and Speed Gauges)
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    gym_bridge_pkg = get_package_share_directory('f1tenth_gym_ros')
    controllers_pkg = get_package_share_directory('controllers')

    # Path to gym bridge launch
    gym_bridge_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gym_bridge_pkg, 'launch', 'gym_bridge_launch.py')
        )
    )

    # Reactive Follow-The-Gap controller node (Obstacle & Anti-Wobble Optimized)
    gap_controller_node = Node(
        package='controllers',
        executable='gap_node',
        name='reactive_gap_controller',
        output='screen',
        parameters=[{
            'bubble_radius': 0.28,
            'safety_width': 0.38,
            'bubble_max_dist': 3.0,
            'max_range': 8.0,
            'fov_deg': 70.0,
            'v_min': 2.5,
            'v_max': 7.8,
            'disparity_threshold': 1.1,
            'ema_alpha': 0.32,
            'max_steer_velocity': 4.0,
            'v_steer_base': 3.6,
            'v_steer_pow': 1.0,
            'd_floor': 1.8,
            'd_range': 4.0,
            'target_laps': 10,
            'log_csv_path': os.path.expanduser('~/Documents/Final Project - Reactive Controller Competition in\nF1Tenth/sakhir_lap_records.csv')
        }]
    )

    # Live Telemetry GUI Overlay (OpenCV Dashboard Window)
    telemetry_gui_node = Node(
        package='controllers',
        executable='telemetry_gui',
        name='telemetry_gui_overlay',
        output='screen'
    )

    return LaunchDescription([
        gym_bridge_launch,
        gap_controller_node,
        telemetry_gui_node
    ])
