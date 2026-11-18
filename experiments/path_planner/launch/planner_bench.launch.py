"""Planner-only Nav2 stack for the A*/Dijkstra comparison: map_server + planner_server on a static map.

No Gazebo, AMCL or controller. A static map -> base_footprint transform stands in for localization because every
request carries its own start pose (ComputePathToPose use_start: true).

    ros2 launch experiments/path_planner/launch/planner_bench.launch.py map:=<map.yaml> [params_file:=<yaml>]
"""

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

DEFAULT_PARAMS = os.path.normpath(os.path.join(os.path.dirname(__file__), '..', 'config', 'navfn_compare.yaml'))


def generate_launch_description():
    params = LaunchConfiguration('params_file')
    return LaunchDescription([
        DeclareLaunchArgument('map', description='map yaml served by map_server (same file as the scenario map)'),
        DeclareLaunchArgument('params_file', default_value=DEFAULT_PARAMS),
        Node(package='tf2_ros', executable='static_transform_publisher', name='bench_map_to_base',
             arguments=['--frame-id', 'map', '--child-frame-id', 'base_footprint']),
        Node(package='nav2_map_server', executable='map_server', name='map_server', output='screen',
             parameters=[params, {'yaml_filename': LaunchConfiguration('map')}]),
        Node(package='nav2_planner', executable='planner_server', name='planner_server', output='screen',
             parameters=[params]),
        Node(package='nav2_lifecycle_manager', executable='lifecycle_manager', name='lifecycle_manager_bench',
             output='screen', parameters=[{'autostart': True, 'node_names': ['map_server', 'planner_server']}]),
    ])
