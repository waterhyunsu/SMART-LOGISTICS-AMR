"""Driving comparison stack: Gazebo Harmonic + TurtleBot3 Waffle Pi + Nav2 (AMCL, NavFn, DWB, BT navigator).

[초안-팀합의필요] B's temporary environment for the A*/Dijkstra driving comparison. It is not D's deployment
launch (amr_navigation); when D's Nav2 environment exists the comparison moves there.

    ros2 launch experiments/path_planner/launch/drive_bench.launch.py \
        world:=<world.sdf> map:=<map.yaml> [params_file:=<yaml>] [gui:=true] [x:=0.0 y:=0.0 yaw:=1.5708]

Needs TURTLEBOT3_MODEL=waffle_pi and the packages of docker/Dockerfile.sim.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import AppendEnvironmentVariable, DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

BENCH = os.path.normpath(os.path.join(os.path.dirname(__file__), '..'))
MODEL = 'turtlebot3_' + os.environ.get('TURTLEBOT3_MODEL', 'waffle_pi')
ROBOT_NAME = 'waffle_pi'


def generate_launch_description():
    tb3 = get_package_share_directory('turtlebot3_gazebo')
    gz_launch = os.path.join(get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')
    world, gui = LaunchConfiguration('world'), LaunchConfiguration('gui')
    return LaunchDescription([
        DeclareLaunchArgument('world', description='Gazebo Harmonic world (sim/worlds/*.sdf)'),
        DeclareLaunchArgument('map', description='map yaml of the same world (the scenario map)'),
        DeclareLaunchArgument('params_file', default_value=os.path.join(BENCH, 'config', 'nav2_drive.yaml')),
        DeclareLaunchArgument('gui', default_value='false', description='also start the Gazebo window'),
        DeclareLaunchArgument('x', default_value='0.0'),
        DeclareLaunchArgument('y', default_value='0.0'),
        DeclareLaunchArgument('yaw', default_value='1.5708'),
        AppendEnvironmentVariable('GZ_SIM_RESOURCE_PATH', os.path.join(tb3, 'models')),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(gz_launch),
                                 launch_arguments={'gz_args': ['-r -s -v2 ', world],
                                                   'on_exit_shutdown': 'true'}.items()),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(gz_launch), condition=IfCondition(gui),
                                 launch_arguments={'gz_args': '-g -v2 '}.items()),
        Node(package='ros_gz_sim', executable='create', output='screen',
             arguments=['-name', ROBOT_NAME, '-file', os.path.join(tb3, 'models', MODEL, 'model.sdf'),
                        '-x', LaunchConfiguration('x'), '-y', LaunchConfiguration('y'), '-z', '0.01',
                        '-Y', LaunchConfiguration('yaw')]),
        Node(package='ros_gz_bridge', executable='parameter_bridge', output='screen',
             arguments=['--ros-args', '-p',
                        'config_file:=' + os.path.join(tb3, 'params', MODEL + '_bridge.yaml')]),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(tb3, 'launch', 'robot_state_publisher.launch.py')),
            launch_arguments={'use_sim_time': 'true'}.items()),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(get_package_share_directory('nav2_bringup'), 'launch',
                                                       'bringup_launch.py')),
            launch_arguments={'map': LaunchConfiguration('map'), 'params_file': LaunchConfiguration('params_file'),
                              'use_sim_time': 'true', 'autostart': 'true', 'use_composition': 'False'}.items()),
    ])
