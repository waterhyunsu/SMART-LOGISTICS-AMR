"""Research 1 Nav2 stack: localization (map_server + AMCL) + navigation (planner, controller, smoother, behaviors,
BT navigator, velocity smoother), all with experiments/r1/nav2/nav2_r1.yaml.

    ros2 launch experiments/r1/nav2/nav2_r1.launch.py map:=<map.yaml> [params_file:=...] [use_sim_time:=true]
                                                     [cmd_vel_out:=cmd_vel_pc]

Same nodes and remappings as nav2_bringup localization_launch.py + navigation_launch.py (Humble), except:
  - the velocity smoother output goes to cmd_vel_out (default cmd_vel_pc), so commands pass through this project's
    command watchdog (scripts/pi_cmd_watchdog.py: cmd_vel_pc -> cmd_vel, stops when commands stop), like the bridge on
    the real robot. Use cmd_vel_out:=cmd_vel only when no watchdog runs
  - waypoint_follower is not started (not used by research 1)
"""
import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from nav2_common.launch import RewrittenYaml

HERE = os.path.dirname(os.path.abspath(__file__))


def generate_launch_description():
    map_yaml = LaunchConfiguration('map')
    params_file = LaunchConfiguration('params_file')
    use_sim_time = LaunchConfiguration('use_sim_time')
    log_level = LaunchConfiguration('log_level')
    cmd_vel_out = LaunchConfiguration('cmd_vel_out')

    params = RewrittenYaml(source_file=params_file, root_key='',
                           param_rewrites={'use_sim_time': use_sim_time, 'yaml_filename': map_yaml,
                                           'autostart': 'true'},
                           convert_types=True)
    remap_tf = [('/tf', 'tf'), ('/tf_static', 'tf_static')]
    args = ['--ros-args', '--log-level', log_level]

    def node(pkg, exe, name, remaps=()):
        return Node(package=pkg, executable=exe, name=name, output='screen', respawn=False,
                    parameters=[params], arguments=args, remappings=remap_tf + list(remaps))

    localization = ['map_server', 'amcl']
    navigation = ['controller_server', 'smoother_server', 'planner_server', 'behavior_server', 'bt_navigator',
                  'velocity_smoother']
    return LaunchDescription([
        SetEnvironmentVariable('RCUTILS_LOGGING_BUFFERED_STREAM', '1'),
        DeclareLaunchArgument('map', description='맵 yaml'),
        DeclareLaunchArgument('params_file', default_value=os.path.join(HERE, 'nav2_r1.yaml')),
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('log_level', default_value='info'),
        DeclareLaunchArgument('cmd_vel_out', default_value='cmd_vel_pc', description='속도 명령 출력 토픽'),
        node('nav2_map_server', 'map_server', 'map_server'),
        node('nav2_amcl', 'amcl', 'amcl'),
        node('nav2_controller', 'controller_server', 'controller_server', [('cmd_vel', 'cmd_vel_nav')]),
        node('nav2_smoother', 'smoother_server', 'smoother_server'),
        node('nav2_planner', 'planner_server', 'planner_server'),
        node('nav2_behaviors', 'behavior_server', 'behavior_server'),
        node('nav2_bt_navigator', 'bt_navigator', 'bt_navigator'),
        node('nav2_velocity_smoother', 'velocity_smoother', 'velocity_smoother',
             [('cmd_vel', 'cmd_vel_nav'), ('cmd_vel_smoothed', cmd_vel_out)]),
        Node(package='nav2_lifecycle_manager', executable='lifecycle_manager', name='lifecycle_manager_localization',
             output='screen', arguments=args,
             parameters=[{'use_sim_time': use_sim_time, 'autostart': True, 'node_names': localization}]),
        Node(package='nav2_lifecycle_manager', executable='lifecycle_manager', name='lifecycle_manager_navigation',
             output='screen', arguments=args,
             parameters=[{'use_sim_time': use_sim_time, 'autostart': True, 'node_names': navigation}]),
    ])
