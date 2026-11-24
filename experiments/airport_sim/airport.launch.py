"""Launch the airport world with Jazzy's packaged TurtleBot3 and Nav2."""

import os
from pathlib import Path
from tempfile import TemporaryDirectory
from xml.etree import ElementTree as ET

import xacro
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    EmitEvent,
    ExecuteProcess,
    IncludeLaunchDescription,
    OpaqueFunction,
    RegisterEventHandler,
    TimerAction,
)
from launch.conditions import IfCondition, UnlessCondition
from launch.event_handlers import OnShutdown
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

ASSETS = Path(__file__).resolve().parent / "assets"


def setup(context):
    if not os.environ.get("ROS_DOMAIN_ID", "").isdigit() or (
            os.environ.get("ROS_AUTOMATIC_DISCOVERY_RANGE") != "LOCALHOST"
            or not os.environ.get("GZ_PARTITION")):
        raise RuntimeError("Set a dedicated SIM ROS_DOMAIN_ID, LOCALHOST discovery and GZ_PARTITION")

    nav2 = Path(get_package_share_directory("nav2_bringup"))
    tb3 = Path(get_package_share_directory("nav2_minimal_tb3_sim"))
    home = yaml.safe_load((ASSETS / "waypoints.yaml").read_text())["waypoints"]["HOME"]
    headless = LaunchConfiguration("headless")
    use_rviz = LaunchConfiguration("use_rviz")

    # Write the SDF before launching the server. Starting a separate xacro
    # child alongside Gazebo can leave Gazebo trying to open a missing world.
    temp = TemporaryDirectory(prefix="moveit-airport-")
    world = Path(temp.name) / "airport.sdf"
    robot = Path(temp.name) / "waffle_observed.sdf"
    # Read-only Gazebo world pose for the experiment. It is never sent to AMCL,
    # TF, the controller, or either of the planners.
    waffle = ET.fromstring(xacro.process_file(str(tb3 / "urdf" / "gz_waffle.sdf.xacro")).toxml())
    observer = ET.SubElement(waffle.find("model"), "plugin",
                             filename="gz-sim-odometry-publisher-system",
                             name="gz::sim::systems::OdometryPublisher")
    for key, value in {"odom_topic": "/airport/world_odom", "odom_frame": "world",
                       "robot_base_frame": "base_footprint", "dimensions": "2",
                       "odom_publish_frequency": "30"}.items():
        ET.SubElement(observer, key).text = value
    ET.ElementTree(waffle).write(robot, encoding="utf-8")
    params_path = Path(temp.name) / "nav2_params.yaml"
    world.write_text(xacro.process_file(
        str(ASSETS / "airport.sdf.xacro"),
        mappings={"headless": headless.perform(context)},
    ).toxml())
    params = yaml.safe_load((nav2 / "params" / "nav2_params.yaml").read_text())
    # Scope localization tuning to this simulated robot. Copy the planner,
    # controller and costmap configuration from the installed Jazzy profile.
    params["amcl"]["ros__parameters"].update({
        "set_initial_pose": True,
        "initial_pose": home,
        "alpha1": 0.008,
        "alpha2": 0.008,
        "alpha3": 0.008,
        "alpha4": 0.008,
        "update_min_d": 0.12,
        "update_min_a": 0.12,
        "laser_model_type": "likelihood_field_prob",
        "laser_max_range": 20.0,
        "max_beams": 90,
        "sigma_hit": 0.15,
    })
    params["controller_server"]["ros__parameters"]["general_goal_checker"]["xy_goal_tolerance"] = 0.15
    params_path.write_text(yaml.safe_dump(params))

    return [
        RegisterEventHandler(OnShutdown(on_shutdown=[OpaqueFunction(
            function=lambda _: temp.cleanup()
        )])),
        ExecuteProcess(cmd=["gz", "sim", "-r", "-s", str(world)], output="screen",
                       on_exit=[EmitEvent(event=Shutdown(reason="Gazebo server stopped"))]),
        ExecuteProcess(cmd=["gz", "sim", "-g"], condition=UnlessCondition(headless),
                       output="screen"),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(tb3 / "launch" / "spawn_tb3.launch.py")),
            launch_arguments={"robot_sdf": str(robot),
                              "x_pose": str(home["x"]), "y_pose": str(home["y"]),
                              "yaw": str(home["yaw"])}.items(),
        ),
        Node(package="ros_gz_bridge", executable="parameter_bridge", name="airport_pose_observer",
             arguments=["/airport/world_odom@nav_msgs/msg/Odometry[gz.msgs.Odometry"]),
        Node(package="robot_state_publisher", executable="robot_state_publisher",
             parameters=[{"use_sim_time": True, "robot_description":
                          (tb3 / "urdf" / "turtlebot3_waffle.urdf").read_text()}],
             output="screen"),
        TimerAction(period=7.0, actions=[IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(nav2 / "launch" / "bringup_launch.py")),
            launch_arguments={"map": str(ASSETS / "airport.yaml"), "use_sim_time": "true",
                              "use_composition": "False", "params_file": str(params_path)}.items(),
        )]),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(nav2 / "launch" / "rviz_launch.py")),
            condition=IfCondition(use_rviz),
            launch_arguments={"use_sim_time": "true"}.items(),
        ),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("headless", default_value="true"),
        DeclareLaunchArgument("use_rviz", default_value="false"),
        OpaqueFunction(function=setup),
    ])
