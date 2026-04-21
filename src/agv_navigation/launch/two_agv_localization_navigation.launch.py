#!/usr/bin/env python3

import copy
import os
import tempfile

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


ROBOTS = [
    ("agv_01", 0.0, -1.0),
    ("agv_02", 0.0, 1.0),
]


def _as_bool(value):
    return str(value).lower() in ("1", "true", "yes", "on")


def _replace_strings(value, robot_id):
    replacements = {
        "base_footprint": f"{robot_id}_base_footprint",
        "base_link": f"{robot_id}_base_link",
        "odom": f"{robot_id}_odom",
        "/agv/odom": f"/{robot_id}/odom",
        "/agv/scan": f"/{robot_id}/scan",
    }
    if isinstance(value, str):
        return replacements.get(value, value)
    if isinstance(value, list):
        return [_replace_strings(item, robot_id) for item in value]
    if isinstance(value, dict):
        return {
            key: _replace_strings(item, robot_id)
            for key, item in value.items()
        }
    return value


def _set_use_sim_time(node_params, use_sim_time):
    for node_cfg in node_params.values():
        if not isinstance(node_cfg, dict):
            continue
        ros_params = node_cfg.get("ros__parameters")
        if isinstance(ros_params, dict):
            ros_params["use_sim_time"] = use_sim_time
        for child_cfg in node_cfg.values():
            if not isinstance(child_cfg, dict):
                continue
            child_params = child_cfg.get("ros__parameters")
            if isinstance(child_params, dict):
                child_params["use_sim_time"] = use_sim_time


def _write_robot_params(base_params, robot_id, initial_x, initial_y,
                        map_file, use_sim_time):
    params = _replace_strings(copy.deepcopy(base_params), robot_id)
    _set_use_sim_time(params, use_sim_time)

    amcl_params = params["amcl"]["ros__parameters"]
    amcl_params["initial_pose"]["x"] = float(initial_x)
    amcl_params["initial_pose"]["y"] = float(initial_y)
    amcl_params["transform_tolerance"] = 0.1

    params["map_server"]["ros__parameters"]["yaml_filename"] = map_file

    fd, path = tempfile.mkstemp(
        prefix=f"{robot_id}_nav2_", suffix=".yaml")
    with os.fdopen(fd, "w") as stream:
        yaml.safe_dump({robot_id: params}, stream, sort_keys=False)
    return path


def _nav2_nodes(robot_id, params_file, use_sim_time, autostart):
    tf_remaps = [("/tf", "/tf"), ("/tf_static", "/tf_static")]
    lifecycle_localization = ["map_server", "amcl"]
    lifecycle_navigation = [
        "controller_server",
        "smoother_server",
        "planner_server",
        "behavior_server",
        "bt_navigator",
        "waypoint_follower",
        "velocity_smoother",
    ]

    common = {
        "namespace": robot_id,
        "parameters": [params_file],
        "output": "screen",
    }

    return [
        Node(
            package="nav2_map_server",
            executable="map_server",
            name="map_server",
            remappings=tf_remaps,
            **common,
        ),
        Node(
            package="nav2_amcl",
            executable="amcl",
            name="amcl",
            remappings=tf_remaps,
            **common,
        ),
        Node(
            package="nav2_lifecycle_manager",
            executable="lifecycle_manager",
            name="lifecycle_manager_localization",
            namespace=robot_id,
            output="screen",
            parameters=[{
                "use_sim_time": use_sim_time,
                "autostart": autostart,
                "node_names": lifecycle_localization,
            }],
        ),
        Node(
            package="nav2_controller",
            executable="controller_server",
            name="controller_server",
            remappings=tf_remaps + [("cmd_vel", "cmd_vel_nav")],
            **common,
        ),
        Node(
            package="nav2_smoother",
            executable="smoother_server",
            name="smoother_server",
            remappings=tf_remaps,
            **common,
        ),
        Node(
            package="nav2_planner",
            executable="planner_server",
            name="planner_server",
            remappings=tf_remaps,
            **common,
        ),
        Node(
            package="nav2_behaviors",
            executable="behavior_server",
            name="behavior_server",
            remappings=tf_remaps,
            **common,
        ),
        Node(
            package="nav2_bt_navigator",
            executable="bt_navigator",
            name="bt_navigator",
            remappings=tf_remaps,
            **common,
        ),
        Node(
            package="nav2_waypoint_follower",
            executable="waypoint_follower",
            name="waypoint_follower",
            remappings=tf_remaps,
            **common,
        ),
        Node(
            package="nav2_velocity_smoother",
            executable="velocity_smoother",
            name="velocity_smoother",
            remappings=tf_remaps + [
                ("cmd_vel", "cmd_vel_nav"),
                ("cmd_vel_smoothed", "cmd_vel"),
            ],
            **common,
        ),
        Node(
            package="nav2_lifecycle_manager",
            executable="lifecycle_manager",
            name="lifecycle_manager_navigation",
            namespace=robot_id,
            output="screen",
            parameters=[{
                "use_sim_time": use_sim_time,
                "autostart": autostart,
                "node_names": lifecycle_navigation,
            }],
        ),
    ]


def _launch_setup(context, *args, **kwargs):
    params_file = LaunchConfiguration("params_file").perform(context)
    map_file = LaunchConfiguration("map").perform(context)
    use_sim_time = _as_bool(
        LaunchConfiguration("use_sim_time").perform(context))
    autostart = _as_bool(LaunchConfiguration("autostart").perform(context))

    with open(params_file, "r") as stream:
        base_params = yaml.safe_load(stream)

    actions = []
    for robot_id, initial_x, initial_y in ROBOTS:
        robot_params = _write_robot_params(
            base_params, robot_id, initial_x, initial_y,
            map_file, use_sim_time)
        actions.extend(
            _nav2_nodes(robot_id, robot_params, use_sim_time, autostart))
    return actions


def generate_launch_description():
    nav_dir = get_package_share_directory("agv_navigation")
    default_params = os.path.join(nav_dir, "config", "nav2_params.yaml")
    default_map = os.path.join(nav_dir, "maps", "warehouse.yaml")

    return LaunchDescription([
        SetEnvironmentVariable("RCUTILS_LOGGING_BUFFERED_STREAM", "1"),
        DeclareLaunchArgument("use_sim_time", default_value="true"),
        DeclareLaunchArgument("autostart", default_value="true"),
        DeclareLaunchArgument("params_file", default_value=default_params),
        DeclareLaunchArgument("map", default_value=default_map),
        OpaqueFunction(function=_launch_setup),
    ])
