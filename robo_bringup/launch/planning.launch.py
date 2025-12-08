#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    # Path to your Nav2 params file
    params_file = "/home/tne/robo_ws_2/src/robo_bringup/config/nav2_params_planning.yaml"

    # Match this with your YAML (you already set use_sim_time: False there)
    use_sim_time = False

    common_params = [params_file, {"use_sim_time": use_sim_time}]

    map_server = Node(
        package="nav2_map_server",
        executable="map_server",
        name="map_server",
        output="screen",
        parameters=common_params,
    )

    amcl = Node(
        package="nav2_amcl",
        executable="amcl",
        name="amcl",
        output="screen",
        parameters=common_params,
    )

    planner_server = Node(
        package="nav2_planner",
        executable="planner_server",
        name="planner_server",
        output="screen",
        parameters=common_params,
    )

    # Lifecycle manager to bring the above 3 nodes through their lifecycle states
    lifecycle_manager = Node(
        package="nav2_lifecycle_manager",
        executable="lifecycle_manager",
        name="lifecycle_manager_navigation",
        output="screen",
        parameters=[{
            "use_sim_time": use_sim_time,
            "autostart": True,
            "node_names": [
                "map_server",
                "amcl",
                "planner_server",
            ],
        }],
    )

    return LaunchDescription([
        map_server,
        amcl,
        planner_server,
        lifecycle_manager,
    ])
