# nav2_planner_only.launch.py

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # Adjust this path to your actual params file
    nav2_params = '/home/tne/robo_ws_2/src/robo_bringup/config/nav2_params_planning.yaml'

    # If you ever want to toggle sim time, change here and in YAML
    use_sim_time = False

    # Common params list for Nav2 nodes
    common_params = [nav2_params, {'use_sim_time': use_sim_time}]

    map_server = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=common_params,
    )

    amcl = Node(
        package='nav2_amcl',
        executable='amcl',
        name='amcl',
        output='screen',
        parameters=common_params,
    )

    planner_server = Node(
        package='nav2_planner',
        executable='planner_server',
        name='planner_server',
        output='screen',
    ) 

    bt_navigator = Node(
        package='nav2_bt_navigator',
        executable='bt_navigator',
        name='bt_navigator',
        output='screen',
        parameters=common_params,
    )

    waypoint_follower = Node(
        package='nav2_waypoint_follower',
        executable='waypoint_follower',
        name='waypoint_follower',
        output='screen',
        parameters=common_params,
    )

    lifecycle_manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_navigation',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'autostart': True,
            'node_names': [
                'map_server',
                'amcl',
                'planner_server',
                'controller_server',  
                'bt_navigator',
                'waypoint_follower',
            ],
        }],
    )

    # Your custom follower only thing that publishes /cmd_vel
    # path_follower = Node(
    #     package='goal_follower',
    #     executable='path_follower',
    #     name='path_follower',
    #     output='screen',
    #     parameters=[{
    #         'cmd_vel_topic': '/cmd_vel',
    #         'path_topic': '/plan',
    #         'global_frame': 'map',
    #         'base_frame': 'base_link',
    #         'waypoint_spacing': 0.3,
    #     }],
    # )

    return LaunchDescription([
        map_server,
        amcl,
        planner_server,
        bt_navigator,
        waypoint_follower,
        lifecycle_manager,
        ##path_follower,
    ])
