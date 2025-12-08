from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

import os


def generate_launch_description():
    # --- Bringup: lidar + rf2o + TF ---
    robo_bringup_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('robo_bringup'),
                'launch',
                'bringup.launch.py'
            )
        )
    )

    # --- slam_toolbox (online async) ---
    slam_config = "/home/tne/robo_ws_2/src/robo_bringup/config/slam_toolbox_mapping.yaml"

    slam_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('slam_toolbox'),
                'launch',
                'online_async_launch.py'
            )
        ),
        launch_arguments={
            'slam_params_file': slam_config
        }.items(),
    )

    return LaunchDescription([
        robo_bringup_launch,
        slam_launch,
    ])
