from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

import os


def generate_launch_description():
    # --- RPLidar A1 driver launch ---
    # Adjust the filename if your launch file is named differently
    rplidar_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('rplidar_ros'),
                'launch',
                'rplidar_a1_launch.py'  # or 'rp_lidar_a1_launch.py' in your setup
            )
        )
    )

    # --- Static transform: base_link -> laser ---
    #
    # Replace x y z roll pitch yaw with your actual lidar pose on the robot.
    # These are in meters and radians.
    static_tf_laser = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf',
        arguments=[
            # x, y, z
            '0.0', '0.0', '0.0',
            # roll, pitch, yaw
            '0.0', '0.0', '0.0',
            # parent frame, child frame
            'base_link', 'laser'
        ]
    )
    
    rf2o_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('rf2o_laser_odometry'),
                'launch',
                'rf2o_laser_odometry.launch.py'
            )
        ),
        launch_arguments={
            'laser_scan_topic': '/scan',
            'odom_topic': '/odom',
            'odom_frame_id': 'odom',
            'base_frame_id': 'base_link',
            'publish_tf': 'true',
        }.items(),
    )
    
    # Remap odom_rf2o → /odom
    # rf2o_remap = Node(
    #     package='rf2o_laser_odometry',
    #     executable='rf2o_laser_odometry_node',
    #     remappings=[('odom_rf2o', '/odom')],
    #     # We *don’t* run this node — we only use it to apply remapping rules
    # )
    
    # --- rf2o laser odometry node ---
    # This is equivalent to:
    # ros2 launch rf2o_laser_odometry rf2o_laser_odometry.launch.py \
    #   laser_scan_topic:=/scan \
    #   odom_frame_id:=odom \
    #   base_frame_id:=base_link \
    #   publish_tf:=true \
    #   --ros-args -r odom_rf2o:=/odom
    
    # rf2o_node = Node(
    #     package='rf2o_laser_odometry',
    #     executable='rf2o_laser_odometry_node',
    #     name='rf2o_laser_odometry',
    #     output='screen',
    #     parameters=[{
    #         'laser_scan_topic': '/scan',
    #         'odom_frame_id': 'odom',
    #         'base_frame_id': 'base_link',
    #         'publish_tf': True,
    #         'odom_topic': '/odom', 
    #         'init_pose_from_topic': '/rf2o_init_pose', ## TRY
    #     }],
    #     # remappings=[
    #     #     ('odom_rf2o', '/odom'),
    #     # ],
    # ) 

    return LaunchDescription([
        rplidar_launch,
        static_tf_laser,
        rf2o_launch,
    ])