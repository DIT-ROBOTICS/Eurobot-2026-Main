import os
from launch import LaunchDescription
from launch.actions import TimerAction, DeclareLaunchArgument
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Get package directory
    pkg_dir = get_package_share_directory('main')

    # Declare launch arguments for parameter files
    map_points_arg = DeclareLaunchArgument(
        'map_points',
        default_value=os.path.join(pkg_dir, 'params', 'map_points.yaml'),
        description='Map points configuration file'
    )

    # Launch configurations
    map_points = LaunchConfiguration('map_points')

    # BT Engine node
    bt_engine_node = TimerAction(
        period=0.5,
        actions=[
            Node(
                package='main',
                executable='bt_engine',
                name='bt_engine',
                output='screen',
                parameters=[
                    map_points,
                    {"frame_id": "base_footprint"},
                    {"tree_name": "MainTree"},
                ]
            )
        ]
    )

    # Startup node (new version)
    startup_node = TimerAction(
        period=0.5,
        actions=[
            Node(
                package='startup',
                executable='startup_new',
                name='startup_node',
                output='screen',
                parameters=[
                    map_points,
                    {"robot_name": "White"},
                    {"plan_code": 1},
                ]
            )
        ]
    )

    return LaunchDescription([
        map_points_arg,
        bt_engine_node,
        startup_node,
    ])
