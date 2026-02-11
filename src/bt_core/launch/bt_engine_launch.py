import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Get source directory (relative to this launch file)
    launch_dir = os.path.dirname(os.path.realpath(__file__))
    src_dir = os.path.dirname(launch_dir)  # main package src dir
    startup_dir = os.path.join(os.path.dirname(src_dir), 'startup')
    ros_domain_id = os.getenv('ROS_DOMAIN_ID')
    
    if ros_domain_id == '11':
        map_points_name = 'map_points_white.yaml'
        robot_config_name = 'robot_config_white.yaml'
    elif ros_domain_id == '13':
        map_points_name = 'map_points_black.yaml'
        robot_config_name = 'robot_config_black.yaml'
    else:
        map_points_name = 'map_points_default.yaml'
        robot_config_name = 'robot_config_default.yaml'
    
    # Declare launch arguments for parameter files
    map_points_arg = DeclareLaunchArgument(
        'map_points',
        default_value=os.path.join(src_dir, 'params', map_points_name),
        description='Map points configuration file'
    )
    
    robot_config_arg = DeclareLaunchArgument(
        'robot_config',
        default_value=os.path.join(startup_dir, 'params', robot_config_name),
        description='Robot configuration file'
    )

    # Launch configurations
    map_points = LaunchConfiguration('map_points')
    robot_config = LaunchConfiguration('robot_config')

    # BT Engine node
    bt_engine_node = Node(
        package='bt_core',
        executable='bt_engine',
        name='bt_engine',
        output='screen',
        parameters=[
            robot_config,
            map_points,
            {"frame_id": "base_footprint"},
            {"tree_name": "MainTree"},
        ]
    )

    return LaunchDescription([
        map_points_arg,
        robot_config_arg,
        bt_engine_node,
    ])
