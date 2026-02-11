import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Get source directory (relative to this launch file)
    launch_dir = os.path.dirname(os.path.realpath(__file__))
    src_dir = os.path.dirname(launch_dir)  # startup package src dir
    ros_domain_id = os.getenv('ROS_DOMAIN_ID')

    # Declare launch arguments
    if ros_domain_id == '11':
        robot_config_name = 'robot_config_white.yaml'
        print('[Startup Launch]: ROS_DOMAIN_ID=11. Use robot_config_white.yaml ')
    elif ros_domain_id == '13':
        robot_config_name = 'robot_config_black.yaml'
        print('[Startup Launch]: ROS_DOMAIN_ID=13. Use robot_config_black.yaml ')
    else:
        robot_config_name = 'robot_config_default.yaml'
        print('[Startup Launch]: ROS_DOMAIN_ID=' + ros_domain_id + '. Use robot_config_default.yaml ')
    
    robot_config_arg = DeclareLaunchArgument(
        'robot_config',
        default_value=os.path.join(src_dir, 'params', robot_config_name),
        description='Robot configuration file'
    )

    # Launch configurations
    robot_config = LaunchConfiguration('robot_config')

    # Startup node
    startup_node = Node(
        package='startup',
        executable='startup_new',
        name='startup_node',
        output='screen',
        parameters=[robot_config]
    )

    return LaunchDescription([
        robot_config_arg,
        startup_node,
    ])
