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

    # Declare launch arguments
    robot_config_arg = DeclareLaunchArgument(
        'robot_config',
        default_value=os.path.join(src_dir, 'params', 'robot_config_default.yaml'),
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
