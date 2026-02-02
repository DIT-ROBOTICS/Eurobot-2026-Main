from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Get package directories
    bt_core_pkg_dir = get_package_share_directory('bt_core')
    startup_pkg_dir = get_package_share_directory('startup')

    # Include startup launch
    startup_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(startup_pkg_dir, 'launch', 'startup_launch.py')
        )
    )

    # Include bt_engine launch
    bt_engine_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bt_core_pkg_dir, 'launch', 'bt_engine_launch.py')
        )
    )

    return LaunchDescription([
        startup_launch,
        bt_engine_launch,
    ])
