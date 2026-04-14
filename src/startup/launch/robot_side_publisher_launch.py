from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='front_side_publisher',
            arguments=['0', '0.22', '0', '0', '0', '0', 'base_footprint', 'front_side']
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='right_side_publisher',
            arguments=['0.22', '0', '0', '0', '0', '0', 'base_footprint', 'right_side']
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='back_side_publisher',
            arguments=['0', '-0.22', '0', '0', '0', '0', 'base_footprint', 'back_side']
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='left_side_publisher',
            arguments=['-0.22', '0', '0', '0', '0', '0', 'base_footprint', 'left_side']
        ),
    ])
