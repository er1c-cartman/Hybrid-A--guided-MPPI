import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    config = os.path.join(
        get_package_share_directory('pointcloud_filter'),
        'config',
        'custom_pointcloud_filter_config.yaml'
    )

    return LaunchDescription([
        Node(
            package='pointcloud_filter',
            executable='custom_pointcloud_filter_node',
            name='custom_pointcloud_filter_node',
            parameters=[config],
            output='screen'
        )
    ])

