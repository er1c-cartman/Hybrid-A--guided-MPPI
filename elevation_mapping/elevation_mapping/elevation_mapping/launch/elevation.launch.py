import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='False')

    # Path to elevation mapping config file
    elevation_mapping_config = os.path.join(
        get_package_share_directory('elevation_mapping'),
        'config',
        'elevation_mapping.yaml'
    )
    # Paths to the grid map configuration files
    config_folder = get_package_share_directory('elevation_mapping')
    grid_map_filters_config = os.path.join(config_folder, 'config', 'grid_map_filters.yaml')
    grid_map_visualization_config = os.path.join(config_folder, 'config', 'grid_map_visualization.yaml')
    grid_map_converter_config = os.path.join(config_folder, 'config', 'grid_map_converter.yaml')

    # Nodes for the launch file
    grid_map_filters_node = Node(
        package='grid_map_demos',
        executable='filters_demo',
        name='grid_map_filters',
        output='screen',
        parameters=[grid_map_filters_config]
    )

    grid_map_visualization_node = Node(
        package='grid_map_visualization',
        executable='grid_map_visualization',
        name='grid_map_visualization',
        output='screen',
        parameters=[grid_map_visualization_config]
    )

    grid_map_converter_node = Node(
        package='scout_navigation',
        executable='gridmap_converter_node',
        name='gridmap_converter',
        output='screen',
        parameters=[grid_map_converter_config, {'use_sim_time': use_sim_time}]
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='False',
            description='Use simulation (Gazebo) clock if true'
        ),
        grid_map_filters_node,
        grid_map_visualization_node,
        grid_map_converter_node,
        Node(
            package='elevation_mapping',
            executable='elevation_mapping_node',
            #name='elevation_mapping',
            output='screen',
            parameters=[elevation_mapping_config, {'use_sim_time': use_sim_time}]
        ),
        Node(
            package='elevation_mapping',
            executable='tf_to_pose_publisher.py',
            name='tf_to_pose_publisher',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}]
        ),
    ])

