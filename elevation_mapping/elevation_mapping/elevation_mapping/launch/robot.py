import os
import launch
import launch_ros

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import FindExecutable, PathJoinSubstitution
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node


def generate_launch_description():
    model_name = 'fake_robot.xacro'
    robot_description_content = Command([
        PathJoinSubstitution([FindExecutable(name="xacro")]), " ",
        PathJoinSubstitution(
            [FindPackageShare("scout_description"), "urdf", model_name]
        ),
    ])


    ahrs_driver=Node(
        package="fdilink_ahrs",
        executable="ahrs_driver_node",
        parameters=[{'if_debug_': False,
            'serial_port_':'/dev/ttyUSB0',
            'serial_baud_':921600,
            'imu_topic':'/imu',
            'imu_frame_id_':'gyro_link',
            'mag_pose_2d_topic':'/mag_pose_2d',
            'Magnetic_topic':'/magnetic',
            'Euler_angles_topic':'/euler_angles',
            'gps_topic':'/gps/fix',
            'twist_topic':'/system_speed',
            'NED_odom_topic':'/NED_odometry',
            'device_type_':1}],
        output="screen"
    )
 
    pointcloud_filter_node = Node(
        package='pointcloud_filter',
        executable='pointcloud_filter_node',
        name='pointcloud_filter_node',
        parameters=['/home/home/ros2_ws/install/pointcloud_filter/share/pointcloud_filter/config/params.yaml'],
        output='screen'
    )
    
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='False',
            description='Use simulation clock if true'),

        launch.actions.LogInfo(msg='use_sim_time: '),
        launch.actions.LogInfo(msg=LaunchConfiguration('use_sim_time')),
        
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                'robot_description': robot_description_content
            }]
        ),
        Node(
            package='scout_description',  # This is your package name
            executable='fake_odometry_publisher.py',
            name='fake_odometry_publisher',
            output='screen'
        ),

        ahrs_driver,
        pointcloud_filter_node,
        Node(
            package='scout_description',
            executable='static_transform_publisher.py',
            name='static_transform_publisher',
            output='screen'
        )
    ])

