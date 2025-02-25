import os
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, Command, PathJoinSubstitution, FindExecutable
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    use_sim_time_arg = DeclareLaunchArgument('use_sim_time', default_value='false',
                                             description='Use simulation clock if true')

    port_name_arg = DeclareLaunchArgument('port_name', default_value='can0',
                                          description='CAN bus name, e.g. can0')
    odom_frame_arg = DeclareLaunchArgument('odom_frame', default_value='odom',
                                           description='Odometry frame id')
    base_link_frame_arg = DeclareLaunchArgument('base_frame', default_value='base_link',
                                                description='Base link frame id')
    odom_topic_arg = DeclareLaunchArgument('odom_topic_name', default_value='odom',
                                           description='Odometry topic name')

    simulated_robot_arg = DeclareLaunchArgument('simulated_robot', default_value='false',
                                                description='Whether running with simulator')
    sim_control_rate_arg = DeclareLaunchArgument('control_rate', default_value='50',
                                                 description='Simulation control loop update rate')

    hunter_base_node = Node(
        package='hunter_base',
        executable='hunter_base_node',
        output='screen',
        emulate_tty=True,
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'port_name': LaunchConfiguration('port_name'),
            'odom_frame': LaunchConfiguration('odom_frame'),
            'base_frame': LaunchConfiguration('base_frame'),
            'odom_topic_name': LaunchConfiguration('odom_topic_name'),
            'simulated_robot': LaunchConfiguration('simulated_robot'),
            'control_rate': LaunchConfiguration('control_rate')
        }]
    )

    robot_description_content = Command([
        PathJoinSubstitution([FindExecutable(name="xacro")]), " ",
        PathJoinSubstitution(
            [FindPackageShare("hunter_description"), "urdf", "hunter_description.urdf"]
        ),
    ])

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'robot_description': ParameterValue(robot_description_content, value_type=str)
        }]
    )

    ahrs_driver_node = Node(
        package="fdilink_ahrs",
        executable="ahrs_driver_node",
        parameters=[{
            'if_debug_': False,
            'serial_port_': '/dev/ttyUSB0',
            'serial_baud_': 921600,
            'imu_topic': '/imu',
            'imu_frame_id_': 'gyro_link',
            'mag_pose_2d_topic': '/mag_pose_2d',
            'Magnetic_topic': '/magnetic',
            'Euler_angles_topic': '/euler_angles',
            'gps_topic': '/gps/fix',
            'twist_topic': '/system_speed',
            'NED_odom_topic': '/NED_odometry',
            'device_type_': 1
        }],
        output="screen"
    )

    return LaunchDescription([
        use_sim_time_arg,
        port_name_arg,
        odom_frame_arg,
        base_link_frame_arg,
        odom_topic_arg,
        simulated_robot_arg,
        sim_control_rate_arg,
        hunter_base_node,
        robot_state_publisher_node,
        ahrs_driver_node
    ])

