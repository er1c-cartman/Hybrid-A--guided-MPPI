import os
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, GroupAction,
                            IncludeLaunchDescription, SetEnvironmentVariable)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
# bringup_dir = get_package_share_directory('fdilink_ahrs')
# launch_dir = os.path.join(bringup_dir, 'launch')
# imu_tf = IncludeLaunchDescription(
#         PythonLaunchDescriptionSource(os.path.join(launch_dir, 'imu_tf.launch.py')),
# )
def generate_launch_description():
    ahrs_driver=Node(
        package="fdilink_ahrs",
        executable="ahrs_driver_node",
        parameters=[{'if_debug_': False,
            # modified by KimJH 'dev/fdilink_ahrs -> dev/ttyUSB2'
            #'serial_port_':'/dev/fdilink_ahrs',
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

    launch_description =LaunchDescription()
    launch_description.add_action(ahrs_driver)
#    launch_description.add_action(imu_tf)
    return launch_description
