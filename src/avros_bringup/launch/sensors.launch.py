"""Launch sensor drivers and robot_state_publisher.

Launches:
  - robot_state_publisher (URDF -> static TF)
  - velodyne driver + transform
  - realsense2_camera
  - xsens_mti_driver
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    pkg_dir = get_package_share_directory('avros_bringup')
    urdf_file = os.path.join(pkg_dir, 'urdf', 'avros.urdf.xacro')
    cyclonedds_file = os.path.join(pkg_dir, 'config', 'cyclonedds.xml')
    velodyne_config = os.path.join(pkg_dir, 'config', 'velodyne.yaml')
    realsense_config = os.path.join(pkg_dir, 'config', 'realsense.yaml')
    xsens_config = os.path.join(pkg_dir, 'config', 'xsens.yaml')
    ntrip_config = os.path.join(pkg_dir, 'config', 'ntrip_params.yaml')

    return LaunchDescription([
        # CycloneDDS shared memory
        SetEnvironmentVariable(
            name='RMW_IMPLEMENTATION',
            value='rmw_cyclonedds_cpp',
            condition=IfCondition(LaunchConfiguration('use_cyclonedds')),
        ),
        SetEnvironmentVariable(
            name='CYCLONEDDS_URI',
            value='file://' + cyclonedds_file,
            condition=IfCondition(LaunchConfiguration('use_cyclonedds')),
        ),

        DeclareLaunchArgument(
            'use_sim_time', default_value='false',
            description='Use simulation clock'
        ),

        DeclareLaunchArgument(
            'enable_ntrip', default_value='true',
            description='Enable NTRIP client for RTK corrections'
        ),

        DeclareLaunchArgument(
            'enable_velodyne', default_value='true',
            description='Enable Velodyne VLP-16 LiDAR'
        ),

        DeclareLaunchArgument(
            'enable_realsense', default_value='true',
            description='Enable RealSense D455 camera'
        ),

        DeclareLaunchArgument(
            'enable_xsens', default_value='true',
            description='Enable Xsens MTi IMU/GNSS driver'
        ),

        DeclareLaunchArgument(
            'use_cyclonedds', default_value='true',
            description='Force CycloneDDS RMW implementation'
        ),

        # robot_state_publisher: URDF -> static TF
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{
                'robot_description': ParameterValue(
                    Command(['xacro ', urdf_file]), value_type=str
                ),
                'use_sim_time': LaunchConfiguration('use_sim_time'),
            }],
            output='screen',
        ),

        # Velodyne VLP-16 driver
        Node(
            package='velodyne_driver',
            executable='velodyne_driver_node',
            name='velodyne_driver_node',
            parameters=[velodyne_config],
            output='screen',
            condition=IfCondition(LaunchConfiguration('enable_velodyne')),
        ),

        # Velodyne raw packets -> PointCloud2
        Node(
            package='velodyne_pointcloud',
            executable='velodyne_transform_node',
            name='velodyne_transform_node',
            parameters=[velodyne_config],
            output='screen',
            condition=IfCondition(LaunchConfiguration('enable_velodyne')),
        ),

        # RealSense D455 (built from source, RSUSB backend)
        Node(
            package='realsense2_camera',
            executable='realsense2_camera_node',
            name='camera',
            parameters=[realsense_config],
            output='screen',
            condition=IfCondition(LaunchConfiguration('enable_realsense')),
        ),

        # Xsens MTi-680G IMU/GNSS
        # Package: xsens_mti_ros2_driver (build from source)
        # git clone --recursive --branch ros2 \
        #   https://github.com/xsenssupport/Xsens_MTi_ROS_Driver_and_Ntrip_Client.git
        Node(
            package='xsens_mti_ros2_driver',
            executable='xsens_mti_node',
            name='xsens_mti_node',
            parameters=[xsens_config],
            output='screen',
            condition=IfCondition(LaunchConfiguration('enable_xsens')),
        ),

        # NTRIP client for RTK corrections (GPGGA -> caster -> RTCM3)
        # Package: ntrip (from Xsens_MTi_ROS_Driver_and_Ntrip_Client repo)
        Node(
            package='ntrip',
            executable='ntrip',
            name='ntrip_client',
            parameters=[ntrip_config],
            remappings=[
                ('nmea', '/nmea'),
                ('rtcm', '/rtcm'),
            ],
            output='screen',
            condition=IfCondition(LaunchConfiguration('enable_ntrip')),
        ),
    ])
