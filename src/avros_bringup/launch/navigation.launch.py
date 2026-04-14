"""Launch full autonomous navigation stack.

Launches:
  - Everything from localization.launch.py (sensors + EKF + navsat)
  - actuator_node (cmd_vel -> Teensy UDP)
  - Nav2 servers (controller, smoother, planner, route_server, behavior,
    velocity_smoother, bt_navigator)
  - Single lifecycle manager (transitions servers in order)
  - foxglove_bridge (WebSocket server for remote visualization)

Nav2 servers are launched directly (not via nav2_bringup) so that
route_server can be included in the lifecycle manager's ordered node list.
The lifecycle manager transitions nodes sequentially, guaranteeing
route_server is active before bt_navigator loads the BT XML.

Cross-distro: ROS_DISTRO selects the correct nav2 params and BT XML.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from nav2_common.launch import RewrittenYaml


def generate_launch_description():
    pkg_dir = get_package_share_directory('avros_bringup')
    actuator_config = os.path.join(pkg_dir, 'config', 'actuator_params.yaml')
    graph_file = os.path.join(pkg_dir, 'config', 'cpp_campus_graph.geojson')
    perception_pkg_dir = get_package_share_directory('avros_perception')
    perception_launch = os.path.join(perception_pkg_dir, 'launch', 'perception.launch.py')

    # Select distro-specific config (params + BT XML)
    ros_distro = os.environ.get('ROS_DISTRO', 'humble')
    if ros_distro == 'humble':
        nav2_config = os.path.join(pkg_dir, 'config', 'nav2_params_humble.yaml')
        bt_xml = os.path.join(pkg_dir, 'config', 'navigate_route_graph_humble.xml')
    else:
        nav2_config = os.path.join(pkg_dir, 'config', 'nav2_params.yaml')
        bt_xml = os.path.join(pkg_dir, 'config', 'navigate_route_graph.xml')

    use_sim_time = LaunchConfiguration('use_sim_time')

    # Rewrite nav2 params with resolved paths and use_sim_time
    configured_params = RewrittenYaml(
        source_file=nav2_config,
        param_rewrites={
            'use_sim_time': use_sim_time,
            'default_nav_to_pose_bt_xml': bt_xml,
            'default_nav_through_poses_bt_xml': bt_xml,
            'graph_filepath': graph_file,
        },
        convert_types=True,
    )

    # Nav2 servers — lifecycle manager transitions these in order,
    # so route_server activates before bt_navigator validates the BT XML
    nav2_servers = [
        ('nav2_controller', 'controller_server'),
        ('nav2_smoother', 'smoother_server'),
        ('nav2_planner', 'planner_server'),
        ('nav2_route', 'route_server'),
        ('nav2_behaviors', 'behavior_server'),
        ('nav2_velocity_smoother', 'velocity_smoother'),
        ('nav2_bt_navigator', 'bt_navigator'),
    ]

    lifecycle_nodes = [name for _, name in nav2_servers]

    nav2_nodes = [
        Node(
            package=package,
            executable=name,
            name=name,
            parameters=[configured_params],
            output='screen',
            respawn=True,
            respawn_delay=2.0,
        )
        for package, name in nav2_servers
    ]

    return LaunchDescription([
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
            'enable_localization', default_value='true',
            description='Enable localization stack'
        ),

        DeclareLaunchArgument(
            'use_cyclonedds', default_value='true',
            description='Force CycloneDDS RMW implementation'
        ),

        DeclareLaunchArgument(
            'enable_mock_tf', default_value='false',
            description='Publish static map->odom and odom->base_link transforms for hardware-free testing'
        ),

        DeclareLaunchArgument(
            'enable_perception', default_value='true',
            description='Enable road segmentation and obstacle bridge'
        ),

        DeclareLaunchArgument(
            'enable_foxglove', default_value='true',
            description='Enable Foxglove WebSocket bridge'
        ),

        DeclareLaunchArgument(
            'perception_venv_python',
            default_value='/home/alexander/github/av-perception/.venv/bin/python',
            description='Python interpreter for the segmentation runtime'
        ),

        DeclareLaunchArgument(
            'perception_project_root',
            default_value='/home/alexander/Desktop/seg',
            description='Path to the segmentation project root'
        ),

        DeclareLaunchArgument(
            'perception_image_dir',
            default_value='/home/alexander/Desktop/seg/img',
            description='Image folder for the demo segmentation stream'
        ),

        DeclareLaunchArgument(
            'perception_weights_path',
            default_value='/home/alexander/Desktop/seg/data/weights/yolopv2.pt',
            description='TorchScript YOLOPv2 weights path'
        ),

        DeclareLaunchArgument(
            'perception_device', default_value='cpu',
            description='Inference device'
        ),

        DeclareLaunchArgument(
            'perception_publish_rate_hz', default_value='1.0',
            description='Segmentation publish rate'
        ),

        # Localization (sensors + EKF + navsat)
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_dir, 'launch', 'localization.launch.py')
            ),
            condition=IfCondition(LaunchConfiguration('enable_localization')),
            launch_arguments={
                'use_sim_time': use_sim_time,
                'enable_ntrip': LaunchConfiguration('enable_ntrip'),
                'enable_velodyne': LaunchConfiguration('enable_velodyne'),
                'enable_realsense': LaunchConfiguration('enable_realsense'),
                'enable_xsens': LaunchConfiguration('enable_xsens'),
                'use_cyclonedds': LaunchConfiguration('use_cyclonedds'),
            }.items(),
        ),

        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='map_to_odom_tf',
            arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom'],
            output='screen',
            condition=IfCondition(LaunchConfiguration('enable_mock_tf')),
        ),

        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='odom_to_base_link_tf',
            arguments=['0', '0', '0', '0', '0', '0', 'odom', 'base_link'],
            output='screen',
            condition=IfCondition(LaunchConfiguration('enable_mock_tf')),
        ),

        # Actuator bridge
        Node(
            package='avros_control',
            executable='actuator_node',
            name='actuator_node',
            parameters=[
                actuator_config,
                {'use_sim_time': use_sim_time},
            ],
            output='screen',
        ),

        # Nav2 servers
        *nav2_nodes,

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(perception_launch),
            condition=IfCondition(LaunchConfiguration('enable_perception')),
            launch_arguments={
                'venv_python': LaunchConfiguration('perception_venv_python'),
                'project_root': LaunchConfiguration('perception_project_root'),
                'image_dir': LaunchConfiguration('perception_image_dir'),
                'weights_path': LaunchConfiguration('perception_weights_path'),
                'device': LaunchConfiguration('perception_device'),
                'publish_rate_hz': LaunchConfiguration('perception_publish_rate_hz'),
            }.items(),
        ),

        # Lifecycle manager — transitions nodes in list order
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_navigation',
            parameters=[{
                'autostart': True,
                'node_names': lifecycle_nodes,
            }],
            output='screen',
        ),

        # Foxglove bridge (WebSocket on port 8765 for remote visualization)
        Node(
            package='foxglove_bridge',
            executable='foxglove_bridge',
            name='foxglove_bridge',
            parameters=[{
                'port': 8765,
                'use_sim_time': use_sim_time,
            }],
            output='screen',
            condition=IfCondition(LaunchConfiguration('enable_foxglove')),
        ),
    ])
