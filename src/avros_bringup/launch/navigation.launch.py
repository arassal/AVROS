"""Launch full autonomous navigation stack.

Launches:
  - Everything from localization.launch.py (sensors + EKF + navsat)
  - actuator_node (cmd_vel -> Teensy UDP)
  - route_server (nav2_route graph-based road planner — must start before Nav2)
  - Nav2 (planner, controller, costmaps, behavior tree, lifecycle manager)

Launch ordering: route_server must be active before bt_navigator activates,
because the BT XML contains a ComputeRoute node that validates the
compute_route action server at load time. On Humble, nav2_bringup does not
include route_server (Jazzy does), so we launch it separately with a
lifecycle manager and delay Nav2 bringup to ensure correct ordering.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    TimerAction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from nav2_common.launch import RewrittenYaml


def generate_launch_description():
    pkg_dir = get_package_share_directory('avros_bringup')
    actuator_config = os.path.join(pkg_dir, 'config', 'actuator_params.yaml')
    nav2_config = os.path.join(pkg_dir, 'config', 'nav2_params.yaml')
    bt_xml = os.path.join(pkg_dir, 'config', 'navigate_route_graph.xml')
    graph_file = os.path.join(pkg_dir, 'config', 'cpp_campus_graph.geojson')

    # Rewrite nav2_params.yaml with resolved BT XML and graph file paths
    configured_params = RewrittenYaml(
        source_file=nav2_config,
        param_rewrites={
            'default_nav_to_pose_bt_xml': bt_xml,
            'default_nav_through_poses_bt_xml': bt_xml,
            'graph_filepath': graph_file,
        },
        convert_types=True,
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time', default_value='false',
            description='Use simulation clock'
        ),

        DeclareLaunchArgument(
            'enable_ntrip', default_value='true',
            description='Enable NTRIP client for RTK corrections'
        ),

        # Include localization launch (sensors + EKF + navsat)
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_dir, 'launch', 'localization.launch.py')
            ),
            launch_arguments={
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                'enable_ntrip': LaunchConfiguration('enable_ntrip'),
            }.items(),
        ),

        # Actuator bridge
        Node(
            package='avros_control',
            executable='actuator_node',
            name='actuator_node',
            parameters=[
                actuator_config,
                {'use_sim_time': LaunchConfiguration('use_sim_time')},
            ],
            output='screen',
        ),

        # Route server (must be active before bt_navigator loads the BT XML)
        Node(
            package='nav2_route',
            executable='route_server',
            name='route_server',
            parameters=[
                configured_params,
                {'use_sim_time': LaunchConfiguration('use_sim_time')},
            ],
            output='screen',
        ),

        # Lifecycle manager for route_server (2s delay for DDS discovery)
        TimerAction(
            period=2.0,
            actions=[
                Node(
                    package='nav2_lifecycle_manager',
                    executable='lifecycle_manager',
                    name='lifecycle_manager_route',
                    parameters=[{
                        'autostart': True,
                        'node_names': ['route_server'],
                        'bond_timeout': 10.0,
                    }],
                    output='screen',
                ),
            ],
        ),

        # Nav2 bringup (delayed 8s so route_server is active first)
        # bt_navigator validates compute_route action server at load time
        TimerAction(
            period=8.0,
            actions=[
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(
                        os.path.join(
                            get_package_share_directory('nav2_bringup'),
                            'launch', 'navigation_launch.py'
                        )
                    ),
                    launch_arguments={
                        'use_sim_time': LaunchConfiguration('use_sim_time'),
                        'params_file': configured_params,
                        'autostart': 'true',
                    }.items(),
                ),
            ],
        ),
    ])
