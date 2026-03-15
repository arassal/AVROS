"""Launch full autonomous navigation stack in Webots simulation.

Includes:
  - Base sim.launch.py (Webots + vehicle + robot_state_publisher)
  - Dual EKF (robot_localization) — local odom + global map
  - navsat_transform — GPS to Cartesian odometry
  - Nav2 servers launched directly (with route_server + lifecycle manager)

Does NOT start: actuator_node, velodyne, realsense, xsens, ntrip, foxglove
(Webots provides all sensor data; Car physics handles actuation)
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from nav2_common.launch import RewrittenYaml


def generate_launch_description():
    sim_pkg = get_package_share_directory('avros_sim')
    bringup_pkg = get_package_share_directory('avros_bringup')

    ekf_config = os.path.join(bringup_pkg, 'config', 'ekf.yaml')
    navsat_config = os.path.join(bringup_pkg, 'config', 'navsat.yaml')
    nav2_config = os.path.join(bringup_pkg, 'config', 'nav2_params.yaml')
    bt_xml = os.path.join(bringup_pkg, 'config', 'navigate_route_graph.xml')
    graph_file = os.path.join(bringup_pkg, 'config', 'cpp_campus_graph.geojson')

    # Rewrite nav2_params.yaml with resolved paths and use_sim_time
    configured_params = RewrittenYaml(
        source_file=nav2_config,
        param_rewrites={
            'use_sim_time': 'true',
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

    # Sim overrides: Jazzy bt_navigator (no plugin_lib_names), slower speed,
    # no collision detection, no camera_depth, larger costmap.
    # Loaded as a second parameter file — values override nav2_params.yaml.
    sim_overrides = os.path.join(sim_pkg, 'config', 'nav2_sim_overrides.yaml')
    sim_override_params = RewrittenYaml(
        source_file=sim_overrides,
        param_rewrites={
            'use_sim_time': 'true',
            'default_nav_to_pose_bt_xml': bt_xml,
            'default_nav_through_poses_bt_xml': bt_xml,
        },
        convert_types=True,
    )

    # Remap Webots LiDAR topic: webots_ros2_driver appends /point_cloud
    # suffix to the Lidar topicName, so /velodyne_points -> /velodyne_points/point_cloud.
    # Costmaps and Nav2 expect /velodyne_points.
    lidar_remap = ('/velodyne_points', '/velodyne_points/point_cloud')

    nav2_nodes = []
    for package, name in nav2_servers:
        # Base params from nav2_params.yaml + sim overrides on top
        # bt_navigator uses ONLY sim overrides (no plugin_lib_names for Jazzy)
        if name == 'bt_navigator':
            params = [sim_override_params]
        else:
            params = [configured_params, sim_override_params]
        remaps = []
        # Costmap nodes (inside controller_server and planner_server)
        # subscribe to /velodyne_points — remap to Webots topic
        if name in ('controller_server', 'planner_server'):
            remaps.append(lidar_remap)
        nav2_nodes.append(Node(
            package=package,
            executable=name,
            name=name,
            parameters=params,
            remappings=remaps,
            output='screen',
        ))

    return LaunchDescription([
        # Base simulation (Webots + vehicle + TF)
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(sim_pkg, 'launch', 'sim.launch.py')
            ),
        ),

        # EKF #1: Local odometry (odom -> base_link)
        # Fuses IMU only for smooth local navigation
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node_odom',
            parameters=[
                ekf_config,
                {'use_sim_time': True},
            ],
            remappings=[
                ('odometry/filtered', '/odometry/filtered'),
            ],
            output='screen',
        ),

        # EKF #2: Global map (map -> odom)
        # Fuses IMU + GPS for GPS-anchored global position
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node_map',
            parameters=[
                ekf_config,
                {'use_sim_time': True},
            ],
            remappings=[
                ('odometry/filtered', '/odometry/global'),
            ],
            output='screen',
        ),

        # navsat_transform_node: GPS -> Cartesian odometry
        Node(
            package='robot_localization',
            executable='navsat_transform_node',
            name='navsat_transform',
            parameters=[
                navsat_config,
                {'use_sim_time': True},
            ],
            remappings=[
                ('imu/data', '/imu/data'),
                ('gps/fix', '/gnss'),
                ('gps/filtered', '/gps/filtered'),
                ('odometry/gps', '/odometry/gps'),
                ('odometry/filtered', '/odometry/global'),
            ],
            output='screen',
        ),

        # Nav2 servers (start immediately, they wait for lifecycle transitions)
        *nav2_nodes,

        # Delay lifecycle manager to allow Webots world to fully load,
        # clock to start, and EKF to begin publishing TF.
        TimerAction(
            period=20.0,
            actions=[
                Node(
                    package='nav2_lifecycle_manager',
                    executable='lifecycle_manager',
                    name='lifecycle_manager_navigation',
                    parameters=[{
                        'use_sim_time': True,
                        'autostart': True,
                        'node_names': lifecycle_nodes,
                    }],
                    output='screen',
                ),
            ],
        ),
    ])
