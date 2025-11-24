#!/usr/bin/env python3
"""
Navigation Launch File for Project Ogre

Launches complete autonomous navigation system:
- Localization: slam_toolbox (localization mode with saved map)
- Sensors: RPLIDAR A1 + RealSense D435 (pointcloud)
- Odometry: Wheel encoders + EKF fusion
- Navigation: Nav2 stack with waypoint navigation and obstacle avoidance
- Visualization: RViz with Nav2 panel

Usage:
    ros2 launch ogre_slam navigation.launch.py map:=path/to/map.yaml

Arguments:
    map: Path to saved map YAML file (REQUIRED)
    use_rviz: Launch RViz visualization (default: true)
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_localization_nodes(context, *args, **kwargs):
    """Generate map server and slam_toolbox localization nodes with expanded map path."""
    map_path = LaunchConfiguration('map').perform(context)
    expanded_path = os.path.expanduser(map_path)
    expanded_path = os.path.expandvars(expanded_path)

    # Extract map file name without extension for slam_toolbox
    # e.g., /home/jetson/ros2_ws/src/ogre-slam/maps/my_map.yaml -> my_map
    map_file_name = os.path.splitext(os.path.basename(expanded_path))[0]
    map_dir = os.path.dirname(expanded_path)

    # Get config directory
    ogre_slam_dir = get_package_share_directory('ogre_slam')
    slam_params_file = os.path.join(ogre_slam_dir, 'config', 'slam_toolbox_params.yaml')

    map_server_node = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[{
            'use_sim_time': False,
            'yaml_filename': expanded_path
        }],
        emulate_tty=True
    )

    # AMCL for localization (publishes map->odom transform)
    # Get config directory
    ogre_slam_dir = get_package_share_directory('ogre_slam')
    amcl_params_file = os.path.join(ogre_slam_dir, 'config', 'amcl_params.yaml')

    amcl_node = Node(
        package='nav2_amcl',
        executable='amcl',
        name='amcl',
        output='screen',
        parameters=[amcl_params_file],
        emulate_tty=True
    )

    return [map_server_node, amcl_node]


def generate_launch_description():
    # Get package directories
    ogre_slam_dir = get_package_share_directory('ogre_slam')
    rplidar_dir = get_package_share_directory('rplidar_ros')
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')

    # Configuration file paths
    nav2_params_file = os.path.join(ogre_slam_dir, 'config', 'nav2_params.yaml')
    ekf_params_file = os.path.join(ogre_slam_dir, 'config', 'ekf_params.yaml')
    slam_params_file = os.path.join(ogre_slam_dir, 'config', 'slam_toolbox_params.yaml')
    odometry_params_file = os.path.join(ogre_slam_dir, 'config', 'odometry_params.yaml')
    rviz_config_file = os.path.join(ogre_slam_dir, 'rviz', 'navigation.rviz')

    # Launch arguments
    map_arg = DeclareLaunchArgument(
        'map',
        default_value='',
        description='Path to saved map YAML file for localization (REQUIRED)'
    )

    use_rviz_arg = DeclareLaunchArgument(
        'use_rviz',
        default_value='true',
        description='Launch RViz for visualization'
    )

    rplidar_model_arg = DeclareLaunchArgument(
        'rplidar_model',
        default_value='a1',
        description='RPLIDAR model (a1, a2, a3, etc.)'
    )

    use_odometry_arg = DeclareLaunchArgument(
        'use_odometry',
        default_value='true',
        description='Launch encoder-based odometry node'
    )

    use_ekf_arg = DeclareLaunchArgument(
        'use_ekf',
        default_value='true',
        description='Use robot_localization EKF for sensor fusion'
    )

    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation time (true for Isaac Sim, false for real robot)'
    )

    # Launch configurations
    map_file = LaunchConfiguration('map')
    use_sim_time = LaunchConfiguration('use_sim_time')
    use_rviz = LaunchConfiguration('use_rviz')
    rplidar_model = LaunchConfiguration('rplidar_model')
    use_odometry = LaunchConfiguration('use_odometry')
    use_ekf = LaunchConfiguration('use_ekf')

    # 1. RPLIDAR launch (2D laser scanner)
    # NOTE: Only launch for real robot - Isaac Sim provides /scan directly
    rplidar_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('rplidar_ros'),
                'launch'
            ]),
            '/rplidar_',
            rplidar_model,
            '_launch.py'
        ]),
        condition=UnlessCondition(use_sim_time)  # Skip if using simulation time
    )

    # 2. RealSense D435 launch (3D depth camera with pointcloud)
    # NOTE: Disabled for Isaac Sim testing - real robot only
    # realsense_launch = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource(
    #         PathJoinSubstitution([
    #             FindPackageShare('realsense2_camera'),
    #             'launch',
    #             'rs_launch.py'
    #         ])
    #     ),
    #     launch_arguments={
    #         'camera_name': 'camera',
    #         'camera_namespace': 'camera',
    #         'enable_color': 'true',
    #         'enable_depth': 'true',
    #         'pointcloud.enable': 'true',  # CRITICAL: Enable pointcloud for Nav2
    #         'align_depth.enable': 'true',
    #         'decimation_filter.enable': 'false',  # Set true to reduce CPU load
    #     }.items()
    # )

    # 3. Static TF: base_link → camera_link
    # Measured position: camera is 15cm forward, 10cm up from robot center
    static_tf_camera = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_to_camera_broadcaster',
        arguments=['0.15', '0.0', '0.10', '0.0', '0.0', '0.0', 'base_link', 'camera_link']
    )

    # 4. Static TF: base_link → laser (RPLIDAR)
    # Mounted at front-center, 27cm up, 180° rotated
    static_tf_laser = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_to_laser_broadcaster',
        arguments=['0.0', '0.0', '0.27', '0.0', '0.0', '3.14159', 'base_link', 'laser']
    )

    # 5. Encoder-based odometry node (conditional)
    odometry_node = Node(
        package='ogre_slam',
        executable='odometry_node',
        name='odometry_node',
        output='screen',
        parameters=[odometry_params_file],
        emulate_tty=True,
        condition=IfCondition(use_odometry)
    )

    # 6. robot_localization EKF node (sensor fusion)
    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[ekf_params_file],
        remappings=[('odometry/filtered', '/odometry/filtered')],
        emulate_tty=True,
        condition=IfCondition(use_ekf)
    )

    # 7-8. Map server + slam_toolbox localization (loads saved map with path expansion)
    localization_launcher = OpaqueFunction(function=generate_localization_nodes)

    # 9. Nav2 Lifecycle Manager (activates Nav2 nodes)
    lifecycle_manager_navigation = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_navigation',
        output='screen',
        parameters=[{
            'use_sim_time': False,
            'autostart': True,
            'node_names': [
                'controller_server',
                'planner_server',
                'behavior_server',
                'bt_navigator',
                'waypoint_follower',
                'velocity_smoother'
            ]
        }],
        emulate_tty=True
    )

    # 10. Nav2 Lifecycle Manager for localization
    lifecycle_manager_localization = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_localization',
        output='screen',
        parameters=[{
            'use_sim_time': False,
            'autostart': True,
            'node_names': ['map_server', 'amcl']
        }],
        emulate_tty=True
    )

    # 11. Controller Server
    controller_server = Node(
        package='nav2_controller',
        executable='controller_server',
        name='controller_server',
        output='screen',
        parameters=[nav2_params_file],
        emulate_tty=True
    )

    # 12. Planner Server
    planner_server = Node(
        package='nav2_planner',
        executable='planner_server',
        name='planner_server',
        output='screen',
        parameters=[nav2_params_file],
        emulate_tty=True
    )

    # 13. Behavior Server
    behavior_server = Node(
        package='nav2_behaviors',
        executable='behavior_server',
        name='behavior_server',
        output='screen',
        parameters=[nav2_params_file],
        emulate_tty=True
    )

    # 14. BT Navigator
    bt_navigator = Node(
        package='nav2_bt_navigator',
        executable='bt_navigator',
        name='bt_navigator',
        output='screen',
        parameters=[nav2_params_file],
        emulate_tty=True
    )

    # 15. Waypoint Follower
    waypoint_follower = Node(
        package='nav2_waypoint_follower',
        executable='waypoint_follower',
        name='waypoint_follower',
        output='screen',
        parameters=[nav2_params_file],
        emulate_tty=True
    )

    # 16. Velocity Smoother
    velocity_smoother = Node(
        package='nav2_velocity_smoother',
        executable='velocity_smoother',
        name='velocity_smoother',
        output='screen',
        parameters=[nav2_params_file],
        emulate_tty=True
    )

    # 17. RViz with Nav2 panel (conditional)
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_file],
        output='screen',
        emulate_tty=True,
        condition=IfCondition(use_rviz)
    )

    return LaunchDescription([
        # Launch arguments
        map_arg,
        use_rviz_arg,
        rplidar_model_arg,
        use_odometry_arg,
        use_ekf_arg,
        use_sim_time_arg,

        # Static TF transforms
        static_tf_camera,
        static_tf_laser,

        # Sensor drivers
        rplidar_launch,
        # realsense_launch,  # Disabled for Isaac Sim

        # Odometry and localization
        odometry_node,
        ekf_node,

        # Map server + SLAM localization (combined)
        localization_launcher,
        lifecycle_manager_localization,

        # Nav2 stack
        controller_server,
        planner_server,
        behavior_server,
        bt_navigator,
        waypoint_follower,
        velocity_smoother,
        lifecycle_manager_navigation,

        # Visualization
        rviz_node
    ])
