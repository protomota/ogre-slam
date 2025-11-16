#!/usr/bin/env python3
"""
Mapping Launch File for ogre-slam
Launches all nodes required for SLAM mapping:
- RPLIDAR
- Odometry node
- robot_localization EKF
- slam_toolbox (mapping mode)
- RViz (visualization)
- ogre_teleop (robot control via web interface)
"""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Get package directories
    ogre_slam_dir = get_package_share_directory('ogre_slam')
    rplidar_dir = get_package_share_directory('rplidar_ros')
    ogre_teleop_dir = get_package_share_directory('ogre_teleop')

    # Configuration file paths
    odometry_params_file = os.path.join(ogre_slam_dir, 'config', 'odometry_params.yaml')
    ekf_params_file = os.path.join(ogre_slam_dir, 'config', 'ekf_params.yaml')
    slam_params_file = os.path.join(ogre_slam_dir, 'config', 'slam_toolbox_params.yaml')
    rviz_config_file = os.path.join(ogre_slam_dir, 'rviz', 'mapping.rviz')

    # Launch arguments
    rplidar_model_arg = DeclareLaunchArgument(
        'rplidar_model',
        default_value='a1',
        description='RPLIDAR model (a1, a2m7, a2m8, a2m12, a3, s1, s2, s3, t1, c1)'
    )

    use_rviz_arg = DeclareLaunchArgument(
        'use_rviz',
        default_value='true',
        description='Launch RViz for visualization'
    )

    use_ekf_arg = DeclareLaunchArgument(
        'use_ekf',
        default_value='true',
        description='Use robot_localization EKF for sensor fusion'
    )

    use_teleop_arg = DeclareLaunchArgument(
        'use_teleop',
        default_value='true',
        description='Launch ogre_teleop for robot control'
    )

    rplidar_model = LaunchConfiguration('rplidar_model')
    use_rviz = LaunchConfiguration('use_rviz')
    use_ekf = LaunchConfiguration('use_ekf')
    use_teleop = LaunchConfiguration('use_teleop')

    # 1. RPLIDAR launch
    rplidar_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(rplidar_dir, 'launch'),
            '/rplidar_', rplidar_model, '_launch.py'
        ])
    )

    # 2. Odometry node
    odometry_node = Node(
        package='ogre_slam',
        executable='odometry_node',
        name='odometry_node',
        output='screen',
        parameters=[odometry_params_file],
        emulate_tty=True
    )

    # 3. robot_localization EKF node (sensor fusion)
    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[ekf_params_file],
        remappings=[
            ('odometry/filtered', '/odometry/filtered')
        ],
        condition=IfCondition(use_ekf)
    )

    # 4. slam_toolbox async node (mapping mode)
    slam_toolbox_node = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[slam_params_file],
        emulate_tty=True
    )

    # 5. Map saver server (for saving maps)
    map_saver_server = Node(
        package='nav2_map_server',
        executable='map_saver_server',
        name='map_saver_server',
        output='screen',
        emulate_tty=True
    )

    # 6. Lifecycle manager for SLAM nodes
    lifecycle_manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_slam',
        output='screen',
        parameters=[{
            'autostart': True,
            'node_names': ['slam_toolbox', 'map_saver_server']
        }],
        emulate_tty=True
    )

    # 7. Static transform: base_link → laser (LIDAR position on robot)
    # LIDAR is mounted at center of robot, 270mm above ground
    static_tf_laser = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_to_laser_broadcaster',
        arguments=['0.0', '0.0', '0.27', '0.0', '0.0', '0.0', 'base_link', 'laser']
    )

    # 8. RViz
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_file],
        condition=IfCondition(use_rviz),
        parameters=[{
            'use_sim_time': False
        }]
    )

    # 9. ogre_teleop launch (motor control + camera + web interface)
    teleop_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ogre_teleop_dir, 'launch', 'web_teleop.launch.py')
        ),
        condition=IfCondition(use_teleop)
    )

    return LaunchDescription([
        # Launch arguments
        rplidar_model_arg,
        use_rviz_arg,
        use_ekf_arg,
        use_teleop_arg,

        # Nodes
        rplidar_launch,
        odometry_node,
        ekf_node,
        static_tf_laser,
        slam_toolbox_node,
        map_saver_server,
        lifecycle_manager,
        rviz_node,
        teleop_launch
    ])
