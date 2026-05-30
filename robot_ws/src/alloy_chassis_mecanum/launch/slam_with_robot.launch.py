from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution, Command
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Get URDF file path
    urdf_path = os.path.join(
        get_package_share_directory('alloy_chassis_mecanum'),
        'urdf',
        'alloy_chassis_mecanum.urdf')

    # Read URDF file
    with open(urdf_path, 'r') as f:
        robot_description_content = f.read()

    return LaunchDescription([
        # 1. Robot State Publisher - 发布机器人URDF
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{
                'robot_description': robot_description_content,
                'publish_frequency': 30.0
            }]
        ),

        # 2. Joint State Publisher - 发布关节状态
        Node(
            package='joint_state_publisher',
            executable='joint_state_publisher',
            name='joint_state_publisher',
            output='screen',
            parameters=[{
                'use_gui': False  # SLAM模式下不需要GUI
            }]
        ),

        # 3. YDLIDAR 雷达驱动
        Node(
            package='ydlidar_ros2_driver',
            executable='ydlidar_ros2_driver_node',
            name='ydlidar_ros2_driver_node',
            output='screen',
            emulate_tty=True,
            parameters=[PathJoinSubstitution([
                FindPackageShare('ydlidar_ros2_driver'),
                'params', 'X2.yaml'
            ])],
            namespace='/'
        ),

        # 4. 静态TF - base_link到laser_frame
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments=['0', '0', '0.1', '0', '0', '0', 'base_link', 'laser_frame'],
            name='base_link_to_laser_tf'
        ),

        # 5. SLAM Toolbox - SLAM建图
        Node(
            package='slam_toolbox',
            executable='async_slam_toolbox_node',
            name='slam_toolbox',
            parameters=[
                PathJoinSubstitution([
                    FindPackageShare('ydlidar_ros2_driver'),
                    'config', 'mapper_params_online_async.yaml'
                ]),
                {
                    'use_scan_matching': True,
                    'use_scan_barycenter': True,
                    'minimum_travel_distance': 0.1,
                    'minimum_travel_heading': 0.1,
                    'scan_buffer_size': 10,
                    'map_update_interval': 1.0,
                    'resolution': 0.05,
                    'max_laser_range': 8.0,
                    'minimum_time_interval': 0.25,
                    'transform_publish_period': 0.02,
                    'tf_buffer_duration': 30.0
                }
            ],
            remappings=[
                ('/scan', '/scan'),
                ('/tf', 'tf'),
                ('/tf_static', 'tf_static')
            ]
        ),

        # 6. RViz2 - 可视化
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', PathJoinSubstitution([
                FindPackageShare('alloy_chassis_mecanum'),
                'config', 'slam_with_robot.rviz'
            ])]
        )
    ])
