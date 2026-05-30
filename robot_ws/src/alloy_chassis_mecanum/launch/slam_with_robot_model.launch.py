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
        # 1. YDLIDAR 雷达（使用官方X2配置文件）
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
        # 2. 静态TF - base_link到laser_frame
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments=['0', '0', '0', '0', '0', '0', 'base_link', 'laser_frame']
        ),
        # 3. 虚拟里程计节点（用于没有实际里程计的情况）
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments=['0', '0', '0', '0', '0', '0', 'odom', 'base_link']
        ),
        # 4. SLAM Toolbox
        Node(
            package='slam_toolbox',
            executable='async_slam_toolbox_node',
            parameters=[
                PathJoinSubstitution([
                    FindPackageShare('ydlidar_ros2_driver'),
                    'config', 'mapper_params_online_async.yaml'
                ])
            ],
            remappings=[  # 确保话题正确
                ('/scan', '/scan'),
                ('/tf', 'tf'),
                ('/tf_static', 'tf_static')
            ]
        ),
        # 5. Robot State Publisher - 发布机器人URDF
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
        # 6. Joint State Publisher - 发布关节状态
        Node(
            package='joint_state_publisher',
            executable='joint_state_publisher',
            name='joint_state_publisher',
            output='screen',
            parameters=[{
                'use_gui': False
            }]
        ),
    ])
