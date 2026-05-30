from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
from launch.actions import DeclareLaunchArgument
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
        # 声明启动参数
        DeclareLaunchArgument(
            'serial_port',
            default_value='/dev/ttyACM0',
            description='Serial port for F407 communication'
        ),
        DeclareLaunchArgument(
            'baudrate',
            default_value='115200',
            description='Baudrate for serial communication'
        ),

        # Robot State Publisher - 发布机器人URDF
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

        # Joint State Publisher - 发布关节状态
        Node(
            package='joint_state_publisher',
            executable='joint_state_publisher',
            name='joint_state_publisher',
            output='screen',
            parameters=[{
                'use_gui': False  # 测试模式下不需要GUI
            }]
        ),

        # 静态TF - base_link到laser_frame
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments=['0', '0', '0', '0', '0', '0', 'base_link', 'laser_frame'],
            name='base_link_to_laser_tf'
        ),

        # 串口里程计接收器 - 从F407接收里程计数据并发布到/odom话题
        Node(
            package='alloy_chassis_mecanum',
            executable='serial_odom_receiver',
            name='serial_odom_receiver',
            output='screen',
            parameters=[{
                'serial_port': LaunchConfiguration('serial_port'),
                'baudrate': LaunchConfiguration('baudrate')
            }]
        ),

        # RViz2 - 可视化
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', PathJoinSubstitution([
                FindPackageShare('alloy_chassis_mecanum'),
                'config', 'display.rviz'
            ])]
        )
    ])
