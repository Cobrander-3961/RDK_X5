from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    return LaunchDescription([
        # 1. YDLIDAR 雷达驱动
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
            arguments=['0', '0', '0.1', '0', '0', '0', 'base_link', 'laser_frame'],
            name='base_link_to_laser_tf'
        ),

        # 3. RViz2 - 可视化
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
