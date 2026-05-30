from launch import LaunchDescription
from launch_ros.actions import Node
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

    print("="*50)
    print("URDF file path:", urdf_path)
    print("URDF content length:", len(robot_description_content))
    print("="*50)

    return LaunchDescription([
        # Robot State Publisher
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
        # Joint State Publisher
        Node(
            package='joint_state_publisher',
            executable='joint_state_publisher',
            name='joint_state_publisher',
            output='screen',
            parameters=[{
                'use_gui': True
            }]
        ),
        # RViz (without config file)
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen'
        )
    ])
