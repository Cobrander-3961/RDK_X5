from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource
import os


def generate_launch_description():
    # Get package directories
    pkg_arm_ros2 = FindPackageShare('arm_ros2')
    pkg_gazebo_ros = FindPackageShare('gazebo_ros')

    # Path to URDF file
    urdf_file = PathJoinSubstitution([pkg_arm_ros2, 'urdf', 'arm.urdf'])

    # Gazebo launch file
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([pkg_gazebo_ros, 'launch', 'gazebo.launch.py'])
        ),
        launch_arguments={'world': os.path.join(pkg_gazebo_ros, 'worlds', 'empty.world')}.items()
    )

    return LaunchDescription([
        # Launch Gazebo
        gazebo_launch,

        # Static transform publisher for base_link to base_footprint
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='tf_footprint_base',
            arguments=['0', '0', '0', '0', '0', '0', 'base_link', 'base_footprint']
        ),

        # Spawn robot in Gazebo
        Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            name='spawn_model',
            arguments=[
                '-file', urdf_file,
                '-urdf',
                '-model', 'arm'
            ],
            output='screen'
        ),

        # Publish calibration status
        Node(
            package='demo_nodes_cpp',
            executable='talker',
            name='fake_joint_calibration',
            parameters=[{'message': 'calibrated'}]
        )
    ])
