from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch_ros.parameter_descriptions import ParameterValue
import os

def generate_launch_description():
    urdf_path = os.path.join(
        get_package_share_directory("arm_ros2"),
        "urdf",
        "arm.urdf")
    
    rviz_config_path = os.path.join(
        get_package_share_directory("arm_ros2"),
        "config",
        "display.rviz")
    
    with open(urdf_path, "r") as f:
        robot_description_content = f.read()
    
    print("URDF file path:", urdf_path)
    print("RViz config path:", rviz_config_path)
    
    return LaunchDescription([
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            name="robot_state_publisher",
            output="screen",
            parameters=[{
                "robot_description": ParameterValue(robot_description_content, value_type=str)
            }]
        ),
        Node(
            package="joint_state_publisher",
            executable="joint_state_publisher",
            name="joint_state_publisher",
            output="screen",
            parameters=[{"use_gui": True}]
        ),
        Node(
            package="rviz2",
            executable="rviz2",
            name="rviz2",
            output="screen"
        )
    ])
