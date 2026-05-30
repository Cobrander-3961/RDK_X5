"""
机械臂视觉伺服控制 launch:
  启动 move_group + arm_motion_planner，
  接收 /target_pose 规划并执行机械臂运动。
"""
from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_move_group_launch
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    moveit_config = MoveItConfigsBuilder(
        "arm", package_name="elite_test"
    ).to_moveit_configs()

    move_group = generate_move_group_launch(moveit_config)

    motion_planner = Node(
        package='elite_test',
        executable='arm_motion_planner.py',
        name='arm_motion_planner',
        output='screen',
        parameters=[{
            'serial_port': '/dev/ttyACM0',
            'baudrate': 115200,
        }],
    )

    return LaunchDescription([
        move_group,
        motion_planner,
    ])
