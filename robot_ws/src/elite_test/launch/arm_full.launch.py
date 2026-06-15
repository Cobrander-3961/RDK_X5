"""机械臂一键启动: RViz + MoveGroup + 串口桥接"""
from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_demo_launch
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    moveit_config = MoveItConfigsBuilder(
        "arm", package_name="elite_test"
    ).to_moveit_configs()

    demo_ld = generate_demo_launch(moveit_config)

    import os, glob
    stm32 = '/dev/ttyACM0'
    for p in glob.glob('/dev/ttyACM*'):
        stm32 = p; break  # 用第一个找到的 ACM 设备
    arm_bridge = Node(
        package='elite_test',
        executable='arm_bridge.py',
        name='arm_bridge',
        output='screen',
        parameters=[{'serial_port': stm32}],
    )

    entities = list(demo_ld.entities)
    entities.append(arm_bridge)
    return LaunchDescription(entities)
