"""
抓取测试: 基于 MoveIt2 demo.launch + arm_fixed_grasp_test.
"""
from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_demo_launch
from launch import LaunchDescription
from launch.actions import TimerAction
from launch_ros.actions import Node


def generate_launch_description():
    moveit_config = MoveItConfigsBuilder(
        "arm", package_name="elite_test"
    ).to_moveit_configs()

    demo_ld = generate_demo_launch(moveit_config)

    grasp_test = TimerAction(
        period=8.0,
        actions=[
            Node(
                package='elite_test',
                executable='arm_fixed_grasp_test.py',
                name='arm_fixed_grasp_test',
                output='screen',
            )
        ],
    )

    entities = list(demo_ld.entities)
    entities.append(grasp_test)
    return LaunchDescription(entities)
