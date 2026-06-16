"""
一体化巡检抓取闭环 — launch 文件。

启动后自动执行: 导航航点 → YOLO 检测 → 自动抓取 → 下一站。

使用方式:
  # 完整闭环 (抓取启用, GUI 显示)
  ros2 launch robot_brain integrated_patrol.launch.py

  # 半集成 (只导航+检测, 不抓取)
  ros2 launch robot_brain integrated_patrol.launch.py grasp_enabled:=false

  # 无头模式 (无 GUI)
  ros2 launch robot_brain integrated_patrol.launch.py headless:=true

  # 循环巡逻
  ros2 launch robot_brain integrated_patrol.launch.py loop:=true

依赖: 需先启动 robot_integrated.launch.py (Nav2 + SLAM + 机械臂)
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # 航点参数 (JSON 二维数组字符串)
    waypoints_arg = DeclareLaunchArgument(
        'waypoints', default_value='[[0.5, 0.5, 0.0], [1.0, 0.0, 1.57], [0.5, -0.5, 3.14], [0.0, 0.0, 0.0]]',
        description='航点列表 (map坐标系): [[x,y,yaw], ...]'
    )
    detection_timeout_arg = DeclareLaunchArgument(
        'detection_timeout_sec', default_value='30.0',
        description='每站检测超时 (秒)'
    )
    grasp_enabled_arg = DeclareLaunchArgument(
        'grasp_enabled', default_value='true',
        description='是否执行抓取 (false=只检测不抓)'
    )
    headless_arg = DeclareLaunchArgument(
        'headless', default_value='false',
        description='无 GUI 模式 (无 cv2.imshow)'
    )
    loop_arg = DeclareLaunchArgument(
        'loop', default_value='false',
        description='是否循环巡逻 (true=无限循环)'
    )
    model_path_arg = DeclareLaunchArgument(
        'model_path', default_value='/home/cobrander/RDK_X5/yolov8n.pt',
        description='YOLO 模型路径'
    )
    serial_port_arg = DeclareLaunchArgument(
        'serial_port', default_value='/dev/ttyACM0',
        description='机械臂串口路径'
    )

    integrated_node = Node(
        package='robot_brain',
        executable='integrated_patrol.py',
        name='integrated_patrol',
        output='screen',
        parameters=[{
            'waypoints': LaunchConfiguration('waypoints'),
            'detection_timeout_sec': LaunchConfiguration('detection_timeout_sec'),
            'grasp_enabled': LaunchConfiguration('grasp_enabled'),
            'headless': LaunchConfiguration('headless'),
            'loop': LaunchConfiguration('loop'),
            'model_path': LaunchConfiguration('model_path'),
            'serial_port': LaunchConfiguration('serial_port'),
        }],
    )

    return LaunchDescription([
        waypoints_arg,
        detection_timeout_arg,
        grasp_enabled_arg,
        headless_arg,
        loop_arg,
        model_path_arg,
        serial_port_arg,
        integrated_node,
    ])
