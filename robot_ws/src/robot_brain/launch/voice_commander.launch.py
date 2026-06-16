"""
语音指挥官 — launch 文件。

语音 → 豆包 Vision Pro 视觉决策 → 导航/检测/抓取 → TTS 反馈。

启动:
  ros2 launch robot_brain voice_commander.launch.py api_key:=YOUR_KEY

  无 API key (关键词 fallback):
  ros2 launch robot_brain voice_commander.launch.py

依赖: 需先启动 robot_integrated.launch.py
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    api_key_arg = DeclareLaunchArgument(
        'api_key', default_value='',
        description='火山引擎豆包 API Key (留空=关键词 fallback 模式)'
    )
    serial_port_arg = DeclareLaunchArgument(
        'serial_port', default_value='/dev/ttyACM0',
        description='机械臂串口路径'
    )
    headless_arg = DeclareLaunchArgument(
        'headless', default_value='false',
        description='无 GUI 模式'
    )
    model_path_arg = DeclareLaunchArgument(
        'model_path', default_value='/home/cobrander/RDK_X5/yolov8n.pt',
        description='YOLO 模型路径'
    )

    voice_node = Node(
        package='robot_brain',
        executable='voice_commander.py',
        name='voice_commander',
        output='screen',
        parameters=[{
            'api_key': LaunchConfiguration('api_key'),
            'serial_port': LaunchConfiguration('serial_port'),
            'headless': LaunchConfiguration('headless'),
            'model_path': LaunchConfiguration('model_path'),
            'named_locations': [
                ['home',   0.0, 0.0, 0.0],
                ['table',  0.5, 0.5, 0.0],
                ['center', 0.0, 0.0, 0.0],
                ['door',   1.0, 0.0, 0.0],
            ],
        }],
    )

    return LaunchDescription([
        api_key_arg,
        serial_port_arg,
        headless_arg,
        model_path_arg,
        voice_node,
    ])
