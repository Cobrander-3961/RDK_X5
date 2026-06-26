"""
果园巡检 — launch 文件。

导航按行巡检 → YOLO 检测苹果 → HSV 质量分析 → 云端上传。

启动:
  # 完整模式
  ros2 launch robot_brain orchard_patrol.launch.py api_key:=YOUR_KEY

  # 定点扫描 (无导航, 如未启动 robot_integrated)
  ros2 launch robot_brain orchard_patrol.launch.py nav_enabled:=false

  # 无头模式 + 无上传
  ros2 launch robot_brain orchard_patrol.launch.py headless:=true api_key:=""
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    api_key_arg = DeclareLaunchArgument(
        'api_key', default_value='',
        description='火山引擎 API Key (留空=不上传云端)'
    )
    cloud_endpoint_arg = DeclareLaunchArgument(
        'cloud_endpoint', default_value='',
        description='云端数据上传端点 URL'
    )
    nav_enabled_arg = DeclareLaunchArgument(
        'nav_enabled', default_value='true',
        description='是否启用导航 (false=定点扫描模式)'
    )
    headless_arg = DeclareLaunchArgument(
        'headless', default_value='false',
        description='无 GUI 模式'
    )
    orchard_rows_arg = DeclareLaunchArgument(
        'orchard_rows', default_value='3',
        description='果树行数'
    )
    row_spacing_arg = DeclareLaunchArgument(
        'row_spacing', default_value='1.5',
        description='行间距 (米)'
    )
    row_length_arg = DeclareLaunchArgument(
        'row_length', default_value='5.0',
        description='每行长度 (米)'
    )
    scan_duration_arg = DeclareLaunchArgument(
        'scan_duration_per_m', default_value='3.0',
        description='每米扫描时长 (秒), 越大越精细'
    )
    conf_arg = DeclareLaunchArgument(
        'confidence_threshold', default_value='0.2',
        description='YOLO 置信度阈值 (苹果检测建议 0.2)'
    )
    model_path_arg = DeclareLaunchArgument(
        'model_path', default_value='/home/sunrise/RDK_X5/yolov8n.pt',
        description='YOLO 模型路径'
    )

    orchard_node = Node(
        package='robot_brain',
        executable='orchard_patrol.py',
        name='orchard_patrol',
        output='screen',
        parameters=[{
            'api_key': LaunchConfiguration('api_key'),
            'cloud_endpoint': LaunchConfiguration('cloud_endpoint'),
            'nav_enabled': LaunchConfiguration('nav_enabled'),
            'headless': LaunchConfiguration('headless'),
            'orchard_rows': LaunchConfiguration('orchard_rows'),
            'row_spacing': LaunchConfiguration('row_spacing'),
            'row_length': LaunchConfiguration('row_length'),
            'scan_duration_per_m': LaunchConfiguration('scan_duration_per_m'),
            'confidence_threshold': LaunchConfiguration('confidence_threshold'),
            'model_path': LaunchConfiguration('model_path'),
        }],
    )

    return LaunchDescription([
        api_key_arg, cloud_endpoint_arg, nav_enabled_arg, headless_arg,
        orchard_rows_arg, row_spacing_arg, row_length_arg, scan_duration_arg,
        conf_arg, model_path_arg,
        orchard_node,
    ])
