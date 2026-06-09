"""
机器人智能大脑启动: 摄像头 + 豆包 Vision Pro + 机械臂控制。
"""
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        # 摄像头
        Node(
            package="usb_cam", executable="usb_cam_node_exe",
            name="usb_cam", output="screen",
            parameters=[{
                "video_device": "/dev/video0",
                "image_width": 640, "image_height": 480,
                "framerate": 15.0
            }],
            condition=None  # 如果没有 usb_cam 会自动跳过
        ),
        # 智能大脑 (交互模式)
        Node(
            package="robot_brain", executable="brain_pipeline.py",
            name="brain_pipeline", output="screen",
            parameters=[{
                "api_key": "",
                "camera_id": 0,
                "serial_port": "/dev/ttyACM0"
            }]
        ),
    ])
