#!/usr/bin/env python3
"""发布 robot_description 到 topic (TransientLocal, JSP 需要)"""
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy
from std_msgs.msg import String

class RobotDescPublisher(Node):
    def __init__(self):
        super().__init__('robot_desc_pub')
        self.declare_parameter('robot_description', '')
        desc = self.get_parameter('robot_description').value
        # TransientLocal: 后订阅者也能收到最后一条消息
        qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.pub = self.create_publisher(String, '/robot_description', qos)
        self.pub.publish(String(data=desc))
        # 每5秒重发一次确保送达
        self.timer = self.create_timer(5.0, lambda: self.pub.publish(String(data=desc)))
        self.get_logger().info('robot_description published (TransientLocal)')

def main():
    rclpy.init()
    rclpy.spin(RobotDescPublisher())

if __name__ == '__main__':
    main()
