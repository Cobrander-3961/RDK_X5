#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped, Quaternion
from tf2_ros import TransformBroadcaster
import math

class FakeOdomPublisher(Node):
    def __init__(self):
        super().__init__('fake_odom_publisher')

        # 创建里程计发布器
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)

        # 创建TF广播器
        self.tf_broadcaster = TransformBroadcaster(self)

        # 定时器，定期发布里程计数据
        self.timer = self.create_timer(0.1, self.publish_odom)

        self.get_logger().info('Fake Odometry Publisher started')

    def quaternion_from_euler(self, roll, pitch, yaw):
        """
        将欧拉角转换为四元数
        """
        cy = math.cos(yaw * 0.5)
        sy = math.sin(yaw * 0.5)
        cp = math.cos(pitch * 0.5)
        sp = math.sin(pitch * 0.5)
        cr = math.cos(roll * 0.5)
        sr = math.sin(roll * 0.5)

        q = Quaternion()
        q.w = cr * cp * cy + sr * sp * sy
        q.x = sr * cp * cy - cr * sp * sy
        q.y = cr * sp * cy + sr * cp * sy
        q.z = cr * cp * sy - sr * sp * cy

        return q

    def publish_odom(self):
        # 创建里程计消息
        odom = Odometry()
        odom.header.stamp = self.get_clock().now().to_msg()
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_link'

        # 设置位置为0
        odom.pose.pose.position.x = 0.0
        odom.pose.pose.position.y = 0.0
        odom.pose.pose.position.z = 0.0

        # 设置姿态为0，使用quaternion_from_euler函数
        odom.pose.pose.orientation = self.quaternion_from_euler(0.0, 0.0, 0.0)

        # 设置速度为0
        odom.twist.twist.linear.x = 0.0
        odom.twist.twist.linear.y = 0.0
        odom.twist.twist.linear.z = 0.0
        odom.twist.twist.angular.x = 0.0
        odom.twist.twist.angular.y = 0.0
        odom.twist.twist.angular.z = 0.0

        # 发布里程计消息
        self.odom_pub.publish(odom)

        # 创建并发布TF变换
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_link'

        # 设置变换为0
        t.transform.translation.x = 0.0
        t.transform.translation.y = 0.0
        t.transform.translation.z = 0.0
        # 使用quaternion_from_euler函数设置旋转
        t.transform.rotation = self.quaternion_from_euler(0.0, 0.0, 0.0)

        # 发布TF变换
        self.tf_broadcaster.sendTransform(t)

def main(args=None):
    rclpy.init(args=args)
    fake_odom_publisher = FakeOdomPublisher()
    rclpy.spin(fake_odom_publisher)
    fake_odom_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
