#!/usr/bin/env python3
"""
双向串口桥接节点：上行接收 STM32 里程计/TF 数据，下行发送 /cmd_vel 速度指令。

上行协议帧: [0xAA][type:1B][length:2B LE][data][0x55]
  - type 0x01: 里程计 (6×double, 48B)
  - type 0x02: TF     (3×double, 24B)

下行协议帧: [0xAA][0x10][0x08 0x00][linear_x:float LE][angular_z:float LE][0x55]
"""
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped, Quaternion, Twist
from tf2_ros import TransformBroadcaster
import serial
import struct
import math
import threading

class SerialOdomReceiver(Node):
    def __init__(self):
        super().__init__('serial_odom_receiver')

        # 串口配置
        self.declare_parameter('serial_port', '/dev/ttyUSB0')
        self.declare_parameter('baudrate', 115200)

        serial_port = self.get_parameter('serial_port').value
        baudrate = self.get_parameter('baudrate').value

        # 打印参数值
        self.get_logger().info(f'Serial port parameter: {serial_port}')
        self.get_logger().info(f'Baudrate parameter: {baudrate}')

        # 初始化串口
        try:
            self.ser = serial.Serial(serial_port, baudrate, timeout=1)
            self.get_logger().info(f'Serial port {serial_port} opened at {baudrate} baud')
        except serial.SerialException as e:
            self.get_logger().error(f'Failed to open serial port: {e}')
            return

        # 串口写锁，防止多线程并发写入
        self.write_lock = threading.Lock()

        # 下行命令: 订阅 /cmd_vel 并下发到 STM32
        self.declare_parameter('watchdog_timeout', 0.5)
        self.watchdog_timeout = self.get_parameter('watchdog_timeout').value
        self.last_cmd_time = self.get_clock().now()
        self.cmd_vel_sub = self.create_subscription(
            Twist, '/cmd_vel', self.cmd_vel_callback, 10)

        # 创建里程计发布器
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)

        # 创建TF广播器
        self.tf_broadcaster = TransformBroadcaster(self)

        # 帧定义
        self.frame_start = 0xAA
        self.frame_end = 0x55

        # 位置保持过滤 (防止编码器噪声导致漂移)
        self.last_x = 0.0
        self.last_y = 0.0
        self.last_yaw = 0.0
        self.pos_hold = True  # 首次收到数据时初始化

        # 接收缓冲区
        self.buffer = bytearray()

        # 定时器，定期处理串口数据
        self.timer = self.create_timer(0.01, self.process_serial_data)

        # 看门狗定时器: 超时自动发送零速指令
        self.watchdog_timer = self.create_timer(0.05, self.watchdog_check)

        self.get_logger().info('Serial Odometry Receiver started')

    def send_frame(self, frame_type, data):
        """
        构建协议帧并通过串口发送。

        Args:
            frame_type: 帧类型 (0x10 = 速度指令)
            data:      载荷字节 (bytes)
        """
        frame = bytearray()
        frame.append(self.frame_start)             # 帧头
        frame.append(frame_type)                    # 类型
        frame.extend(struct.pack('<H', len(data))) # 长度 (LE)
        frame.extend(data)                          # 载荷
        frame.append(self.frame_end)                # 帧尾

        with self.write_lock:
            self.ser.write(frame)

    def cmd_vel_callback(self, msg):
        """
        接收 /cmd_vel 并下发速度指令到 STM32。

        Nav2 controller_server 发布 Twist 消息，linear.x 为线速度 (m/s)，
        angular.z 为角速度 (rad/s)，与 STM32 VelocityCmd_t 对应。
        """
        data = struct.pack('<ff', msg.linear.x, msg.angular.z)
        self.send_frame(0x10, data)
        self.last_cmd_time = self.get_clock().now()

    def watchdog_check(self):
        """
        看门狗检测: 超过 watchdog_timeout 未收到 /cmd_vel 则停车。

        防止通信中断或导航崩溃时机器人继续运动，符合工业安全规范。
        """
        elapsed = (self.get_clock().now() - self.last_cmd_time).nanoseconds * 1e-9
        if elapsed > self.watchdog_timeout:
            data = struct.pack('<ff', 0.0, 0.0)
            self.send_frame(0x10, data)
            # 重置时间戳，避免反复发送零速帧
            self.last_cmd_time = self.get_clock().now()

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

    def process_serial_data(self):
        """处理串口接收到的数据"""
        try:
            # 检查串口是否打开
            if not self.ser.is_open:
                self.get_logger().warn('Serial port is not open')
                return

            # 检查串口是否有数据可读
            in_waiting = self.ser.in_waiting
            if in_waiting > 0:
                # 读取串口数据
                data = self.ser.read(in_waiting)
                self.buffer.extend(data)
                self.get_logger().debug(f'Received {len(data)} bytes, buffer size: {len(self.buffer)}')
                self.get_logger().debug(f'Buffer content: {self.buffer.hex()}')

                # 处理缓冲区中的数据帧
                while len(self.buffer) >= 5:  # 最小帧长度: start(1) + type(1) + length(2) + end(1)
                    # 查找帧头
                    start_idx = self.buffer.find(self.frame_start)
                    if start_idx == -1:
                        # 没有找到帧头，保留最后4个字节，可能是下一个帧的一部分
                        if len(self.buffer) > 4:
                            self.get_logger().debug(f'No frame start found, keeping last 4 bytes')
                            self.buffer = self.buffer[-4:]
                        break

                    # 移除帧头之前的数据
                    if start_idx > 0:
                        self.get_logger().debug(f'Removing {start_idx} bytes before frame start: {self.buffer[:start_idx].hex()}')
                        self.buffer = self.buffer[start_idx:]

                    # 打印当前帧头信息
                    self.get_logger().debug(f'Frame header: {self.buffer[:5].hex()}')

                    # 检查是否有足够的数据来解析帧头和长度
                    if len(self.buffer) < 5:
                        break

                    # 解析帧类型和长度
                    frame_type = self.buffer[1]
                    frame_length = struct.unpack('<H', self.buffer[2:4])[0]

                    # 打印帧信息
                    self.get_logger().debug(f'Frame type: 0x{frame_type:02x}, Frame length: {frame_length}')

                    # 检查帧长度是否合理
                    if frame_length > 256:
                        self.get_logger().warn(f'Invalid frame length: {frame_length}')
                        self.buffer = self.buffer[1:]  # 移除帧头，继续查找
                        continue

                    # 检查是否有完整的帧（包括帧尾）
                    total_frame_length = 5 + frame_length  # start(1) + type(1) + length(2) + data(length) + end(1)
                    if len(self.buffer) < total_frame_length:
                        break

                    # 检查帧尾
                    if self.buffer[total_frame_length - 1] != self.frame_end:
                        self.get_logger().warn(f'Invalid frame end: expected 0x{self.frame_end:02x}, got 0x{self.buffer[total_frame_length - 1]:02x}')
                        self.get_logger().debug(f'Frame data (first 20 bytes): {self.buffer[:20].hex()}')
                        self.get_logger().debug(f'Frame data (last 20 bytes): {self.buffer[max(0, total_frame_length-20):total_frame_length].hex()}')
                        self.get_logger().debug(f'Total frame length: {total_frame_length}, buffer size: {len(self.buffer)}')
                        self.buffer = self.buffer[1:]  # 移除帧头，继续查找
                        continue

                    # 提取帧数据
                    frame_data = self.buffer[4:4+frame_length]

                    # 处理帧数据
                    if frame_type == 0x01:  # 里程计数据
                        self.process_odometry_frame(frame_data)
                    elif frame_type == 0x02:  # TF数据
                        self.process_tf_frame(frame_data)

                    # 移除已处理的帧
                    self.buffer = self.buffer[total_frame_length:]

        except Exception as e:
            self.get_logger().error(f'Error processing serial data: {e}')

    def process_odometry_frame(self, data):
        """处理里程计数据帧"""
        try:
            # 解析里程计数据
            position_x, position_y, orientation_z, orientation_w, linear_velocity_x, angular_velocity_z = struct.unpack('<dddddd', data)

            # 死区过滤: 编码器噪声导致低速漂移, 低于阈值归零
            VEL_DEADBAND = 0.005
            if abs(linear_velocity_x) < VEL_DEADBAND:
                linear_velocity_x = 0.0
            if abs(angular_velocity_z) < VEL_DEADBAND:
                angular_velocity_z = 0.0

            # 位置保持: 速度为零时锁住位置, 防止假积分漂移
            POS_MOVE_THRESH = 0.0001
            if linear_velocity_x == 0.0 and angular_velocity_z == 0.0:
                if self.pos_hold:
                    position_x = self.last_x
                    position_y = self.last_y
                    orientation_z = self.last_yaw
            elif abs(position_x - self.last_x) > POS_MOVE_THRESH or abs(position_y - self.last_y) > POS_MOVE_THRESH:
                self.pos_hold = False
            self.last_x = position_x
            self.last_y = position_y
            self.last_yaw = orientation_z

            # 创建里程计消息
            odom = Odometry()
            odom.header.stamp = self.get_clock().now().to_msg()
            odom.header.frame_id = 'odom'
            odom.child_frame_id = 'base_link'

            # 设置位置
            odom.pose.pose.position.x = position_x
            odom.pose.pose.position.y = position_y
            odom.pose.pose.position.z = 0.0

            # 设置姿态
            odom.pose.pose.orientation.z = orientation_z
            odom.pose.pose.orientation.w = orientation_w

            # 设置速度
            odom.twist.twist.linear.x = linear_velocity_x
            odom.twist.twist.angular.z = angular_velocity_z

            # 发布里程计消息
            self.odom_pub.publish(odom)

            # 发布TF变换
            t = TransformStamped()
            t.header.stamp = self.get_clock().now().to_msg()
            t.header.frame_id = 'odom'
            t.child_frame_id = 'base_link'

            # 设置变换
            t.transform.translation.x = position_x
            t.transform.translation.y = position_y
            t.transform.translation.z = 0.0
            t.transform.rotation.z = orientation_z
            t.transform.rotation.w = orientation_w

            # 发布TF变换
            self.tf_broadcaster.sendTransform(t)

        except Exception as e:
            self.get_logger().error(f'Error processing odometry frame: {e}')

    def process_tf_frame(self, data):
        """处理TF数据帧"""
        try:
            # 解析TF数据
            x, y, theta = struct.unpack('<ddd', data)

            # 将角度转换为四元数
            q = self.quaternion_from_euler(0.0, 0.0, theta)

            # 发布TF变换
            t = TransformStamped()
            t.header.stamp = self.get_clock().now().to_msg()
            t.header.frame_id = 'odom'
            t.child_frame_id = 'base_link'

            # 设置变换
            t.transform.translation.x = x
            t.transform.translation.y = y
            t.transform.translation.z = 0.0
            t.transform.rotation = q

            # 发布TF变换
            self.tf_broadcaster.sendTransform(t)

        except Exception as e:
            self.get_logger().error(f'Error processing TF frame: {e}')

    def __del__(self):
        """析构函数，关闭串口"""
        if hasattr(self, 'ser') and self.ser.is_open:
            self.ser.close()

def main(args=None):
    rclpy.init(args=args)
    serial_odom_receiver = SerialOdomReceiver()
    rclpy.spin(serial_odom_receiver)
    serial_odom_receiver.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
