#!/usr/bin/env python3
"""
MoveIt ↔ F407 桥接: RViz Plan&Execute 时同步驱动物理舵机。

监听 /joint_states, 变化时以 10Hz 节流下发 F407。
"""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import struct, serial, threading, time


class ArmBridge(Node):
    def __init__(self):
        super().__init__('arm_bridge')

        self.declare_parameter('serial_port', '/dev/ttyACM0')
        self.declare_parameter('baudrate', 115200)
        self.ser = None
        try:
            self.ser = serial.Serial(
                self.get_parameter('serial_port').value,
                self.get_parameter('baudrate').value, timeout=1)
            self.get_logger().info(f'Serial OK')
        except Exception as e:
            self.get_logger().error(f'Serial: {e}')

        self.lock = threading.Lock()
        self.last_sent = [0.0]*6
        self.last_time = 0.0

        # joint_state 顺序必须与 URDF 一致
        self.joint_names = ['arm_joint1','arm_joint2','arm_joint3',
                            'arm_joint4','end_joint1','gripper_joint']
        self.joint_positions = [0.0]*6
        self.changed = False

        self.sub = self.create_subscription(
            JointState, '/joint_states', self.js_cb, 10)

        # 10Hz 节流发送
        self.timer = self.create_timer(0.1, self.send_if_changed)
        self.get_logger().info('Arm Bridge ready')

    def js_cb(self, msg: JointState):
        for i, name in enumerate(self.joint_names):
            try:
                idx = msg.name.index(name)
                new_val = float(msg.position[idx])
                if abs(new_val - self.joint_positions[i]) > 0.005:
                    self.changed = True
                self.joint_positions[i] = new_val
            except ValueError:
                pass

    def send_if_changed(self):
        if not self.changed or not self.ser: return
        self.changed = False

        data = struct.pack('<ffffff', *[float(p) for p in self.joint_positions])
        frame = bytearray([0xAA, 0x20, 24, 0]) + data + bytearray([0x55])
        with self.lock: self.ser.write(frame)
        self.get_logger().debug(
            f'-> {[f"{p:.3f}" for p in self.joint_positions]}')

    def __del__(self):
        if self.ser: self.ser.close()


def main():
    rclpy.init()
    node = ArmBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
