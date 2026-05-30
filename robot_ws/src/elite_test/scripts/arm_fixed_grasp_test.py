#!/usr/bin/env python3
"""
固定位置抓取测试 — 关节空间运动序列。
MoveIt 规划 → 串口下发 F407 → PCA9685 舵机执行。
"""
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import Constraints, JointConstraint, MotionPlanRequest
import struct
import serial
import threading
import time


class MoveGroupClient:
    def __init__(self, node: Node):
        self.node = node
        self._client = ActionClient(node, MoveGroup, 'move_action')

    def plan_and_execute(self, positions):
        """同步发送关节目标并等待结果。"""
        constraints = Constraints()
        for i, name in enumerate(['arm_joint1', 'arm_joint2', 'arm_joint3',
                                    'arm_joint4', 'end_joint1', 'gripper_joint']):
            jc = JointConstraint()
            jc.joint_name = name
            jc.position = float(positions[i]) if i < len(positions) else 0.0
            jc.tolerance_above = 0.1
            jc.tolerance_below = 0.1
            jc.weight = 1.0
            constraints.joint_constraints.append(jc)

        req = MotionPlanRequest()
        req.group_name = 'arm_group'
        req.allowed_planning_time = 2.0
        req.max_velocity_scaling_factor = 0.3
        req.num_planning_attempts = 3
        req.goal_constraints = [constraints]

        goal = MoveGroup.Goal(request=req)

        self.node.get_logger().info('Sending goal...')
        fut = self._client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=10.0)
        if not fut.done():
            self.node.get_logger().warn('TIMEOUT: goal not accepted')
            return False

        goal_handle = fut.result()
        if not goal_handle.accepted:
            self.node.get_logger().warn('REJECTED')
            return False

        self.node.get_logger().info('Accepted, waiting...')
        res_fut = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self.node, res_fut, timeout_sec=15.0)
        if not res_fut.done():
            self.node.get_logger().warn('TIMEOUT: no result')
            return False

        err = res_fut.result().result.error_code.val
        ok = err in (1, -4)
        self.node.get_logger().info(f'  {"OK" if ok else "FAILED"} (err={err})')
        return ok


class SerialJointSender:
    """通过串口下发关节角度到 F407 (帧类型 0x20)。"""
    def __init__(self, port='/dev/ttyACM0', baudrate=115200):
        self.ser = None
        self.lock = threading.Lock()
        try:
            self.ser = serial.Serial(port, baudrate, timeout=1)
            print(f'[SerialJointSender] {port} opened at {baudrate}')
        except serial.SerialException as e:
            print(f'[SerialJointSender] WARN: {e} — no hardware connected')

    def send(self, positions):
        """发送 6 个关节角度 (rad) 到 F407。"""
        if self.ser is None or not self.ser.is_open:
            print('[SerialJointSender] Serial not available, skip')
            return

        packed = [float(positions[i]) if i < len(positions) else 0.0
                  for i in range(6)]
        data = struct.pack('<ffffff', *packed)  # 6×float LE = 24 bytes

        frame = bytearray()
        frame.append(0xAA)                       # 帧头
        frame.append(0x20)                        # 类型: 关节角度
        frame.extend(struct.pack('<H', 24))      # 长度 24 LE
        frame.extend(data)                        # 载荷
        frame.append(0x55)                        # 帧尾

        with self.lock:
            self.ser.write(frame)
        print(f'[SerialJointSender] Sent: {[f"{p:.2f}" for p in packed]}')


class FixedGraspTest(Node):
    def __init__(self):
        super().__init__('arm_fixed_grasp_test')
        self.mgc = MoveGroupClient(self)
        self.serial = SerialJointSender()
        self.get_logger().info('Fixed Grasp Test ready')

    HOME   = [ 0.0,  0.0,  0.0,  0.0,  0.0,  0.0]
    PRE    = [ 0.3, -0.2, -0.5,  0.0,  0.0,  0.0]
    GRASP  = [ 0.3, -0.2, -0.5,  0.0,  0.0, -0.5]
    PLACE  = [-0.3, -0.2, -0.3,  0.0,  0.0, -0.5]
    RELEASE= [-0.3, -0.2, -0.3,  0.0,  0.0,  0.3]

    def move_to(self, name, joints):
        """MoveIt 规划 + 串口下发。"""
        self.get_logger().info(f'>>> {name}: {[f"{j:.2f}" for j in joints]}')
        ok = self.mgc.plan_and_execute(joints)
        # 无论 MoveIt 规划结果如何, 都下发目标角度给 F407
        self.serial.send(joints)
        return ok

    def run_sequence(self):
        self.get_logger().info('========== GRASP SEQUENCE START ==========')
        seq = [
            ('Home',      self.HOME),
            ('Pre-grasp', self.PRE),
            ('Grasp',     self.GRASP),
            ('Lift',      self.PRE),
            ('Place',     self.PLACE),
            ('Release',   self.RELEASE),
            ('Home',      self.HOME),
        ]
        for name, joints in seq:
            self.move_to(name, joints)
            time.sleep(1.5)
        self.get_logger().info('========== GRASP SEQUENCE DONE ==========')


def main(args=None):
    rclpy.init(args=args)
    node = FixedGraspTest()
    time.sleep(3.0)
    node.run_sequence()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

