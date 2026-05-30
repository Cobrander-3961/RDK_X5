#!/usr/bin/env python3
"""
MoveIt2 空间位置接收与运动规划节点 (纯 ROS2 Action)。

订阅 /target_pose (PoseStamped)，通过 MoveGroup action 规划并执行。
"""
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import (
    MotionPlanRequest, Constraints, JointConstraint,
    PositionConstraint, BoundingVolume, OrientationConstraint,
)
from shape_msgs.msg import SolidPrimitive
from geometry_msgs.msg import PoseStamped, Pose, Vector3, Quaternion
from std_msgs.msg import Header
import struct
import serial
import threading


class ArmMotionPlanner(Node):
    """接收目标位姿，规划并执行机械臂运动。"""

    def __init__(self):
        super().__init__('arm_motion_planner')

        # ---- 串口 ----
        self.declare_parameter('serial_port', '/dev/ttyACM0')
        self.declare_parameter('baudrate', 115200)
        self.ser = None
        try:
            self.ser = serial.Serial(
                self.get_parameter('serial_port').value,
                self.get_parameter('baudrate').value,
                timeout=1,
            )
            self.get_logger().info('Arm serial opened')
        except serial.SerialException as e:
            self.get_logger().warn(f'Serial not available: {e}')

        self.write_lock = threading.Lock()

        # ---- MoveGroup action 客户端 ----
        self._client = ActionClient(self, MoveGroup, 'move_action')
        if not self._client.wait_for_server(timeout_sec=5.0):
            self.get_logger().warn('move_action server not available')

        # ---- 订阅目标位姿 ----
        self.target_sub = self.create_subscription(
            PoseStamped, '/target_pose', self.target_pose_callback, 10)
        self.get_logger().info('Motion Planner ready')

    def _plan_pose(self, pose: Pose):
        """构建 Pose 目标规划请求。"""
        constraints = Constraints()

        pc = PositionConstraint()
        pc.header = Header(frame_id='base_link')
        pc.link_name = 'end_Link1'
        pc.target_point_offset = Vector3(x=0.0, y=0.0, z=0.0)
        pc.weight = 1.0
        sphere = SolidPrimitive(type=SolidPrimitive.SPHERE, dimensions=[0.01])
        pc.constraint_region = BoundingVolume()
        pc.constraint_region.primitives = [sphere]
        pc.constraint_region.primitive_poses = [pose]

        oc = OrientationConstraint()
        oc.header = Header(frame_id='base_link')
        oc.link_name = 'end_Link1'
        oc.orientation = pose.orientation
        oc.absolute_x_axis_tolerance = 0.1
        oc.absolute_y_axis_tolerance = 0.1
        oc.absolute_z_axis_tolerance = 0.1
        oc.weight = 1.0

        constraints.position_constraints = [pc]
        constraints.orientation_constraints = [oc]

        req = MotionPlanRequest()
        req.group_name = 'arm_group'
        req.allowed_planning_time = 2.0
        req.max_velocity_scaling_factor = 0.3
        req.max_acceleration_scaling_factor = 0.3
        req.goal_constraints = [constraints]

        return MoveGroup.Goal(request=req)

    def target_pose_callback(self, msg: PoseStamped):
        self.get_logger().info(
            f'Target: x={msg.pose.position.x:.3f} '
            f'y={msg.pose.position.y:.3f} z={msg.pose.position.z:.3f}')

        goal = self._plan_pose(msg.pose)
        fut = self._client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, fut, timeout_sec=5.0)
        if not fut.done() or not fut.result().accepted:
            self.get_logger().warn('Planning rejected')
            return

        res_fut = fut.result().get_result_async()
        rclpy.spin_until_future_complete(self, res_fut, timeout_sec=10.0)
        if not res_fut.done():
            self.get_logger().warn('Execution timeout')
            return

        result = res_fut.result().result
        if result.error_code.val == 1:
            self.get_logger().info('Motion executed OK')
            # 发送关节角度到 F407
            if result.trajectory_start and result.trajectory_start.joint_state.position:
                self._send_to_f407(result.trajectory_start.joint_state.position)
        else:
            self.get_logger().error(f'Motion failed: {result.error_code.val}')

    def _send_to_f407(self, positions):
        """打包关节角度 → 串口帧 0x20 下发 F407。"""
        if self.ser is None or not self.ser.is_open:
            return
        packed = [0.0] * 6
        for i in range(min(len(positions), 6)):
            packed[i] = float(positions[i])
        data = struct.pack('<ffffff', *packed)

        frame = bytearray()
        frame.append(0xAA)
        frame.append(0x20)
        frame.extend(struct.pack('<H', len(data)))
        frame.extend(data)
        frame.append(0x55)

        with self.write_lock:
            self.ser.write(frame)
        self.get_logger().info(f'Sent joints: {[f"{p:.2f}" for p in packed]}')

    def __del__(self):
        if self.ser and self.ser.is_open:
            self.ser.close()


def main(args=None):
    rclpy.init(args=args)
    node = ArmMotionPlanner()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
