#!/usr/bin/env python3
"""
Nav2 导航封装 — 可导入模块, 也可独立测试。

从 patrol.py 泛化而来: 封装 NavigateToPose 动作为简洁的 goto() 接口。

两种使用模式:
  1. 组合模式 (被 integrated_patrol.py 使用):
       nav = Navigator(node)     # 复用外部 ROS2 Node
       nav.goto(x, y, yaw)

  2. 独立测试:
       python3 navigator.py --waypoints "0.5,0.5,0.0;1.0,0.0,1.57"
       (需先启动 robot_integrated.launch.py)
"""
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped, Quaternion
from action_msgs.msg import GoalStatus
import math
import sys
import time


def euler_to_quat(yaw):
    """欧拉角 yaw → Quaternion (仅 z/w, x/y=0)。"""
    q = Quaternion()
    q.z = math.sin(yaw / 2.0)
    q.w = math.cos(yaw / 2.0)
    return q


class Navigator:
    """Nav2 NavigateToPose 动作客户端包装器。

    不是 Node 的子类 — 接收外部 Node 引用, 支持组合模式。
    """

    def __init__(self, node: Node):
        """复用已有 ROS2 Node。

        Args:
            node: 已有的 rclpy Node 实例 (日志 / 时钟 / spin 都通过它)
        """
        self._node = node
        self._logger = node.get_logger()
        self._client = ActionClient(node, NavigateToPose, 'navigate_to_pose')
        self._is_navigating = False
        self._last_goal_handle = None
        self._logger.info('Navigator ready')

    def goto(self, x, y, yaw=0.0, timeout_sec=120.0):
        """导航到目标点 (阻塞, 直到到达/超时/失败)。

        内部使用 rclpy.spin_until_future_complete(self._node, ...)
        在组合模式下会同时驱动父 Node 的回调。

        Returns:
            True:  成功到达
            False: 被拒绝 / 超时 / 失败
        """
        goal = NavigateToPose.Goal()
        goal.pose = PoseStamped()
        goal.pose.header.frame_id = 'map'
        goal.pose.header.stamp = self._node.get_clock().now().to_msg()
        goal.pose.pose.position.x = float(x)
        goal.pose.pose.position.y = float(y)
        goal.pose.pose.position.z = 0.0
        goal.pose.pose.orientation = euler_to_quat(yaw)

        self._logger.info(
            f'Navigating to ({x:.2f}, {y:.2f}, yaw={yaw:.2f})...')

        # 等待 action server
        if not self._client.wait_for_server(timeout_sec=5.0):
            self._logger.error('NavigateToPose action server not available')
            return False

        # 发送目标
        self._is_navigating = True
        send_fut = self._client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self._node, send_fut, timeout_sec=10.0)

        if not send_fut.done():
            self._logger.error('Goal send timeout')
            self._is_navigating = False
            return False

        goal_handle = send_fut.result()
        if not goal_handle.accepted:
            self._logger.warn('Goal rejected by Nav2')
            self._is_navigating = False
            return False

        self._last_goal_handle = goal_handle
        self._logger.info('Goal accepted, navigating...')

        # 等待结果
        result_fut = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self._node, result_fut, timeout_sec=timeout_sec)

        self._is_navigating = False

        if not result_fut.done():
            self._logger.warn('Navigation timeout — cancelling goal')
            self.cancel()
            return False

        result = result_fut.result()
        status = result.status
        if status == GoalStatus.STATUS_SUCCEEDED:
            self._logger.info('Arrived!')
            return True
        else:
            self._logger.warn(f'Navigation failed (status={status})')
            return False

    def cancel(self):
        """取消当前导航目标。"""
        if self._last_goal_handle is not None:
            self._logger.info('Cancelling navigation goal...')
            cancel_fut = self._last_goal_handle.cancel_goal_async()
            rclpy.spin_until_future_complete(self._node, cancel_fut, timeout_sec=3.0)
            self._is_navigating = False

    @property
    def is_navigating(self) -> bool:
        return self._is_navigating

    @property
    def node(self):
        """获取内部引用的 Node。"""
        return self._node


# ─── 独立测试 ─────────────────────────────────────────────
class _TestNavigatorNode(Node):
    """独立测试用 — 创建一个临时 Node 并持有 Navigator。"""

    def __init__(self):
        super().__init__('navigator_test')
        self.nav = Navigator(self)


def main():
    rclpy.init(args=sys.argv)

    # 解析航点: --waypoints "x,y,yaw;x,y,yaw"
    waypoints = [(0.5, 0.5, 0.0)]  # 默认: 单点测试
    for i, arg in enumerate(sys.argv):
        if arg == '--waypoints' and i + 1 < len(sys.argv):
            try:
                pts = []
                for s in sys.argv[i + 1].split(';'):
                    parts = s.strip().split(',')
                    if len(parts) >= 2:
                        x, y = float(parts[0]), float(parts[1])
                        yaw = float(parts[2]) if len(parts) >= 3 else 0.0
                        pts.append((x, y, yaw))
                if pts:
                    waypoints = pts
            except ValueError:
                print(f'Invalid waypoints format: {sys.argv[i+1]}')
                print('Expected: "x,y,yaw;x,y,yaw"  (e.g. "0.5,0.5,0.0;1.0,0.0,1.57")')
                rclpy.shutdown()
                return

    test_node = _TestNavigatorNode()

    print(f'\n=== Navigator Test: {len(waypoints)} waypoints ===')
    for i, (x, y, yaw) in enumerate(waypoints):
        print(f'\n--- Waypoint {i+1}/{len(waypoints)} ---')
        t0 = time.time()
        ok = test_node.nav.goto(x, y, yaw)
        elapsed = time.time() - t0
        status = 'OK' if ok else 'FAIL'
        print(f'{status}  ({elapsed:.1f}s)')
        if not ok:
            print('Skipping remaining waypoints due to failure')
            break

    test_node.destroy_node()
    rclpy.shutdown()
    print('\nNavigator test done')


if __name__ == '__main__':
    main()
