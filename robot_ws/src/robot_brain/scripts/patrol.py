#!/usr/bin/env python3
"""自主巡检: 按预设航点巡逻，到达后自动检测抓取"""
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose, NavigateThroughPoses
from geometry_msgs.msg import PoseStamped, Quaternion
import math, time

def euler_to_quat(yaw):
    q = Quaternion()
    q.z = math.sin(yaw/2); q.w = math.cos(yaw/2)
    return q

class PatrolNode(Node):
    def __init__(self):
        super().__init__('patrol')
        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.get_logger().info('Patrol ready')

    def goto(self, x, y, yaw=0.0):
        """导航到目标点。"""
        goal = NavigateToPose.Goal()
        goal.pose = PoseStamped()
        goal.pose.header.frame_id = 'map'
        goal.pose.pose.position.x = x
        goal.pose.pose.position.y = y
        goal.pose.pose.orientation = euler_to_quat(yaw)

        self.get_logger().info(f'Navigating to ({x:.1f}, {y:.1f})...')
        self.nav_client.wait_for_server()
        fut = self.nav_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, fut)
        if not fut.result().accepted:
            self.get_logger().warn('Goal rejected')
            return False

        res_fut = fut.result().get_result_async()
        rclpy.spin_until_future_complete(self, res_fut)
        self.get_logger().info('Arrived!')
        return True

def main():
    rclpy.init()
    node = PatrolNode()

    # 预设航点 (map坐标系, 需根据实际环境调整)
    waypoints = [
        (0.5,  0.5,  0.0),    # 巡检点1
        (1.0,  0.0,  1.57),   # 巡检点2
        (0.5, -0.5,  3.14),   # 巡检点3
        (0.0,  0.0,  0.0),    # 返回起点
    ]

    for i, (x, y, yaw) in enumerate(waypoints):
        print(f'\n=== Waypoint {i+1}/{len(waypoints)} ===')
        ok = node.goto(x, y, yaw)
        if not ok: break
        # 到达后等待5秒 (可以触发视觉检测抓取)
        print('Waiting for detection...')
        time.sleep(5)

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
