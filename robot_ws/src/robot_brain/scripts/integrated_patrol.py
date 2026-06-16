#!/usr/bin/env python3
"""
一体化巡检抓取闭环 — 组合 Navigator + Detector + Grasper。

状态机: NAVIGATING → DETECTING → GRASPING → COOLDOWN → COMPLETED

启动方式:
  # 完整集成 (需要 robot_integrated.launch.py 先启动)
  ros2 launch robot_brain integrated_patrol.launch.py

  # 直接运行 (带默认参数)
  python3 integrated_patrol.py --ros-args -p grasp_enabled:=true

  # 半集成模式 (只导航+检测, 不抓取)
  python3 integrated_patrol.py --ros-args -p grasp_enabled:=false

  # 无头模式 (无 GUI, 纯自动化)
  python3 integrated_patrol.py --ros-args -p headless:=true
"""
import rclpy
from rclpy.node import Node
from enum import Enum
import cv2
import sys
import os
import time
import math

# 追加脚本目录到 path
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from navigator import Navigator
from grasper import Grasper
from detector import Detector


class PatrolState(Enum):
    NAVIGATING = 'navigating'
    DETECTING = 'detecting'
    GRASPING = 'grasping'
    COOLDOWN = 'cooldown'
    COMPLETED = 'completed'


class IntegratedPatrolNode(Node):
    """一体化巡检节点: 导航 → 检测 → 抓取 → 下一站。"""

    # 默认航点 (map 坐标系, 需根据实际环境调整)
    DEFAULT_WAYPOINTS = [
        (0.5,  0.5,  0.0),
        (1.0,  0.0,  1.57),
        (0.5, -0.5,  3.14),
        (0.0,  0.0,  0.0),
    ]

    def __init__(self):
        super().__init__('integrated_patrol')

        # ── ROS2 参数 ──
        self.declare_parameter('waypoints', [
            [0.5, 0.5, 0.0],
            [1.0, 0.0, 1.57],
            [0.5, -0.5, 3.14],
            [0.0, 0.0, 0.0],
        ])
        self.declare_parameter('detection_timeout_sec', 30.0)
        self.declare_parameter('grasp_enabled', True)
        self.declare_parameter('headless', False)
        self.declare_parameter('loop', False)
        self.declare_parameter('model_path', '/home/cobrander/RDK_X5/yolov8n.pt')
        self.declare_parameter('confidence_threshold', 0.3)
        self.declare_parameter('serial_port', '/dev/ttyACM0')
        self.declare_parameter('target_classes', [
            'mouse', 'bottle', 'cup', 'cell phone', 'book', 'banana', 'apple',
        ])

        # 解析航点
        wp_raw = self.get_parameter('waypoints').value
        self.waypoints = [(float(w[0]), float(w[1]), float(w[2])) for w in wp_raw]

        self.detection_timeout = float(self.get_parameter('detection_timeout_sec').value)
        self.grasp_enabled = bool(self.get_parameter('grasp_enabled').value)
        self.headless = bool(self.get_parameter('headless').value)
        self.loop = bool(self.get_parameter('loop').value)
        model_path = str(self.get_parameter('model_path').value)
        conf_thresh = float(self.get_parameter('confidence_threshold').value)
        serial_port = str(self.get_parameter('serial_port').value)
        target_classes = set(str(c) for c in self.get_parameter('target_classes').value)

        # ── 模块初始化 ──
        self.get_logger().info('Initializing modules...')

        # Navigator (复用当前 node)
        self.nav = Navigator(self)

        # Grasper
        self.grasper = Grasper(port=serial_port)
        self.get_logger().info(f'Arm {"connected" if self.grasper.connected else "DRY-RUN"}')

        # Detector
        self.detector = Detector(
            model_path=model_path,
            confidence_threshold=conf_thresh,
            target_classes=target_classes,
            headless=self.headless,
        )

        # ── 状态机 ──
        self.state = PatrolState.NAVIGATING
        self.waypoint_index = 0
        self.grasp_count = 0
        self._detection_start_time = 0.0
        self._cooldown_start = 0.0
        self._cooldown_duration = 5.0  # 抓取后停留 5 秒

        self.get_logger().info(f'Integrated Patrol ready: {len(self.waypoints)} waypoints, '
                               f'grasp={self.grasp_enabled}, loop={self.loop}')

    # ── 状态机推进 ────────────────────────────────────

    def _tick_navigation(self):
        """导航到当前航点。"""
        if self.waypoint_index >= len(self.waypoints):
            if self.loop:
                self.get_logger().info('Loop: restarting from waypoint 0')
                self.waypoint_index = 0
            else:
                self.state = PatrolState.COMPLETED
                return

        x, y, yaw = self.waypoints[self.waypoint_index]
        self.get_logger().info(
            f'=== Waypoint {self.waypoint_index + 1}/{len(self.waypoints)}: '
            f'({x:.2f}, {y:.2f}, yaw={yaw:.2f}) ===')

        ok = self.nav.goto(x, y, yaw, timeout_sec=120.0)

        if ok:
            self.state = PatrolState.DETECTING
            self._detection_start_time = time.time()
        else:
            self.get_logger().warn(f'Navigation failed, skipping waypoint {self.waypoint_index}')
            self.waypoint_index += 1
            # 继续尝试下一个航点
            if self.waypoint_index >= len(self.waypoints):
                self.state = PatrolState.COMPLETED

    def _tick_detection(self, result):
        """检测阶段: 检查触发条件和超时。"""
        elapsed = time.time() - self._detection_start_time

        if self.detector.is_auto_trigger(result):
            self.get_logger().info(
                f'Target detected: {result.class_name} @ {result.distance_m*100:.0f}cm')
            if self.grasp_enabled:
                self.state = PatrolState.GRASPING
                self._grasp_j1 = max(-1.2, min(1.2,
                    (result.cx - result.image_width / 2) / result.image_width * 0.6))
            else:
                self.get_logger().info('Grasp disabled, skipping to next waypoint')
                self.waypoint_index += 1
                self.state = PatrolState.NAVIGATING

        elif elapsed > self.detection_timeout:
            self.get_logger().info(
                f'Detection timeout ({self.detection_timeout:.0f}s), '
                f'no target found at waypoint {self.waypoint_index}')
            self.waypoint_index += 1
            self.state = PatrolState.NAVIGATING

    def _tick_grasping(self):
        """执行抓取序列。"""
        self.get_logger().info(f'Grasping with j1={self._grasp_j1:.2f}')
        self.grasper.execute_grasp_sequence(self._grasp_j1)
        self.grasp_count += 1
        self.detector.reset_cooldown()
        self.waypoint_index += 1
        self._cooldown_start = time.time()
        self.state = PatrolState.COOLDOWN
        self.get_logger().info(f'Grasp #{self.grasp_count} complete')

    def _tick_cooldown(self):
        """抓取后冷却。"""
        if time.time() - self._cooldown_start > self._cooldown_duration:
            self.state = PatrolState.NAVIGATING

    # ── 主循环 ─────────────────────────────────────────

    def run(self):
        """主事件循环: spin_once + detect + display + state machine。"""
        self.get_logger().info('Starting patrol main loop...')

        win_name = 'Integrated Patrol'
        cv2.namedWindow(win_name, cv2.WINDOW_NORMAL)
        cv2.resizeWindow(win_name, 640, 480)

        try:
            while rclpy.ok() and self.state != PatrolState.COMPLETED:
                # 1. ROS2 回调 (10ms)
                rclpy.spin_once(self, timeout_sec=0.01)

                # 2. 状态机
                if self.state == PatrolState.NAVIGATING:
                    self._tick_navigation()

                elif self.state == PatrolState.DETECTING:
                    result = self.detector.detect()

                    # 显示
                    if not self.headless:
                        annotated = self.detector.annotate_frame(result)
                        status = f'WP{self.waypoint_index+1}/{len(self.waypoints)}'
                        if result.class_name:
                            status += f' [{result.class_name}]'
                        self.detector.draw_status_overlay(annotated, status)

                        if result.is_stable and self.detector.is_auto_trigger(result):
                            cv2.putText(annotated,
                                        f'>>> TRIGGER: {result.class_name} '
                                        f'{result.distance_m*100:.0f}cm <<<',
                                        (5, annotated.shape[0] - 10),
                                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)

                        cv2.imshow(win_name, annotated)

                    # 键盘
                    key = cv2.waitKey(1) & 0xFF
                    if key == ord('q'):
                        self.get_logger().info('User quit')
                        break
                    elif key == ord('h'):
                        self.grasper.home()
                    elif key == ord('n'):
                        # 手动跳过当前航点
                        self.get_logger().info('Manual skip to next waypoint')
                        self.waypoint_index += 1
                        self.state = PatrolState.NAVIGATING
                        continue

                    self._tick_detection(result)

                elif self.state == PatrolState.GRASPING:
                    self._tick_grasping()

                elif self.state == PatrolState.COOLDOWN:
                    self._tick_cooldown()

                    # 冷却期间保持显示
                    if not self.headless:
                        result = self.detector.detect()
                        annotated = self.detector.annotate_frame(result)
                        cd_remaining = max(0, self._cooldown_duration -
                                          (time.time() - self._cooldown_start))
                        status = f'COOLDOWN {cd_remaining:.0f}s | Grasps: {self.grasp_count}'
                        self.detector.draw_status_overlay(annotated, status)
                        cv2.imshow(win_name, annotated)
                        if cv2.waitKey(1) & 0xFF == ord('q'):
                            break

        except KeyboardInterrupt:
            self.get_logger().info('Interrupted')

        finally:
            self._shutdown()

    def _shutdown(self):
        """优雅关闭。"""
        self.get_logger().info('Shutting down...')

        # 归位机械臂
        if self.grasper.connected:
            self.get_logger().info('Homing arm...')
            self.grasper.home()

        # 释放相机
        self.detector.release()

        # 清理
        cv2.destroyAllWindows()
        self.get_logger().info(f'Patrol done. {self.grasp_count} grasps completed.')


# ─── main ─────────────────────────────────────────────────
def main():
    rclpy.init(args=sys.argv)
    node = IntegratedPatrolNode()

    print('\n' + '=' * 60)
    print('  Integrated Patrol — 巡检 + 检测 + 抓取 闭环')
    print('=' * 60)
    print(f'  Waypoints: {len(node.waypoints)}')
    for i, (x, y, yaw) in enumerate(node.waypoints):
        print(f'    {i+1}. ({x:.1f}, {y:.1f}, {yaw:.2f} rad)')
    print(f'  Grasp enabled: {node.grasp_enabled}')
    print(f'  Headless: {node.headless}')
    print(f'  Loop: {node.loop}')
    print(f'  Detection timeout: {node.detection_timeout:.0f}s')
    print('=' * 60)
    if not node.headless:
        print('  Keys: q=quit  h=home  n=skip waypoint')
    print()

    if not node.grasper.connected:
        print('⚠️  Warning: Arm not connected (dry-run mode)')
        print()

    node.run()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
