#!/usr/bin/env python3
"""
果园巡检 + 苹果质量分析 + 云端上传 — 全链路编排器。

状态机: NAVIGATING → SCANNING → ANALYZING → REPORTING → COMPLETED

启动:
  # 完整模式 (导航 + 检测 + 上传)
  ros2 launch robot_brain orchard_patrol.launch.py api_key:=YOUR_KEY

  # 定点扫描模式 (不导航, 纯检测+统计)
  python3 orchard_patrol.py --ros-args -p nav_enabled:=false

依赖: 需先启动 robot_integrated.launch.py (导航全家桶)
"""
import rclpy
from rclpy.node import Node
from enum import Enum
import cv2
import sys
import os
import time
import json
import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from navigator import Navigator
from fruit_analyzer import FruitAnalyzer, FruitQuality
from cloud_uploader import CloudUploader, OrchardReport
from detector import Detector


class OrchardState(Enum):
    NAVIGATING = 'navigating'
    SCANNING = 'scanning'
    ANALYZING = 'analyzing'
    REPORTING = 'reporting'
    COMPLETED = 'completed'


class OrchardPatrolNode(Node):
    """果园巡检节点: 按行导航 → 检测苹果 → 质量分析 → 统计 → 上传云端。"""

    def __init__(self):
        super().__init__('orchard_patrol')

        # ── ROS2 参数 ──
        self.declare_parameter('api_key', '')
        self.declare_parameter('cloud_endpoint', '')
        self.declare_parameter('dataset_id', 'orchard-apple-quality')
        self.declare_parameter('serial_port', '/dev/ttyACM0')
        self.declare_parameter('model_path', '/home/sunrise/RDK_X5/yolov8n.pt')
        self.declare_parameter('confidence_threshold', 0.2)
        self.declare_parameter('headless', False)
        self.declare_parameter('nav_enabled', True)
        self.declare_parameter('orchard_rows', 3)
        self.declare_parameter('row_spacing', 1.5)
        self.declare_parameter('row_length', 5.0)
        self.declare_parameter('scan_duration_per_m', 3.0)
        self.declare_parameter('frame_skip', 3)

        self.headless = bool(self.get_parameter('headless').value)

        # ── 模块初始化 ──
        self.get_logger().info('Initializing orchard patrol modules...')

        # 导航 (可选)
        if bool(self.get_parameter('nav_enabled').value):
            self.nav = Navigator(self)
            self.get_logger().info('Navigator ready')
        else:
            self.nav = None
            self.get_logger().info('Navigator disabled (fixed position mode)')

        # 检测 + 质量分析
        detector = Detector(
            model_path=str(self.get_parameter('model_path').value),
            confidence_threshold=float(self.get_parameter('confidence_threshold').value),
            target_classes={'apple'},
            headless=True,
        )
        self.analyzer = FruitAnalyzer(detector=detector, headless=True)

        # 云端上传
        api_key = str(self.get_parameter('api_key').value)
        endpoint = str(self.get_parameter('cloud_endpoint').value)
        dataset_id = str(self.get_parameter('dataset_id').value)
        self.uploader = CloudUploader(
            api_key=api_key,
            endpoint=endpoint,
            dataset_id=dataset_id,
        )

        # ── 巡检参数 ──
        self.orchard_rows = int(self.get_parameter('orchard_rows').value)
        self.row_spacing = float(self.get_parameter('row_spacing').value)
        self.row_length = float(self.get_parameter('row_length').value)
        self.scan_duration_per_m = float(self.get_parameter('scan_duration_per_m').value)
        self.frame_skip = int(self.get_parameter('frame_skip').value)

        # 自动计算: 每行扫描时长
        self.scan_duration = self.row_length * self.scan_duration_per_m

        # ── 状态机 ──
        self.state = OrchardState.NAVIGATING
        self.current_row = 0
        self.row_stats = []         # [{row, summary, thumbnail, location}, ...]
        self._scan_start = 0.0
        self._frame_counter = 0
        self._session_summary = {'total': 0, 'good': 0, 'bad': 0,
                                 'unripe': 0, 'unknown': 0}
        self._best_thumbnail = None  # 质量最高的帧作为缩略图

        self.get_logger().info(
            f'Orchard Patrol ready: {self.orchard_rows} rows × '
            f'{self.row_length}m, spacing={self.row_spacing}m')

    # ── 主循环 ─────────────────────────────────────────

    def run(self):
        self.get_logger().info('Starting orchard patrol...')

        win_name = 'Orchard Patrol'
        if not self.headless:
            cv2.namedWindow(win_name, cv2.WINDOW_NORMAL)
            cv2.resizeWindow(win_name, 640, 480)

        print('\n' + '=' * 60)
        print('  Orchard Patrol — 果园巡检 + 苹果质量分析')
        print('=' * 60)
        print(f'  Rows: {self.orchard_rows} × {self.row_length}m')
        print(f'  Navigation: {"enabled" if self.nav else "disabled"}')
        print(f'  Upload: {"enabled" if self.uploader.endpoint else "disabled"}')
        print(f'  Keys: q=quit  s=save snapshot')
        print('=' * 60)

        try:
            while rclpy.ok() and self.state != OrchardState.COMPLETED:
                rclpy.spin_once(self, timeout_sec=0.01)

                if self.state == OrchardState.NAVIGATING:
                    self._tick_navigating()
                elif self.state == OrchardState.SCANNING:
                    self._tick_scanning()
                elif self.state == OrchardState.ANALYZING:
                    self._tick_analyzing()
                elif self.state == OrchardState.REPORTING:
                    self._tick_reporting()

                self._tick_display()

        except KeyboardInterrupt:
            self.get_logger().info('Interrupted')

        finally:
            self._shutdown()

    # ── 状态实现 ───────────────────────────────────────

    def _tick_navigating(self):
        """导航到当前行起点。"""
        if self.current_row >= self.orchard_rows:
            self.state = OrchardState.COMPLETED
            return

        # 行起点: y = row * spacing, x = 0
        x = 0.0
        y = self.current_row * self.row_spacing
        yaw = 0.0  # 面向行方向

        self.get_logger().info(
            f'=== Row {self.current_row + 1}/{self.orchard_rows}: '
            f'navigate to ({x:.1f}, {y:.1f}) ===')

        if self.nav is not None:
            ok = self.nav.goto(x, y, yaw, timeout_sec=60.0)
            if not ok:
                self.get_logger().warn(f'Navigation to row {self.current_row} failed')
        else:
            # 无导航模式: 等待用户手动放置机器人
            print(f'\n>>> Place robot at row {self.current_row + 1} '
                  f'({x:.1f}, {y:.1f}), then press Enter')
            try:
                input()
            except (EOFError, KeyboardInterrupt):
                self.state = OrchardState.COMPLETED
                return

        self.state = OrchardState.SCANNING
        self._scan_start = time.time()
        self._frame_counter = 0
        self._row_summary = {'total': 0, 'good': 0, 'bad': 0,
                             'unripe': 0, 'unknown': 0}
        self._best_thumbnail = None
        self.get_logger().info(f'Scanning row {self.current_row + 1} '
                               f'for {self.scan_duration:.0f}s...')

    def _tick_scanning(self):
        """沿行扫描: 逐帧分析苹果质量, 累计统计。"""
        elapsed = time.time() - self._scan_start

        # 跳帧 (每 N 帧分析一次, 减少 YOLO 计算)
        self._frame_counter += 1
        if self._frame_counter % self.frame_skip != 0:
            return

        qualities, summary = self.analyzer.analyze_frame()

        # 累计行统计
        for k in self._row_summary:
            self._row_summary[k] += summary[k]

        # 保留最佳缩略图 (苹果最多的帧)
        if summary['total'] > 0:
            if (self._best_thumbnail is None or
                    summary['total'] > self._best_thumbnail[0]):
                frame = self.analyzer.detector.current_frame
                if frame is not None:
                    self._best_thumbnail = (summary['total'], frame.copy())

        # 检查是否到行末
        if elapsed >= self.scan_duration:
            self.get_logger().info(f'Row {self.current_row + 1} scan complete')
            self.state = OrchardState.ANALYZING

    def _tick_analyzing(self):
        """汇总行统计, 准备上报。"""
        row_data = {
            'row': self.current_row + 1,
            'summary': dict(self._row_summary),
            'location': {
                'x': 0.0,
                'y': self.current_row * self.row_spacing,
            },
            'thumbnail': (self._best_thumbnail[1] if self._best_thumbnail
                          else None),
        }
        self.row_stats.append(row_data)

        # 打印行统计
        s = self._row_summary
        total = max(s['total'], 1)
        ratio = s['good'] / total * 100
        self.get_logger().info(
            f'Row {self.current_row + 1} stats: '
            f'total={s["total"]} good={s["good"]} bad={s["bad"]} '
            f'unripe={s["unripe"]} ({ratio:.0f}% good)')

        # 累计全局统计
        for k in self._session_summary:
            self._session_summary[k] += s[k]

        self.state = OrchardState.REPORTING

    def _tick_reporting(self):
        """上传行报告到云端。"""
        row_data = self.row_stats[-1]

        # 构建报告
        report = OrchardReport.from_stats(
            robot_id='RDK-X5-001',
            orchard_id='orchard-1',
            row=row_data['row'],
            location=row_data['location'],
            summary=row_data['summary'],
            thumbnail=row_data['thumbnail'],
        )

        # 上传
        ok = self.uploader.upload_report(report)
        if not ok:
            self.get_logger().warn(f'Row {row_data["row"]} upload failed '
                                   '(cached locally)')

        # 下一行
        self.current_row += 1
        if self.current_row >= self.orchard_rows:
            self.state = OrchardState.COMPLETED
        else:
            self.state = OrchardState.NAVIGATING

    # ── 显示 ───────────────────────────────────────────

    def _tick_display(self):
        """实时显示摄像头画面 + 检测标注 + 状态覆盖。"""
        if self.headless:
            return

        frame = self.analyzer.detector.current_frame
        if frame is None:
            return

        # 扫描中: 显示标注帧
        if self.state == OrchardState.SCANNING:
            qualities, _ = self.analyzer.analyze_frame()
            display = self.analyzer.annotate_frame(qualities, frame)
        else:
            display = frame.copy()

        # 状态覆盖
        row_info = f'Row {self.current_row + 1}/{self.orchard_rows}'
        if self.current_row >= self.orchard_rows:
            row_info = f'Row {self.orchard_rows}/{self.orchard_rows}'

        cv2.putText(display, f'{self.state.value.upper()} — {row_info}',
                    (5, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

        # 行统计覆盖 (SCANNING/ANALYZING/REPORTING 时)
        s = self._row_summary
        total = max(s['total'], 1)
        ratio = s['good'] / total * 100
        y0 = 45
        cv2.putText(display, f'Good:{s["good"]} Bad:{s["bad"]} '
                    f'Unripe:{s["unripe"]} ({ratio:.0f}%)',
                    (5, y0), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (255, 255, 255), 1)

        # 进度条 (SCANNING)
        if self.state == OrchardState.SCANNING:
            elapsed = time.time() - self._scan_start
            progress = min(elapsed / self.scan_duration, 1.0)
            bar_w = 200
            cv2.rectangle(display, (5, 60), (5 + bar_w, 70), (64, 64, 64), -1)
            cv2.rectangle(display, (5, 60), (5 + int(bar_w * progress), 70),
                          (0, 255, 0), -1)

        # 全局统计 (右下角)
        gs = self._session_summary
        gs_total = max(gs['total'], 1)
        cv2.putText(display,
                    f'Session: {gs["good"]}/{gs_total} good ({gs["good"]/gs_total*100:.0f}%)',
                    (5, display.shape[0] - 10),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.45, (200, 200, 200), 1)

        cv2.imshow(win_name, display)
        key = cv2.waitKey(1) & 0xFF
        if key == ord('q'):
            self.get_logger().info('User quit')
            self.state = OrchardState.COMPLETED
        elif key == ord('s'):
            path = f'/tmp/orchard_{time.strftime("%H%M%S")}.jpg'
            cv2.imwrite(path, display)
            print(f'Saved: {path}')

    # ── 清理 ───────────────────────────────────────────

    def _shutdown(self):
        self.get_logger().info('Shutting down...')
        self.analyzer.release()
        cv2.destroyAllWindows()

        # 打印最终报告
        gs = self._session_summary
        total = max(gs['total'], 1)
        print('\n' + '=' * 50)
        print('  ORCHARD PATROL COMPLETE')
        print('=' * 50)
        print(f'  Rows scanned:    {len(self.row_stats)}')
        print(f'  Total apples:    {gs["total"]}')
        print(f'  Good:            {gs["good"]}  ({gs["good"]/total*100:.1f}%)')
        print(f'  Bad:             {gs["bad"]}  ({gs["bad"]/total*100:.1f}%)')
        print(f'  Unripe:          {gs["unripe"]}  ({gs["unripe"]/total*100:.1f}%)')
        print(f'  Good/Bad ratio:  {gs["good"]}:{gs["bad"]}')
        print('=' * 50)

        # 保存本地报告
        report_path = f'/tmp/orchard_report_{time.strftime("%Y%m%d_%H%M%S")}.json'
        try:
            report = {
                'timestamp': time.strftime('%Y-%m-%dT%H:%M:%SZ', time.gmtime()),
                'orchard_rows': self.orchard_rows,
                'session_summary': gs,
                'row_details': [
                    {'row': r['row'], 'summary': r['summary'],
                     'location': r['location']}
                    for r in self.row_stats
                ],
            }
            with open(report_path, 'w') as f:
                json.dump(report, f, ensure_ascii=False, indent=2)
            print(f'\nLocal report saved: {report_path}')
        except Exception as e:
            print(f'Failed to save local report: {e}')

        self.get_logger().info('Orchard patrol done.')


# ─── main ─────────────────────────────────────────────────
def main():
    rclpy.init(args=sys.argv)
    node = OrchardPatrolNode()
    node.run()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
