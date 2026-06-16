#!/usr/bin/env python3
"""
YOLO 目标检测 + 深度估计 + 稳定性过滤 — 可导入模块, 也可独立测试。

从 auto_grasp_full.py 提取核心检测循环, 封装为 Detector 类。

独立测试:
  python3 detector.py              # GUI 模式 (cv2.imshow)
  python3 detector.py --headless   # 无头模式 (stdout 打印)
"""
import cv2
import numpy as np
import time
import sys
import os
from collections import deque
from dataclasses import dataclass, field

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from camera_utils import CameraWrapper


@dataclass
class DetectionResult:
    """单帧检测结果。"""
    class_name: str = None         # 目标类别名
    cx: int = 0                    # 边界框中心 x (图像坐标)
    cy: int = 0                    # 边界框中心 y
    x1: int = 0                    # 边界框左上 x
    y1: int = 0                    # 边界框左上 y
    x2: int = 0                    # 边界框右下 x
    y2: int = 0                    # 边界框右下 y
    distance_m: float = -1.0       # 距离 (m), -1 表示未知
    confidence: float = 0.0        # YOLO 置信度
    is_stable: bool = False        # 是否达到稳定性阈值
    timestamp: float = field(default_factory=time.time)


class Detector:
    """YOLO 目标检测器, 集成深度估计和稳定性过滤。

    设计为可独立使用, 也可被 integrated_patrol.py 组合。
    """

    DEFAULT_TARGETS = {'mouse', 'bottle', 'cup', 'cell phone',
                       'book', 'banana', 'apple'}

    def __init__(self,
                 model_path='/home/cobrander/RDK_X5/yolov8n.pt',
                 confidence_threshold=0.3,
                 target_classes=None,
                 stability_window=5,
                 stability_required=4,
                 grasp_cooldown_frames=30,
                 camera=None,
                 headless=False):
        """
        Args:
            model_path:              YOLO 权重文件路径
            confidence_threshold:    最低置信度阈值
            target_classes:          目标类别集合 (None → 使用 DEFAULT_TARGETS)
            stability_window:        滑动窗口大小 (帧数)
            stability_required:      连续检测到同一目标才判定稳定
            grasp_cooldown_frames:   抓取后冷却帧数 (~3s @ 10fps)
            camera:                  外部 CameraWrapper 实例 (共享), None 则自动创建
            headless:                True 时不调用 cv2.imshow
        """
        # 延迟导入 ultralytics (避免无 ROS2 环境导入失败)
        from ultralytics import YOLO

        print(f'[Detector] Loading YOLO from {model_path}...')
        self.model = YOLO(model_path)
        self.model.conf = confidence_threshold
        self.confidence_threshold = confidence_threshold
        self.target_classes = target_classes if target_classes is not None else self.DEFAULT_TARGETS
        print(f'[Detector] Targets: {sorted(self.target_classes)}')
        print('[Detector] YOLO ready!')

        # 相机
        self._own_camera = camera is None
        self.camera = camera if camera is not None else CameraWrapper()

        # 稳定性检测
        self._detect_history = deque(maxlen=stability_window)
        self._stability_required = min(stability_required, stability_window)

        # 抓取冷却
        self._grasp_cooldown = 0
        self._cooldown_frames = grasp_cooldown_frames

        # 当前帧缓存
        self._current_frame = None
        self._current_depth = None
        self._image_width = 640

        self.headless = headless
        self._fps_t = time.time()
        self._fps = 0.0

    # ── 核心接口 ───────────────────────────────────────

    def detect(self) -> DetectionResult:
        """运行一帧检测: 拍照 → YOLO → 深度 → 稳定性更新。

        Returns:
            DetectionResult — class_name=None 表示无有效检测
        """
        # 1. 拍照
        img, depth = self.camera.read()
        if img is None:
            return DetectionResult()

        self._current_frame = img
        self._current_depth = depth
        h, w = img.shape[:2]
        self._image_width = w

        # 2. YOLO 推理
        results = self.model(img, verbose=False)[0]
        closest_class = None
        min_dist = 999.0
        closest_box = None  # (cx, cy, x1, y1, x2, y2, conf)

        for box in results.boxes:
            cls = self.model.names[int(box.cls)]
            conf = float(box.conf)
            if conf < self.confidence_threshold or cls not in self.target_classes:
                continue

            x1, y1, x2, y2 = map(int, box.xyxy[0].tolist())
            cx, cy = (x1 + x2) // 2, (y1 + y2) // 2
            bw, bh = x2 - x1, y2 - y1

            # 3. 深度估计
            if depth is not None:
                rr = max(min(bw, bh) // 8, 4)
                roi = depth[max(cy - rr, 0):min(cy + rr, h - 1),
                            max(cx - rr, 0):min(cx + rr, w - 1)]
                valid = roi[(roi > 100) & (roi < 10000)]
                dist = float(np.median(valid)) / 1000.0 if valid.size > 0 else -1.0
            else:
                # 无深度: 用边界框宽度估算 (假设目标 ~5cm)
                dist = 0.05 * 320.0 / max(bw, 1)
                dist = max(0.05, min(dist, 2.0))

            if 0.0 < dist < min_dist:
                min_dist = dist
                closest_class = cls
                closest_box = (cx, cy, x1, y1, x2, y2, conf)

        # 4. 稳定性更新
        self._detect_history.append(closest_class)
        hist_list = list(self._detect_history)
        is_stable = (
            closest_class is not None
            and len(hist_list) >= self._stability_required
            and all(d == closest_class for d in hist_list[-self._stability_required:])
        )

        # 5. 冷却递减
        if self._grasp_cooldown > 0:
            self._grasp_cooldown -= 1

        # 6. FPS
        now = time.time()
        self._fps = 0.9 / max(now - self._fps_t, 0.001)
        self._fps_t = now

        if closest_box is None:
            return DetectionResult(is_stable=False)

        return DetectionResult(
            class_name=closest_class,
            cx=closest_box[0], cy=closest_box[1],
            x1=closest_box[2], y1=closest_box[3],
            x2=closest_box[4], y2=closest_box[5],
            distance_m=min_dist,
            confidence=closest_box[6],
            is_stable=is_stable,
            image_width=w,
        )

    def is_auto_trigger(self, result: DetectionResult) -> bool:
        """检测结果是否满足自动抓取触发条件。

        条件: 稳定检测 + 冷却就绪 + 距离在可抓范围 (0.05m ~ 1.0m)
        """
        return (
            result.is_stable
            and result.class_name is not None
            and self._grasp_cooldown == 0
            and 0.05 < result.distance_m < 1.0
        )

    def reset_cooldown(self, frames=None):
        """重置冷却计数器。"""
        self._grasp_cooldown = frames if frames is not None else self._cooldown_frames

    # ── 可视化 ──────────────────────────────────────────

    def annotate_frame(self, result: DetectionResult):
        """在帧上绘制检测框、标签、距离。返回标注后的 BGR 图像。"""
        img = self._current_frame
        if img is None:
            return np.zeros((480, 640, 3), dtype=np.uint8)

        annotated = img.copy()

        if result.class_name is not None:
            x1, y1, x2, y2 = result.x1, result.y1, result.x2, result.y2
            in_range = 0.05 < result.distance_m < 1.0
            color = (0, 255, 0) if in_range else (0, 255, 255)

            cv2.rectangle(annotated, (x1, y1), (x2, y2), color, 2)
            label = f'{result.class_name} {result.confidence:.2f}'
            if result.distance_m > 0:
                label += f' {result.distance_m * 100:.0f}cm'
            if result.is_stable:
                label += ' STABLE'
            cv2.putText(annotated, label, (x1, y1 - 8),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)
            cv2.circle(annotated, (result.cx, result.cy), 4, (0, 255, 255), -1)

        return annotated

    def draw_status_overlay(self, annotated, status=''):
        """绘制 FPS、冷却状态等覆盖信息。"""
        cooldown_info = ''
        if self._grasp_cooldown > 0:
            cooldown_info = f' COOL({self._grasp_cooldown})'

        cv2.putText(annotated, f'FPS:{self._fps:.1f} {status}{cooldown_info}',
                    (5, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

    # ── 属性 ────────────────────────────────────────────

    @property
    def current_frame(self):
        """当前原始帧 (BGR)。"""
        return self._current_frame

    @property
    def fps(self):
        return self._fps

    @property
    def cooldown_remaining(self):
        return self._grasp_cooldown

    def release(self):
        """释放相机资源。"""
        if self._own_camera:
            self.camera.release()
        cv2.destroyAllWindows()


# ─── 独立测试 ─────────────────────────────────────────────
def main():
    import argparse
    parser = argparse.ArgumentParser(description='Detector 独立测试')
    parser.add_argument('--headless', action='store_true', help='无 GUI 模式')
    parser.add_argument('--model', default='/home/cobrander/RDK_X5/yolov8n.pt',
                        help='YOLO 模型路径')
    parser.add_argument('--conf', type=float, default=0.3, help='置信度阈值')
    args = parser.parse_args()

    detector = Detector(
        model_path=args.model,
        confidence_threshold=args.conf,
        headless=args.headless,
    )

    print('\n=== Detector Test ===')
    print('Targets:', sorted(detector.target_classes))
    if args.headless:
        print('Mode: headless (stdout only)')
    else:
        print('Mode: GUI (q=quit, g=grasp trigger test)')

    # 延迟导入 Grasper (独立测试时可选手动抓取)
    grasper = None

    try:
        frame_count = 0
        while True:
            result = detector.detect()
            frame_count += 1

            if args.headless:
                # 每秒打印一次状态
                if frame_count % 10 == 0:
                    if result.class_name:
                        print(f'[{detector.fps:.1f}fps] {result.class_name} '
                              f'{result.distance_m*100:.0f}cm '
                              f'conf={result.confidence:.2f} '
                              f'stable={result.is_stable} '
                              f'cooldown={detector.cooldown_remaining}')
                    else:
                        print(f'[{detector.fps:.1f}fps] (no target)')
            else:
                annotated = detector.annotate_frame(result)
                status = 'AUTO' if detector.cooldown_remaining == 0 else f'COOL'
                if result.is_stable:
                    status = 'STABLE'
                detector.draw_status_overlay(annotated, status)

                if result.is_stable and detector.is_auto_trigger(result):
                    cv2.putText(annotated,
                                f'TRIGGER: {result.class_name} {result.distance_m*100:.0f}cm',
                                (5, annotated.shape[0] - 10),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)

                cv2.imshow('Detector Test', annotated)
                key = cv2.waitKey(1) & 0xFF

                if key == ord('q'):
                    break
                elif key == ord('g') and result.class_name:
                    # 手动触发抓取 (测试用)
                    if grasper is None:
                        from grasper import Grasper
                        grasper = Grasper()
                    j1 = max(-1.2, min(1.2,
                              (result.cx - result.image_width / 2) / result.image_width * 0.6))
                    print(f'\n[Manual] Grasp trigger! {result.class_name} '
                          f'j1={j1:.2f}')
                    grasper.execute_grasp_sequence(j1)
                    detector.reset_cooldown()

    except KeyboardInterrupt:
        print('\nInterrupted')

    finally:
        detector.release()
        if grasper:
            grasper.close()
        print('Detector test done')


if __name__ == '__main__':
    main()
