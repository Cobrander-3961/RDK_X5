#!/usr/bin/env python3
"""
苹果质量分析器 — 基于 HSV 颜色空间的 好果/坏果/未熟 分类。

原理:
  YOLO 检测 apple → 裁剪 ROI → HSV 像素统计:
    - 红色 (H:0-10 / 160-180, S>100)    → "好果" 特征
    - 褐色/暗斑 (V<60 或 S<30 暗区)      → "坏果" 特征 (腐烂/虫蛀)
    - 绿色 (H:35-85, S>50)              → "未熟" 特征

  判定规则:
    好果: 红色占比 > 40% 且 暗斑占比 < 20%
    坏果: 暗斑占比 > 25% 或 颜色总面积 < 30%
    未熟: 绿色占比 > 30%
    未知: 以上均不满足

独立测试:
  python3 fruit_analyzer.py              # GUI 模式
  python3 fruit_analyzer.py --headless   # 仅 stdout 统计
"""
import cv2
import numpy as np
import sys
import os
import time
from dataclasses import dataclass, field

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from detector import Detector, DetectionResult
from camera_utils import CameraWrapper


@dataclass
class FruitQuality:
    """单颗苹果的质量分析结果。"""
    class_name: str = 'apple'
    quality: str = 'unknown'     # 'good' | 'bad' | 'unripe' | 'unknown'
    good_ratio: float = 0.0      # 红色像素占比
    bad_ratio: float = 0.0       # 暗斑像素占比
    unripe_ratio: float = 0.0    # 绿色像素占比
    confidence: float = 0.0      # YOLO 置信度
    bbox: tuple = (0, 0, 0, 0)  # (x1, y1, x2, y2)
    distance_m: float = -1.0     # 距离 (m)
    timestamp: float = field(default_factory=time.time)


class FruitAnalyzer:
    """苹果质量分析器 — 组合 Detector, 在 YOLO 检测基础上做 HSV 分类。

    可与 Detector 共享 CameraWrapper, 避免重复打开摄像头。
    """

    # HSV 阈值 (可调)
    RED_LOWER1  = (0,   100, 50)    # 红色下界 1 (低 hue)
    RED_UPPER1  = (10,  255, 255)   # 红色上界 1
    RED_LOWER2  = (160, 100, 50)    # 红色下界 2 (高 hue, 绕回)
    RED_UPPER2  = (180, 255, 255)   # 红色上界 2
    GREEN_LOWER = (35,  50,  50)    # 绿色下界
    GREEN_UPPER = (85,  255, 255)   # 绿色上界
    # 暗斑: 低饱和度 (S<30) 或 低明度 (V<60)

    # 判定阈值
    GOOD_RED_MIN    = 0.40   # 好果至少 40% 红色
    BAD_DARK_MAX    = 0.20   # 好果最多 20% 暗斑
    BAD_DARK_MIN    = 0.25   # 坏果最少 25% 暗斑
    BAD_COLOR_MIN   = 0.30   # 总颜色占比低于此 = 坏果
    UNRIPE_GREEN_MIN = 0.30  # 未熟果绿色最少 30%

    def __init__(self, detector=None, camera=None, headless=False):
        """复用已有 Detector, 或自动创建。

        Args:
            detector: 已有 Detector 实例 (共享相机)
            camera:   已有 CameraWrapper (detector 为 None 时创建)
            headless: 无 GUI 模式
        """
        if detector is not None:
            self.detector = detector
            self._own_detector = False
        else:
            self.detector = Detector(
                target_classes={'apple'},
                camera=camera,
                headless=True,
            )
            self._own_detector = True

        self.headless = headless
        self._camera = self.detector.camera

    # ── 核心分析 ──────────────────────────────────────

    def analyze_frame(self):
        """一帧分析: YOLO 检测 apple → 逐个 HSV 分类 → 去重统计。

        Returns:
            (qualities, summary_dict)
            qualities:      list[FruitQuality] — 每颗苹果的质量
            summary_dict:   {'total': int, 'good': int, 'bad': int,
                             'unripe': int, 'unknown': int}
        """
        result = self.detector.detect()
        frame = self.detector.current_frame
        if frame is None:
            return [], {'total': 0, 'good': 0, 'bad': 0, 'unripe': 0, 'unknown': 0}

        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        h, w = frame.shape[:2]
        qualities = []

        # 用 YOLO 结果中的 apple 检测 (需要从原始 results 获取所有检测)
        # 因为 Detector.detect() 只返回最近的目标, 这里需要遍历全部 apple
        # 直接调用 YOLO 获取所有检测
        results = self.detector.model(frame, verbose=False)[0]
        for box in results.boxes:
            cls = self.detector.model.names[int(box.cls)]
            if cls != 'apple':
                continue
            conf = float(box.conf)
            if conf < self.detector.confidence_threshold:
                continue

            x1, y1, x2, y2 = map(int, box.xyxy[0].tolist())
            # 边界检查
            x1, y1 = max(0, x1), max(0, y1)
            x2, y2 = min(w, x2), min(h, y2)
            if x2 <= x1 or y2 <= y1:
                continue

            roi = hsv[y1:y2, x1:x2]
            if roi.size == 0:
                continue

            # 像素统计
            total_px = roi.shape[0] * roi.shape[1]

            # 红色像素 (两个 hue 范围)
            red_mask1 = cv2.inRange(roi, self.RED_LOWER1, self.RED_UPPER1)
            red_mask2 = cv2.inRange(roi, self.RED_LOWER2, self.RED_UPPER2)
            red_px = cv2.countNonZero(red_mask1) + cv2.countNonZero(red_mask2)

            # 绿色像素
            green_mask = cv2.inRange(roi, self.GREEN_LOWER, self.GREEN_UPPER)
            green_px = cv2.countNonZero(green_mask)

            # 暗斑: S<30 (低饱和度=灰/褐) 或 V<60 (暗色=腐烂)
            s_channel = roi[:, :, 1]
            v_channel = roi[:, :, 2]
            dark_s = s_channel < 30
            dark_v = v_channel < 60
            dark_px = np.count_nonzero(dark_s | dark_v)

            good_r = red_px / total_px
            bad_r = dark_px / total_px
            unripe_r = green_px / total_px

            # 质量判定
            quality = self._classify(good_r, bad_r, unripe_r)

            # 距离 (从 Detector 的深度图)
            cx, cy = (x1 + x2) // 2, (y1 + y2) // 2
            dist = -1.0
            if self._camera is not None:
                try:
                    dist = self._camera.get_distance(
                        self.detector._current_depth, cx, cy)
                except Exception:
                    pass

            qualities.append(FruitQuality(
                quality=quality,
                good_ratio=round(good_r, 3),
                bad_ratio=round(bad_r, 3),
                unripe_ratio=round(unripe_r, 3),
                confidence=round(conf, 3),
                bbox=(x1, y1, x2, y2),
                distance_m=round(dist, 2),
            ))

        # 汇总
        summary = {'total': len(qualities), 'good': 0, 'bad': 0,
                   'unripe': 0, 'unknown': 0}
        for q in qualities:
            summary[q.quality] += 1

        return qualities, summary

    def _classify(self, good_r, bad_r, unripe_r):
        """根据颜色占比判定质量标签。"""
        if good_r >= self.GOOD_RED_MIN and bad_r <= self.BAD_DARK_MAX:
            return 'good'
        if bad_r >= self.BAD_DARK_MIN:
            return 'bad'
        if (good_r + unripe_r) < self.BAD_COLOR_MIN:
            return 'bad'
        if unripe_r >= self.UNRIPE_GREEN_MIN:
            return 'unripe'
        # fallback: 选择占比最高的
        best = max(('good', good_r), ('bad', bad_r), ('unripe', unripe_r),
                   key=lambda x: x[1])
        return best[0] if best[1] > 0.1 else 'unknown'

    # ── 可视化 ─────────────────────────────────────────

    def annotate_frame(self, qualities, frame=None):
        """绘制质量标注: 绿色框=好果, 红色框=坏果, 黄色框=未熟。"""
        if frame is None:
            frame = self.detector.current_frame
        if frame is None:
            return np.zeros((480, 640, 3), dtype=np.uint8)

        annotated = frame.copy()
        quality_colors = {
            'good':    (0, 255, 0),    # 绿色
            'bad':     (0, 0, 255),    # 红色
            'unripe':  (0, 255, 255),  # 黄色
            'unknown': (128, 128, 128), # 灰色
        }

        for q in qualities:
            x1, y1, x2, y2 = q.bbox
            color = quality_colors.get(q.quality, (128, 128, 128))
            cv2.rectangle(annotated, (x1, y1), (x2, y2), color, 2)

            label = f'{q.quality} R:{q.good_ratio:.0%} B:{q.bad_ratio:.0%}'
            if q.distance_m > 0:
                label += f' {q.distance_m*100:.0f}cm'
            cv2.putText(annotated, label, (x1, y1 - 8),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.4, color, 1)

        return annotated

    def release(self):
        if self._own_detector:
            self.detector.release()


# ─── 独立测试 ─────────────────────────────────────────────
def main():
    import argparse
    parser = argparse.ArgumentParser(description='FruitAnalyzer 独立测试')
    parser.add_argument('--headless', action='store_true', help='无 GUI 模式')
    parser.add_argument('--model', default='/home/sunrise/RDK_X5/yolov8n.pt')
    parser.add_argument('--conf', type=float, default=0.2,
                        help='YOLO 置信度阈值 (苹果检测建议 0.2)')
    args = parser.parse_args()

    # 用较低的置信度阈值 (果园环境远距离检测)
    detector = Detector(
        model_path=args.model,
        confidence_threshold=args.conf,
        target_classes={'apple'},
        headless=args.headless,
    )
    analyzer = FruitAnalyzer(detector=detector, headless=args.headless)

    print('\n=== FruitAnalyzer Test ===')
    print('Quality: Green=good  Red=bad  Yellow=unripe  Gray=unknown')
    print('Keys: q=quit  s=save snapshot')

    total_summary = {'total': 0, 'good': 0, 'bad': 0, 'unripe': 0, 'unknown': 0}
    frame_count = 0

    try:
        while True:
            qualities, summary = analyzer.analyze_frame()
            frame_count += 1

            if args.headless:
                if frame_count % 15 == 0 and summary['total'] > 0:  # ~每1.5秒
                    ratio = summary['good'] / max(summary['total'], 1) * 100
                    print(f'Frame {frame_count}: total={summary["total"]} '
                          f'good={summary["good"]} bad={summary["bad"]} '
                          f'unripe={summary["unripe"]} ({ratio:.0f}% good)')
            else:
                annotated = analyzer.annotate_frame(qualities)

                # 汇总覆盖
                y0 = 25
                for key, label in [('total', 'Total'), ('good', 'Good'),
                                   ('bad', 'Bad'), ('unripe', 'Unripe')]:
                    cv2.putText(annotated, f'{label}: {summary[key]}',
                                (5, y0), cv2.FONT_HERSHEY_SIMPLEX, 0.45,
                                (255, 255, 255), 1)
                    y0 += 20

                # FPS
                fps = analyzer.detector.fps
                cv2.putText(annotated, f'FPS:{fps:.1f}',
                            (5, y0 + 5), cv2.FONT_HERSHEY_SIMPLEX, 0.4,
                            (200, 200, 200), 1)

                cv2.imshow('Fruit Analyzer', annotated)
                key = cv2.waitKey(1) & 0xFF
                if key == ord('q'):
                    break
                elif key == ord('s'):
                    path = f'/tmp/orchard_{time.strftime("%H%M%S")}.jpg'
                    cv2.imwrite(path, annotated)
                    print(f'Saved: {path}')

            # 累计全局统计
            for k in total_summary:
                total_summary[k] += summary[k]

    except KeyboardInterrupt:
        print('\nInterrupted')

    finally:
        print(f'\n=== Final Summary ===')
        total = max(total_summary['total'], 1)
        print(f'  Total apples detected: {total_summary["total"]}')
        print(f'  Good:   {total_summary["good"]:>4}  ({total_summary["good"]/total*100:.1f}%)')
        print(f'  Bad:    {total_summary["bad"]:>4}  ({total_summary["bad"]/total*100:.1f}%)')
        print(f'  Unripe: {total_summary["unripe"]:>4}  ({total_summary["unripe"]/total*100:.1f}%)')
        analyzer.release()
        print('FruitAnalyzer test done')


if __name__ == '__main__':
    main()
