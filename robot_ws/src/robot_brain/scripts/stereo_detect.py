#!/usr/bin/env python3
"""
双目摄像头 + 矿泉水瓶识别 + 可调参测距。

依赖: opencv-python, numpy
用法: python3 stereo_detect.py [camera_id]
     默认 camera_id=2 (3D Webcam)
     按 'q' 退出, 's' 保存快照, 拖动滑块实时调参
"""
import cv2, numpy as np, sys, time


# ─── 可调参数 (通过滑块实时调整) ───
class Params:
    def __init__(self):
        self.baseline = 0.06     # 基线 (m)
        self.focal = 700.0       # 焦距 (pixels)
        self.num_disp = 64       # 最大视差 (16的倍数)
        self.block_size = 5      # SGBM 块大小
        self.uniqueness = 10     # 唯一性

    def create_trackbars(self):
        cv2.namedWindow("Controls")
        cv2.createTrackbar("Baseline(mm)", "Controls", 60, 200, lambda v: setattr(self, 'baseline', v / 1000.0))
        cv2.createTrackbar("Focal(pix)",  "Controls", 700, 2000, lambda v: setattr(self, 'focal', float(v)))
        cv2.createTrackbar("NumDisp(x16)","Controls", 4, 16, lambda v: setattr(self, 'num_disp', max(v * 16, 16)))
        cv2.createTrackbar("BlockSize",    "Controls", 5, 25, lambda v: setattr(self, 'block_size', v | 1))  # odd
        cv2.createTrackbar("UniqueRatio",  "Controls", 10, 50, lambda v: setattr(self, 'uniqueness', v))
        cv2.resizeWindow("Controls", 400, 200)


# ─── 鼠标检测 ───
class MouseDetector:
    # 深色/黑色鼠标主体
    DARK_LOW  = np.array([0,   0,   0])
    DARK_HIGH = np.array([180, 255, 80])
    # 红色滚轮/灯光 (游戏鼠标常有)
    RED_LOW   = np.array([0,   100, 100])
    RED_HIGH  = np.array([10,  255, 255])
    RED2_LOW  = np.array([160, 100, 100])
    RED2_HIGH = np.array([180, 255, 255])
    MIN_AREA   = 300

    @classmethod
    def detect(cls, image):
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        mask = cv2.bitwise_or(
            cv2.inRange(hsv, cls.DARK_LOW, cls.DARK_HIGH),
            cv2.bitwise_or(
                cv2.inRange(hsv, cls.RED_LOW, cls.RED_HIGH),
                cv2.inRange(hsv, cls.RED2_LOW, cls.RED2_HIGH)))
        k = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5,5))
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, k)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, k, iterations=2)
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        results = []
        for cnt in contours:
            if cv2.contourArea(cnt) < cls.MIN_AREA: continue
            x, y, w, h = cv2.boundingRect(cnt)
            results.append((x, y, w, h, x + w // 2, y + h // 2))
        return results


# ─── 主程序 ───
def main():
    cam_id = int(sys.argv[1]) if len(sys.argv) > 1 else 2
    cap = cv2.VideoCapture(cam_id)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

    if not cap.isOpened():
        print(f"Camera {cam_id} not found"); return

    p = Params()
    p.create_trackbars()

    print("Stereo Mouse Detector | 'q'=quit 's'=save | Drag sliders to tune")
    fps_t = time.time()

    while True:
        ret, frame = cap.read()
        if not ret: break
        h, w = frame.shape[:2]

        # ── 拆分左右图 (640×480 → 640×240 → 320×240 ×2, 与官方教程一致) ──
        double_img = cv2.resize(frame, (640, 240))
        left  = double_img[:, :320]
        right = double_img[:, 320:]

        # ── SGBM 视差图 ──
        stereo = cv2.StereoSGBM_create(
            minDisparity=0, numDisparities=p.num_disp,
            blockSize=p.block_size, uniquenessRatio=p.uniqueness,
            speckleWindowSize=100, speckleRange=2,
            disp12MaxDiff=1, P1=8*3*p.block_size**2, P2=32*3*p.block_size**2)
        gray_l = cv2.cvtColor(left, cv2.COLOR_BGR2GRAY)
        gray_r = cv2.cvtColor(right, cv2.COLOR_BGR2GRAY)
        disp = stereo.compute(gray_l, gray_r).astype(np.float32) / 16.0
        disp[disp < 0.5] = 0.5

        # ── 距离图: Z = f * B / d ──
        dist_map = (p.focal * p.baseline) / disp
        dist_map = cv2.medianBlur(dist_map.astype(np.float32), 5)

        # ── 瓶子检测 ──
        bottles = MouseDetector.detect(left)
        result = left.copy()

        for bbox in bottles:
            x, y, wb, hb, cx, cy = bbox
            # 中心区域中位距离
            rx = max(cx - wb//8, 0); rx2 = min(cx + wb//8, dist_map.shape[1]-1)
            ry = max(cy - hb//8, 0); ry2 = min(cy + hb//8, dist_map.shape[0]-1)
            roi = dist_map[ry:ry2, rx:rx2]
            dist = float(np.median(roi[(roi > 0.01) & (roi < 10.0)])) if roi.size > 0 else -1

            color = (0, 255, 0) if 0 < dist < 3.0 else (0, 0, 255)
            cv2.rectangle(result, (x, y), (x+wb, y+hb), color, 2)
            label = f"{dist*100:.0f}cm" if dist > 0 else "?"
            cv2.putText(result, label, (x, y-10), cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2)
            cv2.circle(result, (cx, cy), 3, (0, 255, 255), -1)
            # 串口输出
            if dist > 0:
                print(f"\rMouse: ({cx},{cy}) {dist*100:.1f}cm     ", end="")

        # ── 显示 ──
        disp_viz = cv2.applyColorMap(
            cv2.normalize(disp, None, 0, 255, cv2.NORM_MINMAX).astype(np.uint8),
            cv2.COLORMAP_JET)
        dist_viz = cv2.applyColorMap(
            np.clip(dist_map / 3.0 * 255, 0, 255).astype(np.uint8),
            cv2.COLORMAP_TURBO)

        # 布局: 左上=左视图 右上=右视图 左下=检测结果 右下=距离图
        lr = left; rr = right
        dr = cv2.resize(result, (320, 240)); dv = cv2.resize(dist_viz, (320, 240))
        top_row = np.hstack([lr, rr])
        bot_row = np.hstack([dr, dv])
        display = np.vstack([top_row, bot_row])

        # FPS
        fps = 0.9 * (1.0 / max(time.time() - fps_t, 0.001))
        fps_t = time.time()
        cv2.putText(display, f"FPS:{fps:.1f} B={p.baseline:.3f}m F={p.focal:.0f}",
                    (5, 15), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0,255,0), 1)
        cv2.imshow("Stereo Mouse Detector", display)

        key = cv2.waitKey(1) & 0xFF
        if key == ord('q'): break
        elif key == ord('s'):
            cv2.imwrite("/tmp/left.jpg", left)
            cv2.imwrite("/tmp/right.jpg", right)
            print("\nSnapshot saved to /tmp/left.jpg /tmp/right.jpg")

    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
