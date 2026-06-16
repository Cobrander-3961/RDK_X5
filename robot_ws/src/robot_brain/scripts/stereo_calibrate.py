#!/usr/bin/env python3
"""
深目双目摄像头标定 — 棋盘格法。

拍棋盘格 20+ 张 → stereoCalibrate() → 保存内参+外参到 YAML。

打印准备: 9×6 内角点, 方格 25mm, A4 纸贴硬板
操作: 手持棋盘格在相机前方变换角度 (正视/侧视/旋转/远近)
      检测到角点时自动保存, 空格键手动保存
      收集 20+ 对图像后按 'c' 标定, 按 'q' 退出

输出: config/stereo_calib.yaml
  - camera_matrix (左目内参)
  - dist_coeffs  (左目畸变)
  - R, T         (右目相对左目的外参)
  - baseline_m   (双目基线 米)
"""
import cv2
import numpy as np
import os
import sys
import time
import yaml

# ─── 配置 ────────────────────────────────────────────
CHESSBOARD = (9, 6)        # 内角点数 (列, 行)
SQUARE_SIZE = 0.025        # 方格边长 (米) — 25mm
MIN_SAMPLES = 20           # 最少采集数量
OUTPUT_DIR = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), 'config')
OUTPUT_FILE = os.path.join(OUTPUT_DIR, 'stereo_calib.yaml')

# 棋盘格 3D 坐标 (每个内角点的世界坐标, z=0)
OBJ_POINTS = np.zeros((CHESSBOARD[0] * CHESSBOARD[1], 3), np.float32)
OBJ_POINTS[:, :2] = np.mgrid[0:CHESSBOARD[0], 0:CHESSBOARD[1]].T.reshape(-1, 2)
OBJ_POINTS *= SQUARE_SIZE


def open_stereo_camera():
    """打开深目双目摄像头 (1280×480 → 左640×480 + 右640×480)。"""
    for i in range(4):
        cap = cv2.VideoCapture(i)
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
        ret, frame = cap.read()
        if ret and frame.shape[1] == 1280:
            print(f'[Camera] Stereo camera on /dev/video{i}')
            return cap, i
        cap.release()

    # fallback: try 640×480
    for i in range(4):
        cap = cv2.VideoCapture(i)
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
        ret, frame = cap.read()
        if ret:
            print(f'[Camera] Mono camera on /dev/video{i} (mono calibration mode)')
            return cap, i
        cap.release()

    raise RuntimeError('No camera found!')


def detect_corners(frame, criteria):
    """检测棋盘格角点, 返回 (corners, gray)。"""
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    ret, corners = cv2.findChessboardCorners(gray, CHESSBOARD, None)
    if ret:
        corners_sub = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)
        return corners_sub, gray
    return None, gray


def main():
    print('=== 深目双目摄像头标定 ===')
    print(f'棋盘格: {CHESSBOARD[0]}×{CHESSBOARD[1]} 内角点, {SQUARE_SIZE*1000:.0f}mm 方格')
    print('操作: 手持棋盘格变换角度 → 自动保存 → 按 c 标定 → 按 q 退出')
    print(f'最少 {MIN_SAMPLES} 对图像, 建议 25-30 对\n')

    cap, cam_id = open_stereo_camera()
    stereo_mode = cap.get(cv2.CAP_PROP_FRAME_WIDTH) == 1280

    criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)

    obj_points = []   # 3D 世界坐标
    img_points_l = []  # 左目 2D 角点
    img_points_r = []  # 右目 2D 角点
    auto_cooldown = 0

    print(f'{"Stereo" if stereo_mode else "Mono"} mode — start collecting...\n')

    while True:
        ret, frame = cap.read()
        if not ret:
            break

        if stereo_mode:
            hw = frame.shape[1] // 2
            left = frame[:, :hw]
            right = frame[:, hw:]
        else:
            left = frame
            right = None

        # 每 10 帧检测一次 (降低 CPU)
        if auto_cooldown > 0:
            auto_cooldown -= 1

        corners_l = None
        corners_r = None

        if auto_cooldown == 0:
            corners_l, gray_l = detect_corners(left, criteria)
            if stereo_mode:
                corners_r, gray_r = detect_corners(right, criteria)

            if corners_l is not None and (not stereo_mode or corners_r is not None):
                auto_cooldown = 10  # 自动保存后冷却 10 帧

        # 显示
        display_l = left.copy()
        if corners_l is not None:
            cv2.drawChessboardCorners(display_l, CHESSBOARD, corners_l, True)

        if stereo_mode:
            display_r = right.copy()
            if corners_r is not None:
                cv2.drawChessboardCorners(display_r, CHESSBOARD, corners_r, True)
            display = np.hstack([display_l, display_r])
        else:
            display = display_l

        n = len(obj_points)
        cv2.putText(display, f'Samples: {n}', (5, 20),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
        if n >= MIN_SAMPLES:
            cv2.putText(display, 'READY — press c to calibrate', (5, 45),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 1)
        cv2.imshow('Stereo Calibration', cv2.resize(display, (960, 360)))

        key = cv2.waitKey(1) & 0xFF

        if key == ord('q'):
            break
        elif key == ord(' ') and corners_l is not None:
            # 手动保存
            obj_points.append(OBJ_POINTS)
            img_points_l.append(corners_l)
            if stereo_mode and corners_r is not None:
                img_points_r.append(corners_r)
            print(f'  Saved #{len(obj_points)} (manual)')
        elif key == ord('c') and n >= MIN_SAMPLES:
            break

        # 自动保存 (检测到棋盘格时)
        if auto_cooldown == 9 and corners_l is not None:
            obj_points.append(OBJ_POINTS)
            img_points_l.append(corners_l)
            if stereo_mode and corners_r is not None:
                img_points_r.append(corners_r)
            print(f'  Auto-saved #{len(obj_points)}')

    cap.release()
    cv2.destroyAllWindows()

    n = len(obj_points)
    if n < MIN_SAMPLES:
        print(f'\nNot enough samples ({n} < {MIN_SAMPLES}), aborting')
        return

    print(f'\nCalibrating with {n} image pairs...')

    # 标定
    if stereo_mode and len(img_points_r) == n:
        # 双目标定
        print('Stereo calibration...')
        ret_l, mtx_l, dist_l, _, _ = cv2.calibrateCamera(
            obj_points, img_points_l, left.shape[:2][::-1], None, None)
        ret_r, mtx_r, dist_r, _, _ = cv2.calibrateCamera(
            obj_points, img_points_r, right.shape[:2][::-1], None, None)

        flags = cv2.CALIB_FIX_INTRINSIC
        criteria_stereo = (cv2.TERM_CRITERIA_MAX_ITER + cv2.TERM_CRITERIA_EPS, 100, 1e-5)
        ret, mtx_l, dist_l, mtx_r, dist_r, R, T, E, F = cv2.stereoCalibrate(
            obj_points, img_points_l, img_points_r,
            mtx_l, dist_l, mtx_r, dist_r,
            left.shape[:2][::-1],
            criteria=criteria_stereo, flags=flags)

        baseline = float(np.linalg.norm(T))
        print(f'\nStereo RMS error: {ret:.4f}')
        print(f'Left  camera: fx={mtx_l[0,0]:.1f} fy={mtx_l[1,1]:.1f} '
              f'cx={mtx_l[0,2]:.1f} cy={mtx_l[1,2]:.1f}')
        print(f'Right camera: fx={mtx_r[0,0]:.1f} fy={mtx_r[1,1]:.1f} '
              f'cx={mtx_r[0,2]:.1f} cy={mtx_r[1,2]:.1f}')
        print(f'Baseline: {baseline*1000:.1f}mm ({baseline:.4f}m)')

        result = {
            'image_width': left.shape[1],
            'image_height': left.shape[0],
            'camera_matrix_left': mtx_l.tolist(),
            'dist_coeffs_left': dist_l.tolist(),
            'camera_matrix_right': mtx_r.tolist(),
            'dist_coeffs_right': dist_r.tolist(),
            'R': R.tolist(),
            'T': T.tolist(),
            'baseline_m': float(baseline),
            'stereo_rms_error': float(ret),
            'num_samples': n,
        }
    else:
        # 单目标定
        print('Mono calibration...')
        ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(
            obj_points, img_points_l, left.shape[:2][::-1], None, None)

        print(f'RMS error: {ret:.4f}')
        print(f'Camera: fx={mtx[0,0]:.1f} fy={mtx[1,1]:.1f} '
              f'cx={mtx[0,2]:.1f} cy={mtx[1,2]:.1f}')

        result = {
            'image_width': left.shape[1],
            'image_height': left.shape[0],
            'camera_matrix': mtx.tolist(),
            'dist_coeffs': dist.tolist(),
            'mono_rms_error': float(ret),
            'num_samples': n,
            'note': 'mono calibration (no stereo pair detected)',
        }

    # 保存
    os.makedirs(OUTPUT_DIR, exist_ok=True)
    with open(OUTPUT_FILE, 'w') as f:
        yaml.dump(result, f, default_flow_style=False)
    print(f'\nSaved: {OUTPUT_FILE}')

    # 质量评估
    if ret > 0.5:
        print('\n⚠️  RMS误差偏高 (>0.5px). 建议:')
        print('  1. 增加更多斜视/旋转角度样本')
        print('  2. 确保棋盘格平展, 无反光')
        print('  3. 覆盖图像四角和边缘区域')
    else:
        print('\n✅ 标定质量良好')


if __name__ == '__main__':
    main()
