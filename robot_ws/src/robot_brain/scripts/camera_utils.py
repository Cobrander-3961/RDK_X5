#!/usr/bin/env python3
"""相机统一接口: 自动检测 双目 或 Astra Pro RGB-D"""
import cv2, numpy as np, time

class CameraWrapper:
    """自动适配双目 / Astra Pro RGB-D 摄像头"""
    def __init__(self, cam_id=0):
        self.cap = None
        self.mode = 'rgb'     # 'stereo' | 'rgbd' | 'rgb'
        self.depth_reader = None

        # 尝试打开摄像头
        for i in range(4):
            c = cv2.VideoCapture(i)
            if c.isOpened():
                c.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
                c.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
                ret, f = c.read()
                if ret:
                    self.cap = c
                    w = f.shape[1]
                    # 判断类型: 双目合成图 (左右差异大), 单目RGB
                    if w == 640:
                        left = f[:, :320]; right = f[:, 320:]
                        diff = cv2.absdiff(left, right).mean()
                        if diff > 10:
                            self.mode = 'stereo'
                            print(f"[Cam] Binocular stereo on /dev/video{i}")
                            break
                    self.mode = 'rgb'
                    print(f"[Cam] RGB camera on /dev/video{i}")
                    break
                c.release()
        if self.cap is None:
            raise RuntimeError("No camera found!")

        # 尝试加载深度 SDK
        self._init_depth()

    def _init_depth(self):
        """尝试初始化 Astra Pro 深度流"""
        try:
            from pyorbbecsdk import Pipeline, Config, OBFormat
            self.pipeline = Pipeline()
            config = Config()
            config.enable_video_stream(OBFormat.RGB, 640, 480, 30)
            config.enable_video_stream(OBFormat.DEPTH, 640, 480, 30)
            self.pipeline.start(config)
            self.mode = 'rgbd'
            print("[Cam] Astra Pro RGB-D mode")
            return
        except ImportError:
            pass
        except Exception as e:
            pass
        print("[Cam] Depth not available, RGB only mode")

    def read(self):
        """返回 (rgb_image, depth_map_mm_or_None)"""
        if self.mode == 'stereo':
            ret, frame = self.cap.read()
            if not ret: return None, None
            hw = frame.shape[1]//2
            left = frame[:,:hw]
            right = frame[:,hw:]
            return left, None  # 深度需要外部 SGBM 计算
        elif self.mode == 'rgbd':
            try:
                frames = self.pipeline.wait_for_frames(100)
                rgb = frames.get_color_frame()
                depth = frames.get_depth_frame()
                if rgb is None: return None, None
                rgb = np.frombuffer(rgb.get_data(), dtype=np.uint8).reshape(480,640,3).copy()
                rgb = cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)
                if depth is not None:
                    d = np.frombuffer(depth.get_data(), dtype=np.uint16).reshape(480,640)
                else:
                    d = None
                return rgb, d
            except:
                return None, None
        else:  # rgb only
            ret, frame = self.cap.read()
            if not ret: return None, None
            return frame, None

    def get_distance(self, depth_map, x, y, r=5):
        """深度图中(x,y)处的中位距离 (mm→m)"""
        if depth_map is None: return -1
        h, w = depth_map.shape
        x1, x2 = max(x-r,0), min(x+r,w-1)
        y1, y2 = max(y-r,0), min(y+r,h-1)
        roi = depth_map[y1:y2, x1:x2].astype(float)
        roi = roi[(roi>100)&(roi<10000)]  # 0.1m~10m
        return float(np.median(roi))/1000.0 if len(roi)>0 else -1

    def release(self):
        if self.cap: self.cap.release()
        if hasattr(self, 'pipeline'):
            self.pipeline.stop()
