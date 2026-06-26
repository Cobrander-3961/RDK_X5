#!/usr/bin/env python3
"""YOLO 纯测试 — Astra Pro + YOLOv8n (torch)"""
import cv2, numpy as np, time, sys, os, torch

def main():
    # 加载 YOLOv8n (本地 pt 文件, 需要 ultralytics 包)
    try:
        from ultralytics import YOLO
        model = YOLO("/home/sunrise/RDK_X5/yolov8n.pt")
        model.conf = 0.3
        print("YOLOv8n loaded (ultralytics)")
        use_ultra = True
    except ImportError:
        print("ultralytics not installed, trying torch...")
        sys.exit(1)

    # 找摄像头
    cap = None
    for i in range(4):
        c = cv2.VideoCapture(i)
        if c.isOpened():
            c.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
            c.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
            ret, _ = c.read()
            if ret: cap = c; break
            c.release()
    if cap is None: print("No camera!"); return

    print(f"Camera {i} | Q=quit")
    fps_t = time.time()

    while True:
        ret, frame = cap.read()
        if not ret: break

        results = model(frame, verbose=False)[0]
        for box in results.boxes:
            if float(box.conf) < 0.3: continue
            x1,y1,x2,y2 = map(int, box.xyxy[0].tolist())
            cls = model.names[int(box.cls)]
            conf = float(box.conf)
            cv2.rectangle(frame, (x1,y1), (x2,y2), (0,255,0), 2)
            cv2.putText(frame, f"{cls} {conf:.2f}", (x1,y1-8),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,255,0), 2)

        fps = 0.9/max(time.time()-fps_t, 0.001); fps_t = time.time()
        cv2.putText(frame, f"FPS:{fps:.1f}", (5,20),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,255,0), 2)
        cv2.imshow("YOLO Test", frame)
        if cv2.waitKey(1)&0xFF == ord('q'): break

    cap.release(); cv2.destroyAllWindows()

if __name__=="__main__": main()
