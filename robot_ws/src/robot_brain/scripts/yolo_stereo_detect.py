#!/usr/bin/env python3
"""YOLOv8 + 双目测距 (ultralytics 本地源码, 零下载)"""
import cv2, numpy as np, time, sys, os

# 本地 ultralytics 源码路径
sys.path.insert(0, '/home/cobrander/Share/ultralytics-main')

class Params:
    def __init__(self):
        self.baseline=0.06; self.focal=700.0; self.num_disp=64; self.block_size=5
    def create_trackbars(self):
        cv2.namedWindow("Params")
        cv2.createTrackbar("Baseline(mm)","Params",60,200,lambda v:setattr(self,'baseline',v/1000))
        cv2.createTrackbar("Focal(pix)","Params",700,2000,lambda v:setattr(self,'focal',float(v)))
        cv2.createTrackbar("NumDisp(x16)","Params",4,16,lambda v:setattr(self,'num_disp',max(v*16,16)))
        cv2.resizeWindow("Params",350,120)

def main():
    from ultralytics import YOLO
    print("Loading YOLOv8n...")
    model = YOLO("yolov8n.pt")  # 首次自动下载权重 (6MB)
    print("YOLOv8n ready!")

    # 自动查找 3D Webcam, 支持手动指定
    if len(sys.argv) > 1:
        cam_id = int(sys.argv[1])
        cap = cv2.VideoCapture(cam_id)
    else:
        cap = None
        for i in range(8):
            c = cv2.VideoCapture(i)
            if c.isOpened():
                c.set(cv2.CAP_PROP_FRAME_WIDTH,640); c.set(cv2.CAP_PROP_FRAME_HEIGHT,480)
                ret, _ = c.read()
                if ret: cap = c; print(f"Auto-detected camera {i}"); break
                c.release()
    if cap is None or not cap.isOpened(): print("No camera found!"); return
    cap.set(cv2.CAP_PROP_FRAME_WIDTH,640); cap.set(cv2.CAP_PROP_FRAME_HEIGHT,480)

    p = Params(); p.create_trackbars()
    print("YOLOv8 Stereo | q=quit s=save")
    fps_t = time.time()

    while True:
        ret, frame = cap.read()
        if not ret: break
        hw, fh = frame.shape[1]//2, frame.shape[0]
        left480, right480 = frame[:,:hw], frame[:,hw:]
        left240 = cv2.resize(left480, (320,240))

        # SGBM 视差
        gl = cv2.cvtColor(left240, cv2.COLOR_BGR2GRAY)
        gr = cv2.cvtColor(cv2.resize(right480, (320,240)), cv2.COLOR_BGR2GRAY)
        s = cv2.StereoSGBM_create(minDisparity=0, numDisparities=p.num_disp,
            blockSize=p.block_size, uniquenessRatio=10, speckleWindowSize=100,
            speckleRange=2, disp12MaxDiff=1, P1=8*3*p.block_size**2, P2=32*3*p.block_size**2)
        disp = s.compute(gl, gr).astype(np.float32)/16.0; disp[disp<0.5]=0.5
        dist_map = cv2.medianBlur((p.focal*p.baseline)/disp, 5)

        # YOLOv8 检测
        results = model(left240, verbose=False)[0]
        result = left480.copy()
        for box in results.boxes:
            if float(box.conf) < 0.3: continue
            x1,y1,x2,y2 = map(int, box.xyxy[0].tolist())
            cls = model.names[int(box.cls)]
            sc = fh/240.0
            rx,ry,rw,rh = int(x1*sc), int(y1*sc), int((x2-x1)*sc), int((y2-y1)*sc)
            cx,cy = rx+rw//2, ry+rh//2
            rr = max(min(rw,rh)//8, 4)
            roi = dist_map[max(cy//2-rr//2,0):min(cy//2+rr//2,dist_map.shape[0]-1),
                           max(cx-rr,0):min(cx+rr,dist_map.shape[1]-1)]
            dist = float(np.median(roi[(roi>0.01)&(roi<10)])) if roi.size>0 else -1

            cv2.rectangle(result,(rx,ry),(rx+rw,ry+rh),(0,255,0) if 0<dist<3 else (0,255,255),3)
            label = f"{cls} {float(box.conf):.2f}"
            if dist>0: label += f" {dist*100:.0f}cm"
            cv2.rectangle(result,(rx,ry-25),(rx+len(label)*11,ry),(0,255,0) if 0<dist<3 else (0,255,255),-1)
            cv2.putText(result,label,(rx+4,ry-8),cv2.FONT_HERSHEY_SIMPLEX,0.55,(0,0,0),2)
            cv2.circle(result,(cx,cy),5,(0,255,255),-1)
            print(f"\r{label}   ",end="")

        disp_viz = cv2.applyColorMap(cv2.normalize(disp,None,0,255,cv2.NORM_MINMAX).astype(np.uint8), cv2.COLORMAP_JET)
        # 上: 左眼 + 检测结果  下: 右眼 + 视差热力图
        display = cv2.resize(np.vstack([
            np.hstack([left480, result]),
            np.hstack([right480, cv2.resize(disp_viz,(hw,fh))])
        ]),(480,720))
        cv2.putText(display, f"FPS:{0.9/max(time.time()-fps_t,0.001):.1f} YOLOv8n",(5,20),cv2.FONT_HERSHEY_SIMPLEX,0.5,(0,255,0),2)
        fps_t=time.time()
        cv2.imshow("YOLOv8 Stereo 3D", display)
        key = cv2.waitKey(1)&0xFF
        if key==ord('q'): break
        elif key==ord('s'): cv2.imwrite("/tmp/yolo_stereo.jpg", display); print("\nSaved")
    cap.release(); cv2.destroyAllWindows()

if __name__=="__main__": main()
