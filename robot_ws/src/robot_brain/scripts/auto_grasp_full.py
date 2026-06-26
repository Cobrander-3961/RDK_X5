#!/usr/bin/env python3
"""全流程自动化: 持续检测 → 稳定3秒 → 自动抓取 → 循环"""
import cv2, numpy as np, struct, serial, threading, time, os, sys, glob
from ultralytics import YOLO
from collections import deque

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from camera_utils import CameraWrapper

class ArmSerial:
    def __init__(self):
        ports = glob.glob('/dev/ttyACM*')
        port = ports[0] if ports else '/dev/ttyACM0'
        self.ser = None; self.lock = threading.Lock()
        try: self.ser = serial.Serial(port,115200,timeout=1); print(f'[Arm] {port}')
        except Exception as e: print(f'[Arm] FAIL: {e}')
    def send(self,a):
        if not self.ser: return
        d = struct.pack('<ffffff',*[float(x) for x in a])
        with self.lock: self.ser.write(bytearray([0xAA,0x20,24,0])+d+bytearray([0x55]))

HOME=[0]*6; PRE=[0,-0.3,-0.6,0,0,0]; GRASP=[0,-0.3,-0.6,0,0,-0.5]
LIFT=[0,-0.5,-0.3,0,0,-0.5]; PLACE=[-0.5,-0.3,-0.5,0,0,-0.5]; RELEASE=[-0.5,-0.3,-0.5,0,0,0.3]

def grasp_sequence(arm, j1):
    """自动执行抓取序列。"""
    print(f"[AutoGrasp] j1={j1:.2f}")
    arm.send(HOME); time.sleep(1.5)
    p=PRE.copy(); p[0]=j1; arm.send(p); time.sleep(2)
    g=GRASP.copy(); g[0]=j1; arm.send(g); time.sleep(1)
    l=LIFT.copy(); l[0]=j1; arm.send(l); time.sleep(1.5)
    arm.send(PLACE); time.sleep(2); arm.send(RELEASE); time.sleep(1)
    arm.send(HOME); time.sleep(2)

def main():
    print("Loading YOLOv8n...")
    model = YOLO("/home/sunrise/RDK_X5/yolov8n.pt"); model.conf = 0.3
    print("YOLOv8n ready!")

    cam = CameraWrapper(); arm = ArmSerial()
    TARGET = {'mouse', 'bottle', 'cup', 'cell phone', 'book', 'banana', 'apple'}
    fps_t = time.time()

    # 稳定检测: 连续3帧检测到同一目标才触发
    detect_history = deque(maxlen=5)
    grasp_cooldown = 0  # 抓取冷却时间

    print("\n=== Auto Grasp Full ===")
    print("Targets: mouse, bottle, cup, phone, book, banana, apple")
    print("Auto-trigger when stable detection. Q=quit, H=home, G=manual grasp")
    print("========================")

    while True:
        img, depth = cam.read()
        if img is None: break
        h,w = img.shape[:2]

        results = model(img, verbose=False)[0]
        closest = None; min_d = 999; current_ids = set()

        for box in results.boxes:
            cls = model.names[int(box.cls)]; conf = float(box.conf)
            if conf < 0.3 or cls not in TARGET: continue
            x1,y1,x2,y2 = map(int, box.xyxy[0].tolist())
            cx,cy = (x1+x2)//2, (y1+y2)//2; bw,bh = x2-x1, y2-y1

            if depth is not None:
                rr = max(min(bw,bh)//8,4)
                roi = depth[max(cy-rr,0):min(cy+rr,h-1), max(cx-rr,0):min(cx+rr,w-1)]
                dist = float(np.median(roi[(roi>100)&(roi<10000)]))/1000.0 if roi.size>0 else -1
            else:
                dist = 0.05*320/max(bw,1); dist = max(0.05, min(dist,2.0))

            current_ids.add(f"{cls}_{cx}_{cy}")
            color = (0,255,0) if 0<dist<3 else (0,255,255)
            cv2.rectangle(img,(x1,y1),(x2,y2),color,2)
            label = f"{cls} {conf:.2f}"
            if dist>0: label += f" {dist*100:.0f}cm"
            cv2.putText(img,label,(x1,y1-8),cv2.FONT_HERSHEY_SIMPLEX,0.5,color,2)
            cv2.circle(img,(cx,cy),4,(0,255,255),-1)
            if 0<dist<min_d: min_d=dist; closest=(cls,cx,cy,dist)

        # 稳定检测逻辑
        detect_history.append(closest[0] if closest else None)
        stable = len(detect_history) >= 4 and all(d == detect_history[-1] for d in list(detect_history)[-4:])

        if grasp_cooldown > 0: grasp_cooldown -= 1

        # 自动触发抓取
        if stable and closest and grasp_cooldown == 0 and 0.05 < closest[3] < 1.0:
            print(f"\n[AutoDetect] {closest[0]} @ {closest[3]*100:.0f}cm (stable)")
            j1 = max(-1.2, min(1.2, (closest[1]-w/2)/w*0.6))
            grasp_sequence(arm, j1)
            grasp_cooldown = 30  # 30帧冷却 (~3s)

        # 显示
        fps = 0.9/max(time.time()-fps_t,0.001); fps_t=time.time()
        status = "AUTO" if grasp_cooldown == 0 else f"COOL({grasp_cooldown})"
        cv2.putText(img,f"FPS:{fps:.1f} {status}",(5,20),cv2.FONT_HERSHEY_SIMPLEX,0.4,(0,255,0),2)
        if closest and stable:
            cv2.putText(img, f"DETECTED: {closest[0]} {closest[3]*100:.0f}cm - AUTO GRASP",
                       (5,h-10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,0,255), 2)
        cv2.imshow("Auto Grasp", img)
        key = cv2.waitKey(1)&0xFF

        if key == ord('q'): break
        elif key == ord('h'): arm.send(HOME); time.sleep(1.5); grasp_cooldown = 10
        elif key == ord('g') and closest:
            j1 = max(-1.2, min(1.2, (closest[1]-w/2)/w*0.6))
            grasp_sequence(arm, j1)
            grasp_cooldown = 10

    cam.release(); cv2.destroyAllWindows()

if __name__=="__main__": main()
