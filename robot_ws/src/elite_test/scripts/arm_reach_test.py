#!/usr/bin/env python3
"""机械臂定点测试 — 关节空间模式（已验证可用）。"""
import serial, struct, time
import threading

class SerialSender:
    def __init__(self, port='/dev/ttyACM0', baudrate=115200):
        self.ser = None; self.lock = threading.Lock()
        try:
            self.ser = serial.Serial(port, baudrate, timeout=1)
            print(f'[Serial] {port} OK')
        except Exception as e:
            print(f'[Serial] {e}')
    def send(self, angles):
        if not self.ser: return
        data = struct.pack('<ffffff', *[float(a) for a in angles])
        frame = bytearray([0xAA,0x20,24,0]) + data + bytearray([0x55])
        with self.lock: self.ser.write(frame)
        print(f'  -> {[f"{a:.2f}" for a in angles]}')

ss = SerialSender()
time.sleep(0.5)

# 关节顺序: arm_j1, arm_j2, arm_j3, arm_j4, end_j1, gripper
HOME    = [ 0.0,  0.0,  0.0,  0.0,  0.0,  0.0]

# 正后方 7cm，高度 7cm (~base_link -Y, +Z)
# arm_j1 旋转底座指向后方, j2/j3 伸展臂杆
BEHIND  = [ 1.2, -0.4, -0.6,  0.0,  0.0,  0.0]  # 底座右转 ~69°
GRASP   = [ 1.2, -0.4, -0.6,  0.0,  0.0, -0.5]  # +夹爪闭合

# 前方测试
FORWARD = [-0.5, -0.3, -0.5,  0.0,  0.0,  0.0]

seq = [
    ('HOME',      HOME),
    ('BEHIND',    BEHIND),
    ('HOME',      HOME),
    ('FORWARD',   FORWARD),
    ('HOME',      HOME),
]

for name, angles in seq:
    print(f'[{name}]')
    ss.send(angles)
    time.sleep(2.5)

print('Done')
