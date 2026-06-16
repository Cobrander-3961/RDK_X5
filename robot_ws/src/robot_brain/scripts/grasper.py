#!/usr/bin/env python3
"""
机械臂串口控制 + 抓取序列 — 可导入模块, 也可独立测试。

统一 auto_grasp_full.py / brain_pipeline.py / arm_fixed_grasp_test.py 中
重复的 ArmSerial + 抓取序列逻辑。

串口帧: [0xAA][0x20][24 LE][6×float LE][0x55]
"""
import struct
import serial
import threading
import time
import glob
import sys


class ArmSerial:
    """STM32 机械臂串口桥接。自动检测 /dev/ttyACM*, 线程安全。

    无串口时优雅降级: send() 变 no-op, connected=False。
    """

    def __init__(self, port=None, baudrate=115200):
        self.ser = None
        self.lock = threading.Lock()
        self._port = port

        if port is None:
            ports = glob.glob('/dev/ttyACM*')
            self._port = ports[0] if ports else None

        if self._port:
            try:
                self.ser = serial.Serial(self._port, baudrate, timeout=1)
                print(f'[ArmSerial] {self._port} @ {baudrate}')
            except (serial.SerialException, OSError) as e:
                print(f'[ArmSerial] {self._port} FAIL: {e}')
                self.ser = None
        else:
            print('[ArmSerial] No serial port found, dry-run mode')

    def send(self, angles):
        """发送 6 关节角度 (rad) → STM32 0x20 帧。无串口时 no-op。"""
        if not self.ser:
            print(f'  [dry-run] {[f"{a:.2f}" for a in angles]}')
            return
        try:
            data = struct.pack('<ffffff', *[float(a) for a in angles])
            frame = bytearray([0xAA, 0x20, 24, 0]) + data + bytearray([0x55])
            with self.lock:
                self.ser.write(frame)
        except (serial.SerialException, OSError) as e:
            print(f'[ArmSerial] Write error: {e}')

    def close(self):
        if self.ser and self.ser.is_open:
            self.ser.close()
            print('[ArmSerial] Closed')

    @property
    def connected(self) -> bool:
        return self.ser is not None and self.ser.is_open


class Grasper:
    """高级机械臂控制 — 封装标准抓取序列。

    关节序列来自 auto_grasp_full.py, 与 arm_fixed_grasp_test.py 兼容。
    """

    HOME    = [ 0.0,  0.0,  0.0,  0.0,  0.0,  0.0]
    PRE     = [ 0.0, -0.3, -0.6,  0.0,  0.0,  0.0]
    GRASP   = [ 0.0, -0.3, -0.6,  0.0,  0.0, -0.5]
    LIFT    = [ 0.0, -0.5, -0.3,  0.0,  0.0, -0.5]
    PLACE   = [-0.5, -0.3, -0.5,  0.0,  0.0, -0.5]
    RELEASE = [-0.5, -0.3, -0.5,  0.0,  0.0,  0.3]

    def __init__(self, arm_serial=None, port=None):
        """可复用外部 ArmSerial, 或自动创建。

        Args:
            arm_serial: 已有的 ArmSerial 实例 (共享)
            port:       串口路径, arm_serial 为 None 时生效
        """
        self._own_serial = arm_serial is None
        self.arm = arm_serial if arm_serial is not None else ArmSerial(port=port)

    def send_joints(self, angles):
        """下发任意 6 关节角度。"""
        self.arm.send(angles)

    def home(self, sleep_sec=1.5):
        """归位。"""
        print(f'[Grasper] HOME: {self.HOME}')
        self.arm.send(self.HOME)
        time.sleep(sleep_sec)

    def execute_grasp_sequence(self, j1):
        """完整抓取序列: HOME → PRE(j1) → GRASP(j1) → LIFT(j1) → PLACE → RELEASE → HOME。

        j1 由目标水平位置映射: j1 = (cx - w/2) / w * 0.6, clamp [-1.2, 1.2]。
        """
        j1 = max(-1.2, min(1.2, j1))
        print(f'\n[Grasper] === Grasp Sequence (j1={j1:.2f}) ===')

        # HOME
        print('[Grasper] 1/7 HOME')
        self.arm.send(self.HOME)
        time.sleep(1.5)

        # PRE
        p = self.PRE.copy(); p[0] = j1
        print(f'[Grasper] 2/7 PRE {[f"{x:.2f}" for x in p]}')
        self.arm.send(p)
        time.sleep(2.0)

        # GRASP
        g = self.GRASP.copy(); g[0] = j1
        print(f'[Grasper] 3/7 GRASP {[f"{x:.2f}" for x in g]}')
        self.arm.send(g)
        time.sleep(1.0)

        # LIFT
        l = self.LIFT.copy(); l[0] = j1
        print(f'[Grasper] 4/7 LIFT {[f"{x:.2f}" for x in l]}')
        self.arm.send(l)
        time.sleep(1.5)

        # PLACE
        print(f'[Grasper] 5/7 PLACE {[f"{x:.2f}" for x in self.PLACE]}')
        self.arm.send(self.PLACE)
        time.sleep(2.0)

        # RELEASE
        print(f'[Grasper] 6/7 RELEASE {[f"{x:.2f}" for x in self.RELEASE]}')
        self.arm.send(self.RELEASE)
        time.sleep(1.0)

        # HOME
        print(f'[Grasper] 7/7 HOME')
        self.arm.send(self.HOME)
        time.sleep(2.0)

        print('[Grasper] === Sequence Done ===\n')

    @property
    def connected(self) -> bool:
        return self.arm.connected

    def close(self):
        if self._own_serial:
            self.arm.close()


# ─── 独立测试 ─────────────────────────────────────────────
def main():
    import argparse
    parser = argparse.ArgumentParser(description='Grasper 独立测试')
    parser.add_argument('--port', default=None, help='串口路径 (默认自动检测 /dev/ttyACM*)')
    parser.add_argument('--j1', type=float, default=0.3, help='关节1 角度 (rad), 默认 0.3')
    parser.add_argument('--home-only', action='store_true', help='只归位, 不执行完整序列')
    args = parser.parse_args()

    grasper = Grasper(port=args.port)

    if not grasper.connected:
        print('\n⚠️  无串口连接, 进入 dry-run 模式\n')

    if args.home_only:
        grasper.home()
    else:
        print(f'\n执行完整抓取序列, j1={args.j1:.2f}')
        print('请确认机械臂周围无障碍物, 按 Enter 继续...')
        try:
            input()
        except (EOFError, KeyboardInterrupt):
            print('\n取消')
            grasper.close()
            return
        grasper.execute_grasp_sequence(args.j1)

    grasper.close()
    print('测试完成')


if __name__ == '__main__':
    main()
