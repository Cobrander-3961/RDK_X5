#!/usr/bin/env python3
"""
手眼标定 (Eye-to-hand) — ArUco 标记 + 正运动学 + calibrateHandEye。

准备: 打印 ArUco DICT_4X4_50 ID=0, 5cm×5cm, 贴夹爪上, 标记朝相机
流程: 移动机械臂到 N≥12 个位姿 → 每帧检测 ArUco + 记录关节角度
     → calibrateHandEye() → 求解相机在底座坐标系下的位姿

模式: 1) ROS2: 自动读取 /joint_states (需 arm_bridge 运行)
      2) 独立: 手动输入 6 关节角度 (rad)

启动:
  python3 hand_eye_calibrate.py              # 自动检测 ROS2
  python3 hand_eye_calibrate.py --verify     # 验证已标定结果
"""
import cv2
import numpy as np
import os
import sys
import time
import yaml
import math

# ─── 路径 ────────────────────────────────────────────
SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
CONFIG_DIR = os.path.join(os.path.dirname(SCRIPT_DIR), 'config')
STEREO_CALIB = os.path.join(CONFIG_DIR, 'stereo_calib.yaml')
HAND_EYE_CALIB = os.path.join(CONFIG_DIR, 'hand_eye.yaml')

# ─── ArUco 配置 ──────────────────────────────────────
ARUCO_DICT = cv2.aruco.DICT_4X4_50
ARUCO_ID = 0
MARKER_SIZE = 0.05  # 米 (5cm)

# ─── FK: 从 arm.urdf 提取的 6 个 revolute 关节参数 ──
# 每个关节: (origin_xyz, origin_rpy, axis_xyz)
# 按运动链顺序: base_link → arm_joint1 → arm_joint2 → arm_joint3
#               → arm_joint4 → gripper_joint → end_joint1 → end_Link1
JOINT_PARAMS = [
    # arm_joint1 (base_link → arm_Link1)
    {'origin_xyz': (-0.10713, -0.27, 0.055092),
     'origin_rpy': (1.5708, 0.0, 0.26475),
     'axis': (0, -1, 0)},
    # arm_joint2 (arm_Link1 → arm_Link2)
    {'origin_xyz': (0.0097086, 0.03009, -0.0023964),
     'origin_rpy': (-0.0047341, 0.24187, -0.019756),
     'axis': (-8.0295e-05, 0, -1)},
    # arm_joint3 (arm_Link2 → arm_Link3)
    {'origin_xyz': (-0.0025242, 0.10097, 0.0),
     'origin_rpy': (0.0, -0.00016057, -3.1166),
     'axis': (-8.0295e-05, 0, -1)},
    # arm_joint4 (arm_Link3 → arm_Link4)
    {'origin_xyz': (0.0, -0.0915, 0.00060959),
     'origin_rpy': (0.0, -0.00016059, 3.1416),
     'axis': (-8.0295e-05, 0, -1)},
    # gripper_joint (arm_Link4 → gripper_Link) — ArUco 贴这里
    {'origin_xyz': (-0.0064346, 0.066247, -0.021069),
     'origin_rpy': (3.1416, 0.40529, 3.1416),
     'axis': (0, -1, 0)},
    # end_joint1 (gripper_Link → end_Link1) — 末端指端
    {'origin_xyz': (0.0, 0.028663, -0.01525),
     'origin_rpy': (-0.55028, 1.1853e-05, 4.1989e-05),
     'axis': (1, 4.5406e-05, -4.6232e-05)},
]

# arm_bridge 的关节顺序 → 运动链顺序 的映射
# bridge: [arm_joint1, arm_joint2, arm_joint3, arm_joint4, end_joint1, gripper_joint]
# chain:  [0=arm_j1,   1=arm_j2,   2=arm_j3,   3=arm_j4,   5=gripper,   4=end_j1]
BRIDGE_TO_CHAIN = [0, 1, 2, 3, 5, 4]  # 按运动链重排


def euler_to_rotmat(rpy):
    """roll, pitch, yaw (xyz intrinsic = Rz*Ry*Rx) → 3×3 旋转矩阵。"""
    rx, ry, rz = rpy
    cx, sx = math.cos(rx), math.sin(rx)
    cy, sy = math.cos(ry), math.sin(ry)
    cz, sz = math.cos(rz), math.sin(rz)
    Rx = np.array([[1, 0, 0], [0, cx, -sx], [0, sx, cx]])
    Ry = np.array([[cy, 0, sy], [0, 1, 0], [-sy, 0, cy]])
    Rz = np.array([[cz, -sz, 0], [sz, cz, 0], [0, 0, 1]])
    return Rz @ Ry @ Rx  # intrinsic xyz = Rz*Ry*Rx


def rotmat_to_euler(R_mat, degrees=False):
    """3×3 旋转矩阵 → (roll, pitch, yaw) in radians。"""
    sy = math.sqrt(R_mat[0, 0]**2 + R_mat[1, 0]**2)
    singular = sy < 1e-6
    if not singular:
        rx = math.atan2(R_mat[2, 1], R_mat[2, 2])
        ry = math.atan2(-R_mat[2, 0], sy)
        rz = math.atan2(R_mat[1, 0], R_mat[0, 0])
    else:
        rx = math.atan2(-R_mat[1, 2], R_mat[1, 1])
        ry = math.atan2(-R_mat[2, 0], sy)
        rz = 0
    if degrees:
        rx, ry, rz = math.degrees(rx), math.degrees(ry), math.degrees(rz)
    return (rx, ry, rz)


def make_transform(xyz, rpy):
    """从 (x, y, z) + (roll, pitch, yaw) → 4×4 齐次变换矩阵。"""
    rot = euler_to_rotmat(rpy)
    T = np.eye(4)
    T[:3, :3] = rot
    T[:3, 3] = xyz
    return T


def rotation_around_axis(axis, angle):
    """绕任意轴旋转 angle (rad) 的 4×4 矩阵。"""
    ax = np.array(axis, dtype=float)
    ax = ax / (np.linalg.norm(ax) + 1e-10)
    c = math.cos(angle)
    s = math.sin(angle)
    v = 1 - c
    x, y, z = ax
    R_mat = np.array([
        [c + x*x*v,   x*y*v - z*s, x*z*v + y*s],
        [y*x*v + z*s, c + y*y*v,   y*z*v - x*s],
        [z*x*v - y*s, z*y*v + x*s, c + z*z*v],
    ])
    T = np.eye(4)
    T[:3, :3] = R_mat
    return T


def forward_kinematics(joint_angles_chain):
    """
    正运动学: 6 个关节角度 (rad, 按运动链顺序) → base_T_ee (4×4)。

    返回 base_link → end_Link1 (末端) 的变换矩阵。
    """
    T = np.eye(4)
    for i, angle in enumerate(joint_angles_chain):
        params = JOINT_PARAMS[i]
        T_origin = make_transform(params['origin_xyz'], params['origin_rpy'])
        T_axis = rotation_around_axis(params['axis'], angle)
        T = T @ T_origin @ T_axis
    return T


def reorder_joints(bridge_angles):
    """arm_bridge 顺序 → 运动链顺序。"""
    return [bridge_angles[i] for i in BRIDGE_TO_CHAIN]


def load_camera_calib():
    """加载相机内参。"""
    if not os.path.exists(STEREO_CALIB):
        print(f'[WARN] No stereo calibration found at {STEREO_CALIB}')
        print('  Using default: fx=fy=700, cx=320, cy=240')
        return np.array([[700, 0, 320], [0, 700, 240], [0, 0, 1]], dtype=float), None

    with open(STEREO_CALIB) as f:
        calib = yaml.safe_load(f)

    if 'camera_matrix_left' in calib:
        mtx = np.array(calib['camera_matrix_left'])
        dist = np.array(calib['dist_coeffs_left'])
    else:
        mtx = np.array(calib['camera_matrix'])
        dist = np.array(calib['dist_coeffs']) if calib.get('dist_coeffs') else None

    print(f'[Calib] fx={mtx[0,0]:.1f} fy={mtx[1,1]:.1f} '
          f'cx={mtx[0,2]:.1f} cy={mtx[1,2]:.1f}')
    return mtx, dist


# ─── ROS2 关节读取 (可选) ─────────────────────────────

def try_ros2_joint_reader():
    """尝试通过 ROS2 订阅 /joint_states 获取当前关节角度。返回 reader 函数或 None。"""
    try:
        import rclpy
        if not rclpy.ok():
            rclpy.init(args=[])

        from rclpy.node import Node
        from sensor_msgs.msg import JointState

        class JointReader(Node):
            def __init__(self):
                super().__init__('hand_eye_joint_reader')
                self.joints = [0.0] * 6
                self.names = ['arm_joint1', 'arm_joint2', 'arm_joint3',
                              'arm_joint4', 'end_joint1', 'gripper_joint']
                self.sub = self.create_subscription(
                    JointState, '/joint_states', self._cb, 10)

            def _cb(self, msg):
                for i, name in enumerate(self.names):
                    try:
                        idx = msg.name.index(name)
                        self.joints[i] = float(msg.position[idx])
                    except ValueError:
                        pass

            def read(self):
                rclpy.spin_once(self, timeout_sec=0.05)
                return list(self.joints)

        reader = JointReader()
        # 快速 spin 几次获取初始值
        for _ in range(20):
            rclpy.spin_once(reader, timeout_sec=0.05)
        print(f'[ROS2] Joint reader ready, initial: '
              f'{[f"{j:.2f}" for j in reader.joints]}')
        return reader
    except Exception as e:
        print(f'[ROS2] Not available ({e}), will use manual input')
        return None


# ─── 主流程 ──────────────────────────────────────────

def main():
    import argparse
    parser = argparse.ArgumentParser(description='手眼标定 (Eye-to-hand)')
    parser.add_argument('--verify', action='store_true', help='验证已标定结果')
    args = parser.parse_args()

    if args.verify:
        verify()
        return

    print('=' * 55)
    print('  手眼标定 (Eye-to-hand) — ArUco + FK')
    print('=' * 55)
    print(f'  ArUco: DICT_4X4_50 ID={ARUCO_ID} {MARKER_SIZE*100:.0f}mm')
    print('  准备: ArUco 标记贴夹爪上, 标记朝相机')
    print('  操作: 移动机械臂→按空格记录→重复12+次→按c求解')
    print('=' * 55)

    # 相机
    camera_matrix, dist_coeffs = load_camera_calib()

    for i in range(4):
        cap = cv2.VideoCapture(i)
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
        ret, frame = cap.read()
        if ret:
            print(f'[Camera] /dev/video{i} ({"stereo" if frame.shape[1]==1280 else "mono"})')
            break
        cap.release()

    stereo_mode = frame.shape[1] == 1280

    # ArUco
    aruco_dict = cv2.aruco.getPredefinedDictionary(ARUCO_DICT)
    aruco_params = cv2.aruco.DetectorParameters()
    aruco_params.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX

    # 关节输入: ROS2 或手动
    joint_reader = try_ros2_joint_reader()

    # 收集数据
    R_base_ee_list = []
    t_base_ee_list = []
    R_cam_marker_list = []
    t_cam_marker_list = []
    joint_samples = []

    print('\nStart collecting... (space=record  c=solve  q=quit)')

    while True:
        ret, frame = cap.read()
        if not ret:
            break

        if stereo_mode:
            left = frame[:, :frame.shape[1]//2]
        else:
            left = frame

        # ArUco 检测
        gray = cv2.cvtColor(left, cv2.COLOR_BGR2GRAY)
        corners, ids, _ = cv2.aruco.detectMarkers(gray, aruco_dict,
                                                   parameters=aruco_params)

        marker_found = (ids is not None and ARUCO_ID in ids)

        # 显示
        display = left.copy()
        if marker_found:
            cv2.aruco.drawDetectedMarkers(display, corners, ids)
            rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(
                corners, MARKER_SIZE, camera_matrix, dist_coeffs)
            idx = list(ids.flatten()).index(ARUCO_ID)
            cv2.drawFrameAxes(display, camera_matrix, dist_coeffs,
                              rvecs[idx], tvecs[idx], 0.03)

        n = len(R_base_ee_list)
        status = 'MARKER OK' if marker_found else 'no marker'
        cv2.putText(display, f'Samples: {n} | {status}',
                    (5, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.6,
                    (0, 255, 0) if marker_found else (0, 0, 255), 2)
        if n >= 12:
            cv2.putText(display, 'READY — press c to solve', (5, 45),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 1)

        # 当前关节显示
        if joint_reader is not None:
            joints = joint_reader.read()
            j_str = ' '.join([f'{j:+.2f}' for j in joints])
            cv2.putText(display, f'J: {j_str}', (5, display.shape[0] - 10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.35, (200, 200, 200), 1)

        cv2.imshow('Hand-Eye Calibration', cv2.resize(display, (640, 480)))
        key = cv2.waitKey(1) & 0xFF

        if key == ord('q'):
            break
        elif key == ord('c') and n >= 12:
            break
        elif key == ord(' ') and marker_found:
            # 记录一对数据
            idx = list(ids.flatten()).index(ARUCO_ID)
            rvec = rvecs[idx]
            tvec = tvecs[idx]
            R_mat, _ = cv2.Rodrigues(rvec)
            R_cam_marker_list.append(R_mat)
            t_cam_marker_list.append(tvec.flatten())

            if joint_reader is not None:
                joints_raw = joint_reader.read()
            else:
                # 手动输入
                print('\n输入 6 关节角度 (rad), 空格分隔:')
                print('  arm_j1 arm_j2 arm_j3 arm_j4 end_j1 gripper_j')
                try:
                    inp = input('> ').strip()
                    joints_raw = [float(x) for x in inp.split()]
                    if len(joints_raw) != 6:
                        print('需要6个值!')
                        continue
                except (EOFError, ValueError):
                    continue

            # FK: bridge顺序 → 运动链顺序
            joints_chain = reorder_joints(joints_raw)
            T_base_ee = forward_kinematics(joints_chain)
            R_base_ee_list.append(T_base_ee[:3, :3])
            t_base_ee_list.append(T_base_ee[:3, 3])
            joint_samples.append(joints_raw)

            print(f'  Recorded #{len(R_base_ee_list)}: '
                  f'marker_t={[f"{v:.3f}" for v in tvec.flatten()]} '
                  f'ee_t={[f"{v:.3f}" for v in T_base_ee[:3, 3]]}')

    cap.release()
    cv2.destroyAllWindows()

    n = len(R_base_ee_list)
    if n < 12:
        print(f'\nNot enough samples ({n} < 12), aborting')
        return

    print(f'\nSolving hand-eye with {n} samples...')

    # 多种方法比较
    methods = {
        'Tsai': cv2.CALIB_HAND_EYE_TSAI,
        'Park': cv2.CALIB_HAND_EYE_PARK,
        'Horaud': cv2.CALIB_HAND_EYE_HORAUD,
        'Daniilidis': cv2.CALIB_HAND_EYE_DANIILIDIS,
    }

    best_result = None
    best_method = None
    best_rmse = float('inf')

    for name, method in methods.items():
        try:
            R_cam_base, t_cam_base = cv2.calibrateHandEye(
                R_base_ee_list, t_base_ee_list,
                R_cam_marker_list, t_cam_marker_list,
                method=method)

            # 重投影误差
            errors = []
            T_cam_base = np.eye(4)
            T_cam_base[:3, :3] = R_cam_base
            T_cam_base[:3, 3] = t_cam_base.flatten()

            for i in range(n):
                T_base_ee = np.eye(4)
                T_base_ee[:3, :3] = R_base_ee_list[i]
                T_base_ee[:3, 3] = t_base_ee_list[i]

                T_cam_marker = np.eye(4)
                T_cam_marker[:3, :3] = R_cam_marker_list[i]
                T_cam_marker[:3, 3] = t_cam_marker_list[i]

                # 约束: T_base_ee @ T_ee_marker ≈ T_base_cam @ T_cam_marker
                # 但我们不知道 T_ee_marker, 这里验证 T_base_cam 的一致性
                T_err = T_cam_base @ T_cam_marker
                err = np.linalg.norm(T_err[:3, 3] - T_base_ee[:3, 3])
                errors.append(err)

            rmse = np.sqrt(np.mean(np.array(errors)**2))
            print(f'  {name:12s}: RMSE={rmse*1000:.1f}mm  '
                  f't=[{t_cam_base[0,0]:.3f} {t_cam_base[1,0]:.3f} {t_cam_base[2,0]:.3f}]m')

            if rmse < best_rmse:
                best_rmse = rmse
                best_result = (R_cam_base, t_cam_base)
                best_method = name
        except Exception as e:
            print(f'  {name:12s}: FAILED ({e})')

    if best_result is None:
        print('\nAll methods failed!')
        return

    R_cam_base, t_cam_base = best_result
    t = t_cam_base.flatten()
    euler = rotmat_to_euler(R_cam_base, degrees=True)

    print(f'\n=== 最佳结果 ({best_method}) ===')
    print(f'相机在底座坐标系下的位姿 (T_base_cam):')
    print(f'  位置 (m):  x={t[0]:.4f}  y={t[1]:.4f}  z={t[2]:.4f}')
    print(f'  姿态 (deg): roll={euler[0]:.1f}  pitch={euler[1]:.1f}  yaw={euler[2]:.1f}')
    print(f'  RMSE: {best_rmse*1000:.1f}mm')

    if best_rmse > 0.02:
        print('\n⚠️  RMSE偏高 (>2cm). 建议:')
        print('  1. 增加更多姿态 (间距大, 覆盖工作空间)')
        print('  2. 确保 ArUco 标记平展无反光')
        print('  3. 先跑 stereo_calibrate.py 标定相机')

    # 保存
    os.makedirs(CONFIG_DIR, exist_ok=True)
    result = {
        'method': best_method,
        'T_base_cam': np.eye(4).tolist(),  # placeholder, filled below
        'position_m': {'x': float(t[0]), 'y': float(t[1]), 'z': float(t[2])},
        'euler_deg': {'roll': float(euler[0]), 'pitch': float(euler[1]), 'yaw': float(euler[2])},
        'rmse_mm': float(best_rmse * 1000),
        'num_samples': n,
        'joint_order': ['arm_joint1', 'arm_joint2', 'arm_joint3',
                        'arm_joint4', 'end_joint1', 'gripper_joint'],
        'note': 'T_base_cam: 相机在底座坐标系下的位姿 (eye-to-hand)',
    }
    T_full = np.eye(4)
    T_full[:3, :3] = R_cam_base
    T_full[:3, 3] = t_cam_base.flatten()
    result['T_base_cam'] = T_full.tolist()

    with open(HAND_EYE_CALIB, 'w') as f:
        yaml.dump(result, f, default_flow_style=False)
    print(f'\nSaved: {HAND_EYE_CALIB}')


# ─── 验证模式 ─────────────────────────────────────────

def verify():
    """验证标定结果: 移动机械臂到新位姿，对比 FK vs 视觉估计。"""
    if not os.path.exists(HAND_EYE_CALIB):
        print(f'No calibration file: {HAND_EYE_CALIB}')
        print('Run hand_eye_calibrate.py first')
        return

    with open(HAND_EYE_CALIB) as f:
        calib = yaml.safe_load(f)

    T_base_cam = np.array(calib['T_base_cam'])
    print(f'Loaded: {HAND_EYE_CALIB}')
    print(f'  Method: {calib["method"]}, RMSE: {calib["rmse_mm"]:.1f}mm')
    print(f'  Position: x={calib["position_m"]["x"]:.3f} '
          f'y={calib["position_m"]["y"]:.3f} z={calib["position_m"]["z"]:.3f}m')

    camera_matrix, dist_coeffs = load_camera_calib()

    cap = cv2.VideoCapture(0)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

    aruco_dict = cv2.aruco.getPredefinedDictionary(ARUCO_DICT)
    aruco_params = cv2.aruco.DetectorParameters()
    joint_reader = try_ros2_joint_reader()

    print('\nVerification mode: 移动机械臂, 按空格对比 FK vs 视觉')
    print('q=quit')

    while True:
        ret, frame = cap.read()
        if not ret:
            break

        left = frame[:, :frame.shape[1]//2] if frame.shape[1] == 1280 else frame
        gray = cv2.cvtColor(left, cv2.COLOR_BGR2GRAY)
        corners, ids, _ = cv2.aruco.detectMarkers(gray, aruco_dict,
                                                   parameters=aruco_params)

        display = left.copy()
        marker_found = (ids is not None and ARUCO_ID in ids)

        if marker_found:
            rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(
                corners, MARKER_SIZE, camera_matrix, dist_coeffs)
            idx = list(ids.flatten()).index(ARUCO_ID)
            cv2.drawFrameAxes(display, camera_matrix, dist_coeffs,
                              rvecs[idx], tvecs[idx], 0.03)

            t_cam = tvecs[idx].flatten()

            if joint_reader is not None:
                joints = joint_reader.read()
                joints_chain = reorder_joints(joints)
                T_base_ee = forward_kinematics(joints_chain)
                t_base_fk = T_base_ee[:3, 3]

                # 视觉估计: T_base_ee ≈ T_base_cam @ T_cam_marker
                # (简化: 只比位置, 假设 T_ee_marker ≈ I)
                t_base_vision = T_base_cam[:3, :3] @ t_cam + T_base_cam[:3, 3]
                error = np.linalg.norm(t_base_vision - t_base_fk) * 1000

                cv2.putText(display, f'FK:   [{t_base_fk[0]:.3f} {t_base_fk[1]:.3f} {t_base_fk[2]:.3f}]',
                            (5, display.shape[0] - 40), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 255, 0), 1)
                cv2.putText(display, f'Vis:  [{t_base_vision[0]:.3f} {t_base_vision[1]:.3f} {t_base_vision[2]:.3f}]',
                            (5, display.shape[0] - 20), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 255, 0), 1)
                cv2.putText(display, f'Error: {error:.0f}mm',
                            (5, display.shape[0] - 60), cv2.FONT_HERSHEY_SIMPLEX, 0.5,
                            (0, 255, 0) if error < 20 else (0, 0, 255), 1)

        cv2.imshow('Hand-Eye Verify', cv2.resize(display, (640, 480)))
        key = cv2.waitKey(1) & 0xFF
        if key == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()


if __name__ == '__main__':
    main()
