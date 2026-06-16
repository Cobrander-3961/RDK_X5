# 手眼标定实现 (深目摄像头 + ArUco + FK) — 2026-06-16

## 背景

项目使用深目双目摄像头 + 6 轴舵机机械臂做抓取，但:
- `yolo_stereo_detect.py` 的 SGBM 参数 (基线 0.06m / 焦距 700px) 是粗略估计，未标定
- `auto_grasp_full.py` 用 `j1 = (cx-w/2)/w*0.6` 像素→关节映射，精度低

要达到 "看到目标 → 算出 3D 坐标 → 精准抓取"，必须:
1. 标定相机内参 (焦距/光心/畸变)
2. 标定相机外参 (相机在机械臂底座坐标系下的位姿)

## 方案设计

**Eye-to-hand + ArUco 标记**。深目摄像头左目做单目 PnP（不依赖未标定的双目深度）。

### 两步标定

```
Step 1: stereo_calibrate.py  → 相机内参 (棋盘格法)
Step 2: hand_eye_calibrate.py → 相机外参 (ArUco + FK)
```

## 实现细节

### 1. stereo_calibrate.py (~180 行)

- 自动检测深目双目 (1280×480) 或单目 (640×480)
- 棋盘格: 9×6 内角点, 25mm 方格
- 每 10 帧自动检测+保存角点，空格键手动保存
- 收集 20+ 对 → `cv2.stereoCalibrate()` (固定内参模式)
- 输出 `config/stereo_calib.yaml`:
  - `camera_matrix_left` / `dist_coeffs_left`
  - `camera_matrix_right` / `dist_coeffs_right`
  - `R` / `T` (右目相对左目)
  - `baseline_m` (基线距离)
  - `stereo_rms_error` (质量指标)

### 2. hand_eye_calibrate.py (~320 行)

**FK 实现**: 从 `arm.urdf` 提取 6 个关节的 `<origin>` (xyz + rpy) + `<axis>`:

```python
JOINT_PARAMS = [
    # arm_joint1: origin=(-0.107, -0.27, 0.055), rpy=(1.571, 0, 0.265), axis=(0,-1,0)
    # arm_joint2: origin=(0.010, 0.030, -0.002), rpy=(-0.005, 0.242, -0.020), axis=(~0,0,-1)
    # ...
]

def forward_kinematics(angles):
    T = np.eye(4)
    for i, angle in enumerate(angles):
        T = T @ make_transform(origin_xyz, origin_rpy) @ rotation_around_axis(axis, angle)
    return T  # base_T_ee
```

**关节顺序映射**: `arm_bridge.py` 的顺序与 URDF 运动链不同:
```
bridge: [arm_j1, arm_j2, arm_j3, arm_j4, end_j1, gripper_j]
chain:  [arm_j1, arm_j2, arm_j3, arm_j4, gripper_j, end_j1]
```
用 `BRIDGE_TO_CHAIN = [0, 1, 2, 3, 5, 4]` 重排。

**旋转函数**: 由于 scipy 与 numpy 版本不兼容 (scipy 1.5 需要 numpy<1.25, 当前 numpy 2.2)，手写了 `euler_to_rotmat()` 和 `rotmat_to_euler()` (纯 numpy 实现)。验证 roundtrip 精度 < 0.001rad。

**数据采集**: 支持两种模式:
- ROS2 模式: 自动读 `/joint_states` → 实时 FK
- 独立模式: 手动输入 6 关节角度 (rad)

**求解**: 4 种方法 (Tsai / Park / Horaud / Daniilidis) 并行计算，选 RMSE 最小。

**验证模式**: `--verify` 加载 `hand_eye.yaml`，对比 FK vs 视觉的位置误差。

### 3. ArUco 检测

```python
aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
corners, ids, _ = cv2.aruco.detectMarkers(gray, aruco_dict)
rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(
    corners, MARKER_SIZE, camera_matrix, dist_coeffs)
```

标记尺寸 50mm (5cm×5cm)，贴机械臂夹爪上。

## 遇到的问题

### 1. scipy 与 numpy 版本不兼容

**现象**: `from scipy.spatial.transform import Rotation` → `ValueError: numpy.dtype size changed`

**根因**: 系统 scipy 1.5 需要 numpy < 1.25，但当前 numpy 2.2。

**修复**: 手写 `euler_to_rotmat()` + `rotmat_to_euler()` (纯 numpy + math)。euler→matrix roundtrip 精度 < 0.001rad，对标定足够。

### 2. URDF 关节含复杂 rpy 旋转

**现象**: 部分关节 origin 的 rpy 有非零 roll/pitch (如 gripper_joint rpy="3.1416 0.40529 3.1416")，不是纯 DH 参数。

**修复**: 不做 DH 参数提取，直接用 URDF 的 `<origin xyz rpy>` + `<axis>` 构建变换矩阵。更稳定，不损失精度。

## 新增/修改文件

| 文件 | 操作 |
|------|------|
| `robot_brain/scripts/stereo_calibrate.py` | 🆕 双目相机标定 |
| `robot_brain/scripts/hand_eye_calibrate.py` | 🆕 手眼标定 + FK |
| `robot_brain/CMakeLists.txt` | ✏️ 新增 2 个 PROGRAMS |

## 验证结果

- [x] 两个脚本语法检查通过
- [x] `euler_to_rotmat()` ↔ `rotmat_to_euler()` roundtrip 通过 (误差 < 0.001)
- [x] FK 计算数值稳定 (3 组测试角度, 位置均有限)
- [x] OpenCV 4.12.0: `cv2.aruco`, `cv2.calibrateHandEye`, `cv2.stereoCalibrate` 全部可用
- [ ] 实物: 需打印棋盘格 + ArUco 标记，在 RDK X5 上实际标定

## 使用流程

```bash
# 0. 打印准备: 9×6棋盘格(25mm方格) + ArUco标记(5cm×5cm, DICT_4X4_50 ID=0)
#    ArUco 贴夹爪上, 棋盘格贴硬板

# 1. 相机标定 (~10分钟)
python3 stereo_calibrate.py
# 手持棋盘格在相机前变换角度 → 自动采集 20+ 对 → 按 c 标定

# 2. 手眼标定 (~5分钟)
python3 hand_eye_calibrate.py
# 需要 arm_bridge 运行 (读取关节角度)
# 移动机械臂 12+ 个不同位姿 → 按空格录制 → 按 c 求解

# 3. 验证
python3 hand_eye_calibrate.py --verify

# 4. 标定结果用于抓取
# stereo_calib.yaml: 相机内参
# hand_eye.yaml:    相机→底座 变换矩阵 (T_base_cam)
```

## 后续集成

标定完成后，`hand_eye.yaml` 中的 `T_base_cam` 可以用于:
1. 替换 `auto_grasp_full.py` 的 `j1 = (cx-w/2)/w*0.6` 经验公式
2. 计算 `T_base_object = T_base_cam @ T_cam_object` (目标在底座坐标系下的 3D 位置)
3. 作为 MoveIt2 抓取规划的精确输入
