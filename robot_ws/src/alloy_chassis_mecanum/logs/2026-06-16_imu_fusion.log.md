# IMU 姿态融合实现 — 2026-06-16

## 背景

项目文档 (COMPETITION_DOC.md / README.md) 声称了 MPU6050 IMU 姿态融合，但实际 ROS2 代码中完全没有 IMU 相关实现。Slam Toolbox 纯靠激光 scan matching，在长走廊 / 空旷场景容易丢失定位。

## 目标

1. 串口协议新增 IMU 数据帧 (type 0x03)
2. ROS2 侧发布 `/imu/data_raw` 话题
3. 配置 robot_localization EKF 融合里程计 + IMU
4. 更新 launch 文件
5. 补充 STM32 固件参考代码

## 操作步骤

### Step 1: 串口协议新增 type 0x03

**帧格式**:
```
[0xAA][0x03][48,0][6×double LE][0x55]  (总长 53 字节)
  angular_velocity_x, angular_velocity_y, angular_velocity_z  (gyro, rad/s)
  linear_acceleration_x, linear_acceleration_y, linear_acceleration_z (accel, m/s²)
```

**为什么选 6×double**: 与现有 type 0x01 (里程计) 完全一致的数据格式, STM32 固件只需读 MPU6050 不同寄存器即可复用打包代码。

### Step 2: 修改 serial_odom_receiver.py

- 文件: `alloy_chassis_mecanum/scripts/serial_odom_receiver.py`
- 新增 68 行:
  - `from sensor_msgs.msg import Imu` 导入
  - `self.imu_pub = self.create_publisher(Imu, '/imu/data_raw', 10)`
  - `process_imu_frame(self, data)` — 解包 → 填充 Imu 消息 → publish
  - `elif frame_type == 0x03:` 分发
  - STM32 固件参考注释 (docstring)

**关键设计决策**:
- IMU 消息 `orientation_covariance[0] = -1` 表示姿态无效 (STM32 不做姿态解算)
- EKF 自己从 gyro z 积分得到 yaw, 不需要外部姿态
- gyro z 协方差设为 0.005 (高可信度), xy 为 0.01

### Step 3: 创建 EKF 配置

- 文件: `alloy_chassis_mecanum/config/ekf.yaml` (新建)
- EKF 策略:
  - `two_d_mode: true` — 差速底盘 2D 模式
  - `odom0: /odom` — 使用 vx, vy (约束=0), vyaw
  - `imu0: /imu/data_raw` — 仅使用 vyaw (gyro z)
  - `publish_tf: true` — EKF 发布 odom→base_link TF
  - Remap `/odometry/filtered` → `/odom` — **Nav2 无需任何配置改动**

**为什么不融合 IMU 的加速度**: 差速底盘速度低 (<0.26m/s), 加速度计的信噪比不足以提供有用信息。gyro z 对航向漂移抑制效果最好。

### Step 4: 修改 launch 文件

修改了两个 launch 文件, 均新增:
1. `ekf_node` — `robot_localization` 的 `ekf_node` (加载 ekf.yaml)
2. `imu_tf` — `base_link → imu_link` static transform (0,0,0.05,0,0,0)

位置: serial_odom 之后, SLAM Toolbox 之前。

- `robot_integrated.launch.py` — 一体化 launch
- `navigation_with_robot.launch.py` — 纯导航 launch

### Step 5: 更新依赖

- `alloy_chassis_mecanum/package.xml`:
  - 新增 `<exec_depend>robot_localization</exec_depend>`
  - 新增 `<exec_depend>sensor_msgs</exec_depend>`
- 系统: `sudo apt install ros-humble-robot-localization`

## 遇到的问题

无。本次是纯新增功能，没有修改已有逻辑，不影响现有功能。

## 验证结果

- [x] `serial_odom_receiver.py` 语法检查通过
- [x] `ekf.yaml` YAML 合法
- [x] 两个 launch 文件语法通过
- [x] `package.xml` XML 合法
- [ ] 实物验证: 需要 STM32 固件实现 0x03 帧发送后, 在 RDK X5 上测试 `/imu/data_raw` 话题

## STM32 固件参考 (待实现)

```c
// mpu6050.c — 读取 MPU6050 并发送 type 0x03 帧
void MPU6050_SendIMUFrame(void) {
    int16_t gyro[3], accel[3];
    MPU6050_ReadGyro(gyro);   // 寄存器 0x43-0x48
    MPU6050_ReadAccel(accel);

    // 转换为 double (rad/s, m/s²)
    double imu_data[6];
    imu_data[0] = gyro[0] / 131.0  * 3.14159 / 180.0;  // gyro x (rad/s)
    imu_data[1] = gyro[1] / 131.0  * 3.14159 / 180.0;  // gyro y
    imu_data[2] = gyro[2] / 131.0  * 3.14159 / 180.0;  // gyro z
    imu_data[3] = accel[0] / 16384.0 * 9.8;  // accel x (m/s²)
    imu_data[4] = accel[1] / 16384.0 * 9.8;  // accel y
    imu_data[5] = accel[2] / 16384.0 * 9.8;  // accel z

    // 发送帧 (与 type 0x01 相同的打包模式)
    uint8_t frame[53];
    frame[0] = 0xAA;
    frame[1] = 0x03;
    frame[2] = 48; frame[3] = 0;  // length = 48 LE
    memcpy(frame + 4, imu_data, 48);
    frame[52] = 0x55;
    HAL_UART_Transmit(&huart2, frame, 53, 100);
}

// 在 FreeRTOS 任务中每 10ms 调用一次
// (与里程计发送任务同频, 或独立 100Hz 任务)
```

## 参考

- 现有日志: `f407_uart_receive_fix.log` (串口协议端序修复细节)
- robot_localization 文档: https://docs.ros.org/en/humble/Tutorials/Advanced/Navigation2/Integration/EKF.html
