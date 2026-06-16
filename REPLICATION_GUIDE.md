# RDK X5 智能服务机器人 — 从零复刻指南

> 适用读者：有基本 Linux/Python 基础，想手动复刻本项目的初学者  
> 预计耗时：硬件搭建 2 周 + 软件开发 4 周  
> 总代码量：~8000 行 (ROS2 Python + C)

---

## 目录

1. [项目全景](#1-项目全景)
2. [硬件清单与搭建](#2-硬件清单与搭建)
3. [软件环境搭建](#3-软件环境搭建)
4. [项目结构总览](#4-项目结构总览)
5. [STM32 固件](#5-stm32-固件)
6. [ROS2 底盘驱动 + 导航](#6-ros2-底盘驱动--导航)
7. [MoveIt2 机械臂](#7-moveit2-机械臂)
8. [视觉检测 (YOLO + 双目测距)](#8-视觉检测-yolo--双目测距)
9. [AI 智能控制 (豆包 + 语音)](#9-ai-智能控制-豆包--语音)
10. [一体化集成](#10-一体化集成)
11. [踩坑大全 (21+ 问题)](#11-踩坑大全-21-问题)
12. [验证清单](#12-验证清单)

---

## 1. 项目全景

```
┌──────────────────────────────────────────────────────┐
│                    RDK X5 (RK3588)                    │
│                   Ubuntu 22.04 + ROS2 Humble          │
│                                                       │
│  ┌──────────┐  ┌──────────┐  ┌───────────────────┐  │
│  │ Navigation2│  │ MoveIt2  │  │  robot_brain (AI) │  │
│  │ SLAM+NavFn│  │ OMPL规划 │  │  YOLOv8+豆包+语音 │  │
│  │ +DWB控制  │  │ 6轴运动  │  │  视觉决策+抓取    │  │
│  └─────┬─────┘  └────┬─────┘  └────────┬──────────┘  │
│        │             │                 │              │
│  ┌─────┴─────────────┴─────────────────┴──────────┐  │
│  │         串口协议 (115200 8N1)                    │  │
│  │  [0xAA][type][len LE][data][0x55]               │  │
│  │  ↓0x10(速度) ↓0x20(关节)  ↑0x01(里程计)        │  │
│  └──────────────────────┬──────────────────────────┘  │
└─────────────────────────┼─────────────────────────────┘
                          │ USB CDC (DAPLink)
┌─────────────────────────┼─────────────────────────────┐
│              STM32F407  │  FreeRTOS                    │
│  ┌──────────────────────┴──────────────────────────┐  │
│  │  USART2 逐字节中断接收                           │  │
│  │  ├─ 0x10 → 电机 PID (4路, 50Hz)                │  │
│  │  └─ 0x20 → PCA9685 舵机 (I2C3)                 │  │
│  │  上行 → 0x01 编码器里程计 + 0x02 TF             │  │
│  └─────────────────────────────────────────────────┘  │
│  外设: MPU6050(IMU) + 4×编码器电机 + 6×MG996r舵机   │
└───────────────────────────────────────────────────────┘
```

**核心设计思想**：RDK X5 负责"大脑"（导航规划、AI 决策、视觉识别），STM32 负责"小脑+肌肉"（电机 PID 实时控制、舵机驱动、传感器采集）。双芯片通过自定义二进制串口协议通信，分层解耦。

---

## 2. 硬件清单与搭建

### 2.1 物料清单

| 类别 | 器件 | 型号 | 数量 | 单价(约) |
|------|------|------|------|----------|
| 主控 | RDK X5 | 地瓜机器人 (RK3588, 6TOPS NPU) | 1 | ¥800 |
| 底层控制 | STM32F407VET6 | ARM Cortex-M4, 168MHz | 1 | ¥40 |
| 激光雷达 | YDLIDAR X2 | 360° 激光扫描, 8m | 1 | ¥300 |
| 摄像头 | 3D 双目 USB | 640×480×2, 基线 6cm | 1 | ¥80 |
| IMU | MPU6050 | 6 轴 (3 gyro + 3 accel) | 1 | ¥5 |
| 舵机驱动 | PCA9685 | 16 路 PWM, I2C | 1 | ¥10 |
| 舵机 | MG996R | 180° 金属齿轮 | 6 | ¥15 |
| 编码器电机 | JGB37-520 | 12V, 1024 PPR | 4 | ¥50 |
| 电机驱动 | TB6612 / L298N | 双 H 桥 | 2 | ¥10 |
| 串口调试器 | DAPLink | CMSIS-DAP + VCP | 1 | ¥20 |
| 电池 | 12V 锂电池 | 5000mAh+ | 1 | ¥100 |

### 2.2 连接拓扑

```
STM32F407
  ├── USART2 (PA2 TX, PA3 RX) ── DAPLink VCP ── USB ── RDK X5 (/dev/ttyACM0)
  ├── I2C3  (PA8 SCL, PC9 SDA)
  │     ├── PCA9685 (地址 0x80) ── 6×舵机
  │     └── MPU6050 (地址 0x68) ── 角速度+加速度
  ├── TIM3/4/5/8 编码器模式 ← 4×电机编码器 (A/B相)
  ├── TIM1 CH1-4 → TB6612 → 4×电机
  └── GPIO → 电机方向控制

RDK X5
  ├── USB1 ── DAPLink VCP (/dev/ttyACM0)
  ├── USB2 ── YDLIDAR X2 (/dev/ttyUSB0)
  └── USB3 ── 双目摄像头 (/dev/video0)
```

### 2.3 搭建要点

| 注意点 | 详情 |
|--------|------|
| 舵机供电 | MG996r 堵转电流 2.5A×6=15A, **舵机必须独立 6V 大电流供电**, 不可从 STM32 3.3V 取电 |
| 电机供电 | 4 个 12V 电机独立供电, 地与 STM32 共地 |
| I2C 上拉 | PCA9685 和 MPU6050 的 SDA/SCL 需要 4.7kΩ 上拉到 3.3V |
| 串口确认 | DAPLink VCP 实际连接 STM32 的 **USART2**, 不是 USART1 |
| 编码器线序 | 1024 PPR 编码器 A/B 相必须正确, 否则 PID 方向反 |

---

## 3. 软件环境搭建

### 3.1 RDK X5 系统

```bash
# 确认系统 (必须是 Ubuntu 22.04)
lsb_release -a
# Distributor ID: Ubuntu
# Release: 22.04

# 确认 ROS2 Humble 已安装
source /opt/ros/humble/setup.bash
ros2 --version
# 如果没装:
# sudo apt install ros-humble-desktop
```

### 3.2 安装依赖

```bash
# ROS2 核心包
sudo apt install -y \
  ros-humble-navigation2 \
  ros-humble-nav2-bringup \
  ros-humble-slam-toolbox \
  ros-humble-moveit \
  ros-humble-ros2-control \
  ros-humble-ros2-controllers \
  ros-humble-joint-state-publisher \
  ros-humble-robot-state-publisher \
  ros-humble-xacro \
  ros-humble-robot-localization \
  ros-humble-tf2-ros \
  ros-humble-cv-bridge \
  ros-humble-usb-cam

# Python 依赖
pip install pyserial numpy opencv-python
pip install edge-tts speechrecognition openai-whisper
pip install ultralytics  # YOLOv8 (如果网络慢, 见 11.3 替代方案)

# 系统工具
sudo apt install -y v4l-utils ffmpeg cmake git
```

### 3.3 创建工作区

```bash
mkdir -p ~/RDK_X5/robot_ws/src
cd ~/RDK_X5/robot_ws

# 创建 5 个功能包骨架
cd src
ros2 pkg create alloy_chassis_mecanum --build-type ament_cmake
ros2 pkg create arm_ros2 --build-type ament_cmake
ros2 pkg create elite_test --build-type ament_cmake
ros2 pkg create robot_brain --build-type ament_cmake
# ydlidar_ros2_driver 从 GitHub clone (ARM64 需预编译或交叉编译)
```

### 3.4 Wayland → X11 (解决 MoveIt/RViz 显示问题)

```bash
# RDK X5 默认可能是 Wayland, MoveIt Setup Assistant 在 Wayland 下会无响应
# 在 ~/.bashrc 末尾添加:
echo 'export QT_QPA_PLATFORM=xcb' >> ~/.bashrc
echo 'export GDK_BACKEND=x11' >> ~/.bashrc
source ~/.bashrc
```

---

## 4. 项目结构总览

```
RDK_X5/
├── robot_ws/src/                  # ROS2 工作空间 (5个功能包)
│   ├── alloy_chassis_mecanum/     # ① 底盘驱动 + Navigation2 配置
│   │   ├── config/
│   │   │   ├── nav2_params.yaml   # Nav2 全部参数 (规划器/控制器/行为树)
│   │   │   └── ekf.yaml           # robot_localization EKF 融合配置
│   │   ├── launch/
│   │   │   ├── robot_integrated.launch.py   # ★ 一体化启动 (底盘+机械臂+导航)
│   │   │   ├── navigation_with_robot.launch.py
│   │   │   └── slam_with_robot.launch.py
│   │   ├── scripts/
│   │   │   └── serial_odom_receiver.py      # ★ 双向串口桥接 (0x01/0x02/0x03/0x10)
│   │   └── urdf/
│   │       └── robot_combined.urdf          # 底盘+机械臂合并 URDF
│   ├── arm_ros2/                  # ② 机械臂 URDF 模型
│   │   └── urdf/arm.urdf          # 6轴舵机机械臂 (含限位)
│   ├── elite_test/                # ③ MoveIt2 运动规划
│   │   ├── config/
│   │   │   ├── arm.srdf           # 自碰撞禁表 + virtual_joint
│   │   │   ├── kinematics.yaml    # KDL 运动学参数
│   │   │   └── joint_limits.yaml  # 关节限位
│   │   ├── launch/demo.launch.py  # RViz + MoveGroup 启动
│   │   └── scripts/
│   │       ├── arm_bridge.py      # ★ RViz滑块 → STM32 串口桥接
│   │       └── arm_fixed_grasp_test.py  # 抓取序列测试
│   ├── robot_brain/               # ④ AI 智能控制
│   │   ├── scripts/
│   │   │   ├── camera_utils.py    # 相机自动检测 (双目/RGB-D/RGB)
│   │   │   ├── doubao_client.py   # ★ 豆包 Vision Pro API
│   │   │   ├── detector.py        # YOLO+深度+稳定性检测
│   │   │   ├── navigator.py       # Nav2 导航封装
│   │   │   ├── grasper.py         # 机械臂抓取序列
│   │   │   ├── integrated_patrol.py   # 巡检→检测→抓取 闭环
│   │   │   ├── voice_commander.py     # ★ 语音→视觉→动作 全链路
│   │   │   └── voice_chat.py      # 语音对话 (Whisper + Edge TTS)
│   │   └── launch/
│   │       ├── integrated_patrol.launch.py
│   │       └── voice_commander.launch.py
│   └── ydlidar_ros2_driver/       # ⑤ YDLIDAR X2 驱动 (C++)
│       ├── params/X2.yaml         # 雷达参数
│       └── config/mapper_params_online_async.yaml  # SLAM 参数
├── PORTING_GUIDE.md               # 移植指南
└── ydlidar_ros2_ws/               # YDLIDAR 独立工作区
```

### 关键数据流

```
激光雷达 → /scan → slam_toolbox → /map
                                      ↓
编码器 → STM32(0x01) → /odom → EKF → /odom (filtered) → Nav2 规划器
IMU    → STM32(0x03) → /imu/data_raw ↗
                                      ↓
Nav2 /cmd_vel → STM32(0x10) → 电机PID → 底盘运动
                                      ↓
语音/WIFI → Doubao Vision Pro → JSON决策
  ├─ navigate → Navigator.goto() → /navigate_to_pose
  ├─ search → YOLO检测 → 抓取序列
  └─ speak → Edge TTS → 语音播报
                                      ↓
RViz滑块 → /joint_states → arm_bridge → STM32(0x20) → PCA9685 → 舵机
```

---

## 5. STM32 固件

### 5.1 工程结构

```
F407_Base_Control/
├── Core/
│   ├── Inc/
│   │   ├── main.h
│   │   ├── ros2_uart.h        # 串口协议解析 + 运动学转换 ★
│   │   ├── pca9685.h          # PCA9685 I2C 舵机驱动 ★
│   │   ├── mpu6050.h          # MPU6050 IMU 读数
│   │   ├── encoder.h          # 编码器正交解码
│   │   └── motor_pid.h        # 电机 PID 控制
│   └── Src/
│       ├── main.c             # ★ FreeRTOS 任务 + 串口接收
│       ├── ros2_uart.c        # ★ 帧解析 + 运动学转换
│       ├── pca9685.c          # 舵机角度 → PWM 脉宽
│       ├── mpu6050.c          # 陀螺仪+加速度计读取
│       └── motor_pid.c        # 增量式 PID
└── Drivers/                    # STM32 HAL 库
```

### 5.2 串口协议帧格式 (核心)

```
帧结构: [0xAA][type:1B][length:2B LE][data:N bytes][0x55]

上行 (STM32 → ROS2):
  type 0x01  里程计  6×double LE (48B)  position_x, position_y, orient_z, orient_w, vx, vz
  type 0x02  TF      3×double LE (24B)  x, y, theta
  type 0x03  IMU     6×double LE (48B)  gyro_xyz, accel_xyz  ★ ROS2 侧新增

下行 (ROS2 → STM32):
  type 0x10  速度指令  2×float LE (8B)   linear_x, angular_z
  type 0x20  关节角度  6×float LE (24B)  arm_joint1~4, end_joint, gripper_joint
```

### 5.3 关键代码：逐字节中断接收 (main.c)

**这是整个项目调试时间最长的地方。** STM32F4 的 USART2 在同时 TX/RX 时，DMA 空闲中断 (`HAL_UARTEx_ReceiveToIdle_DMA`) 会不可靠。

```c
// ❌ 不可靠: DMA 空闲中断在 USART2 全双工时失效
HAL_UARTEx_ReceiveToIdle_DMA(&huart2, rx_buf, RX_BUF_SIZE);

// ✅ 正确: 改为逐字节中断接收
HAL_UART_Receive_IT(&huart2, &rx_byte, 1);

// 在 HAL_UART_RxCpltCallback 中逐字节处理:
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
    if (huart->Instance == USART2) {
        cmd_buffer[cmd_index++] = rx_byte;
        // 检测到帧尾 0x55 时触发解析
        if (rx_byte == 0x55) {
            ProcessROS2Command(cmd_buffer, cmd_index);
            cmd_index = 0;
        }
        HAL_UART_Receive_IT(&huart2, &rx_byte, 1);  // 重新使能
    }
}
```

### 5.4 关键代码：帧解析器不要吞非匹配帧 (ros2_uart.c)

```c
// ❌ 错误: 类型不匹配时返回 consumed = total_len, 吞掉数据
int ParseVelocityFrame(...) {
    if (buffer[1] != 0x10) {
        *consumed = length;  // 整帧丢弃! 如果是 0x20 关节帧, 舵机永远不会动
        return 0;
    }
}

// ✅ 正确: 不匹配时 consumed = 0, 让下一个解析器处理
int ParseVelocityFrame(...) {
    if (buffer[1] != 0x10) {
        *consumed = 0;  // 我不认识, 让别人处理
        return 0;
    }
}
// 在 main.c 中链式调用:
ret = ParseVelocityFrame(buf, len, &vel, &consumed);
if (ret == 0 && consumed > 0) continue;  // 被消费了
ret = ParseJointAngleFrame(buf, len, &joints, &consumed);
```

### 5.5 关键代码：长度字段是小端序

```c
// Python 发送端: struct.pack('<H', len)  → 小端序
// 当 len=8 时发送 [0x08, 0x00]

// ❌ 错误: 大端序解析 (0x08 << 8) | 0x00 = 2048
uint16_t frame_len = (buf[idx+2] << 8) | buf[idx+3];

// ✅ 正确: 小端序解析
uint16_t frame_len = buf[idx+2] | (buf[idx+3] << 8);  // = 8
```

### 5.6 运动学转换 (ros2_uart.c)

```c
// 差分驱动逆运动学: cmd_vel → 左右轮速度
// 参数定义 (必须与 ROS2 侧一致!)
#define WHEEL_SEPARATION    0.28f   // 轮距 (m)
#define WHEEL_DIAMETER      0.065f  // 轮径 (m)
#define ENCODER_PPR         1024    // 编码器线数
#define CONTROL_PERIOD_MS   50      // 控制周期 (ms), 与 PID 任务频率一致

void VelocityToMotorSpeed(const VelocityCmd_t *vel, MotorSpeedCmd_t *motor) {
    // 编码器计数/50ms = v(m/s) × PPR × 50ms / (π × D × 1000ms)
    float scale = ENCODER_PPR * CONTROL_PERIOD_MS / (3.1416f * WHEEL_DIAMETER * 1000.0f);
    // = 1024 × 50 / (3.1416 × 0.065 × 1000) ≈ 250.7

    float v_left  = vel->linear_x - vel->angular_z * WHEEL_SEPARATION / 2.0f;
    float v_right = vel->linear_x + vel->angular_z * WHEEL_SEPARATION / 2.0f;

    motor->left_speed  = (int32_t)(v_left  * scale);
    motor->right_speed = (int32_t)(v_right * scale);
    motor->left_dir    = (v_left  >= 0) ? FORWARD : REVERSE;
    motor->right_dir   = (v_right >= 0) ? FORWARD : REVERSE;
    // 负速度 → REVERSE 方向 + 绝对值
    if (motor->left_speed  < 0) motor->left_speed  = -motor->left_speed;
    if (motor->right_speed < 0) motor->right_speed = -motor->right_speed;
}
```

### 5.7 舵机驱动 (pca9685.c)

```c
// PCA9685 通过 I2C3 控制, 地址 0x80 (STM32 HAL 需要左移 1 位)
// 50Hz PWM: prescaler = 25MHz / (4096 × 50Hz) - 1 ≈ 121
// 舵机 0~180° → 脉宽 0.5ms~2.5ms → PWM 计数 102~512

void SetServoAngle(uint8_t channel, float angle_deg) {
    angle_deg = clamp(angle_deg, 0.0f, 180.0f);
    // 角度 → 脉宽 (ms): 0.5 + angle/180 × 2.0
    float pulse_ms = 0.5f + angle_deg / 180.0f * 2.0f;
    // 脉宽 → PWM 计数 (4096 分辨率, 50Hz 周期 = 20ms)
    uint16_t pwm = (uint16_t)(pulse_ms / 20.0f * 4096.0f);
    PCA9685_SetChannel(channel, 0, pwm);
}

void SetJointAngles(float joints[6]) {
    // ROS2 关节角度 (rad) → 舵机角度 (°)
    for (int i = 0; i < 6; i++) {
        float deg = joints[i] * 180.0f / 3.14159f + 90.0f;  // rad→°, 中位=90°
        SetServoAngle(i, deg);
    }
}
```

---

## 6. ROS2 底盘驱动 + 导航

### 6.1 串口里程计接收器 (serial_odom_receiver.py)

这是 ROS2 与 STM32 之间的**唯一通信通道**。核心逻辑：

```python
class SerialOdomReceiver(Node):
    def __init__(self):
        # 订阅 /cmd_vel → 下行发送 0x10 帧到 STM32
        self.cmd_vel_sub = self.create_subscription(Twist, '/cmd_vel', self.cmd_vel_cb, 10)

        # 发布 /odom → 供 Nav2 使用
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)

        # 发布 /imu/data_raw → 供 EKF 融合使用 ★
        self.imu_pub = self.create_publisher(Imu, '/imu/data_raw', 10)

        # 10ms 定时器 → 处理串口缓冲区
        self.timer = self.create_timer(0.01, self.process_serial_data)

        # ★ 看门狗: 0.5s 无 /cmd_vel 自动停车
        self.watchdog_timer = self.create_timer(0.05, self.watchdog_check)

    def process_serial_data(self):
        """逐帧解析: 找 0xAA → 读 type+len → 验证 0x55 → 分发"""
        while len(self.buffer) >= 5:  # 最小帧 5 字节
            start = self.buffer.find(0xAA)
            if start == -1: break
            frame_type = self.buffer[start+1]
            frame_len  = struct.unpack('<H', self.buffer[start+2:start+4])[0]
            if frame_len > 256:  # 异常长度, 丢弃
                self.buffer = self.buffer[start+1:]
                continue
            total_len = 5 + frame_len
            if len(self.buffer) < total_len: break  # 不完整帧
            if self.buffer[start+total_len-1] != 0x55:  # 帧尾不对
                self.buffer = self.buffer[start+1:]
                continue
            frame_data = self.buffer[start+4:start+4+frame_len]
            if frame_type == 0x01:   self.process_odometry_frame(frame_data)
            elif frame_type == 0x02: self.process_tf_frame(frame_data)
            elif frame_type == 0x03: self.process_imu_frame(frame_data)   # ★ 新增
            self.buffer = self.buffer[start+total_len:]

    def process_odometry_frame(self, data):
        """解包 6×double → 发布 /odom + TF"""
        px, py, oz, ow, vx, vz = struct.unpack('<dddddd', data)

        # ★ 死区过滤: 编码器噪声 <0.005 m/s 归零
        if abs(vx) < 0.005: vx = 0.0
        if abs(vz) < 0.005: vz = 0.0

        # ★ 位置保持: 速度为零时锁住位置 (防止编码器噪声漂移)
        if vx == 0.0 and vz == 0.0:
            px, py = self.last_x, self.last_y

        self.last_x, self.last_y = px, py
        # ... 填充 Odometry 消息, publish
```

### 6.2 Nav2 配置关键点 (nav2_params.yaml)

```yaml
# ★ 行为树插件: Humble 不包含 are_error_codes_active, 必须移除
bt_navigator:
  ros__parameters:
    plugin_lib_names:
      - nav2_compute_path_to_pose_action_bt_node
      # ... 其他插件
      - nav2_remove_passed_goals_action_bt_node      # ★ Humble 必须加这个
      - nav2_navigate_through_poses_action_bt_node   # ★ Humble 必须加这个
      # nav2_are_error_codes_active_condition_bt_node # ★ 移除, Humble 不存在

    # ★ 参数名: Humble 是 default_nav_to_pose_bt_xml, 不是 default_bt_xml_filename
    default_nav_to_pose_bt_xml: "/opt/ros/humble/share/nav2_bt_navigator/behavior_trees/navigate_to_pose_w_replanning_and_recovery.xml"

# ★ 生命周期管理器: 节点名需要前缀斜杠
lifecycle_manager_navigation:
  ros__parameters:
    autostart: True
    node_names:
      - /controller_server    # ★ 必须有 /
      - /planner_server
      - /behavior_server
      - /bt_navigator
      - /smoother_server
      - /global_costmap/global_costmap    # ★ 代价地图也要管理
      - /local_costmap/local_costmap

# ★ DWB 控制器参数 (差速底盘)
controller_server:
  ros__parameters:
    controller_plugins: ["FollowPath"]
    FollowPath:
      plugin: "dwb_core::DWBLocalPlanner"
      max_vel_x: 0.26        # 最大 0.26 m/s
      max_vel_theta: 1.0     # 最大 1.0 rad/s
      xy_goal_tolerance: 0.25 # 到达精度 25cm
```

### 6.3 EKF 融合 (ekf.yaml) — 解决"IMU 写了文档没写代码"

```yaml
ekf_filter_node:
  ros__parameters:
    frequency: 30.0
    two_d_mode: true            # 差速底盘 = 2D
    publish_tf: true            # 发布 odom→base_link TF

    odom0: /odom
    odom0_config: [false,false,false, false,false,false,
                   true,true,false, false,false,true,
                   false,false,false]
    # 使用: vx, vy(=0), vyaw

    imu0: /imu/data_raw
    imu0_config: [false,false,false, false,false,false,
                  false,false,false, false,false,true,
                  false,false,false]
    # ★ 只用 gyro z 修正编码器航向漂移
```

### 6.4 编译和运行

```bash
cd ~/RDK_X5/robot_ws
colcon build --symlink-install
source install/setup.bash

# 纯导航
ros2 launch alloy_chassis_mecanum navigation_with_robot.launch.py

# 验证
ros2 topic echo /odom          # 有数据 = 串口 OK
ros2 topic echo /scan          # 有数据 = 激光 OK
ros2 topic echo /imu/data_raw  # 有数据 = IMU OK ★
ros2 lifecycle get /controller_server  # 应显示 active
```

---

## 7. MoveIt2 机械臂

### 7.1 URDF 关键修复

机械臂 URDF 需要在 MoveIt Setup Assistant 生成后手动修复：

```xml
<!-- ❌ 错误: continuous 关节无限制, MoveIt 报错 -->
<joint name="arm_joint1" type="continuous">

<!-- ✅ 正确: revolute + 限位 -->
<joint name="arm_joint1" type="revolute">
  <limit lower="-3.14" upper="3.14" effort="10" velocity="3.14"/>
```

### 7.2 SRDF 自碰撞禁表 + virtual_joint

```xml
<!-- ★ virtual_joint 父帧必须是 world, 不能是 base_link (循环引用!) -->
<virtual_joint name="virtual_joint" type="fixed"
    parent_frame="world" child_link="base_link"/>

<!-- ★ 禁用自碰撞: 相邻连杆无须检测 -->
<disable_collisions link1="base_link" link2="arm_Link1" reason="Adjacent"/>
<disable_collisions link1="arm_Link1" link2="arm_Link2" reason="Adjacent"/>
<!-- ... 共 8 对 -->
```

### 7.3 运动学配置 (kinematics.yaml)

```yaml
arm_group:
  kinematics_solver: kdl_kinematics_plugin/KDLKinematicsPlugin
  kinematics_solver_search_resolution: 0.005
  kinematics_solver_timeout: 0.05  # ★ 0.005→0.05, 否则 KDL 超时返回 0 航点
```

### 7.4 关节限位 (joint_limits.yaml)

```yaml
# ★ 数值必须是浮点, 整数会崩溃
joint_limits:
  arm_joint1:
    has_position_limits: true
    min_position: -3.14   # ★ 必须有这个字段
    max_position: 3.14
    max_velocity: 3.14    # ★ 必须 3.14 不是 3
```

### 7.5 arm_bridge.py — RViz 滑块 → 物理舵机

```python
class ArmBridge(Node):
    def __init__(self):
        # ★ 同时监听两个话题: joint_states + 控制器指令
        self.sub = self.create_subscription(JointState, '/joint_states', self.js_cb, 10)
        self.cmd_sub = self.create_subscription(JointState,
            '/arm_group_controller/command', self.cmd_cb, 10)

        # ★ 10Hz 节流发送 (不用每帧都发串口)
        self.timer = self.create_timer(0.1, self.send_if_changed)

    def send_if_changed(self):
        """变化超过阈值才发送, 避免串口拥堵"""
        if not self.changed: return
        data = struct.pack('<ffffff', *self.joint_positions)
        frame = bytearray([0xAA, 0x20, 24, 0]) + data + bytearray([0x55])
        self.ser.write(frame)
```

### 7.6 MoveGroup Action 死锁问题

```python
# ❌ 错误: 在 ROS2 回调中调用 spin_until_future_complete → 死锁!
def timer_callback(self):
    fut = self.client.send_goal_async(goal)
    rclpy.spin_until_future_complete(self, fut)  # 永远不返回!

# ✅ 正确: 在 main() 中调用 (spin 之前)
def main():
    node = FixedGraspTest()
    time.sleep(3.0)           # 等 MoveGroup 完全启动
    node.run_sequence()       # 同步调用, 此时尚未 spin
    node.destroy_node()
```

---

## 8. 视觉检测 (YOLO + 双目测距)

### 8.1 相机自动适配 (camera_utils.py)

```python
class CameraWrapper:
    """自动检测摄像头类型: 双目 / Astra Pro RGB-D / 普通 USB"""
    def __init__(self, cam_id=0):
        for i in range(4):
            cap = cv2.VideoCapture(i)
            ret, frame = cap.read()
            if not ret: continue
            w = frame.shape[1]
            if w == 1280:  # 双目合成图 (640×2)
                left = frame[:, :640]; right = frame[:, 640:]
                diff = cv2.absdiff(left, right).mean()
                if diff > 10:  # 左右视图差异大 → 真双目
                    self.mode = 'stereo'
                    break
            self.mode = 'rgb'  # 普通摄像头
        # 尝试 Astra Pro SDK
        try:
            from pyorbbecsdk import Pipeline
            self.mode = 'rgbd'  # RGB-D 深度相机
        except ImportError:
            pass
```

### 8.2 YOLO 检测 + 稳定性过滤 (detector.py)

```python
class Detector:
    """YOLOv8n 检测 + 深度估计 + 5帧稳定性过滤"""
    def __init__(self):
        self.model = YOLO("yolov8n.pt")  # 6MB, CPU 3-5 FPS
        self.model.conf = 0.3
        self.target_classes = {'mouse', 'bottle', 'cup', 'cell phone',
                               'book', 'banana', 'apple'}
        self._detect_history = deque(maxlen=5)   # 5帧滑动窗口
        self._grasp_cooldown = 0                 # 30帧冷却 (~3s)

    def detect(self) -> DetectionResult:
        img, depth = self.camera.read()
        results = self.model(img, verbose=False)[0]

        # 找最近的有效目标
        closest = None
        for box in results.boxes:
            cls = self.model.names[int(box.cls)]
            if cls not in self.target_classes: continue

            # 深度估计: 深度图中目标 ROI 的中位值
            if depth is not None:
                roi = depth[cy-r:cy+r, cx-r:cx+r]
                dist = np.median(roi[(roi > 100) & (roi < 10000)]) / 1000.0
            else:
                dist = 0.05 * 320 / max(bw, 1)  # 无深度 → 用框宽估算

        # ★ 稳定性: 连续 N 帧检测到同一类别才算有效
        self._detect_history.append(closest_class)
        is_stable = all(d == closest_class for d in list(self._detect_history)[-4:])

        return DetectionResult(class_name=closest_class, distance_m=dist,
                               is_stable=is_stable, ...)
```

### 8.3 双目测距原理 (yolo_stereo_detect.py)

```
Z = (f × B) / d

  Z   = 距离 (m)
  f   = 焦距 (像素), 默认 700
  B   = 基线 (m), 默认 0.06
  d   = 视差 (像素), SGBM 计算

SGBM 参数:
  numDisparities = 64  (实际=64×16=1024 级)
  blockSize = 5
  uniquenessRatio = 10
  speckleWindowSize = 100
```

---

## 9. AI 智能控制 (豆包 + 语音)

### 9.1 豆包 Vision Pro API (doubao_client.py)

```python
class DoubaoClient:
    def __init__(self, api_key, model="doubao-1.5-vision-pro-32k"):
        self.endpoint = "https://ark.cn-beijing.volces.com/api/v3"

    def chat_with_image(self, user_text, image_path, system_prompt=""):
        """拍照 → base64 → 豆包 Vision Pro → 决策文本"""
        with open(image_path, "rb") as f:
            image_b64 = base64.b64encode(f.read()).decode()
        messages = [
            {"role": "system", "content": system_prompt},
            {"role": "user", "content": [
                {"type": "image_url", "image_url": {"url": f"data:image/jpeg;base64,{image_b64}"}},
                {"type": "text", "text": user_text}
            ]}
        ]
        # POST 请求 → 返回 JSON 决策
```

### 9.2 语音输入输出 (voice_chat.py)

```python
def listen():
    """arecord 4s → Whisper tiny 转写 → 中文文本"""
    subprocess.run(["arecord", "-D", "plughw:1,0", "-d", "4",
                    "-r", "16000", "-f", "S16_LE", "/tmp/voice.wav"])
    model = whisper.load_model("tiny")  # 80MB, CPU 实时
    result = model.transcribe("/tmp/voice.wav", language="zh", fp16=False)
    return result["text"].strip()

def speak(text):
    """Edge TTS → ffplay 播放 (国内可用, 免费)"""
    async def _run():
        await Communicate(text, 'zh-CN-XiaoxiaoNeural').save('/tmp/tts.mp3')
        subprocess.run(['ffplay', '-nodisp', '-autoexit', '/tmp/tts.mp3'])
    threading.Thread(target=lambda: asyncio.run(_run()), daemon=True).start()
```

### 9.3 语音→动作全链路 (voice_commander.py)

```
🎤 后台线程              🧠 主线程 (spin_once + cv2)
──────────              ─────────────────────────────
listen() ─queue→        while rclpy.ok():
  Whisper 4s录音+转写     spin_once(10ms)
                          if voice_cmd:
                            📸 拍照 /tmp/voice_view.jpg
                            🤖 Doubao Vision Pro 决策
                            → JSON 解析 (嵌套支持)
                            ├─ navigate → Navigator.goto(x,y,yaw)
                            ├─ search   → Detector 循环 → Grasper 抓取
                            ├─ arm      → Grasper.send_joints()
                            └─ none     → speak(回复)
                          🖼️ cv2.imshow + cv2.waitKey(1)
```

**系统提示词设计** (发给豆包的 prompt):

```
你是机器人控制专家。根据摄像头图像和用户指令，决定下一步动作。
返回 JSON: {"action": "...", "speech": "...", "thinking": "..."}

可用动作:
- navigate → {"action":"navigate", "target":{"name":"table"}}
  或 {"action":"navigate", "target":{"distance_m":1.5, "angle_rad":0.2}}
- search → {"action":"search", "target_class":"cup"}
- arm → {"action":"arm", "joints":[0,-0.3,-0.6,0,0,-0.5]}
- home / wave / none
```

**无 API key 时的关键词 fallback**:

```python
KEYWORD_ACTIONS = {
    '抓': 'search', '拿': 'search', '取': 'search',
    '去': 'navigate', '走': 'navigate',
    '回': 'home', '归位': 'home',
    '你好': 'wave', '挥手': 'wave',
}
```

---

## 10. 一体化集成

### 10.1 三种运行模式 (全部独立可用)

```bash
# ★ 模式 1: 全栈一体化 (底盘导航 + 机械臂 + RViz)
ros2 launch alloy_chassis_mecanum robot_integrated.launch.py

# ★ 模式 2: 语音AI 全链路 (在上面启动后)
ros2 launch robot_brain voice_commander.launch.py api_key:=YOUR_KEY

# ★ 模式 3: 自动巡检抓取 (无需语音)
ros2 launch robot_brain integrated_patrol.launch.py grasp_enabled:=true loop:=true

# ★ 单独测试某个功能:
python3 robot_brain/scripts/grasper.py --home-only    # 机械臂 dry-run
python3 robot_brain/scripts/detector.py                # 视觉检测 GUI
python3 robot_brain/scripts/navigator.py               # 导航单点
ros2 run elite_test arm_fixed_grasp_test.py            # 抓取序列
```

### 10.2 模块依赖 (零循环)

```
voice_commander.py ──┬── navigator.py ──── nav2_msgs (ROS2)
                     ├── detector.py ───── camera_utils.py + ultralytics
                     ├── grasper.py ────── pyserial (无 ROS2)
                     └── doubao_client.py (纯 HTTP)
```

---

## 11. 踩坑大全 (21+ 问题)

### 11.1 串口通信类

| # | 问题 | 原因 | 修复 | 耗时 |
|---|------|------|------|------|
| 1 | 舵机不动, Python 帧正确 | DMA 空闲中断在 USART2 全双工时不可靠 | 改为逐字节中断 `HAL_UART_Receive_IT` | 2天 |
| 2 | 帧解析器吞掉关节帧 | 非匹配类型返回 `consumed=total_len` | 返回 `consumed=0` 让下个解析器处理 | 半天 |
| 3 | 长度字段解析为 2048 | C 端按大端序, Python 发小端序 | `buf[i] \| (buf[i+1]<<8)` | 半天 |
| 4 | 轮速不对, 量级差 10 倍 | RPM→编码器计数转换错误, 参数不一致 | 统一用 `v × PPR × 50 / (π×D×1000)` | 半天 |
| 5 | 串口冲突: 两个节点同时打开 `/dev/ttyACM0` | arm_bridge 和 serial_odom 互斥 | 进程管理, 不重复启动 | 2h |

### 11.2 MoveIt / URDF 类

| # | 问题 | 原因 | 修复 |
|---|------|------|------|
| 6 | `continuous joint has no limits` | URDF `type="continuous"` | 改为 `"revolute"` + limit |
| 7 | `expected [double] got [integer]` | joint_limits.yaml 整数 | `1` → `1.0` |
| 8 | KDL 规划 0 航点 | `kinematics_solver_timeout: 0.005` 太短 | → `0.05` |
| 9 | MoveGroup future 永远不完成 | 在 timer 回调中 `spin_until_future_complete` | 在 `main()` 中调用 |
| 10 | `base_link` 自碰撞误报 | SRDF 未设禁表 | 加 8 对 `disable_collisions` |
| 11 | SRDF virtual_joint 循环引用 | `parent_frame="base_link"` | → `"world"` |
| 12 | URDF 编译报错 `unclosed token` | 文件截断, 缺 `</joint></robot>` | 补全 XML |
| 13 | `ModuleNotFoundError: moveit_commander` | Humble 无此包 | 用 `rclpy.action.ActionClient` + `moveit_msgs` |
| 14 | KDL 位姿 IK 全部 FAILURE (err=99999) | joint origin 复杂 rpy 旋转 | 改用关节空间控制 (不求解位姿 IK) |
| 15 | MoveIt Setup Assistant 界面无响应 | Wayland 显示环境 | `export QT_QPA_PLATFORM=xcb` |

### 11.3 导航类

| # | 问题 | 原因 | 修复 |
|---|------|------|------|
| 16 | lifecycle 节点全部 inactive | 节点名缺前缀 `/` | nav2_params 加 `/planner_server` 等 |
| 17 | `libnav2_are_error_codes_active...so` 不存在 | ROS2 Humble 不含此插件 | 从 plugin_lib_names 移除 |
| 18 | BT XML `RemovePassedGoals` 不识别 | 缺 `nav2_remove_passed_goals_action_bt_node` | 补充插件 |
| 19 | odom 漂移, 地图跳动 | 编码器噪声无过滤 | 死区 `<0.005` 归零 + 位置保持 |
| 20 | local_costmap "no map received" | TF 树不完整, 缺 odom→base_link | 加 static TF (或 EKF 自动发布) |

### 11.4 其他

| # | 问题 | 原因 | 修复 |
|---|------|------|------|
| 21 | `pip install ultralytics` 超时 | 国内网络 PyPI 慢 | OpenCV DNN 替代 / 离线下载 whl / 镜像源 |
| 22 | YDLIDAR 大量 Check Sum Error | USB 供电不足 / 干扰 | 独立供电 USB Hub |

### 11.5 调试技巧总结

1. **串口调试**: 用 `xxd /dev/ttyACM0 | head` 查看原始字节流，确认帧头 `AA` 频繁出现
2. **ROS2 调试**: `ros2 topic echo` 逐话题验证；`ros2 lifecycle get` 确认节点激活
3. **舵机调试**: 先用 Arduino/PCA9685 测试程序单独确认舵机能动，排除硬件问题
4. **导航调试**: RViz 中逐层检查 TF 树 (`map→odom→base_link→laser_frame`)
5. **编译调试**: `colcon build --symlink-install --event-handlers console_direct+` 看实时日志

### 11.6 详细日志索引

每个子系统的完整调试记录在各自的 `logs/` 目录下:

| 日志文件 | 内容 |
|----------|------|
| `alloy_chassis_mecanum/logs/serial_bridge_fix.log` | 串口桥接端序修复 + 看门狗实现 |
| `alloy_chassis_mecanum/logs/f407_uart_receive_fix.log` | STM32 帧解析器 + 运动学转换 + 引脚确认 |
| `alloy_chassis_mecanum/logs/navigation_fix_detailed.log` | 生命周期管理器 + TF 树 + BT XML 修复 |
| `alloy_chassis_mecanum/logs/navigation_launch_fix.log` | Nav2 启动残留问题 (KDL/TF/GLSL) |
| `alloy_chassis_mecanum/logs/2026-06-16_imu_fusion.log.md` | IMU 融合实现 (type 0x03 帧 + EKF) |
| `elite_test/logs/2026-05-24_debug_log.md` | 机械臂 15+ 问题修复全集 (舵机/URDF/MoveIt/串口) |
| `robot_brain/logs/2026-06-16_modular_architecture.log.md` | 模块化架构设计 (grasper/navigator/detector) |
| `robot_brain/logs/2026-06-16_voice_commander_impl.log.md` | 语音→动作 全链路实现

---

## 12. 验证清单

在 RDK X5 上完成以下检查, 确保系统正常:

### 硬件连接

- [ ] `ls /dev/ttyACM0` → STM32 串口存在
- [ ] `ls /dev/ttyUSB0` → YDLIDAR 存在 (或不插激光时跳过)
- [ ] `v4l2-ctl --list-devices` → 摄像头存在
- [ ] STM32 上电后串口输出启动日志 (115200 8N1)

### 编译

- [ ] `colcon build --symlink-install` 无报错
- [ ] `source install/setup.bash` 无警告

### 串口通信

- [ ] `ros2 topic echo /odom` 有数据 → 里程计正常
- [ ] `/imu/data_raw` 有数据 → IMU 正常 ★
- [ ] 拧动轮子时 `/odom` 数值变化

### SLAM + 导航

- [ ] `ros2 topic echo /scan` 有数据 → 激光正常
- [ ] RViz 显示激光扫描点 (红色)
- [ ] RViz 显示灰色地图区域
- [ ] `ros2 lifecycle get /controller_server` → active
- [ ] 在 RViz 点击 "2D Goal Pose" 能看到规划路径

### 机械臂

- [ ] RViz 拖动关节滑块 → 真实舵机跟随运动
- [ ] `ros2 run elite_test arm_fixed_grasp_test.py` → 完整抓取序列

### 视觉

- [ ] `python3 detector.py` → 能看到摄像头画面 + YOLO 检测框
- [ ] 杯子/瓶子在画面中 → 显示距离 + STABLE 标志

### 语音 + AI

- [ ] `python3 voice_chat.py` → 麦克风录音正常
- [ ] 说"你好" → Edge TTS 回复
- [ ] 有 API key: `voice_commander` → "去抓杯子" → 完整动作链路

### 一体化

- [ ] `robot_integrated.launch.py` 启动无崩溃
- [ ] RViz 同时显示底盘+机械臂模型
- [ ] 所有 lifecycle 节点 active
- [ ] Ctrl+C 优雅退出, 机械臂归位

---

## 附录 A: 快速命令速查

```bash
# 编译
cd ~/RDK_X5/robot_ws && colcon build --symlink-install && source install/setup.bash

# 启动全家桶
ros2 launch alloy_chassis_mecanum robot_integrated.launch.py

# 语音全链路
ros2 launch robot_brain voice_commander.launch.py api_key:=YOUR_KEY

# 自动巡检抓取
ros2 launch robot_brain integrated_patrol.launch.py grasp_enabled:=true

# 单独功能测试
ros2 run elite_test arm_fixed_grasp_test.py           # 抓取测试
python3 robot_brain/scripts/detector.py                # 视觉检测
python3 robot_brain/scripts/grasper.py --home-only    # 机械臂 dry-run
python3 robot_brain/scripts/navigator.py --waypoints "0.5,0.5,0;1,0,1.57"

# 调试
ros2 topic list | grep -E "odom|scan|cmd_vel|joint"
ros2 node list
ros2 lifecycle get /controller_server
rqt_graph  # 可视化节点图

# 串口原始数据
xxd /dev/ttyACM0 | head -n 5
```

## 附录 B: 文件索引 (快速找代码)

| 想实现什么 | 看哪个文件 |
|-----------|-----------|
| 串口协议帧格式 | `serial_odom_receiver.py` 顶部 docstring |
| STM32 接收解析 | `f407_uart_receive_fix.log` 有完整 C 代码 |
| 舵机如何驱动 | `2026-05-24_debug_log.md` 第 10 节 |
| 导航怎么配 | `navigation_fix_detailed.log` 完整问题修复链 |
| MoveIt 怎么修 | `2026-05-24_debug_log.md` 问题 1-9 |
| YOLO 怎么跑 | `detector.py` 的 `detect()` 方法 |
| 豆包 API 怎么调 | `doubao_client.py` 的 `chat_with_image()` |
| 语音怎么接入 | `voice_commander.py` 的 `run()` 主循环 |
| 一体化 launch | `robot_integrated.launch.py` |
| 移植到新机器 | `PORTING_GUIDE.md` |
