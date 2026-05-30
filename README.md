# RDK X5 机器人项目

## 项目概述

基于 RDK X5 + STM32F407 的 ROS2 Humble 机器人系统，包含差分驱动底盘、6 轴舵机机械臂、激光雷达 SLAM 导航。

## 硬件架构

```
RDK X5 (ROS2) ←→ STM32F407 (FreeRTOS)
    ├── YDLIDAR X2 激光雷达
    ├── PCA9685 舵机驱动 (6× 180° 舵机)
    ├── MPU6050 IMU
    ├── USB 摄像头
    └── 4× 编码器电机 (差分驱动)
```

## 目录结构

| 目录 | 说明 |
|------|------|
| `robot_ws/src/alloy_chassis_mecanum/` | 底盘驱动、里程计、Navigation2 配置 |
| `robot_ws/src/arm_ros2/` | 机械臂 URDF 模型 |
| `robot_ws/src/elite_test/` | MoveIt2 配置、运动规划、抓取测试 |
| `F407_Base_Control/` | STM32F407 固件 (FreeRTOS + HAL) |
| `Share/` | 正在开发的最新固件 |

## 通信协议

### ROS2 ↔ STM32 (串口 115200 8N1)

```
帧格式: [0xAA][type:1B][length:2B LE][data][0x55]

上行 (STM32→ROS2):
  type 0x01: 里程计 (6×double)
  type 0x02: TF (3×double)

下行 (ROS2→STM32):
  type 0x10: 速度指令 (2×float)
  type 0x20: 关节角度 (6×float)
```

## 快速启动

### 导航

```bash
source install/setup.bash
ros2 launch alloy_chassis_mecanum navigation_with_robot.launch.py
```

### 机械臂

```bash
# 终端 1: RViz 控制
ros2 launch elite_test demo.launch.py
# 终端 2: 串口桥接 (真实舵机)
ros2 run elite_test arm_bridge.py
```

### 抓取测试

```bash
ros2 run elite_test arm_fixed_grasp_test.py
```

## 依赖

- ROS2 Humble
- Navigation2
- MoveIt2
- slam_toolbox
- ros2_control + mock_components
- YDLIDAR SDK
- Python: pyserial, numpy, opencv-python

## 日志

详见 `robot_ws/src/elite_test/logs/` 和 `robot_ws/src/alloy_chassis_mecanum/logs/`
