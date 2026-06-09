# RDK X5 机器人项目 — 移植指南

## 项目概览

ROS2 Humble 机器人系统：差分驱动底盘 + 6轴舵机机械臂 + 激光SLAM导航 + 双目视觉。

### 硬件架构

```
RDK X5 (Ubuntu 22.04 / ROS2 Humble)
  ├── STM32F407 (USB CDC /dev/ttyACM0) — FreeRTOS + 电机控制 + PCA9685舵机驱动
  ├── YDLIDAR X2 (USB /dev/ttyUSB0) — 激光雷达 SLAM
  ├── 3D 双目摄像头 (USB /dev/video0) — 物体检测 + 测距
  └── 4×编码器电机 (差分驱动)
```

### 软件包结构

```
RDK_X5/robot_ws/src/
├── alloy_chassis_mecanum/   # 底盘核心包
│   ├── launch/              # 导航启动、一体化启动
│   ├── config/              # nav2_params.yaml、RViz、BT XML
│   ├── scripts/             # serial_odom_receiver.py (串口里程计)
│   │                        # robot_desc_pub.py (URDF topic发布)
│   ├── urdf/                # chassis URDF + robot_combined.urdf
│   └── meshes/              # 底盘 STL 模型
├── arm_ros2/                # 机械臂 URDF + STL 模型
├── elite_test/              # MoveIt2 配置 + 运动规划
│   ├── config/              # arm.srdf、joint_limits、kinematics
│   ├── launch/              # demo.launch.py、arm_full.launch.py
│   ├── scripts/             # arm_bridge.py (RViz→串口桥接)
│   │                        # arm_fixed_grasp_test.py (抓取序列)
│   └── logs/                # 调试日志
├── robot_brain/             # 智能控制包
│   ├── scripts/             # brain_pipeline.py (语音→视觉→决策)
│   │                        # doubao_client.py (豆包 Vision API)
│   │                        # yolo_stereo_detect.py (YOLOv8 + 双目)
│   │                        # voice_chat.py (豆包语音对话)
│   └── config/              # api_config.yaml
└── ydlidar_ros2_driver/     # YDLIDAR 驱动 (C++, 需 ARM64 编译)
```

### 通信协议

STM32 ↔ ROS2 通过 USART2 串口 (115200 8N1):

```
帧格式: [0xAA][type:1B][length:2B LE][data][0x55]

上行 (STM32→ROS2):
  type 0x01: 里程计 (6×double, 48B)
  type 0x02: TF     (3×double, 24B)

下行 (ROS2→STM32):
  type 0x10: 速度指令 (2×float, 8B)
  type 0x20: 关节角度 (6×float, 24B)
```

### 关键修复点（已完成，不要回退）

1. `arm_ros2/urdf/arm.urdf`: 关节类型 continuous → revolute, 限位更新
2. `elite_test/config/arm.srdf`: 自碰撞禁表(ACL) 8对, virtual_joint → world
3. `elite_test/config/joint_limits.yaml`: 整数→浮点, 添加 min/max position
4. `elite_test/config/kinematics.yaml`: timeout 0.005→0.05
5. `alloy_chassis_mecanum/config/nav2_params.yaml`: BT XML 插件修复, lifecycle timeout=15s
6. `serial_odom_receiver.py`: 死区过滤(<0.005归零) + 位置保持
7. URDF 路径: 26处 file:// → package://
8. STM32 USART2: DMA→HAL_UART_Receive_IT (逐字节中断)
9. STM32 frame parser: 非匹配帧不消费(consumed=0)
10. `robot_desc_pub.py`: TransientLocal 发布 /robot_description topic (JSP 需要)

---

## 移植步骤

### 1. 前置准备

```bash
# 确认系统版本
lsb_release -a  # 必须是 Ubuntu 22.04 jammy

# 安装 ROS2 Humble (如未装)
# 参考: https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html
sudo apt install ros-humble-desktop ros-humble-navigation2 ros-humble-nav2-bringup \
  ros-humble-slam-toolbox ros-humble-moveit ros-humble-ros2-control \
  ros-humble-ros2-controllers ros-humble-joint-state-publisher \
  ros-humble-robot-state-publisher ros-humble-xacro

# Python 依赖
pip install pyserial gTTS playsound edge-tts opencv-python numpy
pip install torch torchvision --index-url https://download.pytorch.org/whl/cpu  # CPU版
pip install speechrecognition  # 语音识别
```

### 2. 复制项目

```bash
# 从开发机复制整个项目
scp -r user@dev-machine:~/RDK_X5 ~/

# 或者用 U盘/共享文件夹
rsync -av /path/to/RDK_X5 ~/
```

### 3. 编译

```bash
cd ~/RDK_X5/robot_ws
# 先删旧的 build/install
rm -rf build/ install/

# 编译 (Python 包很快, YDLIDAR 是唯一的 C++ 包)
colcon build --symlink-install

# 如果 YDLIDAR 编译失败 (ARM64 SDK问题):
# 方案A: 安装 ARM64 YDLIDAR SDK
# 方案B: 跳过激光, 先跑机械臂
colcon build --symlink-install --packages-skip ydlidar_ros2_driver
```

### 4. 连接硬件

| 设备 | Linux 设备 | 确认命令 |
|------|-----------|---------|
| STM32 串口 | `/dev/ttyACM0` | `ls /dev/ttyACM0` |
| YDLIDAR | `/dev/ttyUSB0` | `ls /dev/ttyUSB0` |
| 双目摄像头 | `/dev/video0` | `v4l2-ctl --list-devices` |

### 5. 启动命令

```bash
source ~/RDK_X5/robot_ws/install/setup.bash

# 方案A: 纯导航
ros2 launch alloy_chassis_mecanum navigation_with_robot.launch.py

# 方案B: 机械臂 RViz 控制
ros2 launch elite_test arm_full.launch.py

# 方案C: 一体化 (底盘 + 机械臂)
ros2 launch alloy_chassis_mecanum robot_integrated.launch.py

# 双目 + YOLO 检测
~/RDK_X5/robot_ws/src/robot_brain/scripts/yolo_stereo_detect.py

# 豆包语音对话
~/RDK_X5/robot_ws/src/robot_brain/scripts/voice_chat.py
```

### 6. 笔记本远程 RViz

```bash
# 笔记本上 (22.04 + Humble)
source /opt/ros/humble/setup.bash
source ~/RDK_X5/robot_ws/install/setup.bash  # 必须同版本编译过
rviz2 -d ~/RDK_X5/robot_ws/src/alloy_chassis_mecanum/config/navigation_with_robot.rviz
```

### 7. 已知问题

| 问题 | 原因 | 解决 |
|------|------|------|
| lifecycle 超时 | controller 配置慢 | 自动重试, 等1分钟 或 `ros2 lifecycle set` 手动激活 |
| "no map received" | SLAM 需要激光数据 | 确认 `/dev/ttyUSB0` 在线 |
| odom TF 缺失 | STM32 串口数据损坏 | 重启 STM32, 检查 USB 线 |
| YOLO 权重下载慢 | 网络 | 手动下载放 `/tmp/` |
| 豆包 API 401 | Key 格式错误 | 确认 Ark 控制台 key (`ark-xxx`) |

### 8. 模型文件位置

| 文件 | 路径 | 大小 |
|------|------|------|
| YOLOv3-tiny cfg | `/tmp/yolov3-tiny.cfg` | 2KB |
| YOLOv3-tiny weights | `/tmp/yolov3-tiny.weights` | 34MB |
| YOLOv8n ONNX | `/tmp/yolov8n.onnx` | 6MB |
| YOLOv5 源码 | `~/Share/ultralytics-main/` | - |
| Edge TTS | pip 安装 | - |

### 9. 豆包 API 配置

文件: `robot_brain/config/api_config.yaml`

```yaml
doubao:
  api_key: "ark-4bf39fb4-59ad-4dbe-a983-3dbc441231f2-f0c41"
  model: "ep-20260609152524-hhxtc"
  endpoint: "https://ark.cn-beijing.volces.com/api/v3"
```

### 10. 快速验证清单

- [ ] `ros2 node list` 能看到节点
- [ ] `/scan` topic 有数据 (激光)
- [ ] `/odom` topic 有数据 (编码器)
- [ ] `/tf` 有 odom→base_link
- [ ] RViz 显示地图 (灰色区域)
- [ ] RViz 显示激光扫描点 (红色)
- [ ] RViz 显示机器人模型
- [ ] 机械臂关节滑块能动
