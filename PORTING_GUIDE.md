# RDK X5 机器人项目 — AI 可执行的移植指南

> **目标读者**: 另一个 AI（或人类），在 RDK X5 ARM64 开发板上从零部署本项目。
> **前置假设**: RDK X5 已刷 Ubuntu 22.04，有网络连接，已通过 SSH 登录。

---

## 0. 前置检查 (5 分钟)

在 RDK X5 上执行以下命令，确认环境正确：

```bash
# 1. 确认 ARM64 架构
uname -m
# 期望输出: aarch64

# 2. 确认 Ubuntu 22.04
lsb_release -a
# 期望输出: Ubuntu 22.04.x LTS (jammy)

# 3. 确认磁盘空间 (> 10GB 可用)
df -h /

# 4. 确认内存
free -h
# 期望: >= 4GB (RDK X5 通常有 8GB)

# 5. 确认网络
ping -c 1 www.baidu.com
```

---

## 1. 安装系统依赖 (15 分钟)

### 1.1 基础工具

```bash
sudo apt update && sudo apt install -y \
  git curl wget python3-pip python3-yaml cmake build-essential \
  v4l-utils ffmpeg alsa-utils usbutils \
  ros-humble-desktop \
  ros-humble-navigation2 ros-humble-nav2-bringup \
  ros-humble-slam-toolbox ros-humble-moveit \
  ros-humble-ros2-control ros-humble-ros2-controllers \
  ros-humble-joint-state-publisher ros-humble-robot-state-publisher \
  ros-humble-xacro ros-humble-robot-localization \
  ros-humble-tf2-ros ros-humble-cv-bridge ros-humble-usb-cam
```

> **如果 ROS2 Humble 未安装**: 参考 https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html

### 1.2 Python 依赖

```bash
# 基础依赖
pip install pyserial numpy opencv-python ultralytics edge-tts

# 语音识别 (Whisper + speech_recognition)
pip install openai-whisper SpeechRecognition

# PyTorch ARM64 CPU 版 — 注意：必须是 aarch64 wheel
pip install torch torchvision --index-url https://download.pytorch.org/whl/cpu

# 如果 PyTorch ARM64 不可用，跳过 Whisper，项目会降级为键盘输入
# 验证安装:
python3 -c "import torch; print('PyTorch OK, device:', 'cpu')" 2>/dev/null || echo "PyTorch SKIPPED"
```

> **注意**: 如果 `ultralytics` 安装超时 (国内网络)，用镜像:
> `pip install ultralytics -i https://pypi.tuna.tsinghua.edu.cn/simple`

### 1.3 YDLIDAR SDK (ARM64 编译)

```bash
cd ~
git clone https://github.com/YDLIDAR/YDLidar-SDK.git
cd YDLidar-SDK && mkdir build && cd build
cmake .. && make -j4 && sudo make install
# 验证:
ls /usr/local/lib/libydlidar_sdk* && echo "YDLIDAR SDK OK"
```

**如果编译失败** (交叉编译问题 / 缺少 ARM64 依赖)，跳过:
```bash
echo "YDLIDAR SDK SKIPPED — 导航模式将无激光雷达数据"
```

---

## 2. 传输项目文件 (5 分钟)

### 2.1 传输方式 (选其一)

**方式 A — SCP 直接拷贝** (推荐，如果 dev 机器和 RDK X5 在同一网络):
```bash
# 在 dev 机器上执行:
ssh user@rdkx5-ip "mkdir -p ~/RDK_X5"
scp -r ~/RDK_X5/* user@rdkx5-ip:~/RDK_X5/
```

**方式 B — U 盘** (离线):
```bash
# 在 RDK X5 上:
sudo mount /dev/sda1 /mnt 2>/dev/null || sudo mount /dev/sdb1 /mnt
cp -r /mnt/RDK_X5 ~/
sudo umount /mnt
```

**方式 C — tar 压缩传输** (大文件):
```bash
# 在 dev 机器上:
cd ~/RDK_X5 && tar czf /tmp/rdk_x5.tar.gz \
  --exclude='build' --exclude='install' --exclude='log' \
  --exclude='__pycache__' --exclude='.git' \
  --exclude='ydlidar_ros2_ws/build' --exclude='ydlidar_ros2_ws/install' \
  --exclude='frames_*.pdf' --exclude='frames_*.gv' \
  robot_ws/ ydlidar_ros2_ws/ *.sh *.md *.pt

# 在 RDK X5 上:
scp user@dev-ip:/tmp/rdk_x5.tar.gz ~/
tar xzf rdk_x5.tar.gz -C ~/RDK_X5/
```

### 2.2 清理开发机残留

传输完成后，删除开发机的编译产物（这些含 x86_64 路径，ARM64 必须重建）:

```bash
# 在 RDK X5 上执行:
cd ~/RDK_X5
rm -rf robot_ws/build/ robot_ws/install/ log/
```

---

## 3. 设备连接验证 (10 分钟)

### 3.1 确认所有硬件已连接

```bash
# 检查 STM32 串口 (DAPLink VCP):
ls /dev/ttyACM* 2>/dev/null && echo "STM32 串口: OK" || echo "STM32 串口: MISSING — 检查 USB 线和 STM32 上电"

# 检查 YDLIDAR:
ls /dev/ttyUSB* 2>/dev/null && echo "激光雷达: OK" || echo "激光雷达: MISSING — 检查 USB 线"

# 检查摄像头 (深目双目):
v4l2-ctl --list-devices 2>/dev/null | head -20
# 找到包含 "USB Camera" 或 "3D" 的设备，记录 /dev/videoX 编号

# 检查音频设备:
arecord -l 2>/dev/null || echo "麦克风: MISSING — 语音功能不可用"
```

### 3.2 记录实际设备名

在移植指南的后续步骤中，你看到的 `/dev/ttyACM0`、`/dev/ttyUSB0`、`/dev/video0` 需要替换为实际设备名。**请现在记录下来**:

| 功能 | 期望设备 | 实际设备 (请填写) |
|------|---------|------------------|
| STM32 串口 | `/dev/ttyACM0` | `____` |
| YDLIDAR | `/dev/ttyUSB0` | `____` |
| 摄像头 | `/dev/video0` | `____` |
| 麦克风 | `plughw:1,0` | `____` (从 `arecord -l` 获取) |

### 3.3 创建稳定设备名 (udev 规则)

```bash
# STM32 DAPLink (VID 0483 PID 5740) → /dev/stm32_arm
echo 'SUBSYSTEM=="tty", ATTRS{idVendor}=="0483", ATTRS{idProduct}=="5740", SYMLINK+="stm32_arm"' | sudo tee /etc/udev/rules.d/99-robot.rules

# YDLIDAR (VID 10c4 PID ea60) → /dev/ydlidar
echo 'SUBSYSTEM=="tty", ATTRS{idVendor}=="10c4", ATTRS{idProduct}=="ea60", SYMLINK+="ydlidar"' | sudo tee -a /etc/udev/rules.d/99-robot.rules

# 摄像头 (通用 USB 摄像头) — 记录设备路径供参考
echo 'SUBSYSTEM=="video4linux", ATTRS{idVendor}=="*", SYMLINK+="robot_cam"' | sudo tee -a /etc/udev/rules.d/99-robot.rules

sudo udevadm control --reload-rules && sudo udevadm trigger
# 重新插拔 USB 设备后验证:
ls -la /dev/stm32_arm /dev/ydlidar 2>/dev/null
```

> **如果 udev 规则不生效**: 直接用实际设备名 (如 `/dev/ttyACM0`)，不依赖 symlink。但每次重启设备名可能变化。

---

## 4. 编译 ROS2 工作空间 (10 分钟)

```bash
cd ~/RDK_X5/robot_ws

# 清理旧编译产物
rm -rf build/ install/ log/

# 编译 (先跳过 YDLIDAR，编译 Python 包)
colcon build --symlink-install --packages-skip ydlidar_ros2_driver 2>&1 | tee /tmp/build_$(date +%Y%m%d).log

# 如果 YDLIDAR SDK 编译成功，再编译 YDLIDAR 驱动:
# colcon build --symlink-install --packages-select ydlidar_ros2_driver

# 检查编译结果:
grep -c "error:" /tmp/build_*.log 2>/dev/null || echo "No errors found"
ls install/ && echo "Build OK"

# 加载环境:
echo "source ~/RDK_X5/robot_ws/install/setup.bash" >> ~/.bashrc
source ~/RDK_X5/robot_ws/install/setup.bash
```

**常见编译错误**:
| 错误 | 原因 | 解决 |
|------|------|------|
| `ModuleNotFoundError: No module named 'xxx'` | Python 依赖未安装 | `pip install xxx` |
| `ament_package not found` | ROS2 环境未 source | `source /opt/ros/humble/setup.bash` |
| `ydlidar_sdk not found` | YDLIDAR SDK 未编译 | `--packages-skip ydlidar_ros2_driver` |
| `cmake: command not found` | cmake 未安装 | `sudo apt install cmake` |

---

## 5. 配置文件调整 (10 分钟)

以下每个文件都需要根据实际设备名调整。**所有路径必须是绝对路径或 package:// 引用**。

### 5.1 设备路径批量替换

如果你的实际设备名与默认不同，执行以下替换:

```bash
cd ~/RDK_X5/robot_ws/src

# 替换 STM32 串口 (如果实际是 /dev/ttyACM1 或其他)
# find . \( -name "*.py" -o -name "*.yaml" -o -name "*.launch.py" \) -exec sed -i 's|/dev/ttyACM0|/dev/ttyACM0|g' {} \;

# 替换 YDLIDAR 端口
# find . \( -name "*.yaml" \) -exec sed -i 's|/dev/ttyUSB0|/dev/ttyUSB0|g' {} \;
```

### 5.2 YAML 配置确认清单

逐项检查以下文件，确保设备路径正确:

| 文件 | 检查项 | 默认值 | 如不同则修改 |
|------|--------|--------|-------------|
| `alloy_chassis_mecanum/config/nav2_params.yaml` | BT XML 路径 | `/opt/ros/humble/...` | 通常不需改 |
| `alloy_chassis_mecanum/config/ekf.yaml` | IMU/odom 话题名 | `/imu/data_raw`, `/odom` | 不需改 |
| `ydlidar_ros2_driver/params/X2.yaml` | `port` | `/dev/ttyUSB0` | 改为实际值 |
| `robot_brain/config/api_config.yaml` | `api_key` | `""` (env var) | `export VOLCENGINE_API_KEY=ark-...` |
| `robot_brain/config/api_config.yaml` | `serial.port` | `/dev/ttyACM0` | 改为实际值 |
| `elite_test/config/kinematics.yaml` | timeout | `0.05` | 不需改 |
| `elite_test/config/joint_limits.yaml` | 关节限位 | 各 ±1.57 | 不需改 |

### 5.3 环境变量

在 `~/.bashrc` 末尾添加:

```bash
# ROS2
source /opt/ros/humble/setup.bash
source ~/RDK_X5/robot_ws/install/setup.bash

# 火山引擎 API (豆包 Vision Pro)
export VOLCENGINE_API_KEY="your-api-key-here"   # ← 替换为你的 API Key
export VOLCENGINE_MODEL="doubao-1.5-vision-pro-32k"

# 显示 (无头模式适配)
export QT_QPA_PLATFORM=xcb
export DISPLAY=${DISPLAY:-:0}

# 摄像头设备 (如果 USB 摄像头不是 /dev/video0)
# export CAMERA_DEV=/dev/video2
```

然后: `source ~/.bashrc`

---

## 6. 预下载模型文件 (5 分钟)

这些文件在首次运行时会自动下载，但 RDK X5 网络可能不稳定。建议提前下载:

### 6.1 YOLOv8n 权重 (~6MB)

```bash
# 方法 1: ultralytics 自动下载 (需要网络)
python3 -c "from ultralytics import YOLO; YOLO('yolov8n.pt')" && echo "YOLO OK"

# 如果自动下载失败:
# wget https://github.com/ultralytics/assets/releases/download/v8.3.0/yolov8n.pt \
#   -O ~/.cache/ultralytics/yolov8n.pt
```

验证: `ls ~/RDK_X5/yolov8n.pt || ls ~/.cache/ultralytics/yolov8n.pt`

### 6.2 Whisper tiny 模型 (~80MB)

```bash
python3 -c "import whisper; whisper.load_model('tiny')" 2>/dev/null && echo "Whisper OK"
# 缓存位置: ~/.cache/whisper/tiny.pt
```

如果下载失败，语音功能会降级为键盘输入模式（不影响其他功能）。

---

## 7. 功能逐项验证 (30 分钟)

### 7.1 编译验证

```bash
cd ~/RDK_X5/robot_ws
source install/setup.bash

# 确认所有包被识别:
ros2 pkg list | grep -E "alloy_chassis|arm_ros2|elite_test|robot_brain|ydlidar"
# 期望输出:
#   alloy_chassis_mecanum
#   arm_ros2
#   elite_test
#   robot_brain
#   ydlidar_ros2_driver  (如果编译了)
```

### 7.2 串口通信验证 (STM32)

```bash
# 确认 STM32 在发送数据 (0xAA 帧头应频繁出现):
# (Ctrl+C 退出)
timeout 5 xxd /dev/ttyACM0 | head -20
# 期望: 看到大量 AA 01 ... 55 或 AA 02 ... 55 的十六进制输出
```

### 7.3 摄像头验证

```bash
# 拍照测试:
ffmpeg -i /dev/video0 -vframes 1 /tmp/test_cam.jpg -y 2>&1 | tail -5
# 如果有多个 /dev/video*，逐个测试:
for i in 0 1 2 3 4; do
  ffmpeg -i /dev/video$i -vframes 1 /tmp/test_cam_$i.jpg -y 2>/dev/null && \
    echo "/dev/video$i OK ($(identify /tmp/test_cam_$i.jpg 2>/dev/null | awk '{print $3}'))" || \
    echo "/dev/video$i FAIL"
done
```

### 7.4 基础 ROS2 节点测试

```bash
# 在不连接真实硬件的情况下，测试 Python 脚本能否启动:

# 测试机械臂 dry-run:
python3 ~/RDK_X5/robot_ws/src/robot_brain/scripts/grasper.py --home-only
# 期望: 打印 "[dry-run]" 和关节数值 (无串口时自动降级)

# 测试摄像头检测 (按 q 退出):
timeout 10 python3 ~/RDK_X5/robot_ws/src/robot_brain/scripts/detector.py --headless 2>&1 | head -20
# 期望: 打印 "Loading YOLOv8n..." 和检测日志

# 测试豆包 API 客户端 (如果有 API key):
python3 ~/RDK_X5/robot_ws/src/robot_brain/scripts/doubao_client.py "$VOLCENGINE_API_KEY"
# 期望: 打印 "Reply: ..." (豆包回复)
```

---

## 8. 启动全系统 (10 分钟)

### 8.1 Step 1: 启动导航全家桶

```bash
# 终端 1:
source ~/RDK_X5/robot_ws/install/setup.bash
ros2 launch alloy_chassis_mecanum robot_integrated.launch.py
```

**验证** (另开终端):
```bash
source ~/RDK_X5/robot_ws/install/setup.bash

# 1. 检查所有节点在运行:
ros2 node list | wc -l
# 期望: >= 15 个节点

# 2. 检查生命週期状态:
ros2 lifecycle get /controller_server 2>/dev/null | grep "state"
# 期望: active (如果不是，等 30 秒后重试)

# 3. 检查核心话题:
ros2 topic list | grep -E "odom|scan|cmd_vel|joint_states|imu"
# 期望输出:
#   /odom
#   /scan          (如果有激光)
#   /cmd_vel
#   /joint_states
#   /imu/data_raw  (如果有 IMU)
```

### 8.2 Step 2: 可选 — 启动机械臂桥接

```bash
# 终端 2:
source ~/RDK_X5/robot_ws/install/setup.bash
ros2 run elite_test arm_bridge.py
```

**验证**:
```bash
# 在 RViz 中拖动关节滑块 → 真实舵机应跟随运动
ros2 topic echo /joint_states --once
```

### 8.3 Step 3: 可选 — 启动 AI 控制

```bash
# 语音指挥官 (需要 API key 和麦克风):
ros2 launch robot_brain voice_commander.launch.py api_key:=$VOLCENGINE_API_KEY

# 果园巡检 (视觉分析 + 云端上传):
ros2 launch robot_brain orchard_patrol.launch.py api_key:=$VOLCENGINE_API_KEY

# 自动巡检抓取:
ros2 launch robot_brain integrated_patrol.launch.py grasp_enabled:=true
```

---

## 9. 常见问题排查

| 现象 | 可能原因 | 排查命令 |
|------|---------|---------|
| `/dev/ttyACM0` 不存在 | STM32 未上电 / USB 线松动 / DAPLink 固件问题 | `lsusb \| grep 0483` (应看到 STM32 DAPLink) |
| `/dev/ttyUSB0` 不存在 | 激光雷达未插 / 驱动未加载 | `lsusb \| grep 10c4` (应看到 YDLIDAR) |
| `/odom` 无数据 | STM32 固件未烧录 / 串口帧格式错误 | `xxd /dev/ttyACM0 \| head` (应看到 AA 01 ...) |
| `/scan` 无数据 | 激光雷达型号 / 波特率不匹配 | 检查 `params/X2.yaml` 中的 `port` 和 `baudrate` |
| YOLO 加载失败 | 权重文件不存在 | `ls ~/.cache/ultralytics/yolov8n.pt` |
| 豆包 API 返回 401 | API Key 错误或过期 | `echo $VOLCENGINE_API_KEY` |
| `cv2.imshow()` 无窗口 | 无头模式 / DISPLAY 未设 | `export DISPLAY=:0` 或用 `--headless` 参数 |
| 编译报 "ydlidar_sdk not found" | SDK 未编译或未安装 | 回到 Step 1.3 |
| odom 数据漂移 | 编码器噪声 | 死区参数已设置在 serial_odom_receiver.py (0.005) |
| lifecycle_manager 超时 | 节点启动慢 | 等 1 分钟自动重试，或 `ros2 lifecycle set /controller_server configure` 手动激活 |

---

## 10. 自动化启动脚本

创建 `~/start_robot.sh`:

```bash
#!/bin/bash
# RDK X5 机器人一键启动脚本

set -e
source /opt/ros/humble/setup.bash
source ~/RDK_X5/robot_ws/install/setup.bash

echo "=== RDK X5 Robot Startup ==="
echo "1. Checking devices..."

# 设备检查
[ -e /dev/ttyACM0 ] && echo "  STM32: OK" || echo "  STM32: MISSING!"
[ -e /dev/ttyUSB0 ] && echo "  LiDAR: OK" || echo "  LiDAR: MISSING"
[ -e /dev/video0 ] && echo "  Camera: OK" || echo "  Camera: MISSING"

echo "2. Launching integrated system..."
ros2 launch alloy_chassis_mecanum robot_integrated.launch.py

# 给系统 30 秒初始化
sleep 30

echo "3. Checking ROS2 nodes..."
ros2 node list | head -20
echo "=== Startup complete ==="
```

```bash
chmod +x ~/start_robot.sh
```

---

## 附录 A: 文件清理清单 (传输前删除)

```bash
# 在 dev 机器上，传输前删除这些 (含开发机路径/编译产物):
rm -rf robot_ws/build/ robot_ws/install/ robot_ws/log/
rm -rf ydlidar_ros2_ws/build/ ydlidar_ros2_ws/install/
rm -rf build/ install/ log/
find . -type d -name __pycache__ -exec rm -rf {} + 2>/dev/null
find . -name "*.pyc" -delete 2>/dev/null
```

## 附录 B: 关键路径速查

| 用途 | 路径 |
|------|------|
| ROS2 workspace | `~/RDK_X5/robot_ws/` |
| 启动脚本 | `~/RDK_X5/robot_ws/install/setup.bash` |
| YOLO 权重 | `~/.cache/ultralytics/yolov8n.pt` |
| Whisper 模型 | `~/.cache/whisper/tiny.pt` |
| 日志目录 | `~/RDK_X5/robot_ws/src/<pkg>/logs/` |
| API 配置 | `~/RDK_X5/robot_ws/src/robot_brain/config/api_config.yaml` |
| Nav2 配置 | `~/RDK_X5/robot_ws/src/alloy_chassis_mecanum/config/nav2_params.yaml` |
| EKF 配置 | `~/RDK_X5/robot_ws/src/alloy_chassis_mecanum/config/ekf.yaml` |
| 离线缓存 | `/tmp/orchard_cache/` (云端上传失败时) |

## 附录 C: 移植前已修复的 Bug

以下问题已在源码中修复，**不需要**再处理:

| 问题 | 修复内容 |
|------|---------|
| `robot_combined.urdf` 26处 `file:///home/cobrander/...` | 全部改为 `package://` |
| `serial_odom_receiver.py` 默认端口 `/dev/ttyUSB0` | 改为 `/dev/ttyACM0` |
| `voice_chat.py` 硬编码 API Key | 改为 `os.environ.get("VOLCENGINE_API_KEY")` |
| `api_config.yaml` 暴露真实 API Key | 改为占位符 |
| `yolo_stereo_detect.py` `sys.path` 开发机路径 hack | 删除 |
| `run_moveit_final.sh` x86_64 Qt 路径 | 改为 `dpkg --print-architecture` 自动检测 |
