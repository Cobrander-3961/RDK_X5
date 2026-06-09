# MoveIt Setup Assistant 启动脚本

## 问题描述
moveit_setup_assistant 在 Wayland 显示环境下可能会出现界面无响应的问题。

## 解决方案
提供了三个启动脚本，按推荐顺序使用：

### 1. run_moveit_final.sh (推荐)
最完整的解决方案，包含：
- 强制使用 X11 后端
- 禁用 Wayland
- 自动创建 X11 认证文件
- 完整的环境变量设置

使用方法：
```bash
./run_moveit_final.sh
```

### 2. run_moveit_assistant_enhanced.sh
增强版启动脚本，包含：
- Qt 和 GDK 后端设置
- ROS 环境检查
- 调试输出

使用方法：
```bash
./run_moveit_assistant_enhanced.sh
```

### 3. run_moveit_assistant.sh
基础版启动脚本，只包含最基本的设置。

使用方法：
```bash
./run_moveit_assistant.sh
```

## 如果仍然有问题

如果上述脚本都无法解决问题，请尝试以下步骤：

1. 重新安装 MoveIt2：
```bash
sudo apt update
sudo apt install --reinstall ros-humble-moveit-setup-assistant
```

2. 检查 ROS 环境：
```bash
source /opt/ros/humble/setup.bash
ros2 --version
```

3. 手动设置环境变量：
```bash
export QT_QPA_PLATFORM=xcb
export GDK_BACKEND=x11
/opt/ros/humble/lib/moveit_setup_assistant/moveit_setup_assistant
```

## 永久解决方案

如果希望永久解决此问题，可以在 ~/.bashrc 文件末尾添加：
```bash
export QT_QPA_PLATFORM=xcb
export GDK_BACKEND=x11
```

然后执行：
```bash
source ~/.bashrc
```
