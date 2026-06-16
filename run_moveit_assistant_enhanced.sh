#!/bin/bash
# 启动 moveit_setup_assistant 的增强脚本
# 解决 Wayland 显示协议问题和界面无响应问题

# 设置 Qt 使用 X11 后端
export QT_QPA_PLATFORM=xcb

# 设置 Qt 插件路径
export QT_PLUGIN_PATH=/usr/lib/$(dpkg --print-architecture 2>/dev/null || uname -m)-linux-gnu/qt5/plugins

# 禁用 Wayland
export GDK_BACKEND=x11

# 设置显示服务器
export DISPLAY=:0

# 启用调试输出
export ROS_LOG_LEVEL=debug

# 检查 ROS 环境
if [ -z "$ROS_DISTRO" ]; then
    echo "ROS 环境未设置，正在加载..."
    source /opt/ros/humble/setup.bash
fi

# 启动 moveit_setup_assistant
echo "正在启动 moveit_setup_assistant..."
/opt/ros/humble/lib/moveit_setup_assistant/moveit_setup_assistant
