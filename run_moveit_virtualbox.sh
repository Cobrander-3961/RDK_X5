#!/bin/bash
# 启动 moveit_setup_assistant 的 VirtualBox 优化脚本
# 解决 VirtualBox + Wayland 环境下的显示问题

# 禁用所有 Wayland 相关设置
unset WAYLAND_DISPLAY
export WAYLAND_DISPLAY=

# 强制使用 X11
export GDK_BACKEND=x11
export QT_QPA_PLATFORM=xcb
export SDL_VIDEODRIVER=x11

# VirtualBox 特定设置
export LIBGL_ALWAYS_SOFTWARE=1
export MESA_GL_VERSION_OVERRIDE=3.3

# Qt 特定设置
export QT_AUTO_SCREEN_SCALE_FACTOR=0
export QT_SCALE_FACTOR=1

# 显示设置
export DISPLAY=${DISPLAY:-:0}

# ROS 环境设置
if [ -z "$ROS_DISTRO" ]; then
    source /opt/ros/humble/setup.bash
fi

# 跳过 X11 认证检查，直接启动
echo "正在启动 moveit_setup_assistant (VirtualBox 模式)..."
echo "显示服务器: $DISPLAY"
echo "Qt 平台: $QT_QPA_PLATFORM"

# 直接启动，不进行 X11 认证
/opt/ros/humble/lib/moveit_setup_assistant/moveit_setup_assistant
