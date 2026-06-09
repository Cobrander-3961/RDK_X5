#!/bin/bash
# 启动 moveit_setup_assistant 的最终解决方案脚本
# 解决 Wayland 显示协议问题和 X11 认证问题

# 设置 Qt 使用 X11 后端
export QT_QPA_PLATFORM=xcb

# 设置 GDK 使用 X11 后端
export GDK_BACKEND=x11

# 设置 SDL 使用 X11 后端
export SDL_VIDEODRIVER=x11

# 禁用 Wayland
export WAYLAND_DISPLAY=

# 启用 X11
export DISPLAY=${DISPLAY:-:0}

# 设置 Qt 插件路径
export QT_PLUGIN_PATH=/usr/lib/x86_64-linux-gnu/qt5/plugins:/usr/lib/qt5/plugins

# 检查 ROS 环境
if [ -z "$ROS_DISTRO" ]; then
    echo "ROS 环境未设置，正在加载..."
    source /opt/ros/humble/setup.bash
fi

# 检查 X11 认证文件
if [ ! -f ~/.Xauthority ]; then
    echo "警告: ~/.Xauthority 文件不存在"
    echo "尝试创建 X11 认证文件..."
    touch ~/.Xauthority
    xauth generate $DISPLAY . trusted
fi

# 启动 moveit_setup_assistant
echo "正在启动 moveit_setup_assistant..."
echo "显示服务器: $DISPLAY"
echo "Qt 平台: $QT_QPA_PLATFORM"

/opt/ros/humble/lib/moveit_setup_assistant/moveit_setup_assistant
