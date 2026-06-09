#!/bin/bash
# 启动 moveit_setup_assistant 的脚本
# 解决 Wayland 显示协议问题

# 设置 Qt 使用 X11 后端
export QT_QPA_PLATFORM=xcb

# 启动 moveit_setup_assistant
/opt/ros/humble/lib/moveit_setup_assistant/moveit_setup_assistant
