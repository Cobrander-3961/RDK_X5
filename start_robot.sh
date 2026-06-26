#!/bin/bash
# RDK X5 机器人一键启动脚本
# 用法: ~/start_robot.sh [full|nav|arm|ai]

set -e

RED="\033[1;31m"; GREEN="\033[1;32m"; YELLOW="\033[1;33m"; NC="\033[0m"
info()  { echo -e "${GREEN}[INFO]${NC} $*"; }
warn()  { echo -e "${YELLOW}[WARN]${NC} $*"; }
error() { echo -e "${RED}[ERROR]${NC} $*"; }

MODE="${1:-full}"

# ── 环境加载 ─────────────────────────────────────────
source /opt/ros/humble/setup.bash
source ~/RDK_X5/robot_ws/install/setup.bash
export QT_QPA_PLATFORM=xcb
export DISPLAY=${DISPLAY:-:0}

echo "=========================================="
echo "   RDK X5 Robot Startup"
echo "   Mode: ${MODE}"
echo "   Time: $(date)"
echo "=========================================="

# ── 设备检查 ─────────────────────────────────────────
echo ""
info "1. Checking devices..."

check_dev() {
    local path="$1" name="$2"
    if [ -e "$path" ]; then
        info "  ${name}: OK (${path})"
        return 0
    else
        warn "  ${name}: MISSING (${path})"
        return 1
    fi
}

STM32_OK=false; LIDAR_OK=false; CAM_OK=false
check_dev "/dev/ttyACM0" "STM32" && STM32_OK=true || true
check_dev "/dev/ttyUSB0" "YDLIDAR" && LIDAR_OK=true || true
check_dev "/dev/video2"  "Camera" && CAM_OK=true || true

# ── 启动 ─────────────────────────────────────────────
echo ""
info "2. Launching (mode=${MODE})..."

case "$MODE" in
    nav)
        info "Starting Navigation2 stack..."
        ros2 launch alloy_chassis_mecanum robot_integrated.launch.py
        ;;
    arm)
        info "Starting MoveIt2 arm control..."
        ros2 launch elite_test arm_full.launch.py
        ;;
    ai)
        info "Starting AI voice commander..."
        ros2 launch robot_brain voice_commander.launch.py api_key:=${VOLCENGINE_API_KEY:-}
        ;;
    full)
        info "Starting full system..."

        # 导航全家桶 (底盘 + 串口 + 雷达 + Nav2)
        gnome-terminal -- bash -c "
            source /opt/ros/humble/setup.bash
            source ~/RDK_X5/robot_ws/install/setup.bash
            echo [NAV] Starting...
            ros2 launch alloy_chassis_mecanum robot_integrated.launch.py
            exec bash
        " 2>/dev/null || \
        xterm -e bash -c "
            source /opt/ros/humble/setup.bash
            source ~/RDK_X5/robot_ws/install/setup.bash
            echo [NAV] Starting...
            ros2 launch alloy_chassis_mecanum robot_integrated.launch.py
            exec bash
        " 2>/dev/null || \
        warn "No terminal emulator, starting nav in background..."
        ros2 launch alloy_chassis_mecanum robot_integrated.launch.py &
        NAV_PID=$!
        sleep 30

        # 系统就绪检查
        echo ""
        info "3. System ready check..."
        sleep 5
        NODE_COUNT=$(ros2 node list 2>/dev/null | wc -l)
        info "  Active nodes: ${NODE_COUNT}"
        ros2 node list 2>/dev/null | head -20

        echo ""
        info "=========================================="
        info "  System started! Use:"
        info "  ros2 launch elite_test arm_full.launch.py  # arm only"
        info "  ros2 launch robot_brain voice_commander.launch.py  # AI"
        info "=========================================="
        ;;
    *)
        error "Unknown mode: ${MODE}"
        echo "Usage: $0 [full|nav|arm|ai]"
        exit 1
        ;;
esac
