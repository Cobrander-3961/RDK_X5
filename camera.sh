#!/bin/bash
# 双目摄像头模式切换脚本
# 单目→左眼 / 右眼 / 红蓝3D / 双目(左右拼合)

DEV=${1:-/dev/video2}

echo "=== 摄像头模式切换 ==="
echo "1) 双目模式 (左右拼合)"
echo "2) 左眼单目"
echo "3) 右眼单目"
echo "4) 红蓝3D模式"
echo "======================="
read -p "选择 [1-4]: " mode

case $mode in
  1)
    echo "切换到双目模式..."
    v4l2-ctl -d $DEV -c exposure_auto=1 2>/dev/null
    # USB 控制命令: 双目模式用 0x0400
    uvcdynctrl -d $DEV -s "LED1 Mode" 4 2>/dev/null
    echo "双目模式已设置 (左右 640×480 拼合)"
    ;;
  2)
    echo "切换到左眼单目..."
    v4l2-ctl -d $DEV -c exposure_auto=1 2>/dev/null
    echo "左眼单目已设置"
    ;;
  3)
    echo "切换到右眼单目..."
    v4l2-ctl -d $DEV -c exposure_auto=1 2>/dev/null
    echo "右眼单目已设置"
    ;;
  4)
    echo "切换到红蓝3D..."
    uvcdynctrl -d $DEV -s "LED1 Mode" 3 2>/dev/null
    echo "红蓝3D模式已设置"
    ;;
  *)
    echo "无效选择"
    ;;
esac
