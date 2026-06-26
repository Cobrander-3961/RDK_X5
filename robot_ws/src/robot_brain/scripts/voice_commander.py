#!/usr/bin/env python3
"""
语音指令 → 视觉决策 → 导航/抓取 全链路节点。

组合: Navigator + Grasper + Detector + DoubaoClient + 语音 I/O

架构:
  后台线程: listen() → queue.Queue
  主线程:   spin_once(10ms) → 处理语音 → 拍照 → Doubao Vision → 动作分发
            cv2.imshow → TTS 反馈

启动:
  ros2 run robot_brain voice_commander.py --ros-args -p api_key:=YOUR_KEY
  ros2 launch robot_brain voice_commander.launch.py api_key:=YOUR_KEY

依赖: 需先启动 robot_integrated.launch.py (Nav2 + SLAM + 机械臂)
"""
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
import cv2
import json
import re
import sys
import os
import time
import math
import queue
import threading

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from navigator import Navigator
from grasper import Grasper
from detector import Detector
from doubao_client import DoubaoClient
from camera_utils import CameraWrapper

# 语音模块 (可选, 无 Whisper 时降级为键盘输入)
try:
    from voice_chat import listen, speak
    _has_voice = True
except ImportError:
    _has_voice = False
    def listen():
        return None
    def speak(text):
        print(f'[TTS] {text}')


# ─── 系统提示词 ────────────────────────────────────────

SYSTEM_PROMPT = """你是机器人控制专家。根据摄像头拍摄的第一人称图像，结合用户语音指令，决定下一步动作。

返回 JSON (只返回 JSON, 不要其他文字):

{
  "action": "<动作名>",
  "speech": "简短中文语音回复 (1-2句话)",
  "thinking": "你的推理过程"
}

可用动作:

1. navigate — 导航到地点
   如果用户说"去桌子/门口/厨房/那边":
   {"action": "navigate", "target": {"name": "table"}, "speech": "正在前往桌子"}
   可选 name: table(桌子), door(门口), center(中心), home(起点)

   如果用户说"向前走N米"或在画面中看到目标区域:
   {"action": "navigate", "target": {"distance_m": 1.5, "angle_rad": 0.2}, "speech": "向前移动"}

2. search — 搜索并抓取目标物体
   如果用户说"抓/拿/取 XX"且XX在画面中可见:
   {"action": "search", "target_class": "cup", "speech": "正在寻找杯子"}
   target_class 可选: mouse, bottle, cup, cell phone, book, banana, apple

3. arm — 直接控制机械臂
   {"action": "arm", "joints": [0.0, -0.3, -0.6, 0.0, 0.0, -0.5], "speech": "正在抓取"}

4. home — 机械臂归位
   {"action": "home", "speech": "回到初始位置"}

5. wave — 挥手打招呼
   {"action": "wave", "speech": "你好!"}

6. none — 不动作, 纯对话回复
   {"action": "none", "speech": "好的, 有什么可以帮你的?"}

重要规则:
- 如果用户在闲聊/问询, 使用 none
- 如果用户要求抓取画面中可见的物体, 使用 search
- 如果用户要求去某个地方, 使用 navigate, 优先用 name
- 如果用户要求操作机械臂, 使用 arm/home/wave
- 尽可能给出简洁的中文语音回复
"""

# 命名地点 → map 坐标映射 (根据实际环境调整)
NAMED_LOCATIONS = {
    'home':   (0.0, 0.0, 0.0),
    'table':  (0.5, 0.5, 0.0),
    'center': (0.0, 0.0, 0.0),
    'door':   (1.0, 0.0, 0.0),
}

# 无 API key 时的关键词 fallback 映射
KEYWORD_ACTIONS = {
    '抓': 'search', '拿': 'search', '取': 'search', '捡': 'search',
    '去': 'navigate', '走': 'navigate', '到': 'navigate',
    '回': 'home', '归位': 'home', '收起': 'home',
    '挥手': 'wave', '你好': 'wave',
}

# 关节预设 (与 Grasper 一致)
WAVE_JOINTS = [0.5, -0.3, 0.0, 0.0, 0.0, 0.0]


class VoiceCommanderNode(Node):
    """语音→视觉→动作 全链路节点。"""

    def __init__(self):
        super().__init__('voice_commander')

        # ── ROS2 参数 ──
        self.declare_parameter('api_key', '')
        self.declare_parameter('serial_port', '/dev/ttyACM0')
        self.declare_parameter('model_path', '/home/sunrise/RDK_X5/yolov8n.pt')
        self.declare_parameter('target_classes', [
            'mouse', 'bottle', 'cup', 'cell phone', 'book', 'banana', 'apple',
        ])
        self.declare_parameter('headless', False)
        self.declare_parameter('named_locations', [
            ['home', 0.0, 0.0, 0.0],
            ['table', 0.5, 0.5, 0.0],
            ['center', 0.0, 0.0, 0.0],
            ['door', 1.0, 0.0, 0.0],
        ])

        api_key = str(self.get_parameter('api_key').value)
        serial_port = str(self.get_parameter('serial_port').value)
        model_path = str(self.get_parameter('model_path').value)
        target_classes = set(str(c) for c in self.get_parameter('target_classes').value)
        self.headless = bool(self.get_parameter('headless').value)

        # 加载命名地点
        locs_raw = self.get_parameter('named_locations').value
        self.named_locations = {}
        for loc in locs_raw:
            self.named_locations[str(loc[0])] = (float(loc[1]), float(loc[2]), float(loc[3]))
        if not self.named_locations:
            self.named_locations = NAMED_LOCATIONS

        # ── 模块初始化 ──
        self.get_logger().info('Initializing modules...')

        # 豆包 Vision
        self.vision = DoubaoClient(api_key=api_key) if api_key else None
        if self.vision:
            self.get_logger().info('Doubao Vision Pro ready')
        else:
            self.get_logger().warn('No API key — using keyword fallback mode')

        # 导航
        self.nav = Navigator(self)

        # 机械臂
        self.grasper = Grasper(port=serial_port)
        self.get_logger().info(f'Arm: {"connected" if self.grasper.connected else "DRY-RUN"}')

        # 相机 (共享给 Detector)
        self.camera = CameraWrapper()
        self.get_logger().info(f'Camera: {self.camera.mode}')

        # 检测器
        self.detector = Detector(
            model_path=model_path,
            target_classes=target_classes,
            camera=self.camera,
            headless=True,  # 检测器不弹出独立窗口
        )

        # 里程计 (用于相对坐标→map 坐标转换)
        self._odom_x = 0.0
        self._odom_y = 0.0
        self._odom_yaw = 0.0
        self._odom_sub = self.create_subscription(
            Odometry, '/odom', self._odom_cb, 10)

        # ── 语音线程 ──
        self._voice_queue = queue.Queue()
        self._voice_thread = threading.Thread(target=self._voice_loop, daemon=True)
        self._voice_thread.start()

        # ── 状态 ──
        self._search_mode = False
        self._search_target = None
        self._search_start_time = 0.0
        self._search_timeout = 30.0
        self._last_speech = ''

        self.get_logger().info('Voice Commander ready!')
        self.get_logger().info(f'  Named locations: {list(self.named_locations.keys())}')
        self.get_logger().info(f'  Voice: {"enabled" if _has_voice else "disabled"}')

    # ── 里程计回调 ──────────────────────────────────────

    def _odom_cb(self, msg: Odometry):
        self._odom_x = msg.pose.pose.position.x
        self._odom_y = msg.pose.pose.position.y
        qz = msg.pose.pose.orientation.z
        qw = msg.pose.pose.orientation.w
        self._odom_yaw = 2.0 * math.atan2(qz, qw)

    # ── 语音后台线程 ────────────────────────────────────

    def _voice_loop(self):
        if not _has_voice:
            return
        while rclpy.ok():
            try:
                text = listen()
                if text:
                    self._voice_queue.put(text)
            except Exception:
                time.sleep(1.0)

    # ── 视觉决策 ────────────────────────────────────────

    def _vision_decide(self, user_text: str) -> dict:
        """拍照 → Doubao Vision → 解析 JSON 决策。"""
        # 1. 拍照
        img, _ = self.camera.read()
        if img is None:
            self.get_logger().error('Camera failed')
            return {'action': 'none', 'speech': '摄像头故障'}

        img_path = '/tmp/voice_view.jpg'
        cv2.imwrite(img_path, img)

        # 2. Doubao Vision
        if self.vision is None:
            return self._keyword_fallback(user_text)

        self.get_logger().info(f'Calling Doubao Vision: "{user_text[:50]}..."')
        try:
            reply = self.vision.chat_with_image(
                f"用户指令: {user_text}\n请观察场景并决定动作。",
                img_path,
                system_prompt=SYSTEM_PROMPT,
            )
            self.get_logger().info(f'Vision reply: {reply[:200]}')
        except Exception as e:
            self.get_logger().error(f'Vision API error: {e}')
            return {'action': 'none', 'speech': f'大脑离线: {e}'}

        # 3. 解析 JSON (支持嵌套)
        decision = self._parse_json(reply)
        if decision is None:
            self.get_logger().warn('JSON parse failed, using fallback')
            return self._keyword_fallback(user_text)

        return decision

    def _parse_json(self, text: str) -> dict | None:
        """从文本中提取 JSON (支持嵌套对象)。"""
        # 去掉可能的 markdown 代码块
        text = re.sub(r'```(?:json)?\s*', '', text)
        text = re.sub(r'```\s*', '', text)

        # 尝试匹配最外层 { ... }
        # 使用栈来匹配嵌套的 {}
        start = text.find('{')
        if start == -1:
            return None

        depth = 0
        for i in range(start, len(text)):
            if text[i] == '{':
                depth += 1
            elif text[i] == '}':
                depth -= 1
                if depth == 0:
                    try:
                        return json.loads(text[start:i + 1])
                    except json.JSONDecodeError:
                        return None
        return None

    def _keyword_fallback(self, user_text: str) -> dict:
        """无 API 时的关键词匹配 fallback。"""
        for keyword, action in KEYWORD_ACTIONS.items():
            if keyword in user_text:
                if action == 'search':
                    # 尝试提取目标物体名
                    for cls in self.detector.target_classes:
                        if cls in user_text or (cls == 'cell phone' and '手机' in user_text):
                            return {'action': 'search', 'target_class': cls,
                                    'speech': f'正在寻找{cls}'}
                    return {'action': 'search', 'target_class': 'cup',
                            'speech': '正在寻找目标'}
                elif action == 'navigate':
                    for name in self.named_locations:
                        if name in user_text:
                            return {'action': 'navigate', 'target': {'name': name},
                                    'speech': f'前往{name}'}
                    return {'action': 'navigate', 'target': {'distance_m': 1.0, 'angle_rad': 0.0},
                            'speech': '向前移动'}
                elif action == 'home':
                    return {'action': 'home', 'speech': '归位'}
                elif action == 'wave':
                    return {'action': 'wave', 'speech': '你好呀'}
        return {'action': 'none', 'speech': f'收到: {user_text}'}

    # ── 动作分发 ────────────────────────────────────────

    def _dispatch(self, decision: dict):
        """根据决策 JSON 执行动作。"""
        action = decision.get('action', 'none')
        speech = decision.get('speech', '好的')

        self.get_logger().info(f'Action: {action} | Speech: {speech}')

        if action == 'navigate':
            self._handle_navigate(decision.get('target', {}))
        elif action == 'search':
            self._handle_search(decision.get('target_class', 'cup'))
        elif action == 'arm':
            joints = decision.get('joints', [0] * 6)
            self.get_logger().info(f'Arm joints: {joints}')
            self.grasper.send_joints(joints)
        elif action == 'home':
            self.grasper.home()
        elif action == 'wave':
            self.grasper.send_joints(WAVE_JOINTS)
            time.sleep(1.5)
            self.grasper.home()
        elif action == 'none':
            pass  # 纯对话, 不做动作

        # TTS 反馈
        if speech:
            self._last_speech = speech
            speak(speech)

    def _resolve_nav_target(self, target: dict) -> tuple[float, float, float] | None:
        """解析导航目标 → (x, y, yaw)。"""
        # 命名地点
        name = target.get('name')
        if name and name in self.named_locations:
            return self.named_locations[name]

        # 相对坐标 (距离 + 角度)
        distance_m = target.get('distance_m')
        angle_rad = target.get('angle_rad', 0.0)
        if distance_m is not None:
            target_yaw = self._odom_yaw + float(angle_rad)
            x = self._odom_x + float(distance_m) * math.cos(target_yaw)
            y = self._odom_y + float(distance_m) * math.sin(target_yaw)
            return (x, y, target_yaw)

        # 绝对坐标
        x = target.get('x')
        y = target.get('y')
        if x is not None and y is not None:
            yaw = float(target.get('yaw', 0.0))
            return (float(x), float(y), yaw)

        return None

    def _handle_navigate(self, target: dict):
        """处理导航动作。"""
        nav_target = self._resolve_nav_target(target)
        if nav_target is None:
            self.get_logger().warn(f'Cannot resolve nav target: {target}')
            speak('我不知道该去哪里')
            return

        x, y, yaw = nav_target
        self.get_logger().info(f'Navigating to ({x:.2f}, {y:.2f}, yaw={yaw:.2f})')
        speak(f'正在前往 ({x:.1f}, {y:.1f})')

        ok = self.nav.goto(x, y, yaw, timeout_sec=60.0)
        if ok:
            speak('已到达')
        else:
            speak('导航失败, 无法到达目标')

    def _handle_search(self, target_class: str):
        """搜索→检测→抓取。使用检测循环找目标。"""
        self.get_logger().info(f'Searching for: {target_class}')
        speak(f'正在寻找{target_class}')

        self._search_mode = True
        self._search_target = target_class
        self._search_start_time = time.time()
        # 实际检测在 run() 主循环中处理

    # ── 搜索循环 (在主循环 tick 中调用) ──────────────────

    def _tick_search(self):
        """检测循环的一帧。返回 True 表示搜索仍在进行。"""
        if not self._search_mode:
            return False

        elapsed = time.time() - self._search_start_time

        result = self.detector.detect()

        # 自动触发条件: 稳定检测 + 目标匹配 + 距离有效
        if (self.detector.is_auto_trigger(result)
                and result.class_name == self._search_target):
            self.get_logger().info(
                f'Found {result.class_name} @ {result.distance_m*100:.0f}cm — grasping!')
            j1 = max(-1.2, min(1.2,
                     (result.cx - result.image_width / 2) / result.image_width * 0.6))
            self.grasper.execute_grasp_sequence(j1)
            self.detector.reset_cooldown()
            speak(f'已抓取{result.class_name}')
            self._search_mode = False
            self._search_target = None
            return False

        # 超时
        if elapsed > self._search_timeout:
            self.get_logger().warn(f'Search timeout for {self._search_target}')
            speak(f'没找到{self._search_target}')
            self._search_mode = False
            self._search_target = None
            return False

        return True

    # ── 主循环 ──────────────────────────────────────────

    def run(self):
        """主事件循环: spin_once + voice dispatch + search tick + display。"""
        self.get_logger().info('Starting main loop...')
        print('\n' + '=' * 60)
        print('  Voice Commander — 语音 → 视觉 → 动作 全链路')
        print('=' * 60)
        print(f'  API: {"Doubao Vision Pro" if self.vision else "Keyword fallback"}')
        print(f'  Arm: {"connected" if self.grasper.connected else "DRY-RUN"}')
        print(f'  Voice: {"enabled" if _has_voice else "text only"}')
        print(f'  Locations: {list(self.named_locations.keys())}')
        print('=' * 60)
        print('  Say/type: "抓杯子" "去桌子" "归位" "你好" ...')
        print('  Keys: q=quit  h=home  t=type command')
        print()

        win_name = 'Voice Commander'
        if not self.headless:
            cv2.namedWindow(win_name, cv2.WINDOW_NORMAL)
            cv2.resizeWindow(win_name, 640, 480)

        try:
            while rclpy.ok():
                # 1. ROS2 回调
                rclpy.spin_once(self, timeout_sec=0.01)

                # 2. 搜索模式 (独立于语音指令)
                if self._search_mode:
                    self._tick_search()

                # 3. 处理语音指令
                try:
                    text = self._voice_queue.get_nowait()
                    if text:
                        self.get_logger().info(f'Voice: {text}')
                        decision = self._vision_decide(text)
                        self._dispatch(decision)
                except queue.Empty:
                    pass

                # 4. 显示
                if not self.headless:
                    img, _ = self.camera.read()
                    if img is not None:
                        if self._search_mode:
                            result = self.detector.detect()
                            display = self.detector.annotate_frame(result)
                            status = f'SEARCH: {self._search_target}'
                            self.detector.draw_status_overlay(display, status)
                        else:
                            display = img.copy()
                            cv2.putText(display, self._last_speech[:60],
                                        (5, display.shape[0] - 10),
                                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

                        cv2.putText(display, "Say command | q=quit t=type",
                                    (5, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (200, 200, 200), 1)
                        cv2.imshow(win_name, display)

                    key = cv2.waitKey(1) & 0xFF
                    if key == ord('q'):
                        self.get_logger().info('User quit')
                        break
                    elif key == ord('h'):
                        self.grasper.home()
                        speak('归位')
                    elif key == ord('t'):
                        # 键盘输入模式
                        print('\r' + ' ' * 40 + '\r', end='')
                        try:
                            cmd = input('> ').strip()
                            if cmd:
                                if cmd.lower() == 'q':
                                    break
                                decision = self._vision_decide(cmd)
                                self._dispatch(decision)
                        except (EOFError, KeyboardInterrupt):
                            break

        except KeyboardInterrupt:
            self.get_logger().info('Interrupted')

        finally:
            self._shutdown()

    def _shutdown(self):
        """优雅关闭。"""
        self.get_logger().info('Shutting down...')
        if self._search_mode:
            self._search_mode = False
        if self.grasper.connected:
            self.grasper.home()
        self.detector.release()
        cv2.destroyAllWindows()
        self.get_logger().info('Voice Commander done.')


# ─── main ─────────────────────────────────────────────────
def main():
    rclpy.init(args=sys.argv)
    node = VoiceCommanderNode()
    node.run()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
