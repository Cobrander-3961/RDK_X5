#!/usr/bin/env python3
"""
机器人智能控制管线: 语音 → 视觉 → 豆包 Vision Pro 决策 → 语音反馈 → 执行动作。

流程:
  1. 等待语音唤醒/文本输入
  2. 拍摄摄像头图像
  3. 图像 + 指令 → 豆包 Vision Pro 决策
  4. 解析决策 → 执行机械臂/底盘动作
  5. TTS 语音反馈
"""
import rclpy, cv2, json, re, struct, serial, threading, time, os
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

# 追加脚本目录到 path 以导入 doubao_client
import sys
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from doubao_client import DoubaoClient


# ─── 机械臂串口控制器 ───
class ArmSerial:
    def __init__(self, port="/dev/ttyACM0", baudrate=115200):
        self.ser = None; self.lock = threading.Lock()
        try:
            self.ser = serial.Serial(port, baudrate, timeout=1)
            print(f"[Arm] {port} OK")
        except Exception as e:
            print(f"[Arm] {e}")

    def send_joints(self, angles):
        if not self.ser: return
        data = struct.pack("<ffffff", *[float(a) for a in angles])
        frame = bytearray([0xAA, 0x20, 24, 0]) + data + bytearray([0x55])
        with self.lock: self.ser.write(frame)
        print(f"[Arm] -> {[f'{a:.2f}' for a in angles]}")


# ─── 主管线节点 ───
class BrainPipeline(Node):
    def __init__(self):
        super().__init__("brain_pipeline")

        # 参数
        self.declare_parameter("api_key", "")
        self.declare_parameter("camera_id", 0)
        self.declare_parameter("serial_port", "/dev/ttyACM0")

        api_key = self.get_parameter("api_key").value
        if not api_key:
            self.get_logger().warn("No API key set! Use --ros-args -p api_key:=YOUR_KEY")

        # 豆包 Vision 客户端
        self.vision = DoubaoClient(api_key=api_key) if api_key else None

        # 摄像头
        self.bridge = CvBridge()
        self.cap = cv2.VideoCapture(self.get_parameter("camera_id").value)

        # 机械臂串口
        self.arm = ArmSerial(port=self.get_parameter("serial_port").value)

        # 预定义关节动作
        self.ACTIONS = {
            "home":   [ 0.0,  0.0,  0.0,  0.0,  0.0,  0.0],
            "grasp":  [ 0.3, -0.2, -0.6,  0.0,  0.0, -0.5],
            "place":  [-0.3, -0.2, -0.3,  0.0,  0.0,  0.0],
            "wave":   [ 0.5, -0.3,  0.0,  0.0,  0.0,  0.0],
            "left":   [-0.5,  0.0,  0.0,  0.0,  0.0,  0.0],
            "right":  [ 0.5,  0.0,  0.0,  0.0,  0.0,  0.0],
        }

        # 决策系统提示
        self.SYSTEM_PROMPT = """你是机器人控制专家。根据摄像头图像和用户指令，决定下一步动作。

返回 JSON 格式:
{
  "action": "home|grasp|place|wave|left|right|none",
  "speech": "简短的中文语音回复",
  "thinking": "你的推理过程"
}

可用动作: home(归位), grasp(抓取), place(放置), wave(挥手), left(左转), right(右转), none(不动作)
"""

        # 指令输入 (topic)
        self.cmd_sub = self.create_subscription(
            Image, "/camera/image_raw", self.image_cb, 10)
        self.latest_frame = None

        # 定时器: 等待指令
        self.get_logger().info("Brain Pipeline ready. Send text via /brain_command topic")

    def image_cb(self, msg):
        self.latest_frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")

    def capture(self):
        """拍摄并保存当前图像。"""
        if self.latest_frame is not None:
            path = "/tmp/robot_view.jpg"
            cv2.imwrite(path, self.latest_frame)
            return path
        ret, frame = self.cap.read()
        if ret:
            path = "/tmp/robot_view.jpg"
            cv2.imwrite(path, frame)
            return path
        return None

    def process_command(self, text: str):
        """处理自然语言指令。"""
        self.get_logger().info(f"Command: {text}")

        # 1. 拍照
        img_path = self.capture()
        if not img_path:
            self.get_logger().error("Camera failed")
            return

        # 2. 视觉决策
        if self.vision is None:
            self.get_logger().error("No API key")
            return

        self.get_logger().info("Calling Doubao Vision...")
        reply = self.vision.chat_with_image(
            f"用户指令: {text}\n请观察场景并决定动作。",
            img_path,
            system_prompt=self.SYSTEM_PROMPT
        )
        self.get_logger().info(f"Vision reply: {reply[:200]}")

        # 3. 解析 JSON
        try:
            json_match = re.search(r'\{[^{}]*\}', reply, re.DOTALL)
            if json_match:
                decision = json.loads(json_match.group())
            else:
                decision = {"action": "none", "speech": reply}
        except json.JSONDecodeError:
            decision = {"action": "none", "speech": reply}

        # 4. 执行动作
        action = decision.get("action", "none")
        if action in self.ACTIONS:
            self.get_logger().info(f"Executing: {action}")
            self.arm.send_joints(self.ACTIONS[action])
            time.sleep(1.5)

        # 5. 语音反馈 (打印)
        speech = decision.get("speech", "好的")
        self.get_logger().info(f"Speech: {speech}")

    def __del__(self):
        if hasattr(self, "cap") and self.cap:
            self.cap.release()


def main():
    rclpy.init()
    node = BrainPipeline()
    try:
        # 交互循环
        print("\n=== Robot Brain Ready ===")
        print("输入自然语言指令 (q 退出):")
        while rclpy.ok():
            # 非阻塞 spin
            rclpy.spin_once(node, timeout_sec=0.1)
            try:
                cmd = input("> ").strip()
                if cmd.lower() == "q":
                    break
                if cmd:
                    node.process_command(cmd)
            except (EOFError, KeyboardInterrupt):
                break
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
