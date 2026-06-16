# 语音→视觉→动作 全链路实现 — 2026-06-16

## 背景

项目有三个独立模块各做各的:
- `voice_chat.py` — 语音对话 (Whisper + Edge TTS + 豆包文本), 但对话结果不触发任何机器人动作
- `brain_pipeline.py` — 文本指令 + 豆包 Vision Pro 决策, 但动作用固定关节角度, 无导航/无 YOLO
- `integrated_patrol.py` — 导航 + YOLO + 抓取闭环, 但没有语音入口

项目简介声称 "支持语音指令理解与视觉场景分析，赋能机器人进行复杂的任务规划与动态决策"，但实际缺少一条完整链路。

## 目标

语音 → 豆包 Vision Pro 视觉决策 → 导航/检测/抓取 → TTS 反馈 全链路。

## 架构设计

```
🎤 后台线程              🧠 主线程 (spin_once + cv2)
──────────              ─────────────────────────────
listen() ──queue.Queue→ while rclpy.ok():
  arecord 4s              spin_once(10ms)
  Whisper tiny 转写        if voice_cmd:
                            ① 📸 拍照 /tmp/voice_view.jpg
                            ② 🤖 Doubao Vision Pro 决策
                            ③ 解析 JSON (嵌套对象)
                            ④ 分发动作:
                             ├─ navigate → Navigator.goto(x,y,yaw)
                             ├─ search   → Detector 循环 → Grasper 抓取
                             ├─ arm      → Grasper.send_joints()
                             ├─ home     → Grasper.home()
                             └─ none     → speak(回复)
                            ⑤ 🗣️ Edge TTS 语音反馈
                          🖼️ cv2.imshow + cv2.waitKey(1)
```

## 实现细节

### 1. 语音线程 + 队列桥接

```python
self._voice_queue = queue.Queue()
self._voice_thread = threading.Thread(target=self._voice_loop, daemon=True)

def _voice_loop(self):
    while rclpy.ok():
        text = listen()         # 阻塞 4s 录音 + Whisper 转写
        if text:
            self._voice_queue.put(text)
```

**为什么后台线程**: `listen()` 中 `subprocess.run(["arecord", "-d", "4"])` 阻塞 4 秒, 然后 `whisper.transcribe()` 再阻塞 2-3 秒。如果放主线程, ROS2 spin 和 OpenCV 显示会冻结 6-7 秒。

**为什么用 Queue 而不是直接回调**: Queue 是 Python 内置的线程安全队列, 不需要额外的锁。主线程 `get_nowait()` 非阻塞检查。

### 2. 嵌套 JSON 解析

`brain_pipeline.py` 的 JSON 解析使用 `r'\{[^{}]*\}'` 正则, 只匹配**平层** JSON。新系统提示词需要嵌套的 `target` 对象:

```json
{"action": "navigate", "target": {"distance_m": 1.5, "angle_rad": 0.2}}
```

**修复**: 用栈深度匹配最外层 `{...}`:

```python
def _parse_json(self, text):
    text = re.sub(r'```(?:json)?\s*', '', text)  # 去 markdown 围栏
    start = text.find('{')
    if start == -1: return None
    depth = 0
    for i in range(start, len(text)):
        if text[i] == '{': depth += 1
        elif text[i] == '}':
            depth -= 1
            if depth == 0:
                return json.loads(text[start:i+1])
    return None
```

### 3. 相对坐标 → map 坐标转换

VLM 只能看到摄像头第一人称视角, 输出的是相对机器人的距离和角度。需要转换为 map 坐标系下的绝对位置:

```python
def _resolve_nav_target(self, target):
    # 命名地点 → 直接查表
    name = target.get('name')
    if name in self.named_locations:
        return self.named_locations[name]

    # 相对坐标 → 根据当前里程计位姿转换
    distance_m = target.get('distance_m')
    if distance_m is not None:
        angle_rad = target.get('angle_rad', 0.0)
        target_yaw = self._odom_yaw + angle_rad       # 机器人朝向 + 偏移
        x = self._odom_x + distance_m * cos(target_yaw)
        y = self._odom_y + distance_m * sin(target_yaw)
        return (x, y, target_yaw)

    # 绝对坐标 → 直接使用
    return (target['x'], target['y'], target.get('yaw', 0))
```

### 4. 搜索模式 (search action)

VLM 决定 `search` 后, 进入**搜索模式**—在主循环中持续运行 YOLO 检测直到找到目标或超时:

```python
def _tick_search(self):
    result = self.detector.detect()
    if self.detector.is_auto_trigger(result) and result.class_name == self._search_target:
        j1 = clamp((cx - w/2) / w * 0.6, -1.2, 1.2)
        self.grasper.execute_grasp_sequence(j1)
        speak(f'已抓取{result.class_name}')
        self._search_mode = False
    elif timeout:
        speak(f'没找到{self._search_target}')
        self._search_mode = False
```

搜索期间, 语音指令处理被暂停 (队列消息被丢弃), 防止用户重复指令导致状态混乱。

### 5. 无 API Key 降级模式

```python
KEYWORD_ACTIONS = {
    '抓': 'search', '拿': 'search', '取': 'search', '捡': 'search',
    '去': 'navigate', '走': 'navigate', '到': 'navigate',
    '回': 'home', '归位': 'home', '收起': 'home',
    '挥手': 'wave', '你好': 'wave',
}
```

用户说"抓杯子" → 匹配 '抓' → `search` action + 尝试提取物体名 (杯子 → cup)。

### 6. 键盘备用输入

在 GUI 模式下按 `t` 键可以键盘输入指令, 用于:
- 开发调试 (不需要对着麦克风喊)
- Whisper 出问题时的手动 fallback
- 环境噪音大时的替代方案

## 遇到的问题

### 问题: 相机重复打开

**现象**: voice_commander 和 Detector 各自打开同一个 `/dev/video0`, 导致 "device busy" 错误。

**修复**: voice_commander 创建 `CameraWrapper` 后传给 Detector 共享:
```python
self.camera = CameraWrapper()                          # 主相机
self.detector = Detector(camera=self.camera, ...)      # 共享
```

Detector 构造函数支持 `camera` 外部注入 (之前模块化时已设计好)。

### 问题: 语音线程中访问 ROS2 API 不安全

**现象**: 不能从后台线程调用 `self.get_logger()`, `self.create_publisher()` 等 ROS2 API。

**修复**: 后台线程**只做 listen → queue.put()**, 所有 ROS2 操作在主线程的 `spin_once` 循环中完成。这是 ROS2 的线程模型约束。

### 问题: VLM 输出不可靠

**现象**: 豆包有时不返回 JSON, 有时 JSON 格式错误, 有时在 JSON 前后加解释文字。

**修复**: 
1. 去除 markdown 代码围栏 (` ```json ... ``` `)
2. 栈深度匹配 `{...}` (处理嵌套)
3. `json.loads` 失败时 → 自动 fallback 到关键词匹配
4. 关键词匹配也失败时 → 当作纯对话, `speak(reply)` 直接播放豆包原文

## 新增文件

- `robot_brain/scripts/voice_commander.py` (~410 行)
- `robot_brain/launch/voice_commander.launch.py` (~50 行)

## 修改文件

- `robot_brain/CMakeLists.txt` — 加 `voice_commander.py`

## 验证结果

- [x] `voice_commander.py` 语法检查通过
- [x] `voice_commander.launch.py` 语法检查通过
- [x] 模块导入链验证: `from voice_chat import listen, speak` (try/except fallback)
- [x] 无 API Key 模式下关键词匹配逻辑验证
- [ ] 实物验证: 需要在 RDK X5 上连接摄像头+麦克风+豆包 API key 进行端到端测试

## 启动方式

```bash
# 有 API Key — 豆包 Vision Pro 视觉决策
ros2 launch robot_brain voice_commander.launch.py api_key:=YOUR_KEY

# 无 API Key — 关键词 fallback 模式
ros2 launch robot_brain voice_commander.launch.py

# 键盘交互 (无头模式)
ros2 launch robot_brain voice_commander.launch.py headless:=true
```
