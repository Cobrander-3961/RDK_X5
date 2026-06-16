# robot_brain 模块化架构重构 — 2026-06-16

## 背景

项目初始状态有 3 个独立脚本各做各的:
- `patrol.py` — 只导航到航点, sleep 5s 空等, 不抓取
- `auto_grasp_full.py` — 只 YOLO 检测+抓取, 不导航
- `brain_pipeline.py` — 语音+豆包 Vision Pro, 但用固定关节角度, 没有 YOLO 反馈

代码重复严重:
- `ArmSerial` 类在 3 个文件中各写了一遍
- 抓取序列 (`HOME→PRE→GRASP→LIFT→PLACE→RELEASE→HOME`) 在 3 个文件中各写了一遍
- `patrol.py` 的 `goto()` 逻辑只在自己文件里用, 其他脚本无法复用

## 目标

1. 提取可复用模块, 每个模块独立可测
2. 一体化编排器串联成 `导航 → 检测 → 抓取 → 下一站` 闭环
3. 所有旧脚本保持原样不动

## 新增文件

### 1. grasper.py (~130 行)

统一 `auto_grasp_full.py` / `brain_pipeline.py` / `arm_fixed_grasp_test.py` 中重复的 ArmSerial + 抓取序列。

```python
class ArmSerial:
    """无串口时优雅降级: send() → no-op, connected=False"""
    def __init__(self, port=None)  # 自动检测 /dev/ttyACM*

class Grasper:
    HOME = [0,0,0,0,0,0]
    PRE  = [0, -0.3, -0.6, 0, 0, 0]   # j1 动态替换
    # ...
    def execute_grasp_sequence(self, j1)  # 7步标准序列
```

**独立测试**: `python3 grasper.py --home-only` — dry-run 验证通过

### 2. navigator.py (~140 行)

从 `patrol.py` 泛化为可复用类。**不继承 Node** — 通过组合模式复用已有节点。

```python
class Navigator:
    def __init__(self, node: Node)        # 复用外部 Node
    def goto(self, x, y, yaw=0.0) -> bool # 阻塞直到到达/超时
```

**设计决策**: 不是 Node 子类。`integrated_patrol.py` 和 `voice_commander.py` 共享同一个 Node 实例的 spin 循环, 避免多 Node 单进程的 spin 冲突。

**独立测试**: `python3 navigator.py --waypoints "0.5,0.5,0;1,0,1.57"` (需 Nav2 运行)

### 3. detector.py (~250 行)

从 `auto_grasp_full.py` 主循环提取。可配置、可导入、支持无头模式。

```python
@dataclass
class DetectionResult:
    class_name, cx, cy, distance_m, confidence, is_stable, image_width

class Detector:
    def __init__(self, model_path, target_classes, camera=None, headless=False)
    def detect(self) -> DetectionResult     # 一帧: 拍照→YOLO→深度→稳定性
    def is_auto_trigger(self, result) -> bool
    def annotate_frame(self, result) -> np.ndarray
```

**稳定性过滤**: `deque(maxlen=5)` 滑动窗口, 连续 4 帧同一类别才判定稳定。防误触发。

**独立测试**: `python3 detector.py` (GUI) / `python3 detector.py --headless`

### 4. integrated_patrol.py (~260 行)

状态机编排器: `NAVIGATING → DETECTING → GRASPING → COOLDOWN → ... → COMPLETED`

```python
class IntegratedPatrolNode(Node):
    def __init__(self):
        self.nav = Navigator(self)               # 共享 node
        self.grasper = Grasper(port=serial_port)
        self.detector = Detector(...)

    def run(self):
        while rclpy.ok() and state != COMPLETED:
            rclpy.spin_once(self, timeout_sec=0.01)   # ROS2 回调
            # 状态机推进 + cv2.imshow + cv2.waitKey(1)
```

**ROS2 + OpenCV 共存**: `spin_once(10ms)` + `cv2.waitKey(1)` 交替, 导航时 `Navigator.goto()` 内部 `spin_until_future_complete` 临时接管 spin。

## 修改文件

- `robot_brain/CMakeLists.txt` — 补全所有脚本到 install(PROGRAMS)
- `robot_brain/package.xml` — 补加 `nav2_msgs` 依赖 (patrol.py 用了但没声明)
- `robot_brain/launch/integrated_patrol.launch.py` — 新建

## 遇到的问题

### 问题: Navigator 和 IntegratedPatrolNode 都是 Node 子类

**现象**: 最初 Navigator 继承 `Node`, 和 `IntegratedPatrolNode` 是两个独立 Node。同一进程 spin 两个 Node 需要 MultiThreadedExecutor。

**修复**: Navigator 改为纯组合模式 — 接收一个 `Node` 引用, 不继承 Node。ActionClient 和 spin 操作都通过这个引用。

**影响**: 这个改动也影响了独立测试, 需要创建临时 `_TestNavigatorNode` 包装类。但好处是 `voice_commander.py` 可以直接复用同一个模式。

## 验证结果

- [x] 4 个新文件语法检查全部通过
- [x] `grasper.py` dry-run 模式正常
- [x] 模块导入链验证: `from grasper import ArmSerial, Grasper` OK
- [x] 旧文件 `auto_grasp_full.py`, `patrol.py`, `brain_pipeline.py` 未改动
- [x] launch 文件语法通过

## 模块依赖图 (零循环)

```
integrated_patrol.py ──┬── navigator.py ────── nav2_msgs (ROS2 Action)
                       ├── detector.py ─────── camera_utils.py + ultralytics
                       └── grasper.py ──────── pyserial (无 ROS2)
```

三个叶子模块之间无任何 import 关系, 各自 `python3 xxx.py` 可直接独立测试。
