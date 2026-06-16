# 果园苹果质量分析 + 云端上传 — 2026-06-16

## 背景

项目定位为智慧农业场景，需要机器人具备:
1. 果园巡检 — 按行导航，扫描果树
2. 苹果质量分析 — 区分好果/坏果/未熟果，统计比例
3. 云端上传 — 统计报告 + 缩略图 → 火山引擎

## 目标

- 零额外模型训练 (只用现有的 YOLOv8n COCO 权重 + HSV 颜色分析)
- 模块独立可测 (延续项目模块化架构)
- 离线容错 (上传失败时本地缓存)

---

## 新增文件

### 1. fruit_analyzer.py (~280 行) — 苹果质量分析器

**检测原理**:

```
YOLO 检测 "apple" → 裁剪 ROI → HSV 像素统计

红色 (H:0-10/160-180, S>100)    → good_ratio  (好果特征)
暗斑 (S<30 灰褐 或 V<60 暗色)     → bad_ratio   (腐烂/虫蛀)
绿色 (H:35-85, S>50)             → unripe_ratio(未熟特征)

判定规则:
  好果: 红色 > 40% 且 暗斑 < 20%
  坏果: 暗斑 > 25% 或 颜色总面积 < 30%
  未熟: 绿色 > 30%
  未知: 以上均不满足 → 选占比最高的颜色
```

**接口**:

```python
class FruitAnalyzer:
    def __init__(self, detector=None, camera=None, headless=False)
    def analyze_frame(self) -> (list[FruitQuality], dict)
        # 一帧: YOLO检测apple → 逐个HSV分析 → 返回质量列表 + 汇总字典
    def annotate_frame(self, qualities, frame) -> np.ndarray
        # 画框: 绿色=好果, 红色=坏果, 黄色=未熟
```

**设计决策**:
- 组合 Detector 而非继承: 可以共享已有的 Detector/Camera 实例
- `analyze_frame()` 遍历 YOLO results 的全部 apple 检测 (而非 Detector 的最近目标)
- YOLO 置信度建议 0.2 (果园远距离检测，而非室内近距离)

### 2. cloud_uploader.py (~200 行) — 云端上传客户端

**模式**: 完全复用 `doubao_client.py` 的 `urllib.request` 模式 (项目约定: 不使用 requests 库)。

**接口**:

```python
@dataclass
class OrchardReport:
    robot_id, timestamp, orchard_id, row
    total_apples, good, bad, unripe
    good_ratio, location, thumbnail_base64
    @classmethod from_stats(...)  # 从统计字典 + 缩略图构建

class CloudUploader:
    def __init__(self, api_key, endpoint, dataset_id)
    def upload_report(self, report: OrchardReport) -> bool
        # POST JSON → retry 3次 → 失败则本地缓存
    def upload_batch(self, reports) -> int
    def flush_cache(self) -> int  # 重传缓存报告
    @staticmethod encode_image(image, quality=60) -> str  # BGR → base64 JPEG
```

**离线缓存机制**: 上传失败时报告写为 JSON 到 `/tmp/orchard_cache/`，调用 `flush_cache()` 重传。

**缩略图压缩**: 自动缩放至 320px 宽 + JPEG quality=60，控制在 ~10KB 以内。

### 3. orchard_patrol.py (~300 行) — 果园巡检编排器

**状态机**: `NAVIGATING → SCANNING → ANALYZING → REPORTING → COMPLETED`

```
NAVIGATING: 导航到行起点 (或手动放置)
SCANNING:   逐帧分析苹果质量，累计行统计，保留最佳缩略图
ANALYZING:  汇总行统计
REPORTING:  构建 OrchardReport → 上传云端 → 下一行
COMPLETED:  打印全局报告, 保存本地 JSON
```

**关键参数** (ROS2 params):
- `orchard_rows` / `row_spacing` / `row_length` — 果园布局
- `scan_duration_per_m` — 每米扫描时长 (3s/m × 5m = 15s/行)
- `frame_skip` — 跳帧 (每 3 帧分析一次，减少 YOLO 计算)
- `nav_enabled` — 可关闭导航做定点测试
- `confidence_threshold` — 默认 0.2 (远距离检测)

**缩略图选择策略**: 保留苹果检测数量最多的帧作为代表性缩略图。

**双模式**:
- `nav_enabled=true` — 导航到每行起点，完整巡检
- `nav_enabled=false` — 定点扫描，等待手动放置机器人 (按 Enter 开始每行)

## 修改文件

- `robot_brain/config/api_config.yaml` — 新增 `cloud_upload` 配置段
- `robot_brain/CMakeLists.txt` — 新增 3 个 PROGRAMS
- `robot_brain/launch/orchard_patrol.launch.py` — 新建

## 遇到的问题

无。本次是纯新增功能，没有修改已有逻辑。所有模块遵循项目约定 (urllib.request / 组合模式 / ROS2 param / 模块独立可测)。

## 验证结果

- [x] 3 个新文件 + 1 launch 文件 语法检查通过
- [x] `cloud_uploader.py` import 测试通过
- [x] api_config.yaml YAML 结构兼容
- [x] CMakeLists.txt 已更新
- [ ] 实物验证: 需要在 RDK X5 上连接摄像头 + 苹果实物测试 HSV 分类准确度
- [ ] 云端验证: 需要配置火山引擎 DataArk endpoint 进行实际上传测试

## HSV 阈值调优建议

以下参数可针对实际苹果品种微调:

| 参数 | 默认值 | 调优方向 |
|------|--------|----------|
| `RED_UPPER1` hue | 10 | 红富士偏深红 → 加大到 15 |
| `GOOD_RED_MIN` | 0.40 | 严格质检 → 0.50; 宽松 → 0.30 |
| `BAD_DARK_MIN` | 0.25 | 允许少量瑕疵 → 0.35 |
| `GREEN_UPPER` hue | 85 | 青苹果 → 加大到 100 |

建议在真实果园采集 50+ 样本图像，手动标注质量标签，用 `fruit_analyzer.py` 跑一遍对比准确率后调整。

## 启动命令

```bash
# 完整巡检 (导航 + 检测 + 上传)
ros2 launch robot_brain orchard_patrol.launch.py \
    api_key:=YOUR_KEY \
    cloud_endpoint:=https://your-endpoint \
    orchard_rows:=3 row_spacing:=1.5 row_length:=5.0

# 定点测试 (无导航无上传)
ros2 launch robot_brain orchard_patrol.launch.py \
    nav_enabled:=false api_key:=""

# 单独测试质量分析
python3 fruit_analyzer.py
python3 fruit_analyzer.py --headless  # 仅 stdout

# 单独测试云上传
python3 cloud_uploader.py --test --api-key YOUR_KEY --endpoint https://...
```
