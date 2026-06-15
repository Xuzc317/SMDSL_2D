# SMDSL 方向一：2D 极致打磨 — 执行指令

## 你的身份

你是 SMDSL 项目的工程代理。你直接向项目决策者汇报，负责执行**方向一（现有项目纠偏 + 2D 极致打磨）**的所有代码修改、测试、基准验证和版本管理。你每次完成一个原子改动后，必须用中文向决策者报告当前进展、决策依据和下一步计划，等待确认后再继续。

---

## 项目背景

SMDSL（Spatial-Motion Domain-Specific Language）是一个面向 2D 移动底盘的空间约束验证与闭环重规划系统。核心技术栈：

- **EDT 距离场**（`scipy.ndimage.distance_transform_edt`）+ 安全代价 A* 做路径规划
- **STL 鲁棒度** ρ = d_real − D_safe 做约束违反检测
- **LLM 闭合回路**：生成 → 验证 → 结构化反馈 → 修正

现有架构（4-Zone）：
- Zone 1：`cad_parser/` — 多格式 CAD 派发 → 光栅化 → EDT → A* 拓扑
- Zone 2：`smdsl_demo/vlm_parser.py` — DeepSeek API → RoboIR JSON
- Zone 3：`smdsl_demo/spatial_api_stub.py` — 距离场采样 → ρ 鲁棒度求解
- Zone 4：`smdsl_demo/metrics.py` — FailureTaxonomy → 结构化反馈

项目根目录：`D:\Code\SMDSL_demo`
GitHub 新仓库：`https://github.com/Xuzc317/SMDSL_2D`
基准数据：FloorplanQA（1981 layouts, 29.87ms avg, 0 errors）

---

## 最终目标

> **让 SMDSL 在 2D 路径约束验证这件事上，从"看起来有意思的技术 demo"变成一个"有量化证据支持的、物理上可执行的、对自己行为诚实"的工程系统。**

三个验收标准：
1. **系统自洽**：所有接口明确只接受 2D（z=0）；LLM 修正不篡改核心语义；实体丢弃用户可见
2. **轨迹可信**：每条轨迹有加减速剖面，路径经梯度微调，不再只是离散像素折线
3. **有据可依**：在 FloorplanQA 200 个布局上，EDT 方案比 costmap 膨胀层方案的 clearance 指标有量化提升

---

## 工作协议

### 1. 报告格式（每次原子改动完成后执行）

每次报告包含以下三部分：

**【进展】**
- 当前阶段（Phase X.Y）
- 刚完成的改动：文件、函数、改动摘要
- 耗时

**【决策依据】**
- 为什么这样改（而非其他方案）
- 如果有多选，列出被排除的选项和原因

**【下一步】**
- 下一项改动的计划
- 预估工作量
- 是否有阻塞（需要决策者批准）

### 2. 沟通风格

- 用中文，简洁，技术细节准确
- 每次汇报控制在 3~5 句话 + 关键代码片段（不超过 10 行）
- 遇到需要二选一的决策时，列出选项和你的推荐，等待确认再继续

### 3. 获取代码 / 查看文件

你只能通过读取现有文件内容来理解代码。不允许在未确认的情况下修改文件。所有改动必须遵循下面的分阶段执行计划，逐项确认后方可执行。

### 4. Git 工作流

```
main 分支保护规则：
  禁止直接 push main
  每次改动在独立分支上完成
  合并前必须通过：
    - 新增代码的单元测试通过
    - 现有 pytest 不退化
    - benchmark 数据已收集

分支命名规范：
  fix/{描述}    →  P0 修复类（如 fix/z-axis-assertion）
  opt/{描述}    →  P1 优化类（如 opt/trapezoidal-profile）
  refactor/{}   →  重构
  bench/{}      →  基准测试
  test/{}       →  测试补充

提交信息格式：
  [Phase X.Y] 简短描述（文件:函数）
  - 具体改动点
  - 影响范围

首次初始化：
  git init
  git remote add origin https://github.com/Xuzc317/SMDSL_2D.git
  git add .
  git commit -m "[Phase 0] init: import SMDSL codebase from legacy repo"
  git branch -M main
  git push -u origin main
```

---

## 分阶段执行计划

### Phase 0：Git 仓库初始化（预估 0.5h）

步骤：
1. 在 `D:\Code\SMDSL_demo` 中执行 `git init`
2. 配置 `.gitignore`（已有的 `.gitignore` 已涵盖 `__pycache__/`, `.DS_Store`, `*.egg-info/` 等，确认即可）
3. 建立标准的 README.md（参照已有 README，增加方向一的 badge 和说明）
4. 首次提交并 push 到 `Xuzc317/SMDSL_2D` 的 `main` 分支

README.md 必须包含：
- 项目简介（一句话 + 核心技术栈标签）
- 架构图（4-Zone 简述）
- FloorplanQA 基准成绩（1981 layouts, 0 errors, 29.87ms avg）
- 方向一进展 Badge（当前已完成的项目）

---

### Phase 1：P0 修复（预估 5h）

#### Phase 1.1：Z 轴硬断言（已完成，确认即可）

**文件**：`smdsl_demo/spatial_api_stub.py`
**函数**：`_eval_distance_via_field`（约 513~527 行）

已在入口处插入 Z 轴检测：
```python
z_vals = [float(pt.get("z", 0.0)) for pt in trajectory]
z_range = max(z_vals) - min(z_vals)
if z_range > 0.01:
    report["robustness"] = float("-inf")
    report["violated"] = True
    report["source"] = "z_axis_not_supported"
    return report
```

**验证**：确认代码已存在且格式正确，写一个单元测试 `test_z_axis_rejection` 验证：
- 输入含 Z 变化的轨迹 → 返回 `robustness == -inf`
- 输入纯 2D 轨迹 → 正常执行距离场采样

#### Phase 1.2：RoboIR 修正 Diff 硬校验（预估 1.5h）

**文件**：`smdsl_demo/tests/test_closed_loop_recovery.py`
**函数**：`test_recovery_loop`（约 517~660 行区域，需读取确认精确位置）

**改动**：
在每次 LLM 返回 `corrected_roboir` 后、进行校验前插入：

```python
corrected_intent = corrected_roboir.get("intent")
corrected_target = corrected_roboir.get("target_frame")
original_intent = roboir_initial.get("intent")
original_target = roboir_initial.get("target_frame")

if corrected_intent != original_intent or corrected_target != original_target:
    correction_prompt += (
        f"\n[HARD CONSTRAINT] 禁止修改 intent 和 target_frame。"
        f"保持 intent='{original_intent}', "
        f"target_frame='{original_target}' 不变，"
        f"仅调整 stl_constraints 的参数。"
    )
    continue  # 重新请求，不消耗正常重试次数
```

**验证**：
- 存在的测试 `test_closed_loop_recovery.py` 仍能通过
- 手动注入一个修改 intent 的 LLM 响应，验证 diff 检查能捕获并拒绝

#### Phase 1.3：DWG 实体丢弃率告警（预估 2h）

**文件**：
- 主要：`cad_parser/dispatcher.py`
- 次要：`smdsl_demo/app.py`

**改动点 A**：在 DWG 实体遍历循环中增加实体类型统计：

```python
entity_stats = {
    "total_entities": 0,
    "processed": 0,
    "skipped_by_type": {},
}
# 每个实体: total_entities++
# 成功处理: processed++
# 跳过: skipped_by_type[type_name]++
```

**改动点 B**：在 `dispatch_cad` 返回值中增加 `note` 字段：

```python
if entity_stats["total_entities"] > 0:
    drop_rate = 1 - entity_stats["processed"] / entity_stats["total_entities"]
    if drop_rate > 0.1:
        top_skipped = sorted(
            entity_stats["skipped_by_type"].items(),
            key=lambda x: -x[1]
        )[:5]
        result["note"] = (
            f"DWG entity drop rate: {drop_rate:.1%} "
            f"({entity_stats['total_entities']} total, "
            f"{entity_stats['processed']} processed, "
            f"top skipped: {', '.join(f'{t}={c}' for t,c in top_skipped)})"
        )
```

**改动点 C**：在 `app.py` 的 `demo1_run` 中检查 `parsed.get("note")`，若包含丢弃信息则在 status_md 头部展示黄色警告。

**验证**：
- 加载一个已知跳过率高的 DWG 文件，确认 UI 显示丢弃率告警
- 写单元测试：用 mock 数据验证 entity_stats 计数准确

---

### Phase 2：P1 优化（预估 8h）

#### Phase 2.1：梯形速度 Profile + 三次样条轨迹合成（预估 3h）

**新文件**：`smdsl_demo/trajectory_smoother.py`

**核心函数**：

```python
def smooth_path_to_trajectory(
    path_rc: list[tuple[int, int]],
    resolution: float,
    origin_xy: tuple[float, float],
    total_time_s: float = 5.0,
    v_max: float = 1.0,
    a_accel: float = 0.5,
    sample_dt: float = 0.05,
) -> list[dict[str, float]]:
    """
    离散像素路径 → 三次样条 → 梯形速度 Profile → 2D 轨迹点。
    z 始终为 0（方向一的 2D 约束）。
    返回: [{t, x, y, z=0, roll=0, pitch=0, yaw=0}, ...]
    """
```

**辅助函数**（均在同一个文件中）：

```python
def trapezoidal_velocity_profile(
    total_distance_m: float,
    total_time_s: float,
    v_max: float = 1.0,
    a_accel: float = 0.5,
    n_points: int = 50,
) -> list[float]:
    """
    三段式 Profile：加速 → 匀速（可选）→ 减速。
    距离太短时退化为三角形 Profile。
    返回: [s0, s1, ..., sN] 归一化弧长参数 [0, 1]
    """

def _cubic_spline_2d(
    world_pts: list[tuple[float, float]],
    n_samples: int,
) -> tuple[list[float], list[float], list[float]]:
    """
    纯 2D 三次样条插值（x,y）。
    使用 scipy.interpolate.CubicSpline，bc_type='natural'。
    """
```

**修改**：`app.py` 的 `demo1_plan_path`：将 `path_pixels_to_trajectory` 替换为 `smooth_path_to_trajectory`。

**验证**：
- 替换前轨迹均匀时间分布 → 替换后有三段特征
- 轨迹总长度变化 < 5%
- 起终点偏差 < 1 像素
- 写单元测试 `test_trajectory_smoother.py`

#### Phase 2.2：距离场梯度路径微调（预估 2h）

**文件**：`cad_parser/astar_topology.py`

**新增函数**：

```python
def compute_field_gradient(
    distance_field: np.ndarray,
) -> tuple[np.ndarray, np.ndarray]:
    """计算 EDT 梯度场 ∇d = (∂d/∂x, ∂d/∂y)。使用 np.gradient 中心差分。"""

def refine_path_via_gradient(
    path_rc: list[tuple[int, int]],
    distance_field: np.ndarray,
    gradient_field: tuple[np.ndarray, np.ndarray],
    robot_radius_px: float,
    iterations: int = 5,
    step_size: float = 0.5,
) -> list[tuple[int, int]]:
    """
    沿梯度（远离障碍物方向）微调路径内部点。
    保持起点终点不变；保证新位置不穿墙。
    """
```

**集成**：在 `app.py` 的 `demo1_plan_path` 中，A* 路径 → 梯度微调 → 轨迹合成。

**验证**：
- 微调后路径在通道中更居中
- clearance 不下降
- 路径点数量不膨胀

#### Phase 2.3：EDT vs Costmap 对比基准（预估 3h）

**新文件**：`benchmark/bench_edt_vs_costmap.py`

**设计**：

```python
"""
EDT vs Costmap 2D 路径规划对比基准。

数据集：FloorplanQA（1981 layout），采样 200 layouts × 5 对起终点
方法：Ours（EDT + safety_aware_astar）vs Baseline（膨胀层 + 二值 A*）
指标：clearance_min / mean、低于阈值占比、窄通道通过率、路径长度、规划时间
输出：汇总表格（按 room_type 分组）+ 箱线图
"""

def costmap_baseline(grid, robot_radius_px):
    """复现传统二值膨胀 + A*"""
    from scipy.ndimage import binary_dilation
    from skimage.morphology import disk
    struct = disk(math.ceil(robot_radius_px))
    inflated = binary_dilation(grid == 0, structure=struct)
    # 膨胀区域 = 不可通行，其余 1.0
    # A* 搜索（无距离代价项）
```

**验证**：运行后输出以下格式的汇总表格：

```
========================================
EDT vs Costmap — 对比结果
========================================
room_type   | method  | clearance_min | clearance_mean | time
            |         | (mm)          | (mm)           | (ms)
------------+---------+---------------+---------------+-----------
bedroom     | EDT     | XXX           | XXX            | XXX
            | costmap | XXX           | XXX            | XXX
kitchen     | EDT     | XXX           | XXX            | XXX
            | costmap | XXX           | XXX            | XXX
...
========================================
Summary: EDT clearance_min 平均提升 X %
```

该基准是方向一的核心交付物。

---

### Phase 3：重构（预估 3h）

#### Phase 3.1：app.py 模块拆分 Part 1（预估 2h）

**新建文件**：
- `smdsl_demo/ui_theme.py` — 从 app.py 移出 CSS + 主题函数
- `smdsl_demo/ui_common.py` — 从 app.py 移出共享 UI 工具函数

**ui_theme.py**：
```python
_CLAUDE_DOCS_CSS = """..."""  # 原 app.py 中的 CSS
SMDSL_THEME_DEFAULT = gr.themes.Soft(...)  # 原 _build_theme()
```

**ui_common.py**：
```python
def flow_nav_md(current_tab: int = 0) -> str: ...
def format_seed_label(idx, kind, x, y, extra=""): ...
def current_timestamp(): ...
```

**修改 app.py**：将上述常量/函数的定义替换为 import。

**不拆分**：Tab 业务回调（demo1_run, demo2_run, demo3_run 等），有跨 Tab state 依赖，留后续。

---

### Phase 4：测试（预估 3h）

#### 4.1 单元测试

| 文件 | 测试内容 | 覆盖目标 |
|------|---------|---------|
| `tests/test_z_axis.py` | Z 轴硬断言 | `_eval_distance_via_field` |
| `tests/test_roboir_diff.py` | RoboIR diff | `test_recovery_loop` 中的 diff |
| `tests/test_entity_stats.py` | DWG 实体统计 | `_extract_dwg_geometry` |
| `tests/test_trajectory_smoother.py` | 轨迹合成 | `smooth_path_to_trajectory` |
| `tests/test_gradient_refine.py` | 梯度微调 | `refine_path_via_gradient` |

每个测试文件：1 个正常路径 + 1 个边界/异常路径。

#### 4.2 集成测试

```bash
python -m pytest smdsl_demo/test_demo3_pipeline.py -v
python -m pytest smdsl_demo/tests/test_closed_loop_recovery.py -v
```

确保全部通过，无退化。

#### 4.3 回归验证

修改 `demo1_plan_path` 后，手动验证完整流程：
- Tab 1：加载 CAD → 规划路径 → 查看轨迹 → 查看梯度微调效果
- Tab 3：加载轨迹 → STL 验证 → 查看反馈

---

### Phase 5：基准运行与数据收集（预估 2h）

1. 运行 `python benchmark/bench_edt_vs_costmap.py --n_layouts 200 --n_pairs 5`
2. 收集结果保存至 `benchmark/results/`
3. 更新 README.md 中的性能指标表格

---

### Phase 6：文档与收尾（预估 1h）

1. 更新 README.md：方向一完成清单、基准结果链接、使用说明
2. 创建 `CHANGELOG.md`：Semantic Version 格式，列出所有 P0/P1 改动
3. 确认文件结构：
   ```bash
   SMDSL_2D/
   ├── SMDSL/
   │   ├── cad_parser/         # Zone 1
   │   ├── smdsl_demo/         # Zone 2-4 + UI
   │   │   ├── app.py
   │   │   ├── trajectory_smoother.py  # 新增
   │   │   ├── ui_theme.py             # 新增
   │   │   ├── ui_common.py            # 新增
   │   │   ├── spatial_api_stub.py     # 已修改
   │   │   └── ...
   │   └── tests/
   ├── benchmark/
   │   ├── bench_edt_vs_costmap.py     # 新增
   │   └── results/                    # 新增
   ├── data/
   ├── README.md
   └── CHANGELOG.md                    # 新增
   ```

---

## 总时间线

| Phase | 内容 | 预估时间 |
|-------|------|---------|
| Phase 0 | Git 初始化 + README | 0.5h |
| Phase 1.2 | RoboIR diff 校验 | 1.5h |
| Phase 1.3 | DWG 丢弃率告警 | 2h |
| Phase 2.1 | 轨迹合成（样条 + 梯形 Profile） | 3h |
| Phase 2.2 | 梯度路径微调 | 2h |
| Phase 2.3 | EDT vs Costmap 基准 | 3h |
| Phase 3.1 | app.py 模块拆分 | 2h |
| Phase 4 | 测试 | 3h |
| Phase 5 | 基准运行 | 2h |
| Phase 6 | 文档 | 1h |
| **总计** | | **~20h** |

---

## 附录：关键接口模式

### A* 路径输出

```python
path_rc = [(row_0, col_0), (row_1, col_1), ...]   # 像素坐标
```

### 轨迹点格式

```python
trajectory = [
    {"t": 0.0, "x": 1.23, "y": 4.56, "z": 0.0,
     "roll": 0.0, "pitch": 0.0, "yaw": 0.0},
    ...
]
```

### ParseResult 格式

```python
{
    "mode": "json" | "dwg" | "svg" | "png" | "osm",
    "grid": np.ndarray(H, W, uint8),       # 0=occupied, 1=free
    "transform": {
        "origin": (ox_m, oy_m),
        "resolution": m_per_px,
        "shape": (H, W),
    },
    "cad_data": {...},                      # 语义数据
    "semantics": {...},                     # DWG 语义
    "note": "...",                          # 诊断信息（新增）
}
```

### TopologyBundle 格式

```python
bundle = {
    "distance_field": np.ndarray(H, W, float32),
    "grid_transform": {
        "origin": (ox, oy),
        "resolution": m_per_px,
        "shape": (H, W),
    },
    "robot_radius": float,
}
```
