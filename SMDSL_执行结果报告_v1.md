# SMDSL 方向一 v1 修改方案 — 执行结果报告

> 版本：v1 · 执行日期：2026-06-15
> 状态：✅ 全部完成（38/38 单元测试通过）
> GitHub：`Xuzc317/SMDSL_2D`，已推送 `5d7e17e..1d2ecee`

---

## 一、提交记录

| SHA | 说明 |
|-----|------|
| `a22cfbe` | Config — 添加 python-dotenv，从 `.env` 加载 DeepSeek API Key |
| `081d2b5` | Cleanup — 删除 motion_profile.py（与 trajectory_smoother.py 100% 重叠） |
| `df706ba` | UI — Phase 1.3：路径修正、说明精简、3D 区块删除、Demo 3→2D |
| `1d2ecee` | Docs/Benchmark — 基准框架、CHANGELOG v0.2.1、README 更新 |

---

## 二、Phase 0 — 已有文件审查

### 2.1 `trajectory_smoother.py` ✅

| 功能 | 函数 | 结论 |
|------|------|------|
| 三次样条 | `_cubic_spline_2d()` | ✅ scipy CubicSpline，bc_type="natural" |
| 梯形速度 Profile | `trapezoidal_velocity_profile()` | ✅ 三段式（加速→匀速→减速），距离短时退化为三角形 |
| 轨迹合成入口 | `smooth_path_to_trajectory(...)` | ✅ 输出 `{t, x, y, z=0, roll, pitch, yaw}` |
| 2D 约束 | z = 0 | ✅ 始终为 0 |

### 2.2 `ui_theme.py` ✅

| 功能 | 内容 |
|------|------|
| CSS | `SMDSL_THEME_CSS`（536 行），Apple 设计语言，亮/暗双模式 |
| 主题构建 | `build_theme()` + `build_theme_compact()` |
| 主题切换 JS | `THEME_TOGGLE_JS`（含键盘快捷键 D） |

### 2.3 `ui_common.py` ✅

| 函数 | 用途 |
|------|------|
| `flow_nav_md(active)` | 流程导航条 HTML |
| `format_seed_label(idx, kind, x, y, extra)` | 种子标签格式化 |
| `current_timestamp()` | 时间戳 |

### 2.4 `motion_profile.py` 🔁 → 已删除

- 与 `trajectory_smoother.py` **100% 功能重叠**（三次样条 + 梯形 Profile）
- 提交 `081d2b5`：删除 1 文件，-129 行

---

## 三、Phase 1 — P0 修复 + UI 修正

### 3.1 RoboIR diff 硬校验 ✅（已存在）

**文件**：`SMDSL/tests/test_closed_loop_recovery.py` L586-606

**机制**：
- LLM 修正返回 `corrected_roboir` 后，对比 `intent` 和 `target_frame` 是否与原始一致
- 不一致 → `[DIFF-REJECT]`，prompt 追加硬约束，不消耗重试次数
- 一致 → 继续物理校验

**单元测试**：`tests/test_roboir_diff.py` — 6/6 通过

### 3.2 DWG 实体丢弃率告警 ✅（已存在）

**文件**：`SMDSL/cad_parser/dispatcher.py` L470-575

**机制**：
- `_extract_dwg_geometry()` → `entity_stats` 计数器（按类型统计 processed/skipped）
- `_format_entity_drop_note()` → 丢弃率 >10% 时生成告警字符串
- `_parse_dwg()` 末尾自动追加到 `result["note"]`

**单元测试**：`tests/test_entity_stats.py` — 10/10 通过

### 3.3 app.py UI 修正 ✅（提交 `df706ba`）

| # | 问题 | 文件位置 | 改动 | 状态 |
|---|------|----------|------|------|
| A | Tab 1 说明文字过冗 | L1845-1857 | 40+ 行 → 4 行操作流程 | ✅ |
| B | `_DATA_ROOT` 路径错误 | L72 | `data/cad_samples` → `SMDSL/data/cad_samples` | ✅ |
| C | 预设样本路径错误 | L81-82 | 加 `SMDSL/` 前缀 | ✅ |
| C | 测试预设路径硬编码 | L1878-1916 | 6 个硬编码路径 → `_REPO_ROOT` 动态拼接 | ✅ |
| D | 机器人半径 slider | L1975-1978 | 保留，info 简化为"影响路径与障碍物的最小距离" | ✅ |
| E | 3D 拓扑白模预览 | L1995-1997 | Accordion + Button + Plot **全部删除** | ✅ |
| F | 3D 空间场景图 | L1999-2014 | Accordion + Markdown + Button + Plot **全部删除** | ✅ |
| G1 | handler `demo1_3d_preview` | L1145-1163 | **已删除** | ✅ |
| G2 | handler `demo1_scene_graph_3d` | L1166-1227 | **已删除** | ✅ |
| H | 3D 回调绑定 | L2132-2147 | `d1_3d_btn.click` + `d1_scene3d_btn.click` **已删除** | ✅ |
| I | Demo 3 3D→2D | L2465 + L1609-1629 | Plotly 3D → matplotlib 2D 路径叠加图 + ρ 曲线 | ✅ |

#### 3.3.I Demo 3 2D 叠加图详情

**之前**：
```python
from smdsl_demo.visualize_demo3 import generate_3d_dashboard
fig_3d, fig_rho = generate_3d_dashboard(...)
```

**现在**：
- `fig_traj`：matplotlib imshow（距离场背景）+ plot（路径线）+ scatter（起终点 + 违规节点）
- `fig_rho`：matplotlib 柱状图（每个约束的 ρ 值）
- 标签从"📊 3D Trajectory Sandbox"改为"2D 轨迹叠加图"
- 移除了 `generate_3d_dashboard`、`render_robustness_curve`、`render_trajectory_overlay` 全部 import

---

## 四、Phase 2 — P1 优化（已实装）

### 4.1 轨迹合成接入 ✅

**文件**：`app.py` L814, L874

`_plan_path_core` 已调用 `smooth_path_to_trajectory`：

```
A* 路径 → 三次样条平滑 → 梯形速度 Profile → [{t, x, y, z, roll, pitch, yaw}]
```

### 4.2 距离场梯度微调 ✅

**文件**：`cad_parser/astar_topology.py` L893-941

| 函数 | 行号 | 说明 |
|------|------|------|
| `compute_field_gradient(df)` | L893-898 | `np.gradient` 中心差分，返回 `(gx, gy)` |
| `refine_path_via_gradient(path, df, grad, r_px, iters=5)` | L901-941 | 梯度上升微调，保 clearance ≥ `robot_radius_px` |

**集成点**（`app.py` L866-872）：
```
A* 最短路径 → compute_field_gradient → refine_path_via_gradient → smooth_path_to_trajectory
```

**单元测试**：`tests/test_gradient_refine.py` — 7/7 通过

---

## 五、Phase 3 — 模块拆分（已实装）

所有拆分已在本次会话前完成。`app.py` L87-88 直接导入：

```python
from smdsl_demo.ui_common import (
    format_seed_label as _format_seed_label,
    flow_nav_md as _flow_nav_md,
    current_timestamp as _ts,
)
from smdsl_demo.ui_theme import (
    SMDSL_THEME_CSS as _CLAUDE_DOCS_CSS,
    build_theme as _build_theme,
    THEME_TOGGLE_JS,
)
```

---

## 六、Phase 4 — 测试结果

### 6.1 单元测试：38/38 通过 ✅

| 测试文件 | 测试数 | 结果 |
|----------|--------|------|
| `tests/test_z_axis.py` | 5 | ✅ PASSED |
| `tests/test_roboir_diff.py` | 6 | ✅ PASSED |
| `tests/test_entity_stats.py` | 10 | ✅ PASSED |
| `tests/test_trajectory_smoother.py` | 11 | ✅ PASSED |
| `tests/test_gradient_refine.py` | 7 | ✅ PASSED |
| **合计** | **38** | **✅ 100%** |

### 6.2 集成测试：⏳ 待数据

`smdsl_demo/test_demo3_pipeline.py` — 需要 FloorplanQA 数据集（`room_24.json` 不在仓库中）。下载数据后可运行：

```bash
cd SMDSL && python -m pytest smdsl_demo/test_demo3_pipeline.py -v
```

---

## 七、Phase 5 — 基准 + 文档 + 推送

### 7.1 基准脚本 ✅

**文件**：`SMDSL/benchmark/bench_edt_vs_costmap.py`

| 功能 | 说明 |
|------|------|
| EDT 方案 | `safety_aware_astar_flood`（连续距离场） |
| Costmap 方案 | `binary_dilation` + 二值 A* |
| 指标 | `clearance_min`、`clearance_mean`、窄通道通过率、`planning_time` |
| 合成数据 | 随机墙体生成器（含缺口/窄通道） |

**验证运行（5 布局）**：

| 布局 | EDT | Costmap | Δ clearance_min |
|------|-----|---------|----------------|
| 0 | OK (137.9cm) | OK (137.9cm) | +0.0cm |
| 1 | OK (28.3cm) | OK (25.0cm) | **+3.3cm** |
| 2 | OK (28.3cm) | OK (25.0cm) | **+3.3cm** |
| 3 | OK (39.1cm) | OK (39.1cm) | +0.0cm |
| 4 | **OK** (25.0cm) | **FAIL** (0.0cm) | **+25.0cm** |

> EDT 在 costmap 失败的窄通道场景下成功规划，展示了连续距离场在狭窄空间的优势。

### 7.2 文档 ✅

| 文件 | 改动 |
|------|------|
| `CHANGELOG.md` | 新增 v0.2.1：UI 修正、3D 删除、安全措施、基准框架 |
| `README.md` | 更新 Git HEAD 引用为 `df706ba` |
| `SMDSL_Modification_v1.md` | 修改方案 v1（已入仓） |
| `SMDSL_Execute_Prompt_v1.md` | 执行指令（已入仓） |

### 7.3 安全验证 ✅

| 检查项 | 结果 |
|--------|------|
| `.env` 在 `.gitignore` | ✅ |
| `.env` 零次出现在 Git 历史 | ✅ |
| DeepSeek API Key 未泄露 | ✅ |

### 7.4 Git 推送 ✅

```
5d7e17e..1d2ecee  main -> main
```

远程仓库：`git@github.com:Xuzc317/SMDSL_2D.git`

---

## 八、待办项

| # | 事项 | 说明 |
|---|------|------|
| 1 | 下载 FloorplanQA 数据集 | 1997 个布局 JSON，放入 `SMDSL/data/cad_samples/floorplanqa/layouts/` |
| 2 | 运行集成测试 | `test_demo3_pipeline.py` 需要 room_24.json |
| 3 | 200 布局全量基准 | `bench_edt_vs_costmap.py --n_layouts 200` |
| 4 | OSM 数据 | 需代理/VPN 访问 nominatim.openstreetmap.org |
| 5 | Demo 2/3 VLM 端到端验证 | 需 DeepSeek API 可连通 |

---

## 九、文件变更清单

| 操作 | 文件 |
|------|------|
| 删除 | `SMDSL/smdsl_demo/motion_profile.py` |
| 新建 | `SMDSL/benchmark/bench_edt_vs_costmap.py` |
| 新建 | `SMDSL/benchmark/results/`（目录） |
| 新增 | `SMDSL_Modification_v1.md` |
| 新增 | `SMDSL_Execute_Prompt_v1.md` |
| 修改 | `SMDSL/smdsl_demo/app.py`（+80 / -167 行） |
| 修改 | `requirements.txt`（+1 行：python-dotenv） |
| 修改 | `SMDSL/smdsl_demo/mcp_server.py`（+5 行：dotenv 加载） |
| 修改 | `CHANGELOG.md` |
| 修改 | `README.md` |
