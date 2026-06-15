# SMDSL 方向一 v2 — 执行结果报告

> 版本：v2 · 执行日期：2026-06-15
> 状态：✅ 全部完成（63/63 单元测试通过）
> GitHub：`Xuzc317/SMDSL_2D`，已推送 `8842825..dc36c49`

---

## 一、新增提交

| SHA | 说明 |
|-----|------|
| `5e3a687` | Phase 2.2 — 新增 Architecture Tab + Demo Recording Tab |
| `dc36c49` | Phase 1.2 — 基准脚本重写增强（箱线图 + 窄通道分析） |

---

## 二、Phase 1 — P0 修复

### 2.1 Phase 1.1：RoboIR diff 硬校验 ✅ 验证确认

**文件**：`tests/test_closed_loop_recovery.py` L586-606、`tests/test_roboir_diff.py`

**改动摘要**：
- v1 已实现完整逻辑 — 每次 LLM 修正返回 `corrected_roboir` 后对比 `intent` 和 `target_frame`
- 不一致 → `[DIFF-REJECT]`，prompt 追加硬约束 `[HARD CONSTRAINT] 禁止修改 intent 和 target_frame`，不消耗重试次数
- 一致 → 继续物理校验 + 轨迹重规划

**验证**：
- `test_roboir_diff.py`：6 个测试（intact、intent 修改、target 修改、两者同时、None 处理、缺失字段），全部通过
- `test_closed_loop_recovery.py`：闭环自愈测试可独立运行

**代码证据**：
```python
# test_closed_loop_recovery.py L586-606
corrected_intent = corrected_roboir.get("intent")
corrected_target = corrected_roboir.get("target_frame")
original_intent = roboir.get("intent")
original_target = roboir.get("target_frame")
if corrected_intent != original_intent or corrected_target != original_target:
    print(f"[DIFF-REJECT] LLM 非法修改 intent/target_frame: ...")
    correction_prompt += (
        f"\n[HARD CONSTRAINT] 禁止修改 intent 和 target_frame。"
        f"保持 intent='{original_intent}', target_frame='{original_target}' 不变，"
        f"仅调整 stl_constraints 的参数。"
    )
    continue
```

---

### 2.2 Phase 1.2：EDT vs Costmap 对比基准 ✅ 重写增强

**文件**：`benchmark/bench_edt_vs_costmap.py`（+316 行，重写）

**改动摘要**：

| 功能 | v1 骨架版 | v2 增强版 |
|------|----------|----------|
| 起终点采样 | 1 对/布局 | **n_pairs 对/布局**（可配置） |
| 窄通道检测 | 无 | `_narrow_passage_exists()` — 宽度 ≤ 1.5×机器人直径 |
| 指标数量 | 4 个 | **7 个**：clearance_min、clearance_mean、below_radius_%、窄通道通过率、path_length、planning_time |
| 可视化 | 无 | **箱线图**（按 room_type 分组，EDT vs Costmap 并排） |
| 数据源 | 仅合成 | 合成 + FloorplanQA（`--use_floorplanqa` 开关） |
| 输出 | 仅 console | console + **JSON** + **PNG** → `benchmark/results/` |
| 汇总表 | 全局均值 | **按 room_type 分组**，含窄通道分析 |

**新增函数**：
- `_path_clearance()` — 返回 `(min_mm, mean_mm, below_radius_pct)`
- `_narrow_passage_exists()` — 检测窄通道存在性
- `_find_seed_pairs()` — 多对随机起终点采样
- `run_layout_benchmark()` — 单布局上 EDT vs Costmap 对比
- `_generate_boxplots()` — matplotlib 箱线图生成
- `_print_summary_table()` — 按 room_type 分组的汇总表格

**验证运行**（20 布局 × 3 对）：

```
Room Type                N  EDT OK%   CM OK%   EDT min    CM min     Δ mm   EDT ms    CM ms
===========================================================================================
synthetic_bedroom       15   100.0%    80.0%   1171.6   1189.2   -17.6     127     159
synthetic_kitchen       15    80.0%    80.0%   1012.9    869.5  +143.4     132     111
synthetic_living        15    86.7%    80.0%    582.0    471.9  +110.1     111      94
synthetic_narrow        15    86.7%    86.7%    915.7    541.9  +373.8     141     123

  Narrow passage (30/60 layouts):
    EDT pass rate:  76.7%
    Costmap pass rate: 63.3%
```

**输出文件**：
- `benchmark/results/bench_results.json` — 原始数据
- `benchmark/results/boxplot_clearance.png` — 箱线图

---

## 三、Phase 2 — P1 优化

### 3.1 Phase 2.1：操作说明精简 ✅ v1 已完成

**文件**：`app.py` L1801-1808

**改动摘要**：Tab 1 顶部"这一步在做什么"从 40+ 行（格式对比表 + 处理流程 + 输出说明）精简为 4 行操作流程。

**改动前**（40+ 行）：
```
**输入**：CAD 平面图。**支持三种格式 → 不同语义能力**：
| 格式 | 解析方式 | Demo 2 词汇能力 | Demo 3 路径规划 |
| ... | ... | ... | ... |
**处理**：栅格化 → 外部剔除 → 形态学破壁 → 距离场 (EDT) → ...
**输出**：① 4 联可视化；② 候选起终点清单；③ ...
```

**改动后**（4 行）：
```markdown
### 操作流程
1. 选择/上传 CAD 文件 → 自动解析
2. 查看 4 联图结果（栅格/距离场/A* 热力/拓扑）
3. 选起终点 → 点击「规划路径」→ 查看路径
4. 路径轨迹自动同步至 Demo 3
```

---

### 3.2 Phase 2.2：新模块集成到 app.py ✅ 新增 2 个 Tab

**文件**：`app.py`（+45 行）、`architecture_viz.py`（1 行修复）

**改动摘要**：

**(A) Architecture Tab**：
- 位置：`build_ui()` 末尾，`return demo` 之前
- 内容：调用 `get_architecture_html()` 渲染 SMDSL 4-Zone 架构图

```python
with gr.Tab("Architecture (N-layer Zone)"):
    from smdsl_demo.architecture_viz import get_architecture_html
    gr.HTML(get_architecture_html())
```

**(B) Demo Recording Tab**：
- 输入：App URL（默认 `http://127.0.0.1:7860`）+ 场景选择器（full/cad/compile/verify/all）
- 输出：录制视频 + 状态提示

```python
with gr.Tab("Demo Recording"):
    gr.Markdown("## SMDSL Demo Recording")
    drec_url = gr.Textbox(label="App URL", value="http://127.0.0.1:7860")
    drec_scenario = gr.Dropdown(
        choices=["full", "cad", "compile", "verify", "all"],
        value="full", label="Scenario",
    )
    drec_btn = gr.Button("Record Demo", variant="primary")
    drec_video = gr.Video(label="Recorded Demo")
```

**(C) Handler 函数**（`build_ui()` 之前）：

```python
def _run_recording(app_url: str, scenario: str):
    from smdsl_demo.demo_recorder import record_demo_video
    results = record_demo_video(app_url=app_url, scenario=scenario)
    video_path = next((v for v in results.values() if v), None)
    if video_path:
        return video_path, f"✅ Saved: {video_path}"
    return None, f"❌ Recording failed: {e}"
```

**(D) architecture_viz.py 修复**：
- 问题：f-string 中 `{z['id']}` 在 Python 3.11 下引号冲突导致 `SyntaxError`
- 修复：外层 f-string 改用双引号 `f"..."` 包裹，内部 HTML 属性用 `\"` 转义

**验证**：
- `get_architecture_html()` 导入成功（11,272 字符 HTML）
- 全量测试 63/63 无退化
- `py_compile` 语法检查通过

---

### 3.3 Phase 2.3：轨迹合成接入 ✅ v1 已实装

**文件**：`app.py` `_plan_path_core` L814, L874

**改动摘要**：`_plan_path_core` 已调用 `smooth_path_to_trajectory`（来自 `trajectory_smoother`），管线为：

```
A* 像素路径 → 三次样条平滑 → 梯形速度 Profile → [{t, x, y, z=0, roll, pitch, yaw}]
```

**对应单元测试**：`tests/test_trajectory_smoother.py` — 11/11 通过

---

### 3.4 Phase 2.4：梯度微调 ✅ v1 已实装

**文件 A**：`cad_parser/astar_topology.py` L893-941

| 函数 | 行号 | 说明 |
|------|------|------|
| `compute_field_gradient(df)` | L893-898 | `np.gradient` 中心差分，返回 `(gx, gy)` |
| `refine_path_via_gradient(path, df, grad, r_px, iterations=5)` | L901-941 | 梯度上升微调内部点，保持起终点不变，保 `clearance ≥ robot_radius_px` |

**文件 B**：`app.py` `_plan_path_core` L866-872

```python
from cad_parser.astar_topology import compute_field_gradient, refine_path_via_gradient
gx, gy = compute_field_gradient(df)
path = refine_path_via_gradient(path_rc, df, (gx, gy), safe_radius_px)
```

**对应单元测试**：`tests/test_gradient_refine.py` — 7/7 通过

---

### 3.5 Phase 2.5：DWG entity_stats 完整验证 ✅ 链路确认

**文件**：`cad_parser/dispatcher.py` L470-575、`app.py` L360, L516-522

**完整链路**：

| 环节 | 位置 | 状态 |
|------|------|------|
| ① 实体统计 | `dispatcher.py` `_extract_dwg_geometry()` → `entity_stats` 计数器 | ✅ |
| ② 丢弃率格式化 | `dispatcher.py` `_format_entity_drop_note()` → 丢弃率 >10% 时生成告警 | ✅ |
| ③ 注入 ParseResult | `dispatcher.py` `_parse_dwg()` → `result["note"]` 末尾追加 | ✅ |
| ④ UI 消费 | `app.py` L360 `parsed["note"]` 存入 `_dataset_note` | ✅ |
| ⑤ 黄色告警 | `app.py` L516-522 检测"实体丢弃率"关键词，显示黄色 banner | ✅ |

**对应单元测试**：`tests/test_entity_stats.py` — 10/10 通过

---

### 3.6 Phase 2.6：Git 提交 ✅ 已推送

```
8842825..dc36c49  main -> main
```

---

## 四、最终状态

### 4.1 测试结果

| 测试文件 | 测试数 | 结果 |
|----------|--------|------|
| `test_z_axis.py` | 5 | ✅ |
| `test_roboir_diff.py` | 6 | ✅ |
| `test_entity_stats.py` | 10 | ✅ |
| `test_trajectory_smoother.py` | 11 | ✅ |
| `test_gradient_refine.py` | 7 | ✅ |
| `test_metrics.py` | 7 | ✅ |
| `test_rust_compiler.py` | 18 | ✅ |
| **合计** | **63** | **✅ 100%** |

### 4.2 基准结果

| 指标 | EDT (Ours) | Costmap (Baseline) | 差异 |
|------|-----------|-------------------|------|
| 窄通道通过率 | 76.7% | 63.3% | **+13.4pp** |
| 路径找到率 | 88.3% | 81.7% | +6.7pp |
| 平均规划时间 | 128ms | 122ms | +6ms |

### 4.3 文件变更清单

| 操作 | 文件 | 行数变化 |
|------|------|----------|
| 修改 | `app.py` | +45 行（Architecture Tab + Recording Tab） |
| 修改 | `architecture_viz.py` | 1 行修复（f-string 兼容） |
| 重写 | `benchmark/bench_edt_vs_costmap.py` | +316 行（箱线图 + 窄通道 + 多指标） |
| 新增 | `benchmark/results/bench_results.json` | 基准原始数据 |
| 新增 | `benchmark/results/boxplot_clearance.png` | 箱线图 |
