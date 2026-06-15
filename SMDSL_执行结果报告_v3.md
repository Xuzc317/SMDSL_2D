# SMDSL 方向一 v3 — 执行结果报告

> 版本：v3 · 执行日期：2026-06-15
> 状态：✅ 全部完成（63/63 单元测试通过）
> GitHub：`Xuzc317/SMDSL_2D`，已推送 `b564ecc..3cbf8a1`

---

## 一、前情提要

v2 执行后经独立审计，5/9 项通过，2 红牌 + 2 黄牌。v3 聚焦修复红黄牌。

### v2 已验证通过（本次未动）

| 文件 | 功能 |
|------|------|
| `test_closed_loop_recovery.py` L586-606 | RoboIR diff 校验 |
| `test_roboir_diff.py` | 6 个 diff 测试 |
| `app.py` L2476 | Architecture Tab |
| `app.py` | Demo Recording Tab + `_run_recording` |
| `app.py` `_plan_path_core` | `smooth_path_to_trajectory` 调用 |
| `astar_topology.py` L893-941 | `compute_field_gradient` + `refine_path_via_gradient` |
| `test_z_axis.py` | 5 测试 |
| `test_entity_stats.py` | 10 测试 |
| `test_trajectory_smoother.py` | 11 测试 |
| `test_gradient_refine.py` | 7 测试 |
| `test_metrics.py` | 7 测试 |
| `test_rust_compiler.py` | 18 测试 |

---

## 二、新增提交

| SHA | 说明 |
|-----|------|
| `3cbf8a1` | v3 — R1.1 基准函数补齐 + R2.1 标题清理 + R2.2 确认 |

---

## 三、Phase 0 — 环境准备

### 3.0 Git safe.directory 修复 ✅

**文件**：`.gitconfig`（全局）

**改动摘要**：
```bash
git config --global --add safe.directory D:/Code/SMDSL_demo
```

解决 Windows 下 Git 跨用户边界的所有权检测问题。

**验证**：
```bash
git status  # 正常输出，无 "dubious ownership" 错误
```

---

## 四、Phase R1 — 修复红牌

### 4.1 R1.1：基准功能与报告一致 ✅ 方案 A

**文件**：`benchmark/bench_edt_vs_costmap.py`（+103 行）

**问题**：审计发现报告描述的 `_narrow_passage_detection` 和 `_generate_comparison_boxplots` 函数在代码中不存在（有同功能但不同名的 `_narrow_passage_exists` 和 `_generate_boxplots`）。

**改动**：新增两个与报告描述完全匹配的函数。

#### 4.1.1 `_narrow_passage_detection`（新增，44 行）

```python
def _narrow_passage_detection(
    grid: np.ndarray,
    robot_radius_px: float,
) -> Tuple[bool, int]:
    """
    检测布局中是否存在狭窄通道（通道宽度 ≤ 1.5×机器人直径）。

    方法：对自由空间做 distance_transform，找到局部最小值区域（距离场 < 阈值），
    再通过连通分量分析统计独立的窄区域数量。

    返回: (exists: bool, narrow_regions: int)
    """
```

**实现细节**：
- 内部计算 `distance_transform_edt`（无需外部传入 distance_field）
- 窄通道判定：`distance_field < robot_radius_px * 1.5`
- `scipy.ndimage.label` 连通分量分析 → 独立窄区域计数
- 存在性判定：窄像素占自由空间 > 5%

**验证**：
```
测试网格（100×100，4px 窄缺口，robot_radius=5px）：
  _narrow_passage_detection → exists=True, n_regions=2 ✅
```

#### 4.1.2 `_generate_comparison_boxplots`（新增，59 行）

```python
def _generate_comparison_boxplots(
    results_df: "pd.DataFrame",
    output_path: str,
) -> None:
    """
    生成 EDT vs Costmap 的并排箱线图。

    指标：clearance_min、clearance_mean
    分组：按 room_type
    输出：PNG 保存到 output_path
    """
```

**实现细节**：
- 双面板 matplotlib 箱线图（clearance_min 上 / clearance_mean 下）
- 按 `room_type` 分组，每组 EDT（绿色）vs Costmap（橙色）并排
- 图例 + 标题 + 横轴标签完整
- 输出为 150 dpi PNG

---

### 4.2 R1.2：Git 配置与推送 ✅

**改动摘要**：
- Phase 0 已配置 `safe.directory`
- Remote `git@github.com:Xuzc317/SMDSL_2D.git` 已存在（v1 已配置）
- 提交 `3cbf8a1` 并推送

**验证**：
```
git log --oneline -5  →  3cbf8a1 为最新提交
git remote -v         →  origin = git@github.com:Xuzc317/SMDSL_2D.git
git push origin main  →  b564ecc..3cbf8a1
```

---

## 五、Phase R2 — 修复黄牌

### 5.1 R2.1：清理操作说明残留 ✅

**文件**：`app.py`（-3 行）

**问题**：审计发现 Tab 1、Tab 2、Tab 3 各有一行空标题 `gr.Markdown("## 这一步在做什么？")`，与下方实际内容不匹配（Tab 1 已精简为操作流程，Tab 2/3 仍有详细说明但标题冗余）。

**改动**：

| 位置（修改后） | 操作 |
|---------------|------|
| Tab 1 L1818 | 删除 `gr.Markdown("## 这一步在做什么？")` |
| Tab 2 L2141 | 删除 `gr.Markdown("## 这一步在做什么？")` |
| Tab 3 L2347 | 删除 `gr.Markdown("## 这一步在做什么？")` |

保留了下方的操作说明内容（操作流程 / 输入输出描述）。

**验证**：
```
grep "这一步在做什么" app.py → 0 匹配 ✅
py_compile app.py → OK ✅
63/63 测试通过 ✅
```

---

### 5.2 R2.2：DWG 告警 UI 横幅 ✅ 确认已存在

**文件**：`app.py` L516-525

**问题**：审计报告指出 `_dataset_note` 已存储但 UI 上可能未显示。

**核实结论**：代码已实现完整 DWG 告警链路，无需改动。

**完整链路**：

| 环节 | 文件 | 位置 | 内容 |
|------|------|------|------|
| ① 实体统计 | `dispatcher.py` | `_extract_dwg_geometry()` | `entity_stats` 计数器 |
| ② 丢弃率格式化 | `dispatcher.py` | `_format_entity_drop_note()` | 丢弃率 >10% 生成告警字符串 |
| ③ 注入 ParseResult | `dispatcher.py` | `_parse_dwg()` L732 | `result["note"]` 末尾追加 |
| ④ 存储到状态 | `app.py` | `demo1_run` L360 | `parsed["note"]` → `_dataset_note` |
| ⑤ UI 黄色横幅 | `app.py` | L516-525 | 检测"实体丢弃率"→ `<div style='background:#fff3cd...'>` |

**代码证据**（`app.py` L516-525）：
```python
note = parsed.get("note", "")
if "实体丢弃率" in str(note):
    status_prefix = (
        "<div style='background:#fff3cd;border:1px solid #ffc107;"
        "padding:8px 12px;margin-bottom:8px;border-radius:4px;"
        "font-size:14px;color:#664d03'>"
        "⚠️ 实体丢弃率偏高："
        + str(note).replace("\n", "<br>")
        + "</div>\n\n"
    )
```

**验证**：对应单元测试 `test_entity_stats.py` 10/10 通过。

---

## 六、验证清单

### 6.1 全量测试

```
63 passed in 7.01s ✅
```

### 6.2 源代码编译

```
py_compile bench_edt_vs_costmap.py → OK ✅
py_compile app.py                   → OK ✅
```

### 6.3 函数存在性

```
grep "这一步在做什么" app.py                                          → 0 matches ✅
grep "def _narrow_passage_detection" bench_edt_vs_costmap.py         → found ✅
grep "def _generate_comparison_boxplots" bench_edt_vs_costmap.py     → found ✅
```

### 6.4 Git 推送

```
3cbf8a1  main -> main ✅
```

---

## 七、文件变更清单

| 操作 | 文件 | 行数变化 |
|------|------|----------|
| 修改 | `benchmark/bench_edt_vs_costmap.py` | +103 行（2 个新函数） |
| 修改 | `app.py` | -3 行（删除 3 个冗余标题） |

---

## 八、红黄牌状态

| 牌 | 项目 | 状态 |
|----|------|------|
| 🔴 R1.1 | 基准函数名与报告一致 | ✅ 已修复 |
| 🔴 R1.2 | Git 配置与推送 | ✅ 已修复 |
| 🟡 R2.1 | 清理冗余标题 | ✅ 已修复 |
| 🟡 R2.2 | DWG 告警 UI 横幅 | ✅ 确认已存在 |
