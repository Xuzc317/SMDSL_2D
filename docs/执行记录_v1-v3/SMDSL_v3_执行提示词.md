# SMDSL 方向一 v3 执行提示词

> 基于 v2 审计结果修正

---

## 前情提要

v2 执行后经独立审计，5/9 项通过，2 红牌、2 黄牌。本提示词聚焦修复红黄牌。

### ✅ v2 已验证通过的代码（不要动）
- RoboIR diff（`test_closed_loop_recovery.py` L586-606）
- Architecture Tab（`app.py` L2476）
- Demo Recording Tab + handler
- 轨迹合成接入（`smooth_path_to_trajectory`）
- 梯度微调（`astar_topology.py` 两个函数 + app.py 调用）
- 6 个测试文件（文件存在，行数正确）

---

## Phase 0：环境准备

```bash
# 修复 Git owner 问题（必须先做，否则后续 git 命令全部失败）
git config --global --add safe.directory D:/Code/SMDSL_demo

# 验证
git status
```

---

## Phase R1：修复红牌（必须做）

### R1.1 基准功能与报告一致

**文件**：`benchmark/bench_edt_vs_costmap.py`（现有 348 行）

**【方案 A：补充代码】**（推荐，2h）
在现有脚本基础上，新增以下函数：

```python
def _narrow_passage_detection(grid, robot_radius_px):
    """
    检测布局中是否存在狭窄通道（通道宽度 ≤ 1.5×机器人直径）。
    方法：对自由空间做 distance_transform，找到局部最小值区域。
    返回: (exists: bool, narrow_regions: int)
    """

def _generate_comparison_boxplots(results_df, output_path):
    """
    生成 EDT vs Costmap 的并排箱线图。
    指标：clearance_min、clearance_mean
    分组：按 room_type
    输出：PNG 保存到 benchmark/results/
    """
```

然后更新 `print_summary_table` 使其在输出中包含窄通道通过率。

**【方案 B：修正报告描述】**（0.5h）
如果窄通道 + 箱线图本期不做，就编辑 `SMDSL_审计报告_v2.md` 中对应的描述，使其与现有代码一致。

**我建议方案 A**，因为基准是方向一的核心交付物，窄通道分析是这个基准最有说服力的指标之一。

### R1.2 Git 配置与推送

```bash
# 已经在 Phase 0 配置了 safe.directory

# 查看当前状态
git status

# 如果无 remote：
git remote add origin https://github.com/Xuzc317/SMDSL_2D.git

# 提交（如果有未提交的改动）
git add .
git commit -m "[方向一 v3] 修复: 基准增强 + DWG 告警 + 说明清理"

# 推送到远端
git push -u origin main
```

**验证**：
```bash
git log --oneline -5
git remote -v
```

---

## Phase R2：修复黄牌

### R2.1 清理操作说明残留（10 分钟）

**文件**：`app.py`

grep 找到以下三处，**仅删除空标题行**（"## 这一步在做什么？"），保留下方的操作说明内容：

```python
# L1819 — Tab 1 顶部
gr.Markdown("## 这一步在做什么？")       # ← 删除这一行
# 保留下面的 "### 操作流程" 内容

# L2142 — Tab 2
gr.Markdown("## 这一步在做什么？")       # ← 删除这一行

# L2348 — Tab 3
gr.Markdown("## 这一步在做什么？")       # ← 删除这一行
```

### R2.2 DWG 告警 UI 横幅（30 分钟）

**文件**：`app.py`，在 `demo1_run` 函数的返回值构造部分

当前状态：`_dataset_note` 已存储（L360），但 UI 上未显示。

在 `demo1_run` 返回 `status_md` 之前，添加：

```python
# 检查 DWG 实体丢弃率，在 status 顶部追加黄色告警
note = parsed.get("note", "")
if note and "drop" in note.lower():
    warning_banner = f"⚠️ {note}\n\n"
    status_md = warning_banner + status_md
```

**注意**：`parsed["note"]` 的内容格式类似 `"DWG entity drop rate: 45.2% (total: 16234, processed: 8900, top skipped: SPLINE=14226, ELLIPSE=3068)"`。UI 上应该能在 Tab 1 状态栏看到黄色警告。

---

## Phase V：验证已存在的代码

### V.1 确认测试可运行

```bash
cd D:\Code\SMDSL_demo
python -m pytest SMDSL\tests\test_z_axis.py -v --tb=short
python -m pytest SMDSL\tests\test_roboir_diff.py -v --tb=short
python -m pytest SMDSL\tests\test_entity_stats.py -v --tb=short
python -m pytest SMDSL\tests\test_trajectory_smoother.py -v --tb=short
python -m pytest SMDSL\tests\test_gradient_refine.py -v --tb=short
python -m pytest SMDSL\smdsl_demo\test_demo3_pipeline.py -v --tb=short
```

**预期**：全部通过或跳过（因缺少 `openai` 包而跳过的可以接受，但不能 crash）。

### V.2 确认基准脚本可运行

```bash
cd D:\Code\SMDSL_demo
python benchmark\bench_edt_vs_costmap.py --n_layouts 5 --n_pairs 2
```

**预期**：输出汇总表格、无 ImportError。

### V.3 确认 app.py 启动不崩溃

```bash
cd D:\Code\SMDSL_demo
python -c "import sys; sys.path.insert(0, 'SMDSL'); from smdsl_demo.app import build_ui; print('build_ui 导入成功 ✅')"
```

**预期**：打印 "build_ui 导入成功 ✅"

---

## 汇报要求

每次改动完成后按以下格式汇报：

```
【完成】Phase: 名称
文件：路径
改动摘要：改了哪、怎么改的
验证：我验证通过的方式
```

> **⚠️ 审计说明**：你的报告会经过 Codex 独立代码审计。不要虚报未落地的功能和 Git 记录——审计会逐行检查代码是否存在。基准脚本的函数名和报告描述不一致会被标记为红牌。

---

## 附录

### v2 已验证通过的代码列表（不要动这些文件）

| 文件 | 已验证的功能 |
|------|-------------|
| `tests/test_closed_loop_recovery.py` | RoboIR diff 校验 L586-606 |
| `tests/test_roboir_diff.py` | 6 个 diff 测试用例 |
| `app.py` L2476 | Architecture Tab |
| `app.py` | Demo Recording Tab + _run_recording |
| `app.py` _plan_path_core | smooth_path_to_trajectory 调用 |
| `cad_parser/astar_topology.py` L893-941 | compute_field_gradient + refine_path_via_gradient |
| `tests/test_z_axis.py` (145行) | Z 轴断言测试 |
| `tests/test_entity_stats.py` (138行) | 实体统计测试 |
| `tests/test_trajectory_smoother.py` (172行) | 轨迹合成测试 |
| `tests/test_gradient_refine.py` (127行) | 梯度微调测试 |
| `dispatcher.py` | entity_stats 计数器 |
