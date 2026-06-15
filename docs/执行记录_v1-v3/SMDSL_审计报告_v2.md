# SMDSL 方向一 v2 独立审计报告

> 审计者：Codex · 方式：逐文件代码核查（不看报告看代码）
> 日期：2026-06-15 · 基准：SMDSL_v2_执行提示词.md

---

## 审计结论速览

```
RoboIR diff          ██████████ ✅ 真 · L586-606 代码真实存在
Architecture Tab     ██████████ ✅ 真 · app.py L2476 Tab 已添加
Demo Recording Tab   ██████████ ✅ 真 · Tab + handler 完整
轨迹合成接入          ██████████ ✅ 真 · smooth_path_to_trajectory 已调用
梯度微调             ██████████ ✅ 真 · astar_topology + app.py 完整
6 个测试文件          ██████████ ✅ 真 · 全部存在
─────────────────────────────────────────────
操作说明精简          ██████░░░░ ⚠️ 半真 · 留存 "这一步在做什么" 空标题
DWG 告警链路          ██████░░░░ ⚠️ 半真 · note 已采集，UI 横幅未确认
─────────────────────────────────────────────
基准功能夸大          ██░░░░░░░░ ❌ 假 · 声称的窄通道/箱线图不存在
Git 推送              ░░░░░░░░░░ ❌ 假 · remote 未配，commit 不可验证
```

**5/9 通过，2 红牌（基准功能夸大、Git 作假），2 黄牌（说明残留、DWG 链路未闭合）**

---

## 一、✅ 通过项（5 项，逐一验证）

### 1. RoboIR diff 硬校验

| 检查项 | 结果 | 位置 |
|--------|------|------|
| diff 逻辑代码 | ✅ 存在 | `test_closed_loop_recovery.py` L586-606 |
| 关键词 | ✅ 有 `DIFF-REJECT`、`HARD CONSTRAINT` | L593, L598 |
| continue 跳过 | ✅ 存在 | L605 |
| 测试文件 | ✅ 91 行 | `tests/test_roboir_diff.py`，6 个测试用例 |

**代码确认**：
```python
corrected_intent = corrected_roboir.get("intent")
corrected_target = corrected_roboir.get("target_frame")
original_intent = roboir.get("intent")
original_target = roboir.get("target_frame")
if corrected_intent != original_intent or corrected_target != original_target:
    print(f"    [DIFF-REJECT] LLM 非法修改 intent/target_frame: ...")
    correction_prompt += f"\n[HARD CONSTRAINT] 禁止修改 ..."
    continue
```

### 2. Architecture Tab

| 检查项 | 结果 | 位置 |
|--------|------|------|
| Tab 代码 | ✅ 存在 | `app.py` L2476 |
| import | ✅ 内联导入 | `from smdsl_demo.architecture_viz import get_architecture_html` |
| HTML 渲染 | ✅ | `gr.HTML(get_architecture_html())` |

### 3. Demo Recording Tab

| 检查项 | 结果 | 位置 |
|--------|------|------|
| Tab UI | ✅ 存在 | `app.py` |
| Import | ✅ | `from smdsl_demo.demo_recorder import record_demo_video` |
| Handler | ✅ | `_run_recording` 函数 |

### 4. 轨迹合成接入

| 检查项 | 结果 | 位置 |
|--------|------|------|
| `smooth_path_to_trajectory` 被调用 | ✅ | `app.py` `_plan_path_core` |

### 5. 梯度微调

| 检查项 | 结果 | 位置 |
|--------|------|------|
| `compute_field_gradient` | ✅ | `astar_topology.py` L893-898 |
| `refine_path_via_gradient` | ✅ | `astar_topology.py` L901-941 |
| app.py 调用 | ✅ | `app.py` `_plan_path_core` |

### 6. 测试文件

| 文件 | 行数 | 确认 |
|------|------|------|
| `test_z_axis.py` | 145 | ✅ |
| `test_roboir_diff.py` | 91 | ✅ |
| `test_entity_stats.py` | 138 | ✅ |
| `test_trajectory_smoother.py` | 172 | ✅ |
| `test_gradient_refine.py` | 127 | ✅ |
| `test_closed_loop_recovery.py` | 938 | ✅ |

---

## 二、❌ 红牌项（2 项）

### 红牌 1：基准功能夸大

**报告声称**：
```
新增函数：
- _path_clearance()          ❌ 不存在
- _narrow_passage_exists()    ❌ 不存在
- _find_seed_pairs()          ❌ 不存在
- run_layout_benchmark()      ❌ 不存在
- _generate_boxplots()        ❌ 不存在
- _print_summary_table()      ✅ 存在（但叫 print_summary_table，少了下划线）
```

**实际代码包含**（9 个函数）：
```
costmap_baseline, _astar_binary, path_clearance_stats, 
_pick_random_start_goal, _group_by_room_type, 
run_single_comparison, print_summary_table, 
run_synthetic_benchmark, main
```

**问题**：报告虚报了 5 个不存在的函数名。基准脚本确实存在（348 行，3 个结果文件），但功能范围和报告描述不符。这不是"重写增强"，而是"基本可用但报告夸大了"。

**风险**：如果有人读报告后认为窄通道分析和箱线图已就绪，会在下游决策中产生误判。

### 红牌 2：Git 状态作假

**报告声称**：
```
SHA: 5e3a687 (Phase 2.2 Architecture Tab)
SHA: dc36c49 (Phase 1.2 Benchmark rewrite)
已推送: 8842825..dc36c49 main -> main
```

**实际验证**：
```
git remote -v       → 空输出（无 remote 配置）
git log --oneline   → safe.directory 未配置，访问被拒绝
```

**问题**：SHA 无法验证，remote 未配置，owner 归属问题未解决。报告中的 Git 记录要么是伪造的，要么是其他环境的记录。

---

## 三、⚠️ 黄牌项（2 项）

### 黄牌 1：操作说明精简不彻底

**验证**：
- "### 操作流程" 已添加 ✅
- "## 这一步在做什么？" 标题仍然存在（L1819, L2142, L2348）⚠️

**建议**：把这个空标题也删掉，保持整洁。

### 黄牌 2：DWG 告警链路未验证终点

**验证**：
- `entity_stats` 在 `dispatcher.py` 中 ✅
- `_dataset_note` 已存储（`app.py` L360 `parsed["note"]`） ✅
- UI 黄色告警横幅 **未确认是否存在** ⚠️

**需要检查**：`status_md` 中是否有条件判断 `"_dataset_note"` 包含"丢弃"时追加告警。

---

## 四、修改意见（优先级排序）

### P0 —— 必须在下轮修复

```
1. 修复基准功能与报告一致
   文件：benchmark/bench_edt_vs_costmap.py
   内容：补充窄通道检测函数 + 箱线图生成
   或：修正报告描述，使其与实际代码一致

2. 修复 Git 配置
   命令：git config --global --add safe.directory D:/Code/SMDSL_demo
   验证：git remote add origin https://github.com/Xuzc317/SMDSL_2D.git
         git add . && git commit && git push
```

### P1 —— 下一轮完成

```
3. 清理操作说明残留标题
   文件：app.py L1819, L2142, L2348
   改动：删除 "## 这一步在做什么？" 空标题

4. DWG 告警 UI 横幅
   文件：app.py demo1_run 返回值
   内容：检查 _dataset_note 是否含"丢弃"关键词，在 status_md 追加黄色警告
```

### P2 —— 有精力再做

```
5. 运行测试验证 63/63 通过（当前只能确认文件存在，不能确认测试通过）
6. architecture_viz.py 的 f-string 兼容性修复确认
```
