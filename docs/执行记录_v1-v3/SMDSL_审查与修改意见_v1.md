# SMDSL 方向一 独立审查报告与修改意见 v1

> 审查者：Codex · 方式：逐文件代码审计（不看报告看代码）
> 日期：2026-06-15

---

## 一、审查结果总表

| 项 | 状态 | 优先级 | 说明 |
|---|------|--------|------|
| Z 轴硬断言 | ✅ 通过 | — | 代码确实存在，逻辑正确 |
| 3D 区块清理 | ✅ 通过 | — | app.py 中 3 个 3D 区块和 handler 全部删除 |
| DATA_ROOT 路径 | ✅ 通过 | — | _REPO_ROOT / "SMDSL" / "data" / "cad_samples" 正确 |
| DWG entity_stats | ⚠️ 待确认 | — | 代码存在，但需验证是否注入 ParseResult.note 并在 UI 消费 |
| motion_profile.py 删除 | ⚠️ 待确认 | — | trajectory_smoother.py 已覆盖，需确认边界情况完整 |

| RoboIR diff 硬校验 | ❌ 不存在 | **P0** | 安全线，LLM 修正可能篡改 intent/target_frame |
| EDT vs Costmap 基准 | ❌ 不存在 | **P0** | 方向一核心交付物，缺失 = 无量化证据 |
| 新模块集成到 app.py | ❌ 不存在 | P1 | architecture + demo_recording 两个 Tab 未接入 |
| 操作说明精简 | ❌ 不存在 | P1 | Tab 1 顶部 40 行说明未动 |
| 轨迹合成接入 app.py | ❌ 不存在 | P1 | 调用仍指向旧 path_pixels_to_trajectory |
| 梯度路径微调 | ❌ 不存在 | P1 | astar_topology.py 无新增函数 |
| 单元测试 | ❌ 不存在 | P2 | 5 个测试文件全空 |
| Git 配置 | ❌ 未解决 | P1 | safe.directory 未配置，commit 被阻塞 |
| 文档更新 | ❌ 不存在 | P2 | README + CHANGELOG 未动 |

---

## 二、P0 修复项（必须做）

### 2.1 RoboIR diff 硬校验

**文件**：`tests/test_closed_loop_recovery.py`
**位置**：LLM 修正循环内，每次 `corrected_roboir` 返回后
**改动**：

```python
# 对比 intent + target_frame，不一致则拒绝
corrected_intent = corrected_roboir.get("intent")
corrected_target = corrected_roboir.get("target_frame")
original_intent = roboir_initial.get("intent")
original_target = roboir_initial.get("target_frame")

if corrected_intent != original_intent or corrected_target != original_target:
    correction_prompt += (
        f"\n[HARD CONSTRAINT] intent 和 target_frame 不可变。"
        f"保持 intent='{original_intent}'、"
        f"target_frame='{original_target}' 不变，"
        f"仅调整 stl_constraints 参数。"
    )
    continue  # 重新请求，不消耗正常重试次数
```

**行数**：~15 行
**验证方法**：手动注入一个修改 intent 的 LLM 响应，确认 diff 检查触发拒绝

### 2.2 EDT vs Costmap 基准（方向一核心交付物）

**新建文件**：`benchmark/bench_edt_vs_costmap.py`
**设计**：

```
数据集：FloorplanQA 200 layouts × 5 random pairs = ~1000 条路径
对比方法：
  Ours:    EDT + safety_aware_astar_flood（当前实现）
  Baseline: binary_dilation + 二值 A*（ROS costmap_2d 风格）
指标：
  - clearance_min (mm)
  - clearance_mean (mm)
  - clearance < robot_radius 占比 (%)
  - 窄通道通过率 (通道宽度 ≤ 1.5×机器人直径)
  - 路径长度 (m)
  - 规划时间 (ms)
输出：按 room_type 分组的汇总表格 + 箱线图
```

**行数**：~200 行
**验证方法**：运行脚本后输出表格可读、数字合理

---

## 三、P1 优化项（分批完成）

### 3.1 新模块集成到 app.py

**文件**：`app.py` — `build_ui()` 函数尾部，`return demo` 之前
**改动**：追加两个 `gr.Tab()` block：

```python
# Architecture Tab
with gr.Tab("Architecture (N-layer Zone)"):
    from smdsl_demo.architecture_viz import get_architecture_html
    gr.HTML(get_architecture_html())

# Demo Recording Tab
with gr.Tab("Demo Recording"):
    # ... UI 控件 + callback
```

**前置条件**：确认 `architecture_viz.py` 和 `demo_recorder.py` 在 `sys.path` 中可被 import

### 3.2 操作说明精简

**文件**：`app.py` Tab 1 顶部（约 L1837~L1880）
**改动**：替换为简洁版本：

```
### 操作流程
1. 选择/上传 CAD 文件 → 自动解析
2. 查看 4 联图（栅格/距离场/A* 热力/拓扑标签）
3. 选起终点 → 点击"规划路径" → 查看路径
4. 轨迹自动同步至 Demo 3
```

### 3.3 轨迹合成接入

**文件**：`app.py` — `demo1_plan_path` 函数
**改动**：

```python
# 替换前
trajectory = path_pixels_to_trajectory(path_rc, transform, total_time_s, sample_dt)

# 替换后
from smdsl_demo.trajectory_smoother import smooth_path_to_trajectory
trajectory = smooth_path_to_trajectory(
    path_rc=path_rc, resolution=transform["resolution"],
    origin_xy=transform["origin"], total_time_s=total_time_s,
)
```

### 3.4 梯度路径微调

**新建函数**：`cad_parser/astar_topology.py`

```python
def compute_field_gradient(distance_field)
    → (gx, gy)
def refine_path_via_gradient(path_rc, df, (gx, gy), robot_radius_px, iterations=5)
    → path_rc_refined
```

**集成位置**：`app.py` — `demo1_plan_path`，在 A* 路径后、轨迹合成前插入

### 3.5 DWG entity_stats 完整验证

**检查点**：
- `dispatch_cad` 返回值中 `note` 字段已注入 ✅/❌
- `app.py` 的 `demo1_run` 已消费 `parsed["note"]` 并显示告警 ✅/❌
- 丢弃率 > 10% 时触发 ✅/❌

### 3.6 Git 配置修复

```bash
git config --global --add safe.directory D:/Code/SMDSL_demo
git add .
git commit -m "方向一 v1：修复汇总"
git remote add origin https://github.com/Xuzc317/SMDSL_2D.git
git push -u origin main
```

---

## 四、P2 收尾项（有精力再做）

- 单元测试：5 个新建测试文件
- 文档：README.md 更新 + CHANGELOG.md 新建
- `trajectory_smoother.py` 边界情况验证

---

## 五、下次审查的核查清单

每次外部代理汇报后，我（Codex）必须独立核查：

```
□ RoboIR diff 代码是否存在（grep tests/test_closed_loop_recovery.py）
□ benchmark/bench_edt_vs_costmap.py 是否存在且可运行
□ app.py 中 architecture_viz / demo_recorder 的 import 是否存在
□ Tab 1 顶部说明是否已精简
□ demo1_plan_path 是否调用 smooth_path_to_trajectory
□ astar_topology.py 是否有 gradient 相关函数
□ test 文件是否存在（grep tests/*.py）
□ git log 是否有 commit 记录
□ README.md 是否已更新
```
