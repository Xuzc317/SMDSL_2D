# SMDSL 方向一 v1 执行指令

## 背景

这是 SMDSL 方向一（2D 极致打磨 + 纠偏）的完整修改方案 v1。之前已经完成了一部分工作，现在需要你根据下面的清单继续执行。

**项目根目录**：`D:\Code\SMDSL_demo`
**GitHub 仓库**：`https://github.com/Xuzc317/SMDSL_2D`（空仓库，git remote 未配置）
**数据目录**：`D:\Code\SMDSL_demo\SMDSL\data\cad_samples`
**核心文件**：`smdsl_demo/app.py`（2552 行，111 KB）

---

## 执行要求

1. **每完成一个改动**：用中文汇报做了什么、改了哪个文件哪一行、为什么这么改、下一步是什么
2. **涉及文件删除**：请确认后再执行
3. **Git**：每次改动完成后 `git add` + `git commit`，最后一次性 push 到 `Xuzc317/SMDSL_2D`

---

## Phase 0：审查已有文件（先做，0.5h）

审查以下 4 个已有文件的内容，告诉我它们是否功能完整：

| 文件 | 审查目标 |
|------|---------|
| `smdsl_demo/trajectory_smoother.py` | 是否已实现三次样条 + 梯形 Profile？核心函数名是什么？ |
| `smdsl_demo/ui_theme.py` | 是否可独立导入？CSS 内容是否完整？ |
| `smdsl_demo/ui_common.py` | 提供了哪些共享函数？ |
| `smdsl_demo/motion_profile.py` | 与本会话中创建，功能是否与 trajectory_smoother.py 重叠？ |

**决定**：`trajectory_smoother.py` 和 `motion_profile.py` 谁去谁留。

---

## Phase 1：P0 修复 + UI 修正（4h）

### 1.1 RoboIR diff 硬校验 —— `tests/test_closed_loop_recovery.py`

在 LLM 修正循环中，每次返回 `corrected_roboir` 后插入：
- 对比 `corrected_roboir["intent"]` 和 `corrected_roboir["target_frame"]` 是否与原始值一致
- 不一致则拒绝本次修正，要求 LLM 重试（不消耗最大重试次数）
- 仅在指向修正 prompt 中加约束

### 1.2 DWG 实体丢弃率告警 —— `cad_parser/dispatcher.py`

- 在 DWG 实体遍历循环中增加 `entity_stats` 计数器
- 在 `dispatch_cad` 返回的 `ParseResult["note"]` 中添加丢弃率字符串
- 丢弃率 > 10% 时才生成告警

### 1.3 路径修正 —— `app.py`

#### A. `_DATA_ROOT` 路径修正（L72）
```python
# 当前（错误）：
_DATA_ROOT = str(_REPO_ROOT / "data" / "cad_samples")
# 修正为：
_DATA_ROOT = str(_REPO_ROOT / "SMDSL" / "data" / "cad_samples")
```

#### B. 预设样本路径修正（L81~L87）
将 `"data/cad_samples/..."` 改为 `"SMDSL/data/cad_samples/..."`

#### C. 示例文件路径修正（L1886~L1898）
将硬编码的 `D:\code_backup_dwg\...` 改为相对路径或用 `_DATA_ROOT` 拼接

#### D. 精简 Tab 1 顶部说明（L1837~L1880 区域）
将 40 行的"这一步在做什么 / 输入 / 处理 / 输出 / Note"替换为简洁操作流程：

```markdown
### 操作流程
1. 选择/上传 CAD 文件 → 自动解析
2. 查看 4 联图结果（栅格/距离场/A* 热力/拓扑）
3. 选起终点 → 点击"规划路径" → 查看路径
4. 路径轨迹自动同步至 Demo 3
```

#### E. 删除 3D 拓扑白模预览区块（L1995~L1997）
删除整个 Accordion（"🌐 3D 拓扑白模预览"）+ Button + Plot

#### F. 删除 3D 空间场景图区块（L1999~L2012）
删除整个 Accordion（"🌆 3D 空间场景图"）+ Markdown + Button + Plot

#### G. 删除对应的 handler 函数
- 删除 `demo1_3d_preview`（L1145~L1163）
- 删除 `demo1_scene_graph_3d`（L1166~L1227）

#### H. 删除对应的回调绑定（L2157~L2172）
- 删除 `d1_3d_btn.click` → `demo1_3d_preview`
- 删除 `d1_scene3d_btn.click` → `demo1_scene_graph_3d`

#### I. Demo 3 的 3D Trajectory Sandbox 改为 2D（L2465 + L1698~L1700）
- Plot label 从 "📊 3D Trajectory Sandbox" 改为 "2D 轨迹叠加图"
- 用 matplotlib 2D 路径图替换 Plotly 3D 场景
- 保留 ρ 曲线（本身就是 2D）
- import 中移除 `generate_3d_dashboard`

---

## Phase 2：P1 优化（5h）

### 2.1 轨迹合成接入 —— `app.py`

将 `demo1_plan_path` 中的 `path_pixels_to_trajectory` 调用替换为 `trajectory_smoother.py` 中对应函数的调用。保持输入输出接口一致。

### 2.2 距离场梯度微调 —— `cad_parser/astar_topology.py`

新增两个函数：
```python
def compute_field_gradient(distance_field) -> tuple[ndarray, ndarray]
def refine_path_via_gradient(path_rc, distance_field, gradient_field, robot_radius_px, iterations=5)
```

在 `app.py` 的 `demo1_plan_path` 中插入：`A* 路径 → 梯度微调 → 轨迹合成`

---

## Phase 3：重构（1.5h）

### 3.1 app.py 模块拆分

- 从 `app.py` 中移出 `_CLAUDE_DOCS_CSS` 到 `ui_theme.py`
- 从 `app.py` 中移出 `_build_theme()` 到 `ui_theme.py`
- 从 `app.py` 中移出 `_flow_nav_md()` / `_format_seed_label()` / `_ts()` 到 `ui_common.py`
- `app.py` 将上述函数改为 import

---

## Phase 4：测试（3h）

### 4.1 单元测试

| 文件 | 测试内容 |
|------|---------|
| `tests/test_z_axis.py` | Z 轴硬断言：含 z 变化的轨迹 → 返回 -inf；纯 2D → 正常 |
| `tests/test_roboir_diff.py` | RoboIR diff：修改 intent 被拒绝 |
| `tests/test_entity_stats.py` | DWG 实体统计计数准确 |
| `tests/test_trajectory_smoother.py` | 轨迹合成：总长度变化 < 5% |
| `tests/test_gradient_refine.py` | 梯度微调后 clearance 不下降 |

### 4.2 集成测试

```bash
python -m pytest smdsl_demo/test_demo3_pipeline.py -v
```

---

## Phase 5：基准 + 收尾（3h）

### 5.1 EDT vs Costmap 对比

新建 `benchmark/bench_edt_vs_costmap.py`，在 200 个 FloorplanQA 布局上对比：
- EDT 方案：`safety_aware_astar_flood`
- Costmap 方案：`binary_dilation` + 二值 A*
- 指标：`clearance_min`、`clearance_mean`、窄通道通过率、`planning_time`

### 5.2 Git + 文档

- `git init`（如果尚未） + `git remote add origin https://github.com/Xuzc317/SMDSL_2D.git`
- 每次改动后 `git add` + `git commit`，最后 `git push`
- 更新 `README.md`
- 创建 `CHANGELOG.md`

---

## 附录：关键接口参考

### trajectory_smoother.py（预置）
如果它已经实现了 `smooth_path_to_trajectory(waypoints_xy, ...)`，则直接使用。

### A* 路径格式
```python
path_rc = [(row_0, col_0), (row_1, col_1), ...]
```

### 轨迹点格式
```python
[
    {"t": 0.0, "x": 1.23, "y": 4.56, "z": 0.0,
     "roll": 0.0, "pitch": 0.0, "yaw": 0.0},
    ...
]
```

### TopologyBundle 格式
```python
{
    "distance_field": ndarray(H, W, float32),
    "grid_transform": {"origin": (ox,oy), "resolution": m_per_px, "shape": (H,W)},
    "robot_radius": float,
}
```
