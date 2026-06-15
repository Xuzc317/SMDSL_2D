# SMDSL 方向一 v2 执行提示词

之前外部代理已经完成了部分工作：删除了 app.py 中的 3D 区块、修正了 DATA_ROOT 路径、实现了 DWG entity_stats、删除了 redundant 的 `motion_profile.py`。以下是本次需要完成的剩余工作。

---

## 执行前准备工作

### Git owner 修复（先做）

```bash
git config --global --add safe.directory D:/Code/SMDSL_demo
```

### 确认当前文件状态

```bash
cd D:\Code\SMDSL_demo
Get-ChildItem .\SMDSL\smdsl_demo\*.py | Select-Object Name, Length
Get-ChildItem .\benchmark -ErrorAction SilentlyContinue
```

---

## 执行顺序

## Phase 1：P0 修复（不可跳过）

### 1.1 RoboIR diff 硬校验（~15 行，1h）

**文件**：`tests/test_closed_loop_recovery.py`
**位置**：LLM 修正循环内，每次 `corrected_roboir` 返回后、校验前

```python
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
    continue
```

**验证**：现有测试不退化。手动注入修改 intent 的 LLM 响应 → diff 检查触发拒绝。

### 1.2 EDT vs Costmap 对比基准（~200 行，3h）

**新建**：`benchmark/bench_edt_vs_costmap.py`

```python
"""
数据集：FloorplanQA 200 layouts（bedroom×50, kitchen×50, living_room×50, hssd×50）
采样：每 layout 5 对随机起终点（排除不可达对）
对比：
  Ours:    EDT + safety_aware_astar_flood
  Baseline: binary_dilation + A*（二值可通行/不可通行）
指标：
  - clearance_min (mm)            ← 主要指标
  - clearance_mean (mm)
  - clearance < robot_radius 占比 (%)
  - 窄通道（宽度 ≤ 1.5×机器人直径）通过率 (%)
  - path_length (m)
  - planning_time (ms)
输出：按 room_type 分组的汇总表格 + 箱线图保存到 benchmark/results/
"""

def costmap_baseline(grid, robot_radius_px):
    from scipy.ndimage import binary_dilation
    from skimage.morphology import disk
    struct = disk(math.ceil(robot_radius_px))
    inflated = binary_dilation(grid == 0, structure=struct)
    cost_grid = np.where(inflated, 1.0, float("inf"))
    # A*（无安全代价项）
    return astar_on_costmap(cost_grid, start, goal)
```

---

## Phase 2：P1 优化（按顺序做）

### 2.1 操作说明精简（10 行，0.5h）

**文件**：`app.py` Tab 1 顶部（grep "这一步在做什么" 找到位置，约 L1837~L1880）
**替换为**：

```python
gr.Markdown("""
### 操作流程
1. 选择/上传 CAD 文件 → 自动解析
2. 查看 4 联图（栅格/距离场/A* 热力/拓扑标签）
3. 选起终点 → 点击"规划路径" → 查看路径
4. 轨迹自动同步至 Demo 3
""")
```

### 2.2 新模块集成到 app.py（30 行，1h）

**文件**：`app.py` — `build_ui()` 函数尾部，`return demo` 之前

追加两个 Tab：

```python
        # ── Architecture Tab ──
        with gr.Tab("Architecture (N-layer Zone)"):
            from smdsl_demo.architecture_viz import get_architecture_html
            gr.HTML(get_architecture_html())

        # ── Demo Recording Tab ──
        with gr.Tab("Demo Recording"):
            gr.Markdown("## SMDSL Demo Recording")
            gr.Markdown("Record a video walkthrough. Requires Playwright.")
            with gr.Row():
                drec_url = gr.Textbox(label="App URL", value="http://127.0.0.1:7860", scale=3)
                drec_scenario = gr.Dropdown(
                    choices=["full", "cad", "compile", "verify", "all"],
                    value="full", label="Scenario", scale=2,
                )
            drec_btn = gr.Button("Record Demo", variant="primary")
            drec_status = gr.Markdown("*Ready...*")
            drec_video = gr.Video(label="Recorded Demo")
            drec_btn.click(fn=_run_recording, inputs=[drec_url, drec_scenario], outputs=[drec_video, drec_status])
```

并在 `build_ui()` 之前添加 handler：

```python
def _run_recording(app_url, scenario):
    from smdsl_demo.demo_recorder import record_demo_video
    results = record_demo_video(app_url=app_url, scenario=scenario)
    paths = list(results.values())
    if paths and paths[0]:
        return paths[0], f"Saved: {paths[0]}"
    return None, "Recording failed"
```

### 2.3 轨迹合成接入（3 行，0.5h）

**文件**：`app.py` — `demo1_plan_path`
**改动**：

```python
# 在文件顶部加 import
from smdsl_demo.trajectory_smoother import smooth_path_to_trajectory

# 在函数内部替换调用
trajectory = smooth_path_to_trajectory(
    path_rc=path_rc,
    resolution=transform["resolution"],
    origin_xy=transform["origin"],
    total_time_s=total_time_s,
)
```

**验证**：启动 app → Tab 1 规划路径 → 轨迹点具有加速/匀速/减速三段特征

### 2.4 梯度路径微调（30 行，2h）

**文件 A**：`cad_parser/astar_topology.py` 新增

```python
def compute_field_gradient(distance_field: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    """计算 EDT 梯度场 ∇d = (∂d/∂x, ∂d/∂y)。np.gradient 中心差分。"""
    gy, gx = np.gradient(distance_field.astype(np.float64))
    return gx, gy

def refine_path_via_gradient(
    path_rc: list[tuple[int, int]],
    distance_field: np.ndarray,
    gradient_field: tuple[np.ndarray, np.ndarray],
    robot_radius_px: float,
    iterations: int = 5,
    step_size: float = 0.5,
) -> list[tuple[int, int]]:
    """沿梯度方向（远离障碍物）微调路径内部点，保持起终点不变。"""
    gx, gy = gradient_field
    result = list(path_rc)
    for _ in range(iterations):
        for i in range(1, len(result) - 1):
            r, c = result[i]
            dr = gy[r, c] * step_size
            dc = gx[r, c] * step_size
            nr, nc = int(round(r + dr)), int(round(c + dc))
            if (0 <= nr < distance_field.shape[0] and
                0 <= nc < distance_field.shape[1] and
                distance_field[nr, nc] >= robot_radius_px):
                result[i] = (nr, nc)
    return result
```

**文件 B**：`app.py` — `demo1_plan_path`，在 A* 结果后插入

```python
from cad_parser.astar_topology import compute_field_gradient, refine_path_via_gradient

gx, gy = compute_field_gradient(distance_field)
path_rc = refine_path_via_gradient(path_rc, distance_field, (gx, gy), robot_radius_px, iterations=5)
```

### 2.5 DWG entity_stats 完整验证

**检查**：
- `dispatch_cad` 返回的 `ParseResult` 中 `note` 字段已注入 ✅/❌
- `app.py` 的 `demo1_run` 中已消费 `parsed.get("note")` ✅/❌
- 丢弃率 > 10% 时在 UI 显示黄色告警 ✅/❌

### 2.6 Git 提交

```bash
git add .
git commit -m "[方向一 v2] P0: RoboIR diff + EDT benchmark; P1: 精简/集成/轨迹/梯度"
git push -u origin main
```

**如果 push 失败**（remote 未配置或无权限），记录下来即可，代码改动不阻塞。

---

## 汇报要求

每次改动完成，按以下格式汇报：

```
【完成】Phase X.Y: 改动名称
文件：路径
改动摘要：改了哪、怎么改的
行数/工作量：+
验证：如何验证通过
```

Codex 收到汇报后会独立审计代码，核实后再更新状态。

---

## 附录：预置文件参考

### trajectory_smoother.py（已有，147 行）
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
    """像素路径 → 三次样条 → 梯形 Profile → 2D 轨迹点 [{t,x,y,z=0,...}]"""
    # 内部调用 _cubic_spline_2d + trapezoidal_velocity_profile
```

### ui_theme.py（已有，562 行）
```python
def build_theme() -> gr.themes.Soft:  # macOS Apple 风格主题
def build_theme_compact() -> gr.themes.Soft:  # 紧凑版
```

### ui_common.py（已有，87 行）
```python
def flow_nav_md(active: int) -> str:  # 顶部导航指示条
def format_seed_label(idx, kind, x, y, extra="") -> str:  # 种子点格式化
def current_timestamp() -> str:  # 时间戳
```
