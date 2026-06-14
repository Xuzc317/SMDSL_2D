# Demo 3 — STL Robustness on Distance Field

**目标**：让 STL 约束 `Distance > D_safe` 的求解从"对参考点的单点欧氏距离"
升级为"对 Demo 1 全局 EDT 距离场的逐点真实物理距离"，并在违规时输出
带 `FailureTaxonomy.COLLISION` 的结构化反馈。

> Demo 2 的 LLM 自我修正循环依赖 Demo 3 的反馈，所以 Demo 3 必须先于 Demo 2 完成。

---

## 1. 执行架构（Zone 3 ⇄ Zone 4）

```
Demo 1 (Zone 3 spatial)         Demo 3 (Zone 3 STL solver)
──────────────────────          ──────────────────────────
CAD JSON                        Trajectory  ──┐
  │                                          │
  ▼                                          ▼
rasterize → grid              check_stl_constraint_violation(
  │                              trajectory,
  ▼                              constraints,
EDT → distance_field  ──────►   topology_bundle = {        )
  │                                distance_field, grid_transform
  ▼                              }
to_topology_bundle()                          │
  │                                          ▼
  └────────────────►  per-point: ρ = d_real_m − D_safe
                                              │
                                              ▼
Demo 4 (Zone 4 feedback)        run_diagnostic_pipeline()
──────────────────────          ──────────────────────────
                                _classify_violation()
                                  │
                                  ▼
                                FailureTaxonomy.COLLISION
                                  │
                                  ▼
                                generate_structured_feedback({
                                  context.violation_nodes: [
                                    {t, x, y, row, col, d_real_m, ρ}, ...
                                  ],
                                  context.worst_node: {...}
                                })
                                  │
                                  ▼
                                LLM (Demo 2) reads → revises plan
```

## 2. STL 鲁棒性度量数学定义

对约束 `Distance > D_safe`，逐时间步 t 计算：

```
(row_f, col_f) = world_to_grid(x_t, y_t)
d_real_px      = bilinear_sample(distance_field, row_f, col_f)
d_real_m       = d_real_px × resolution
ρ_t            = d_real_m − D_safe
```

整条约束的鲁棒度：`ρ = min_t ρ_t`。

| 取值 | 语义 | FailureTaxonomy |
|---|---|---|
| ρ > 0 | 安全冗余 | none / info |
| ρ = 0 | 临界 | warning |
| ρ < 0 | **碰撞 / 过度贴近** | **COLLISION** |

边界外的点（row, col 超出栅格）→ 视为已撞 → `d_real_m = 0` → `ρ = -D_safe`。

## 3. API

### 3.1 距离场查询（Zone 3 内部）

```python
from smdsl_demo.spatial_api_stub import query_clearance_at

d = query_clearance_at(x_m=4.30, y_m=2.10, topology_bundle=bundle)
# d == d_real_m，单位米
```

### 3.2 STL 检测主入口

```python
from smdsl_demo.spatial_api_stub import check_stl_constraint_violation

reports = check_stl_constraint_violation(
    trajectory,
    [{"type": "stl_constraint", "expr": "Distance > 0.30",
      "unit": "m", "reference": "obstacle"}],
    topology_bundle=bundle,           # 优先：距离场
    reference_poses=None,             # 兜底：单参考点
)
```

返回单条报告关键字段：

| 字段 | 类型 | 含义 |
|---|---|---|
| `rule_expr` | str | 原始 STL 表达式 |
| `source` | str | `distance_field` / `reference_pose_fallback` / `missing_reference` |
| `robustness` | float | min ρ |
| `max_violation` | float | max(-ρ, 0) |
| `violation_duration` | float | ρ<0 累计时长（秒） |
| `violated` | bool | 是否触发硬约束破坏 |
| `violation_nodes` | List[dict] | 每个 ρ<0 点的 {t,x,y,row,col,d_real_m,rho,in_bounds} |
| `details` | List[dict] | 全部时间步的 ρ 剖面 |

### 3.3 结构化反馈（给 LLM 的 JSON）

```python
from smdsl_demo.metrics import run_diagnostic_pipeline

feedback_list = run_diagnostic_pipeline(
    robo_ir_str,
    trajectory,
    constraints,
    topology_bundle=bundle,
)
```

每条 feedback：

```jsonc
{
  "error": {
    "type": "collision",          // FailureTaxonomy.COLLISION
    "severity": "error",          // 由 ρ 量级决定
    "robustness_score": -0.30
  },
  "diagnosis": {
    "summary": "轨迹相对真实环境的鲁棒度违反（距离场驱动）",
    "suggestion": "降低任务安全余量、增加避障路径点，或在 STL 中放宽 Distance 阈值。",
    "hint": "查看 context.violation_nodes 获取每个违规栅格点的 (t, x, y, d_real_m, ρ)，针对最深违规点重规划。",
    "violation_magnitude": 0.30
  },
  "context": {
    "rule": "Distance > 0.300",
    "source": "distance_field",
    "max_violation": 0.30,
    "violation_duration": 1.30,
    "n_violation_nodes": 26,
    "worst_node": {
      "t": 1.90, "x": 0.52, "y": 2.94,
      "row": 69, "col": 20,
      "d_real_m": 0.0, "rho": -0.30,
      "in_bounds": true
    },
    "violation_nodes_preview": [ /* 前 5 个违规点 */ ]
  },
  "recommended_action": "adjust_params"
}
```

## 4. 端到端运行

```powershell
# 跑示范脚本，生成 trajectories.png + robustness_report.json
python -m smdsl_demo.demo3_robustness `
    --layout data/cad_samples/floorplanqa/layouts/kitchen/room_24.json `
    --d_safe 0.30 --robot_radius 0.25 --resolution 0.05 `
    --out_dir out/demo3
```

输出（4 条对照轨迹 — 同一房间，同一 D_safe = 0.30 m）：

| 轨迹 | min ρ | n_violation_nodes | FailureTaxonomy |
|---|---|---|---|
| A_SAFE        | +0.150 m | 0  | none / info |
| B_GRAZE_WALL  | −0.150 m | 53 | **collision** / error |
| C_THROUGH_OBJ | −0.300 m | 26 | **collision** / error |
| D_OUT_OF_BOUND | −0.300 m | 38 | **collision** / error |

## 5. 烟囱测试

```powershell
python -m smdsl_demo.test_demo3_pipeline
```

校验：
- 编译期通过（最小 RoboIR 满足 schema）
- A_SAFE 不产生 `collision` 反馈
- C_THROUGH_OBJ 必须产生 `FailureTaxonomy.COLLISION`

## 6. 设计决策与"四大执行区"对齐

| 决策 | 理由 |
|---|---|
| Distance 求解 *优先* 用 `topology_bundle`，回退到 `reference_poses` | "全局环境真值" > "单参考点"；保留兼容。 |
| 越界点视为 d=0（碰撞），而非抛异常 | 让 LLM 看到"越界=碰撞"的统一惩罚信号，无需额外规则。 |
| 双线性插值采样距离场 | 子像素轨迹避免阶梯量化，便于评估细微违规。 |
| 新增 `FailureTaxonomy.COLLISION`（与 `COLLISION_WALL` 并存） | 区分编译期几何相交（`COLLISION_WALL`）vs 运行期距离场鲁棒度违反（`COLLISION`）。 |
| `violation_nodes` 列表 + `worst_node` 双轨 | 列表 → 时序剖面；worst → LLM 一次性命中最严重处。 |
| metrics.py 双重 import 兜底 | 同时支持 `python -m smdsl_demo.X`（包名）与 `python smdsl_demo/X.py`（裸名）。 |

## 7. 与 Demo 2 的接口契约

LLM 在 Demo 2 自我修正循环中需消费的最小字段集合：

```python
{
  "error": {"type": "collision", "robustness_score": -0.30},
  "context": {
    "rule": "Distance > 0.300",
    "worst_node": {"t": 1.90, "x": 0.52, "y": 2.94, "rho": -0.30},
    "n_violation_nodes": 26
  }
}
```

→ LLM 可据此修改路径（在 t≈1.90 附近增加避障 waypoint）或调整 D_safe。
