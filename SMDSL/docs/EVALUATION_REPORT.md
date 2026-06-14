# 《SMDSL 项目客观成果与效能评估终版报告》

> **审计对象**：`D:\code\SMDSL`
> **审计范围**：`smdsl_demo/`、`cad_parser/`、`tests/`、`out/`（含真实运行产物）
> **审计原则**：拒绝猜测、事实为本、标明推测、逻辑拆解
> **审计基线**：`smdsl_demo/PROJECT_CONTEXT.md` 中声明的「四大执行区」分离原则
> **归档时间**：2026-05-19（代码底座封存前的最终复盘）

---

## 第一章 · 目标对齐度核查 (Goal vs. Reality)

### 1.1 初衷一：解决 VLA 模型的空间黑盒与幻觉 → **已落地（事实级）**

| 防幻觉护栏 | 证据位置 | 落地形式 |
|---|---|---|
| 禁止 LLM 计算坐标 / 距离 / 角度 | `smdsl_demo/vlm_parser.py` L107‑114 | System Prompt 内 4 条「【绝对禁止】」硬性条款 |
| 禁止时序词（先/然后/接着） | 同上，L110 | 强制声明式 STL 输出 |
| 输出 Schema 4 字段封死 | `vlm_parser.py` L130‑141；L805‑833 `_validate_shape` | `intent / target_frame / grasp_type / stl_constraints` 严格校验 |
| `grasp_type` 必须 ∈ 枚举 | `vlm_parser.py` L58‑63 `VALID_GRASP_TYPES`（`pinch / suction / magnetic_gripper / none`） | 静态字典校验 |
| STL 表达式必须含合法操作符 | `vlm_parser.py` L64‑68 `VALID_STL_OPS`；L830‑832 软校验 | 强制 `Time / Distance / Orientation / OrientationDiff / Velocity / Force / Torque` 之一 |
| 引用对象必须在 nearest_objects | `vlm_parser.py` L847‑934 `validate_roboir_references` | 返回 warnings（含通配符 `obstacle / wall / floor` 白名单 L841‑844） |
| Spatial‑RAG 局部上下文注入 | `vlm_parser.py` L116‑126；`app.py` L1209‑1217 | `local_context["_global_scene_profile"]` 把 Demo 1 拓扑画像写进 Prompt |
| JSON 解析 3 级容错 | `vlm_parser.py` L754‑795 `_safe_parse_json` | 纯 JSON → 剥 ```json fence → 暴力提 `{...}` |
| 失败重试 | `vlm_parser.py` L279‑300 | 3 次重试 + 指数回退 |

**结论**：初衷一已**完整落地**，不是 PPT。LLM 在结构上不可能输出坐标——一旦输出非法字段会被 `_validate_shape` 直接抛 `VlmParserError`。

### 1.2 初衷二：建立强类型与时空约束 (STL) 中间层 → **已落地（事实级）**

| 类型/约束机制 | 证据位置 | 内容 |
|---|---|---|
| 强类型别名 | `spatial_api_stub.py` L30‑47 | `Pose / NodeId / AreaGraph / ConstraintRule / Trajectory / TopologyBundle` 6 个 TypeAlias |
| Frame 一致性枚举 | `vlm_parser.py` L54‑57 `VALID_FRAMES` | 6 个合法 Frame：`base_link / tool0 / world / map / camera_color_optical_frame / object_frame` |
| 静态编译错误枚举 | `rust_compiler_stub.py` L24‑33 `ValidationErrorType` | 8 类（`JSON_PARSE_ERROR / MISSING_FIELD / UNKNOWN_FRAME / FRAME_MISMATCH / INVALID_POSE_REF / INVALID_GRASP_TYPE / STL_SYNTAX_ERROR / CONSTRAINT_WEIGHT_INVALID`） |
| Frame 层级 & 族 | `rust_compiler_stub.py` L64‑81 `FRAME_HIERARCHY` + `FRAME_FAMILY` | `tool0 → robot_end`、`map↔world` 同族；用于拦截「`tool0` 直接加 `map`」 |
| STL 鲁棒度 ρ 主求解 | `spatial_api_stub.py` L499‑570 `_eval_distance_via_field` | 距离场逐点采样 |
| STL 双轨求解 | `spatial_api_stub.py` L255‑264, L390‑422 | 优先 `topology_bundle`；兜底 `reference_poses`；都缺 → 返回 `source="missing_reference"` + `ρ = -∞` |
| 双线性子像素采样 | `spatial_api_stub.py` L67‑102 `_bilinear_sample` | 含 4 角加权、越界 → `out_of_bounds_value=0.0`（碰撞） |
| OrientationDiff 周期归一 | `spatial_api_stub.py` L453‑457 | `diff_rad % 2π`，避免 ±π 翻转伪违反 |

**结论**：初衷二已落地，并且把 ρ 的语义清晰落到代码里（不是文档里）。

### 1.3 初衷三：提供结构化失败反馈以支持闭环自愈 → **已落地（事实级）**

| 反馈机制 | 证据位置 | 内容 |
|---|---|---|
| 失败病理学枚举 | `smdsl_demo/metrics.py` L31‑69 `FailureTaxonomy` | 9 类：`FRAME_MISMATCH / STL_VIOLATION / COLLISION_WALL / COLLISION / GROUNDING_FAILED / POSE_UNDEFINED / GRASP_TYPE_INVALID / STL_SYNTAX_ERROR / TIMEOUT` |
| 结构化反馈生成 | `metrics.py` L76‑227 `generate_structured_feedback` | 输出三段式 JSON：`error / diagnosis / context` + `worst_node` 几何证据注入（L203‑216） |
| 分级 severity 判定 | `metrics.py` L118‑126 | ρ<-0.5 → critical；ρ<0 → error；ρ<0.1 → warning；其他 → info |
| 编译‑运行双轨诊断 | `metrics.py` L441‑510 `run_diagnostic_pipeline` | 编译期错误 → `error_type` 映射；运行期 STL 违反 → `_classify_violation` |
| Distance 来源判定 | `metrics.py` L419‑438 `_classify_violation` | `source=="distance_field"` → `COLLISION`；否则 `STL_VIOLATION`；`Time` → `TIMEOUT` |
| LLM 自愈 Prompt 模板 | `tests/test_closed_loop_recovery.py` L399‑416 `CORRECTION_PROMPT_TEMPLATE` | 三条「最小责任层恢复」规则 |

**结论**：初衷三已落地。

---

## 第二章 · 各模块核心功能与性能客观评价

### 2.1 Demo 1 — 环境解析器：从「神经网络逐像素分类」到「数学图论 + 形态学」

**它实际上做了什么（按 `astar_topology.py` 真实函数链）**：

1. **矢量化栅格化**（L180‑285 `rasterize_to_grid`）：FloorplanQA JSON → Bresenham 线段（`skimage.draw.line`）+ 多边形填充（`skimage.draw.polygon`）→ H×W uint8。**完全无神经网络**。
2. **外部泛洪剔除**（L292‑396 `remove_exterior_freespace`）：用 4‑连通 `scipy.ndimage.label` 标记自由像素 → 取四边缘连通域为「外部」→ 覆写为 0；带「安全护栏」（L386‑393）：若剔除后内部 free 像素 < 0.5% 则回退（防止线稿 PNG 被清零）。
3. **形态学闭包合并**（L292‑360 + L403‑454 `bridge_thin_walls`）：对障碍层做 `cv2.MORPH_CLOSE`（cv2 缺失则 `ndimage.binary_closing`），核为 `ELLIPSE`，半径 `kernel_px`，专门「焊接」≤ 4px 的门框/标注细线。
4. **双线墙焊接**（L456‑545 `weld_double_line_walls`，2026‑05‑19 补齐）：对障碍层做 `MORPH_CLOSE`，核半径 `ceil(max_gap_m / resolution / 2)`，专门焊死工业 DWG 双线墙的内腔。
5. **EDT 距离场**（L461‑474 `compute_distance_field`）：`scipy.ndimage.distance_transform_edt` 一行调用，O(N) 解析。
6. **拓扑分类**（L481‑550 `classify_topology_global`）：纯矩阵三规则——`obstacle / inflated / path`，再 8‑连通分量统计「主区占比 + 孤岛个数」。**无 BFS 漫水**，纯向量化。
7. **安全代价 A***（L565‑660 `safety_aware_astar_flood`）：堆 + 8 邻域 + corner‑cutting 防穿（L645‑651）+ `safety_weight / max(1, distance_field[n])` 安全奖励。
8. **两点 A***（L667‑757 `astar_shortest_path`）：欧氏启发式（admissible），含 4 道硬性拦截（墙 / 距离不足 / 对角穿墙 / 对角距离不足）。

**性能（真实运行产物 `out/demo1_sweep/summary.json`）**：

| 指标 | 全局 1981 layouts（无错误） |
|---|---|
| `elapsed_ms` mean | **29.87 ms** |
| `elapsed_ms` median | **25.79 ms** |
| `elapsed_ms` p90 | 54.50 ms |
| `elapsed_ms` max | 215.77 ms |
| 总 wall_clock | **8.67 s**（8 worker 并行） |
| `path_fraction` mean | 0.489（房间内 ≈ 49% 像素可走） |
| `deadzone_fraction` mean | **0.044**（仅 4.4% 内部像素不可达——主拓扑诚实度高） |
| 错误数 | **0 / 1981**（按 room_type 拆：bedroom 600、hssd 200、kitchen 581、living_room 600） |

**结论**：Demo 1 的「可用标准」达标——**毫秒级、零失败、可解释**。这是一个数学闭环，不是 AI 黑盒。

### 2.2 Demo 2 — 语义编译器：如何限制 LLM 的幻觉

**Prompt 护栏设计**（`smdsl_demo/vlm_parser.py`）：

- **L85‑195 `ROBOIR_SYSTEM_PROMPT`**：
  - L90‑94：声明式 vs 命令式核心原则（类比 SQL `WHERE` vs `for` 循环）
  - L95‑104：4 组「错误 ✗ / 正确 ✅」对照示例锁定 STL 风格
  - L107‑114：5 条【绝对禁止】（不算坐标、不写时序词、不写动作步骤、不引用未声明对象、不返回 Markdown）
  - L116‑126：【环境约束】区显式声明「严格参考 recommended_global_constraints」
  - L144‑191：3 个工业 Few‑Shot（基础避障 / 高危搬运双约束 / 不确定性探测）
- **L726‑752 `_call_api`**：使用 `response_format={"type": "json_object"}`（DeepSeek 软兼容；TypeError 时自动剥离重试）；温度 0.1。

**Spatial‑RAG 局部环境提取**：

- 场景画像生成在 `cad_parser/semantic_profiler.py`（22,960 字节模块），由 `app.py` L1088 调用 `profile_scene` → 输出 `safety_distance_m / max_velocity_ms / requires_precise_grasp / high_risk_zones`；
- 注入路径：`app.py` L1209‑1217 把上述约束写进 `local_context["_global_scene_profile"]` 后调用 LLM。
- 降维切片纪律：`PROJECT_CONTEXT.md` L58 明确「严禁传入全局巨大矩阵」——代码上的体现是 Tab 2 输入永远是 `local_context` JSON（关键字段 `nearest_objects`），从不传整张距离场。

**结构化校验后置**：`_validate_shape`（L797‑833）+ `validate_roboir_references`（L847‑934）+ `normalize_stl_constraints`（L941‑980）。

**结论**：Demo 2 的护栏是**编译器级别的硬约束**，不是「祈祷型」Prompt。

### 2.3 Demo 3 — 物理求解与反馈

**STL 鲁棒度 ρ 计算 — 距离场双线性采样逻辑**（`spatial_api_stub.py` L499‑570 `_eval_distance_via_field`）：

```python
for pt in trajectory:
    x, y, cur_t = float(pt["x"]), float(pt["y"]), float(pt["t"])
    row_f, col_f = _world_to_grid(x, y, tx)
    d_px        = _bilinear_sample(df, row_f, col_f, out_of_bounds_value=0.0)
    d_real_m    = d_px * res
    rho         = d_real_m - min_dist_m
```

`_bilinear_sample`（L67‑102）严格执行 4 角加权 `v00·(1-fc)·(1-fr) + v01·fc·(1-fr) + v10·(1-fc)·fr + v11·fc·fr`，越界返回 `0.0`（即「越界 ≡ 已撞墙」，与 `DEMO3_README.md` L68 一致）。

**FailureTaxonomy 真实分类**（`metrics.py` L419‑438 `_classify_violation`）：

- `Distance > X` 且 `source == "distance_field"` → `COLLISION`
- `Distance > X` 仅参考点回退 → `STL_VIOLATION`
- `Time < X` → `TIMEOUT`
- 其他 → `STL_VIOLATION`

**真实运行结果**（`out/demo3/robustness_report.json`，kitchen/room_24，`D_safe=0.30 m`）：

| 轨迹 | `min ρ (m)` | `n_violation_nodes` | `violation_duration` (s) | 分类 |
|---|---:|---:|---:|---|
| A_SAFE | **+0.150** | 0 | 0.0 | info / none |
| B_GRAZE_WALL | **−0.150** | **53** | 2.65 | **collision** / error |
| C_THROUGH_OBJ | **−0.300** | **26** | 1.30 | **collision** / error |
| D_OUT_OF_BOUND | **−0.300** | **38** | 1.90 | **collision** / error |

ρ 的语义与数值在代码、文档、JSON 三处**完全自洽**。这是一个真正的**连续语义梯度**。

---

## 第三章 · 端到端闭环能力审计

### 3.1 链路逐段证明（基于 `tests/test_closed_loop_recovery.py`，680 行）

| 阶段 | 代码位置 | 行为 |
|---|---|---|
| ① 场景初始化 | L108‑174 `init_scene` | JSON 分支：`run_pipeline` → `to_topology_bundle` → 取出 `distance_field + grid_transform` |
| ② 寻找远离的安全中心 | L177‑238 `find_free_centroids` | 贪心最远点选择 |
| ③ 诱导失败指令 | L245‑265 `build_trap_instruction` | 「最短直线穿过、5 秒内、不要绕路」——故意省略安全距离 |
| ④ 合成穿墙轨迹 | L272‑298 `synthesize_trajectory` | 两个中心点之间 50 点直线插值——**几何上必然穿墙** |
| ⑤ 物理校验 | L305‑392 `validate_roboir` | 若 LLM 漏写 Distance → 强制注入 `Distance > d_safe`；走距离场分支 → `FailureTaxonomy.COLLISION` |
| ⑥ 自愈循环 | L419‑517 `run_correction_loop` | 反馈整段塞回 `CORRECTION_PROMPT_TEMPLATE`，重新生成；最多 3 轮 |
| ⑦ 诊断报告 | L524‑616 `print_diagnostic_report` | 6 个面板：耗时、ρ 演化、LLM 调用、修正成功率、违规节点改善、约束变化 diff |

### 3.2 工程价值

- **闭环结构完整**：`生成 → 校验 → 错误结构化 → 反馈给同一个 LLM → 重新生成 → 再校验`。
- **失败不是 −1**，而是携带 `worst_node = {t, x, y, row, col, d_real_m, rho, in_bounds}` 的**坐标级几何证据**。
- **可量化的修正信号**：`Δρ`、`n_violation_nodes` 改善、约束 diff——指标级**评测就绪**。

---

## 第四章 · 现存局限性与真实短板

### 4.1 硬编码 / 几何降维

| 局限 | 证据 | 影响 |
|---|---|---|
| **整套系统是 2.5D 而非 3D** | `astar_topology.py` L794 默认 `z=0.0`；`_eval_distance_via_field` L523‑526 只读 `x, y`；3D 预览 `zaxis=[0, 3.5]` 写死。 | 真实机械臂 6 DoF 抓取的 z 维不会被距离场惩罚——**详见根 `README.md` 与 `PROJECT_CONTEXT.md` 的【系统局限性声明】章节**。 |
| **距离场是「拍一张」** | 一次性 `compute_distance_field` 后存入 `topology_bundle`。 | 动态障碍物完全未处理。 |
| **几何参数 4 处硬编码** | `tests/test_closed_loop_recovery.py` L67‑71：`DEFAULT_RESOLUTION / DEFAULT_ROBOT_RADIUS / DEFAULT_D_SAFE / MAX_RETRIES / N_TRAJECTORY_POINTS` | 换平台需要全仓搜替换。 |
| **直线轨迹合成是测试桩** | `synthesize_trajectory` 仅线性插值，没有运动学约束。 | 只能诱导失败，不能当真实规划器。 |

### 4.2 评测真值缺口

- `precision_recall`（`metrics.py` L260‑302）当前只是接口存在，**未在 `data/`、`out/` 中找到** RoboInter‑VQA 的 SMDSL 真值对照集。
- Demo 2 的 NL→RoboIR 准确率**未在代码/日志中找到**端到端基准数据。

### 4.3 LLM 修正的「软约束」性质

- `CORRECTION_PROMPT_TEMPLATE` 虽明确写了「不要改变 intent 与 target_frame」「仅增/强避障约束」，但**未在代码中找到**对修正前后 RoboIR diff 的硬性结构校验（仅 L596‑604 打印给人看）。当前是软约束。

### 4.4 体量与维护债

- `smdsl_demo/app.py` 单文件 **2349 行**，主题 CSS / Gradio 组件 / 业务逻辑混合（`DEVELOPMENT_JOURNAL.md` L114‑118 自承「维护债」）；3D 预览只是 `Plotly Scatter3d` 把墙体每像素画一条垂直线段，**不是真正的 mesh / extrusion**。

### 4.5 已修复的历史短板（2026‑05‑19 封存前补齐）

| 修复项 | 修复前 | 修复后 |
|---|---|---|
| `weld_double_line_walls` 未定义 | `tests/test_closed_loop_recovery.py` L128/135 导入未存在的函数 → DWG 分支 ImportError | `cad_parser/astar_topology.py` 新增完整实现（障碍层 `MORPH_CLOSE`，核半径 `ceil(max_gap_m/res/2)`） |
| `get_astar_path_cost` 启发式 = 0 | 实际退化为 Dijkstra，但接口名为 A* | 新增可选 `node_positions + resolution`，提供时启用欧氏启发式（admissible），未提供时显式降级并写明 docstring |

---

## 总评（事实层）

- **PROJECT_CONTEXT 三大初衷**：全部在代码里看得见。
- **「四大执行区」边界**：从 `_validate_shape` → `RustCompilerStub` → `_eval_distance_via_field` → `FailureTaxonomy` 的调用链是清晰且不交叉的。
- **Demo 1 数据**：1981 layouts、0 错误、median 25.79 ms、wall_clock 8.67 s。
- **Demo 3 数据**：A/B/C/D 四条轨迹的 ρ 与违规节点数与 README 完全自洽，连续语义梯度真实存在。
- **闭环结构**：真闭环，封存时已通过完整真测落盘 → `out/milestone_closed_loop_success.log`。
- **真实短板**：2.5D（z 维丢失）、动态障碍物零处理、precision/recall 真值集缺失、app.py 体量维护债。

---

【推测】下一阶段往 6‑DoF 机械臂迁移时，**第一优先级建议**：

1. 把 2D EDT 升级为 **3D 体素距离场**（参考 fVDB / OctoMap）；
2. 给 `_eval_distance_via_field` 增加 z 维采样（保留 2D 回退作为楼层级导航的 fast path）；
3. 把 `CORRECTION_PROMPT_TEMPLATE` 的软约束升级为**硬性 diff 校验**（修正后必须保持 intent + target_frame 不变，否则拒绝并重试）；
4. 给 `precision_recall` 接 RoboInter‑VQA 真值集，跑出真实的 F1 基线。
