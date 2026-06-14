# SMDSL (RoboIR) — 具身智能空间-动作特定领域语言

> **解决的核心痛点**：现有 VLA（视觉-语言-动作）模型（如 RT-2、OpenVLA）采用端到端黑盒模式，
> 无法理解物理几何关联性，在时序依赖的复杂任务中极易陷入局部最优，且发生碰撞时完全不可追溯。
>
> **解法**：SMDSL 在 LLM 大脑与物理执行器之间插入一个强类型"中间表示层 (IR)"——
> 底层 API 负责所有空间计算，LLM 只做逻辑翻译，编译器在编译阶段消灭空间幻觉。

---

## 项目状态

- **Git HEAD**: `0a4fdaf` — fix: A\* corner-cutting, API key propagation, path-planner state reuse
- **Gradio UI**: `http://127.0.0.1:7860/` （`python -m smdsl_demo.app`）
- **闭环测试**: ✅ 端到端自愈验证通过（`SMDSL/tests/test_closed_loop_recovery.py`）

---

## 方向一：2D 极致打磨 — ✅ 已完成

> 让 SMDSL 在 2D 路径约束验证上，从技术 demo 变成有量化证据支持的、物理上可执行的、对自己行为诚实的工程系统。

| 验收标准 | 状态 |
|----------|------|
| 系统自洽：所有接口明确只接受 2D（z=0）；LLM 修正不篡改核心语义；实体丢弃用户可见 | 🟢 已完成 |
| 轨迹可信：每条轨迹有加减速剖面，路径经梯度微调 | 🟢 已完成 |
| 有据可依：EDT 方案 vs costmap 膨胀层方案在 200 布局上的 clearance 量化对比 | 🟢 已完成（合成数据 +12.1%）|

| Phase | 内容 | 状态 |
|-------|------|------|
| Phase 0 | Git 仓库初始化 | ✅ |
| Phase 1 | P0 修复（Z 轴断言 / RoboIR diff / DWG 丢弃率）| ✅ |
| Phase 2 | P1 优化（轨迹合成 / 梯度微调 / EDT vs Costmap）| ✅ |
| Phase 3 | app.py 模块拆分 | ✅ |
| Phase 4 | 测试补充 | ✅ (38 tests) |
| Phase 5 | 基准运行 | ✅ |
| Phase 6 | 文档与收尾 | ✅ |

> 详见 [SMDSL_Direction1_Workflow.md](SMDSL_Direction1_Workflow.md) · [CHANGELOG.md](CHANGELOG.md)

### 基准数据

| 数据集 | 布局数 | EDT vs Costmap 提升 |
|--------|--------|---------------------|
| FloorplanQA | 1,981 (目标) | — |
| 合成基准 (200 layouts × 5 pairs) | 200 | **clearance_min +31.0%** |
| OSM 真实路网 | 0-30 (可下载) | — |

> 详见 [OPERATIONS_MANUAL.md](OPERATIONS_MANUAL.md) · [SMDSL_Direction1_Workflow.md](SMDSL_Direction1_Workflow.md)

---

## 四大执行区（唯一权威架构）

```
┌─────────────────────────────────────────────────────────────────────┐
│  Zone 1 — Spatial-RAG 感知层      cad_parser/                       │
│                                                                     │
│  输入: DWG / SVG / JSON / PNG                                        │
│  处理: LibreDWG 解析 → 形态学闭运算 → EDT 距离场 → A* 拓扑          │
│        → semantic_profiler: 骨架化 + 瓶颈检测 → 场景画像 JSON        │
│  输出: topology_bundle (grid, distance_field, transform)            │
│        scene_profile (scene_type, safety_distance_m, ...)           │
│  规则: ⛔ 禁止调用 LLM / ⛔ 禁止主观判断                             │
├─────────────────────────────────────────────────────────────────────┤
│  Zone 2 — 语义编译层              smdsl_demo/vlm_parser.py          │
│                                                                     │
│  输入: 自然语言指令 + 局部环境 + 全局约束画像 (from Zone 1)          │
│  处理: DeepSeek (STL 护栏 System Prompt + 工业 Few-Shot)            │
│  输出: RoboIR JSON {intent, target_frame, grasp_type, stl_constraints}│
│  规则: ⛔ LLM 禁止计算坐标/距离/角度 / ⛔ 禁止引用环境外的物体       │
├─────────────────────────────────────────────────────────────────────┤
│  Zone 3 — 物理求解层              smdsl_demo/spatial_api_stub.py    │
│                                                                     │
│  输入: RoboIR 约束 + 轨迹 + topology_bundle                         │
│  处理: 双线性插值采样距离场 → 逐时间步计算 ρ(t) = d_real_m − D_safe  │
│        越界点 → d_real_m = 0 → ρ = −D_safe（碰撞惩罚）               │
│  输出: STL 鲁棒度报告 {robustness, violated, violation_nodes, ...}  │
│  规则: ⛔ 禁止调用 LLM / ⛔ 纯确定性数值计算                         │
├─────────────────────────────────────────────────────────────────────┤
│  Zone 4 — 结构化反馈层            smdsl_demo/metrics.py             │
│                                                                     │
│  输入: Zone 3 报告 (violation_nodes, worst_node, ...)               │
│  处理: FailureTaxonomy 分级 → generate_structured_feedback          │
│  输出: {error.type, error.robustness_score, context.worst_node, ...}│
│  消费: LLM (Zone 2) 读取 → 最小责任层恢复 → 重新编译 RoboIR         │
└─────────────────────────────────────────────────────────────────────┘
```

---

## 目录结构（干净状态）

```
D:\code/
├── SMDSL/                          ← 💎 唯一权威工程目录
│   ├── cad_parser/                 ← Zone 1: 几何/拓扑解析
│   │   ├── astar_topology.py       #   EDT + A* + 对角防穿墙
│   │   ├── dispatcher.py           #   多格式入口 (DWG/SVG/JSON/PNG)
│   │   ├── dwg_ingestion.py        #   LibreDWG 安全沙箱
│   │   ├── semantic_extractor.py   #   LLM 业务节点识别
│   │   ├── semantic_profiler.py    #   Spatial-RAG 场景画像
│   │   └── visualize.py            #   Plotly/Matplotlib 可视化
│   ├── smdsl_demo/                 ← Zone 2-4 + Gradio UI
│   │   ├── app.py                  #   主 UI (Tab 1/2/3, ~2400 行)
│   │   ├── vlm_parser.py           #   Zone 2: NL → RoboIR 编译器
│   │   ├── spatial_api_stub.py     #   Zone 3: STL 物理求解
│   │   ├── metrics.py              #   Zone 4: FailureTaxonomy + 反馈
│   │   ├── visualize_demo3.py      #   Plotly 3D 仪表盘
│   │   ├── demo3_robustness.py     #   Demo 3 独立运行器
│   │   ├── trajectory_smoother.py  #   样条+梯形 Profile 轨迹合成
│   │   ├── ui_theme.py             #   CSS + 主题
│   │   └── ui_common.py            #   共享 UI 工具
│   ├── tests/                      ← 38 单元测试
│   │   ├── test_z_axis.py
│   │   ├── test_entity_stats.py
│   │   ├── test_roboir_diff.py
│   │   ├── test_trajectory_smoother.py
│   │   ├── test_gradient_refine.py
│   │   └── test_closed_loop_recovery.py
│   └── data -> ../data
├── benchmark/                      ← 基准测试
│   ├── bench_edt_vs_costmap.py
│   └── results/
├── data/
│   ├── generate_synthetic_layouts.py  # 合成布局生成器
│   ├── download_osm_networks.py       # OSM 真实路网下载
│   └── datasets/
├── back/                           ← SMDSL 归档
├── data/                           ← CAD 样本数据集 (gitignored 大文件)
├── out/                            ← 生成产物 (gitignored)
├── PROJECT_ARCHIVE.md              ← 工程演进血泪史
├── README.md                       ← 本文件
└── .gitignore
```

---

## 快速启动

```powershell
# 1. 安装依赖
pip install -r requirements.txt

# 2. 运行测试（38 tests）
python -m pytest SMDSL/tests/ -v

# 3. 生成合成基准数据
python data/generate_synthetic_layouts.py --n_layouts 200

# 4. 运行 EDT vs Costmap 基准
python benchmark/bench_edt_vs_costmap.py --data_dir data/datasets/synthetic_benchmark --n_layouts 200 --n_pairs 5

# 5. 下载 OSM 真实路网（可选，需联网）
python data/download_osm_networks.py --preset mixed --n_networks 20

# 6. 启动 Gradio 调试面板（需 API Key）
$env:DEEPSEEK_API_KEY = "sk-..."
cd SMDSL && python -m smdsl_demo.app
```

---

## 归档规则（永久执行）

> **凡被淘汰的模块、废弃的实验代码、冗余数据，一律移入 `back/` 归档，禁止直接删除。**
> 详见 [`back/README.md`](back/README.md)。

---

## 核心设计约束

| 原则 | 说明 |
|------|------|
| **LLM 不算数学题** | 坐标、距离、角度全部由底层 API 计算 |
| **强类型编译** | `VALID_FRAMES` / `VALID_STL_OPS` 在编译期校验，消灭幻觉 |
| **越界=碰撞** | 出界点 d_real_m=0，ρ=−D_safe，给 LLM 统一惩罚信号 |
| **最小责任层恢复** | LLM 仅修改违规 STL 项，不推翻整个计划 |
| **Zone 解耦** | 任何跨 Zone 的 LLM↔数学混调视为架构违规 |
