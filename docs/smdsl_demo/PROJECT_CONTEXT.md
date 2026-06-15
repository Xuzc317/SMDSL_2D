# SMDSL (RoboIR) 项目全局上下文与开发指南

## 一、 项目愿景与第一性原理

本项目旨在开发“面向具身智能的空间-动作领域特定语言（SMDSL，标准实现命名为 RoboIR）”。

核心痛点：现有 VLA（视觉-语言-动作）模型端到端输出底层轨迹，缺乏空间几何精度，且存在不可解释的“黑盒”问题。

核心解法：放弃让大模型直接做数学计算，转而让其输出带有**时空约束（STL）\**和\**强类型空间声明**的 DSL 代码。底层数学计算与碰撞检测交由 Rust API 和经典求解器完成。

## 二、 核心架构：四大执行区 (The 4 Execution Zones)

在开发任何模块时，必须严格遵守以下物理与逻辑的解耦边界：

1. **多模态感知与锚定区 (Multimodal Grounding):** 将自然语言指令中的实体锚定到 CAD 的 3D 节点上。
2. **语义编译与 IR 生成区 (Semantic Compilation):** VLM **仅负责**输出带有时空约束的 RoboIR 抽象语法树。绝对禁止 VLM 在代码中自行计算欧氏距离或矩阵求逆。
3. **约束求解与时空规划区 (Constraint Solving):** 底层 API 根据 RoboIR 约束和 A* 拓扑图，进行纯几何与物理的代数求解。
4. **闭环执行与结构化反馈区 (Structured Feedback):** 执行动作并返回结构化的失败分类（如任务规划错误、运动规划错误、执行控制错误），而不是简单的强化学习 +/-1 奖励。

## 三、 当前工程状态 (Phase 0 已完成)

- **代码骨架 (`D:\code\`)**: 已建立 `cad_parser` 和 `smdsl_demo` 双目录架构，包含 VLM 接口、Rust 编译器 Stub、指标评估 (`metrics.py`) 等核心桩代码。

- **数据集就绪 (`D:\code\data\`)**:

  - `FloorPlanCAD` (3306 files): 提供纯矢量（SVG/CAD）标注，用于无纹理的 A* 空间拓扑解析。

  - `RoboInter-Data / VQA`: 包含 23 万+ episode，提供 10+ 种中间表示（IR）的密集逐帧标注（子任务、抓取位姿、接触点等） ，是验证 VLM 生成 RoboIR 能力的绝对基准 。  

    

    

## 四、 各 Demo 目标效果与 Cursor 开发任务

### Demo 1: CAD 2D Spatial Parsing (环境解析器)

- **目标效果**: 输入一张 `FloorPlanCAD` 的矢量图，彻底弃用 U-Net 等依赖纹理的视觉网络，使用 A* 算法进行“漫水探测”。
- **验证标准**: A* 走不通的封闭多边形被自动 JSON 标记为 `Obstacle/Wall`；A* 高频途径的连通区域被标记为 `Path/Road`。
- **Cursor 任务**: 完善 `cad_parser/astar_topology.py`。实现读取矢量线段 -> 栅格化/Voronoi图 -> A* 拓扑图元提取的算法闭环。

### Demo 2: NL to RoboIR Semantic Compiler (语义转换端)

- **目标效果**: 输入自然语言（如“将杯子保持垂直移动到桌子上”）+ 局部空间 JSON 切片，VLM 能够输出零语法错误的 RoboIR 代码。
- **验证标准**: 输出必须包含内置空间类型（`Pose`, `Frame`, `Transform`, `Grasp`），并且包含类似 STL 的时空声明（如 `Within t, Keep(Vertical(cup))`）。
- **Cursor 任务**: 完善 `smdsl_demo/vlm_parser.py`。基于 `RoboInter` 数据集的真值，编写 Few-Shot Prompt，强制模型只做逻辑映射，不做数学计算。

### Demo 3: Spatial API & Structured Feedback (目标数据计算与反馈端)

- **目标效果**: 接收 Demo 2 编译出的 RoboIR 约束，调用底层的几何 API 计算真实物理参数，并在模拟违规时抛出“失败病理学”报告。
- **验证标准**: `spatial_api_stub.py` 能够快速计算两点间的 A* 最短路径代价；`metrics.py` 能够根据轨迹偏移量，计算出连续的 STL 鲁棒性得分（Robustness Degree），并分类抛出 `FRAME_MISMATCH` 或 `COLLISION` 等结构化反馈。
- **Cursor 任务**: 完善 `spatial_api_stub.py` 中的几何计算逻辑，以及 `metrics.py` 中的结构化失败反馈机制。

## 五、 Cursor AI 编码纪律 (Coding Rules)

1. **类型先行**: 所有 Python 代码必须包含严格的 Type Hints。这为了未来用 Rust (PyO3) 重写核心库做准备。
2. **禁止幻觉计算**: 如果让 LLM 编写 VLM 的 Prompt，必须在 Prompt 中写明“DO NOT CALCULATE COORDINATES, ONLY OUTPUT LOGICAL CONSTRAINTS”。
3. **可微与可采样原则**: 在处理 `RoboInter` 或 CAD 数据时，任何输入给 VLM 的环境变量，必须先写一个函数对其进行“降维切片”，只保留目标物体半径 $N$ 米内的拓扑节点，严禁传入全局巨大矩阵。

## 六、 ⚠️ 系统局限性声明 (Limitations)

> 本节为算法底座封存前的**强制阅读**章节。所有基于本代码继续二次开发的工程师都必须知晓以下边界。

**本系统当前版本为 2.5D 架构，距离场未引入 Z 轴维度；仅适用于机器人底盘的 2D 避障导航与宏观任务规划。对于 6‑DoF 机械臂的三维空间避障，未来需将 2D EDT 升级为 3D 体素距离场或 fVDB。**

### 6.1 维度边界

| 模块 | 当前实现 | 物理含义 |
|---|---|---|
| `cad_parser/astar_topology.py::compute_distance_field` | `scipy.ndimage.distance_transform_edt`（输入 H×W 2D 栅格） | 距离场只在 XY 平面有效 |
| `smdsl_demo/spatial_api_stub.py::_eval_distance_via_field` | 仅使用 `pt["x"], pt["y"]` 采样 | **轨迹点的 `z` 字段被完全忽略** |
| `path_pixels_to_trajectory` | 默认 `z = 0.0` | 所有规划轨迹被压平到地面 |
| `cad_parser/visualize.py::generate_3d_topology_preview` | `Plotly Scatter3d` + `zaxis=[0, 3.5]` 写死 | 视觉白模，**非物理碰撞体** |

### 6.2 适用范围

- ✅ 工厂 AGV / 移动底盘的全局避障路径规划
- ✅ FloorplanQA / 户型图级的语义拓扑分析
- ✅ 楼层级宏观任务规划（不涉及垂直方向碰撞）
- ❌ **6‑DoF 机械臂的桌面级抓取规划**
- ❌ **多层空间**（楼梯 / 上下料台 / 货架）的连续 Z 维避障
- ❌ **动态障碍物**（移动行人 / 其他机器人）——距离场是一次性快照，无增量更新

### 6.3 已知硬编码（按封存时点统计）

- `tests/test_closed_loop_recovery.py` L67‑71：`DEFAULT_RESOLUTION=0.05`、`DEFAULT_ROBOT_RADIUS=0.30`、`DEFAULT_D_SAFE=0.30`、`MAX_RETRIES=3`、`N_TRAJECTORY_POINTS=50`；
- `wall_thickness_m=0.10` 在 5+ 处重复出现；
- `synthesize_trajectory` 是测试桩级别的线性插值，没有运动学约束。

### 6.4 未来升级路径（建议）

1. **3D 体素 SDF**：把 2D EDT 替换为 3D 体素距离场（fVDB / OctoMap / Voxblox）；
2. **z 维采样**：给 `_eval_distance_via_field` 增加 z 轴采样，保留 2D 路径作为楼层级 fast path；
3. **增量距离场**：支持动态障碍物的增量 EDT 更新（KinectFusion 风格）；
4. **硬性 diff 校验**：把 LLM 自愈循环的「软约束」`CORRECTION_PROMPT_TEMPLATE` 升级为修正前后 RoboIR 结构 diff 的硬性校验。

完整局限性分析与历史短板修复记录见 `docs/EVALUATION_REPORT.md` 第四章。