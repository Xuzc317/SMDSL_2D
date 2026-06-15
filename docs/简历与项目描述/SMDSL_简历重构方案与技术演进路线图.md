# SMDSL_DEMO 简历重构方案与技术演进路线图

> **产出目标**：(1) 实事求是、具指标支撑的简历项目描述；(2) 两阶段技术优化 Roadmap
> **撰写日期**：2026-06-08
> **参考依据**：工作区完整代码审计 + 李飞飞世界模型分类学 + SMDSL 架构诊断白皮书

---

# 第一部分：简历重构与项目重新定义

## A. 详细版项目介绍（供后续精简）

---

### 项目名称
**SMDSL (Spatial-Motion Domain-Specific Language) — 面向具身智能的空间约束验证与闭环重规划系统**

### 一句话概述
从零构建了一个基于欧氏距离场 (EDT) 与信号时序逻辑 (STL) 的 2.5D 移动底盘避障验证系统，实现了"CAD图纸解析 → 空间状态构建 → STL约束编译 → 物理约束验证 → LLM驱动的闭环自愈重规划"的完整 Agent 链路。系统在 1,981 个 FloorplanQA 户型布局上以平均 29.87ms/布局完成拓扑提取，零失败；支持 5 种工业/学术输入格式（DWG/SVG/JSON/PNG/OSM），并已通过端到端闭环自愈测试。

### 技术架构：四区解耦设计

项目严格遵循**声明式空间约束**的设计理念，将系统拆分为四个不交叉的执行区（Zone 1-4），每个 Zone 有明确的输入/输出契约和禁止事项：

```
Zone 1: Spatial-RAG 感知层 (cad_parser/, ~2,800行)
  ├── 多格式派发器: DWG(LibreDWG)/SVG(XML解析)/JSON(FloorplanQA)/PNG(二值化)/OSM
  ├── 栅格化引擎: 矢量→占据栅格 (Bresenham + 多边形填充)
  ├── 工业后处理三步: weld_double_line_walls → remove_exterior_freespace → bridge_thin_walls
  ├── EDT距离场: scipy.ndimage.distance_transform_edt
  ├── 安全代价A*: 8邻域 + corner-cutting防穿墙 + safety_weight/(1+distance)
  └── 输出: topology_bundle {distance_field, grid_transform, robot_radius}

Zone 2: 语义编译层 (smdsl_demo/vlm_parser.py, ~1,050行)
  ├── DeepSeek API 编译器: System Prompt 5条"绝对禁止"护栏
  ├── STL声明式约束: RoboIR JSON {intent, target_frame, grasp_type, stl_constraints}
  ├── 编译期校验: _validate_shape + 枚举值检查 + 引用一致性验证
  └── 输出: 标准化STL约束列表

Zone 3: 物理约束求解层 (smdsl_demo/spatial_api_stub.py, ~650行)
  ├── 双线性子像素采样: 4角加权插值，越界→d=0（越界=碰撞）
  ├── STL鲁棒度: ρ(t) = d_real_m(t) − D_safe (连续语义梯度)
  ├── 双轨求解: 优先topology_bundle距离场，回退reference_pose单点
  └── 输出: {robustness, violated, violation_nodes[{t,x,y,d_real,ρ}], ...}

Zone 4: 结构化反馈层 (smdsl_demo/metrics.py, ~510行)
  ├── FailureTaxonomy: 9类错误枚举 (COLLISION/STL_VIOLATION/FRAME_MISMATCH/...)
  ├── 结构化反馈生成: worst_node几何证据注入（t,x,y,d_real_m,ρ）
  └── 消费方: LLM读取反馈→最小责任层恢复→重新编译RoboIR
```

### 关键工程挑战与解决过程

**挑战1：工业 DWG 双线墙"漏水"问题**

工业建筑CAD中墙体通常用两条平行线段（外皮+内皮）绘制，栅格化后两条线之间的缝隙被EDT识别为"自由空间"，导致A*路径穿越墙体内部。

- **解决**：实现 `weld_double_line_walls` 函数——对障碍层做形态学闭运算（MORPH_CLOSE），核半径动态计算为 `ceil(max_gap_m / resolution / 2)`。30cm墙厚@5cm/px → 3px核半径，毫秒级完成焊接。
- **效果**：historical_museum.dwg 的连通分量从87个碎片→4个连通主区域。

**挑战2：外部自由空间泛洪 (Exterior Flooding)**

PNG图片白色背景被二值化为自由空间，SVG的grid初始化为全1——外部自由空间与室内同值，导致EDT在建筑外部也计算距离场，A*漫水泄漏到外部。

- **解决**：实现 `remove_exterior_freespace`——4-连通域分析→四边缘接触域=外部→覆写为0。加入安全护栏：若覆写后内部free<0.5%则回退（防止线稿PNG被清零）。
- **效果**：PNG/SVG路径的A*搜索被严格限制在建筑内部。

**挑战3：A*对角线"穿墙" (Corner-Cutting)**

8邻域A*在对角移动时，若相邻两个正交邻居是墙体，机器人会在物理上穿过墙角。

- **解决**：在 `safety_aware_astar_flood` 中显式检测角切条件——对角移动时检查两个正交邻居的占据状态和距离场，任一不满足R_robot即阻止该移动。
- **效果**：轨迹不再出现"幽灵穿墙"路径。

**挑战4：Z轴静默忽略导致假阴性碰撞**

`_eval_distance_via_field` 中仅使用轨迹点的 (x,y) 进行距离场采样，轨迹点的 `z` 字段被完全忽略。对于6-DoF机械臂轨迹（存在显著Z轴变化），系统会报告ρ>0（假阴性——实际存在碰撞但未被检测）。

- **当前状态**：已识别并记录为已知局限。建议下一版本添加Z轴硬断言。

**挑战5：距离场为静态快照**

`compute_distance_field` 在初始化时调用一次，之后存入 `topology_bundle` 不再更新。系统无法处理动态障碍物（移动行人/其他机器人/临时堆放物）。

- **当前状态**：适用于静态场景（建筑平面图、仓库布局）。动态障碍物支持列入P1优化计划。

### 量化交付物

| 指标 | 数值 | 说明 |
|:---|:---|:---|
| **代码规模** | ~10,000+ 行 Python | 21个.py文件，4 Zone架构 |
| **支持输入格式** | 5种 | DWG (LibreDWG工业解析), JSON (FloorplanQA schema), SVG (XML解析), PNG/JPG (二值化), OSM (osmAG拓扑) |
| **FloorplanQA 基准** | 1,981 layouts, 0 errors | 覆盖 bedroom×600, hssd×200, kitchen×581, living_room×600 |
| **平均拓扑提取耗时** | 29.87ms/layout | median 25.79ms, p90 54.50ms, max 215.77ms (8 workers并行) |
| **DWG 工业图纸** | 19/20 成功解析 | 1个失败: fish_processing_plant.dwg (320MB JSON溢出，已记录ISSUES_LOG) |
| **STL鲁棒度语义** | 4条验证轨迹 (kitchen/room_24) | A_SAFE ρ=+0.150, B_GRAZE ρ=−0.150(53违规节点), C_THROUGH ρ=−0.300(26节点), D_OUT ρ=−0.300(38节点) |
| **闭环自愈** | 3轮内成功修正 | 故意穿墙指令→碰撞检测→LLM重规划→A*绕行 |
| **DWG安全沙箱** | 7层防护 | 扩展名白名单→文件大小上限(50MB)→魔数校验→CVE-2025-61154 R2004缓解→dwgread可用性→subprocess timeout(30s)→编码回退链(UTF-8→Latin-1→cp1252) |
| **ISSUES_LOG** | 6个已记录问题 | 含严重程度/影响文件/根因分析/建议修复 |

### 核心技术组合的理性评估

**EDT距离场 + STL鲁棒度语义 + LLM闭环自愈**

这一技术组合在当前2.5D/2D约束验证场景下的表现：

**实际效果**：
- EDT距离场提供连续的（非二值）安全度量——每个自由像素对应精确的物理距离值，使得A*的1/distance惩罚项有连续梯度可循（优于ROS costmap_2d的简单膨胀层）
- STL鲁棒度ρ = d_real − D_safe 提供了数值化、可微分的约束违反信号——与纯布尔"碰撞/未碰撞"不同，ρ的连续值支持更细粒度的反馈（critical/error/warning/info分级）
- LLM闭环自愈在受控实验条件下（已知起点/终点/静态环境）可在3轮内完成修正

**已知局限**：
- EDT为静态快照，不支持动态障碍物
- 仅2D距离场（z轴被忽略）——对6-DoF机械臂不适用
- A*离散搜索 vs 基于梯度的轨迹优化——路径由离散栅格点连接，非连续最优
- LLM修正为"软约束"——未对修正前后RoboIR做diff硬校验，LLM理论上可推翻原意图
- 组合在学术界不独特——EDT在medical imaging (Maurer 2003)、STL在formal methods (Maler & Nickovic 2004, Donzé 2013)、LLM闭环在robotics (Liang et al. 2024, Code-as-Policies) 均有先例。**本项目的价值不在单点新颖性，而在将这三种技术无缝集成到一个可工作的、有UI的、端到端流水线中。**

### 技术栈

`Python 3.11+` | `Gradio 4.x` | `NumPy/SciPy/scikit-image` | `OpenAI SDK (DeepSeek兼容)` | `Matplotlib/Plotly` | `LibreDWG (dwgread)` | `OpenCV (cv2, 可选fallback到scipy.ndimage)` | `Pillow (PNG处理)`

### 项目状态

- **GitHub**: github.com/Xuzc317/SMDSL_demo (feature/smdsl-upload 分支)
- **UI**: Gradio Web 界面 (3个Tab: CAD解析 / 语义编译 / 物理验证)
- **测试**: 端到端闭环自愈测试通过
- **文档**: README.md + PROJECT_ARCHIVE.md + 4份子模块README + ISSUES_LOG.md (6个已记录的技术问题及根因分析)

---

# 第二部分：项目修剪、去幻觉与未来研发Roadmap

## 开篇：抛弃执念，拥抱务实

**矫正声明**：以下 Roadmap 基于一个核心前提——**放弃"不使用CUDA"的执念**。NVIDIA的CUDA生态在可预见的未来仍是物理仿真与3D视觉的工业标准。走向真正的空间智能需要正确的工具，而非对特定厂商的规避。同时，**放弃"通用物理模拟器"的定位妄想**——接受项目当前是一个2D/2.5D约束验证系统，并用务实的工程节奏将其打磨到极致，再渐进式迈向3D。

---

## 方向一：现有项目纠偏与 2D 极致打磨（差异化深耕）

### 1.1 立即修复清单（P0 — 安全性硬伤，本周完成）

**🔴 Fix 1: Z轴静默忽略 → 硬断言**

当前 `_eval_distance_via_field` (spatial_api_stub.py:519-590) 在逐轨迹点采样时仅使用 `(x, y)`：

```python
# 现状（第544-545行附近）
x = float(pt.get("x", 0.0))
y = float(pt.get("y", 0.0))
# z 被完全忽略——假阴性碰撞风险
```

**修改方案**：

```python
# 在函数入口添加 Z轴检测
def _eval_distance_via_field(trajectory, min_dist_m, bundle, report):
    # 检测轨迹是否存在显著Z轴变化
    z_vals = [float(pt.get("z", 0.0)) for pt in trajectory]
    z_range = max(z_vals) - min(z_vals)
    if z_range > 0.01:  # 1cm以上视为3D轨迹
        report["robustness"] = float("-inf")
        report["violated"] = True
        report["source"] = "z_axis_not_supported"
        report["details"].append({
            "error": (
                f"轨迹包含显著的Z轴变化 (Δz={z_range:.3f}m)，"
                f"当前距离场仅支持2D (x,y)平面。"
                f"请使用2D轨迹或升级到3D体素距离场。"
            )
        })
        return report
    # 继续原有2D采样逻辑...
```

**🔴 Fix 2: RoboIR 修正 diff 硬校验**

当前 `run_correction_loop` (test_closed_loop_recovery.py:517-660) 中的LLM修正结果只做了内容打印，未对修正后的RoboIR做结构化diff校验。

**修改方案**：

```python
# 在LLM修正返回后、校验前插入
CORRECTED_INTENT = corrected_roboir.get("intent")
CORRECTED_TARGET = corrected_roboir.get("target_frame")
ORIGINAL_INTENT = roboir_initial.get("intent")
ORIGINAL_TARGET = roboir_initial.get("target_frame")

if CORRECTED_INTENT != ORIGINAL_INTENT or CORRECTED_TARGET != ORIGINAL_TARGET:
    print(f"    [REJECTED] LLM改变了核心语义: "
          f"intent {ORIGINAL_INTENT}→{CORRECTED_INTENT}, "
          f"target {ORIGINAL_TARGET}→{CORRECTED_TARGET}")
    # 重新请求修正（最多1次额外机会）
    if attempt < max_retries:
        correction_prompt += (
            f"\n\n[硬性约束] 你的上一次修正被拒绝，因为它改变了原始的 "
            f"intent='{ORIGINAL_INTENT}' 和 target_frame='{ORIGINAL_TARGET}'。"
            f"请保持这两个字段不变，仅调整 stl_constraints 中的约束参数。"
        )
        continue  # 重新请求而非接受
```

**🔴 Fix 3: DWG实体丢弃率告警**

当前 dispatcher 在 `_extract_dwg_geometry` 中静默跳过 SPLINE(14,226)/ELLIPSE(3,068)/HATCH(1,482) 等实体类型。需要在 UI 和日志中显式告警。

**修改方案**：在 `_parse_dwg` 返回的 `ParseResult["note"]` 中追加实体丢弃率统计，当丢弃率 >10% 时在 Gradio UI 显示黄色警告横幅。

### 1.2 核心优化清单（P1 — 2D 极致打磨，本月完成）

**⚡ Optimize 1: 梯形速度 Profile 替代线性插值**

当前 `synthesize_trajectory` / `path_pixels_to_trajectory` 使用时间均匀分配——轨迹点在时间上等间距分布，没有加速/减速阶段。这不符合任何真实机器人的运动学特性。

**改进方案**：实现三段式梯形速度profile：

```python
def trapezoidal_velocity_profile(
    total_distance_m: float,
    total_time_s: float,
    v_max: float = 1.0,       # 最大线速度 m/s
    a_accel: float = 0.5,     # 加速度 m/s²
    n_points: int = 50,
) -> List[float]:
    """
    生成梯形速度profile的时刻表（T1加速段 → T2匀速段 → T3减速段）。
    返回每个时刻的累计弧长参数 s(t) ∈ [0, 1]。
    """
    t_accel = v_max / a_accel
    d_accel = 0.5 * a_accel * t_accel**2

    if 2 * d_accel > total_distance_m:
        # 纯三角形profile（距离太短，达不到v_max）
        t_accel = np.sqrt(total_distance_m / a_accel)
        t_const = 0.0
        v_peak = a_accel * t_accel
    else:
        t_const = (total_distance_m - 2 * d_accel) / v_max
        v_peak = v_max

    t_total = 2 * t_accel + t_const
    times = np.linspace(0, t_total, n_points)

    def s_at_t(t):
        if t <= t_accel:
            return 0.5 * a_accel * t**2
        elif t <= t_accel + t_const:
            return d_accel + v_peak * (t - t_accel)
        else:
            t_dec = t - t_accel - t_const
            return total_distance_m - 0.5 * a_accel * (t_accel - t_dec)**2

    return [s_at_t(t) / total_distance_m for t in times]
```

**评测指标设计**：

| 指标 | 当前（线性插值） | 目标（梯形profile） | 测量方法 |
|:---|:---:|:---:|:---|
| 最大加速度 | ∞（瞬时速度突变） | ≤ a_accel | 轨迹二阶差分 |
| 路径长度偏差 | 0（完美均匀） | <5% vs 原始A*路径 | arc length积分 |
| ρ稳健度变化 | 基准值 | 不劣于基准（加速段不切角） | ρ_min 对比 |
| 物理可行性评分 | 0/1（不可行/可行） | 1（可行） | 运动学约束满足 |

**⚡ Optimize 2: Incremental EDT — 局部动态更新**

当前 `compute_distance_field` 在初始化时全量计算，整个运行时不再更新。对于静态CAD解析场景这已足够，但对于"少量动态障碍物"场景需要增量更新。

**技术方案**：实现 Ryoo (2004) 的 incremental EDT 算法。核心思路：当栅格中少量像素从"自由"(1)变为"占据"(0)（或反之），只重算受影响区域的EDT，而非全量 O(H×W)。

**实施步骤**：
1. 将距离场存储为 `np.ndarray(H, W, float32)` + 一个脏区域 `AABB`
2. 障碍物更新时，标记其包围盒为脏
3. 使用约束传播（constrained propagation）仅重算脏区域内的EDT——复杂度 O(area_dirty) 而非 O(H×W)

**评测对比**：

| 场景 | 全量EDT (当前) | Incremental EDT (目标) | 加速比 |
|:---|:---:|:---:|:---:|
| 500×500 栅格，1个物体(5×5 px)移动 | ~3ms | ~0.02ms | ~150x |
| 500×500 栅格，5个物体同时移动 | ~3ms | ~0.1ms | ~30x |
| 2000×2000 栅格，1个物体移动 | ~40ms | ~0.05ms | ~800x |

**⚡ Optimize 3: 距离场梯度计算 — 支持基于梯度的轨迹优化**

当前A*在离散栅格上搜索，路径由离散像素点连接。梯度信息 ∇d(x,y) 可以支持连续空间的轨迹微调。

**实现**：

```python
def compute_distance_field_gradient(distance_field: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
    """计算EDT的梯度场 ∇d = (∂d/∂x, ∂d/∂y)，使用中心差分"""
    gy, gx = np.gradient(distance_field.astype(np.float64))
    return gx, gy  # gx = ∂d/∂x, gy = ∂d/∂y（像素单位）

def project_point_to_safe_region(x_px, y_px, df, gradient_magnitude_threshold=0.1):
    """
    将不安全点沿梯度方向（远离障碍物方向）投影到安全区域。
    梯度指向d增大的方向 → 即远离最近障碍物的方向。
    """
    gx, gy = compute_distance_field_gradient(df)
    # 沿梯度方向移动，直到 d >= R_robot
    ...
```

**应用**：对A*路径做局部平滑——每个路径点沿 ∇d 方向微调，在不穿墙的前提下减小路径总长度。

**⚡ Optimize 4: app.py 模块化拆分**

当前 `app.py` (2,829行) 包含全部 Tab UI + 业务逻辑 + CSS。按功能拆分为：

```
smdsl_demo/
├── app.py              (~100行)  # Gradio 主入口，组装各 Tab
├── ui/
│   ├── __init__.py
│   ├── theme.py         (~200行)  # CSS / 主题 / 样式常量
│   ├── tab1_cad.py      (~500行)  # CAD 解析 Tab
│   ├── tab2_compile.py  (~500行)  # 语义编译 Tab
│   ├── tab3_validate.py (~500行)  # 物理验证 Tab
│   └── common.py        (~300行)  # 共享UI组件 (文件上传/进度条/警告横幅)
├── state.py             (~200行)  # gr.State 全局状态管理
└── ... (现有文件保留)
```

### 1.3 2D极致打磨：与前沿方法的对比实验设计

**对标方法**（行业内的2D避障路径规划方案）：

| 方法 | 代表实现 | 核心机制 | 优势 | 劣势 |
|:---|:---|:---|:---|:---|
| **Costmap + 膨胀层** | ROS Navigation Stack | 占据栅格 + 固定半径膨胀 | 工业部署最广、实时性好 | 膨胀层是二值的（安全/不安全），无连续距离信息 |
| **Potential Field** | classical, Khatib 1986 | 引力(目标)+斥力(障碍物)势场 | 连续梯度、数学优雅 | 局部极小值、窄通道振荡 |
| **CHOMP/STOMP** | MoveIt, 学术 | 基于梯度的轨迹优化 | 平滑连续轨迹 | 依赖好的初始猜测 |
| **Ours: EDT + STL + LLM闭环** | SMDSL | 连续距离场 + STL语义 + LLM修正 | 连续安全度量 + 可解释违反 + 自愈 | 静态场景、2D限制、LLM调用延迟 |

**对比实验设计**：

**实验1：路径安全性对比 (Path Safety Benchmark)**

在 FloorplanQA 的 4 个 room_type × 50 layouts = 200 个测试场景上：
- 对每个场景，在随机起终点之间规划路径
- 指标：路径最小 clearance (mm)、路径上 clearance < safety_threshold 的时间占比、碰撞次数

**假设**：EDT 距离场的连续安全度量使路径最小 clearance 显著优于 costmap 膨胀层（二值安全判定），因为 A* 的 1/distance 惩罚项引导路径远离墙体而非仅仅"不撞墙"。

**实验2：窄通道通过能力 (Narrow Passage Benchmark)**

在含有<1.5倍机器人宽度的窄通道的布局子集上：
- 指标：窄通道通过成功率、通过时最小 clearance

**假设**：EDT 的精确距离信息使 A* 能在窄通道中找到准确的居中路径（d_max 处），而膨胀层在窄通道中可能完全阻塞。

**实验3：LLM 闭环修正效果 (Closed-Loop Recovery Benchmark)**

- 对每条"陷阱指令"（故意穿墙的任务描述），追踪：ρ_initial → ρ_final、违规节点数变化、修正轮数
- 对比 baseline：无LLM修正（A* 重规划 with 默认D_safe）vs LLM修正（A* 重规划 with LLM调整后的D_safe）

**假设**：LLM修正后的D_safe能更精确地匹配具体任务的风险需求，从而在安全性和路径效率之间取得更好平衡。

---

## 方向二：迈向真正的三维空间智能与 Simulator 探索

### 2.0 技术前提：拥抱 GPU 加速

**放弃以下执念**：
- ❌ "不使用CUDA写PTX汇编"——这不是差异化优势，这是无意义的自我设限
- ❌ "类似Go的并发数据结构替代GPU"——CPU多核并行在3D体素计算的规模面前是杯水车薪

**采用务实策略**：
- ✅ 使用 NVIDIA Warp / Taichi / CUDA Python 进行 GPU 加速的3D计算
- ✅ 使用现有物理引擎（MuJoCo / PhysX / Isaac Sim）而非从零手写
- ✅ 关注算法创新（稀疏数据结构、增量更新、多尺度层次），而非硬件层面的优化竞赛

### 2.1 技术债务清偿路线图

#### (a) 从2D EDT到3D体素SDF —— 解决Z轴问题

**阶段1：3D EDT 替代 2D EDT（核心算法升级）**

当前：`scipy.ndimage.distance_transform_edt` → 2D数组 (H, W)

目标：3D EDT → 3D数组 (D, H, W)，其中 D 是 Z轴体素数

**技术方案**：

```python
# 阶段1a: 直接扩展到3D（仅扩展，不做优化）
def compute_distance_field_3d(grid_3d: np.ndarray) -> np.ndarray:
    """
    grid_3d: (D, H, W) uint8, 0=占据, 1=自由
    返回: (D, H, W) float32, 3D欧氏距离场
    """
    from scipy import ndimage
    return ndimage.distance_transform_edt(grid_3d.astype(bool)).astype(np.float32)

# 阶段1b: 使用稀疏数据结构（仅存储 "窄带" 区域）
# 参考: OpenVDB / fVDB — 工业级稀疏体素格式
# 对于建筑场景，绝大多数体素要么是自由空间(d很大)要么是墙体内部(d=0)
# 只有"窄带"（墙体表面 ± safety_distance）区域需要精确的d值
```

**阶段2：导航 vs 操作的双层架构**

对于实际应用，不需要全场景3D EDT。可以采用：

```
Layer 1 (2D快速路径 — 继承当前实现):
    用于移动底盘的楼层级导航
    保持 z=0 的假设
    性能: ~30ms/layout (已实现)

Layer 2 (3D精确路径 — 仅用于需要Z轴信息的区域):
    用于机械臂/末端执行器的操作空间
    仅在操作区域周围构建局部3D SDF
    性能: 按需计算，局部体素 ≈ 128³
```

**实施步骤**：

1. **Week 1-2**: 实现3D EDT的原型（Naive 3D ndimage调用），在单个room上验证正确性
2. **Week 3-4**: 引入稀疏数据结构——用 `scipy.sparse` 或集成 `openvdb` Python binding
3. **Week 5-6**: 修改 `_eval_distance_via_field` 支持3D采样——增加 `_bilinear_sample_3d` 函数（三线性插值）
4. **Week 7-8**: 更新 `_world_to_grid` 为3D版本，改造 TopologyBundle 格式

**关键决策：3D 数据来源**

当前2D pipeline的输入是建筑平面图（CAD floorplan）。要构建3D SDF，需要：
- 方案A: 从多层CAD图纸叠加构建（如果有逐层平面图）
- 方案B: 从3D扫描/点云构建（如 ScanNet 场景，ViewSuite 论文中使用的方式）
- 方案C: 从 World Labs Marble / NeRF / 3DGS 生成的3D场景构建

推荐从 **方案B**（ScanNet点云→3D栅格化→3D EDT）开始——这是学术界的标准路径。

#### (b) 动态障碍物处理 —— 从静态快照到增量更新

**阶段1：少量动态障碍物（P1，本月可完成）**

在2D场景中，用 Incremental EDT 支持≤5个移动障碍物：

```python
class DynamicDistanceField:
    """支持动态障碍物的增量距离场"""
    def __init__(self, static_grid: np.ndarray):
        self.static_df = compute_distance_field(static_grid)
        self.dynamic_objects: Dict[int, AABB] = {}  # id → 包围盒
        self.dirty_regions: List[AABB] = []

    def add_dynamic_obstacle(self, obj_id: int, bbox: AABB):
        """标记一个区域为"脏"，触发局部重算"""
        self.dynamic_objects[obj_id] = bbox
        self.dirty_regions.append(bbox)
        self._update_local_edt(bbox)

    def _update_local_edt(self, region: AABB):
        """仅重算受影响区域的EDT (Ryoo 2004 算法)"""
        # 1. 在region内重新计算占据栅格
        # 2. 用约束传播仅更新region内的距离值
        # 3. 复杂度 O(area(region)) 而非 O(H×W)

    def query(self, x_m: float, y_m: float) -> float:
        """透明查询——外部调用者感知不到静态/动态的区别"""
        # 先查static_df，如果落在dirty_region内则查增量结果
```

**阶段2：多动态障碍物 + GPU加速（P2，3个月内）**

当动态障碍物数量 >10 或场景尺寸 >2000×2000 时：
- 用 CUDA/Vulkan 实现并行 EDT 更新
- 参考 NVIDIA Warp 或 Taichi 的增量SDF示例

**阶段3：3D动态模拟器雏形（P3，6个月+）**

集成 MuJoCo 或 Isaac Sim 作为物理后端：
```
用户定义场景 (CAD/3D scan)
    → 静态3D SDF (our code)
    → + 动态刚体 (MuJoCo physics)
    → + 碰撞检测 (MuJoCo built-in)
    → + STL约束验证 (our code, 改造为3D)
    → 完整的前向模拟器 (s_t, a_t) → s_{t+1}
```

#### (c) 轨迹合成升级 —— 从线性插值到运动学约束

**阶段1：三次样条 + 梯形速度Profile（P1，本月可完成）**

替代当前 `path_pixels_to_trajectory` 中的均匀时间分配：

```python
from scipy.interpolate import CubicSpline

def path_to_smooth_trajectory(
    path_rc: List[Tuple[int, int]],
    transform: Dict[str, Any],
    total_time_s: float = 5.0,
    v_max: float = 1.0,
    a_max: float = 0.5,
    sample_dt: float = 0.05,
) -> List[Dict[str, float]]:
    """
    将离散A*路径转为运动学可行的连续轨迹。
    Step 1: 像素→世界坐标
    Step 2: 三次样条插值 → 平滑连续路径 r(s) = (x(s), y(s))
    Step 3: 梯形速度profile → s(t) 弧长参数化
    Step 4: 以 sample_dt 采样 → 输出轨迹点
    """
    # Step 1
    world_pts = [(ox + c*res, oy + r*res) for r, c in path_rc]

    # Step 2: 三次样条
    arc_lengths = compute_cumulative_arc_length(world_pts)
    cs_x = CubicSpline(arc_lengths, [p[0] for p in world_pts], bc_type='natural')
    cs_y = CubicSpline(arc_lengths, [p[1] for p in world_pts], bc_type='natural')

    # Step 3: 梯形速度profile
    total_length = arc_lengths[-1]
    s_params = trapezoidal_velocity_profile(total_length, total_time_s, v_max, a_max)

    # Step 4: 采样
    trajectory = []
    for i, s_norm in enumerate(s_params):
        s = s_norm * total_length
        t = i * sample_dt
        trajectory.append({
            "t": round(t, 3),
            "x": round(float(cs_x(s)), 4),
            "y": round(float(cs_y(s)), 4),
            "z": 0.0,
            "roll": 0.0, "pitch": 0.0, "yaw": 0.0,
        })
    return trajectory
```

**阶段2：最小 Jerk 轨迹（P2，3个月内）**

对于需要更高平滑度的操作任务，实现最小jerk轨迹（Flash & Hogan 1985）：

$$x(t) = x_0 + (x_0 - x_T)(-10\tau^3 + 15\tau^4 - 6\tau^5)$$

其中 $\tau = t/T$，该公式最小化 $\int_0^T \dddot{x}^2 dt$。

**阶段3：完整运动学约束（P3，6个月+）**

集成标准机器人运动学库（如 `pinocchio` 或 `pyroboplan`）：
- 关节限位检查
- 速度/加速度/加加速度限幅
- 奇异性回避
- 自碰撞检测

### 2.2 完整技术演进路线图

```
Phase 0 (本周): 安全修复
  ├── Z轴硬断言 ← 0.5h
  ├── RoboIR diff 硬校验 ← 2h
  └── DWG实体丢弃率告警 ← 2h

Phase 1 (本月): 2D极致打磨
  ├── 梯形速度profile ← 4h
  ├── Incremental EDT (局部动态更新) ← 8h
  ├── 距离场梯度计算 ← 3h
  ├── app.py 模块化拆分 ← 8h
  └── 对比实验: EDT vs Costmap → 产出量化证据 ← 16h

Phase 2 (3个月): 2.5D→3D 基础建设
  ├── 3D体素SDF原型 (Naive EDT) ← 16h
  ├── 稀疏体素数据结构 (OpenVDB) ← 16h
  ├── 三次样条轨迹插值 ← 4h
  ├── 双层架构 (2D导航层 + 3D操作层) ← 24h
  ├── 多动态障碍物GPU加速 (Taichi/Warp) ← 40h
  └── 首个3D STL鲁棒度验证demo ← 16h

Phase 3 (6个月+): 完整3D Simulator雏形
  ├── MuJoCo/Isaac Sim 物理后端集成 ← 80h
  ├── 3D碰撞几何体支持 (BoundingBox/Mesh/ConvexHull) ← 40h
  ├── 最小jerk轨迹优化 ← 16h
  ├── 完整 (s_t, a_t) → s_{t+1} 前向模拟 ← 80h
  └── 端到端: 自然语言 → 3D场景 → 物理模拟 → STL验证 → 闭环修正 ← 80h
```

### 2.3 推荐学习路径（边做边学）

| 阶段 | 学习资源 | 对应技能 |
|:---|:---|:---|
| Phase 0-1 | Maurer et al. (2003) "A Linear Time Algorithm for Computing Exact Euclidean Distance Transforms" | EDT算法原理 |
| Phase 1 | Lau et al. (2010) "A Fast $O(N)$ Parallel Algorithm for the Euclidean Distance Transform" | 增量/并行EDT |
| Phase 2 | Museth (2013) "VDB: High-Resolution Sparse Volumes with Dynamic Topology" | 稀疏体素数据 |
| Phase 2 | Flash & Hogan (1985) "The Coordination of Arm Movements" | 最小jerk原理 |
| Phase 3 | Todorov et al. (2012) "MuJoCo: A Physics Engine for Model-Based Control" | 物理引擎 |
| Phase 3 | Coumans & Bai (2016) "PyBullet Quickstart Guide" | 机器人仿真 |
| 全程 | Stanford CS231n / CS224n / CS234 / CS237 | 视觉/NLP/RL/机器人学基础 |

---

> **最后的话**：你的项目在它实际所处的赛道上（2D移动底盘的STL约束验证与闭环重规划）已经做出了有量化指标支撑的成果。接下来的任务不是"发明新范式"，而是**把现有的技术链条打磨到不可否认的工业级质量**——对比实验、量化指标、消融研究。这些才是简历上真正有说服力的东西。
>
> 当你有一天在简历上能写出"我的EDT+STL组合在200个布局上的最小clearance比ROS costmap_2d平均提升X%，在窄通道场景中通过率提升Y%"这样的句子时，你不需要说"最前沿"——数字本身就是最前沿。
