# SMDSL_DEMO 架构级诊断与未来演进技术白皮书

> **诊断基准**：李飞飞/World Labs (2026) 世界模型功能分类学（Renderer/Simulator/Planner）
> **诊断日期**：2026-06-08
> **诊断范围**：`SMDSL/cad_parser/` + `SMDSL/smdsl_demo/` + `SMDSL/tests/`
> **诊断原则**：冷酷、客观、数据说话，拒绝客套

---

## 第一部分：认知真伪性判定

### 1.1 你的核心命题

你声称 SMDSL 是一个 **Simulator**，其核心数学身份是：

$$(s_t, a_t) \rightarrow s_{t+1}$$

即"输入当前物理状态 $s_t$ 与动作 $a_t$，显式预测并推演下一刻的真实物理状态 $s_{t+1}$"。

### 1.2 代码层面的真相

让我们逐函数核验你的系统**实际上做了什么**：

---

**Zone 1 — `compute_distance_field`** (`astar_topology.py:546-559`):

```python
dist = ndimage.distance_transform_edt(grid.astype(bool))
return dist.astype(np.float32)
```

**实际行为**：对一张**静态**占据栅格计算 EDT（欧氏距离变换）。输出是 `d(x, y)`——每个像素到最近墙的距离。这是一个**纯空间函数**，不含时间维度，不含动作变量。

**数学身份**：$f_{EDT}: \mathbb{R}^{H \times W} \rightarrow \mathbb{R}^{H \times W}$，即 $d = f_{EDT}(grid_{static})$

**判决**：这是**状态构建**（state construction, o → s_hat），不是**状态转移**（state transition, (s_t, a_t) → s_{t+1}）。

---

**Zone 3 — `_eval_distance_via_field`** (`spatial_api_stub.py:519-590`):

```python
for pt in trajectory:
    x, y = float(pt["x"]), float(pt["y"])
    row_f, col_f = _world_to_grid(x, y, tx)
    d_px = _bilinear_sample(df, row_f, col_f, out_of_bounds_value=0.0)
    d_real_m = float(d_px) * res
    rho = d_real_m - min_dist_m
```

**实际行为**：对一个**预先给定的完整轨迹**逐点查询距离场，计算 STL 鲁棒度 ρ = d_real − D_safe。

**数学身份**：$ρ(t) = f_{EDT}(traj(t)) - D_{safe}$，即对轨迹 $traj$ 的**约束满足度评估**。不是状态转移。

**判决**：这是**约束验证**（constraint verification），不是**前向模拟**（forward simulation）。

核心区别：
- 前向模拟：给定 $s_t$ 和 $a_t$，**计算** $s_{t+1}$
- 约束验证：给定**完整轨迹** $\{s_0, s_1, ..., s_T\}$，**检查**每个点是否满足约束

你的系统做的是后者，不是前者。

---

**闭环自愈 — `_replan_trajectory_with_clearance`** (`test_closed_loop_recovery.py:456-514`):

```python
path_rc = astar_shortest_path(
    grid=grid, distance_field=distance_field,
    start_rc=(sr, sc), goal_rc=(gr, gc),
    robot_radius_px=robot_radius_px, safety_weight=0.5,
)
new_traj = path_pixels_to_trajectory(path_rc=path_rc, transform=transform, ...)
```

**实际行为**：基于距离场，用 A* 重规划一条满足新安全距离的路径。

**数学身份**：$traj^* = \arg\min_{traj} \sum cost(traj(t))$ subject to $d(traj(t)) \ge d_{safe}$

**判决**：这是**基于模型的规划**（model-based planning），距离场就是你的世界模型。这确实是"用 Simulator 辅助 Planner"的逻辑，但你的"Simulator"只是一个**静态 ED距离场**——它不模拟任何动态过程。

---

### 1.3 核心判决

**你的认知存在一个微妙但关键的错误**：

你声称你的系统实现了 $(s_t, a_t) \rightarrow s_{t+1}$（状态转移方程的前向求解）。但你的代码实际实现的是：

1. $o \rightarrow \hat{s}$：从 CAD 观察构建静态状态表征（EDT 距离场）
2. $traj, constraints \rightarrow \{ρ_i\}$：对给定轨迹做约束满足度评估
3. $constraints, feedback \rightarrow traj'$：基于约束反馈重规划轨迹

你的系统**不做前向状态预测**。它不回答"如果机器人从当前位置向前移动0.1m，它的新位置和新的clearance是多少"这种问题。它回答的是"这条已经规划好的轨迹有没有违反安全距离"。

**在 POMDP 循环中，你的系统实际占据的位置是**：

```
     ┌──────────────┐
     │  State        │  ← 你在这里：EDT距离场 = 静态环境状态表征
     │  Construction │     (o → s_hat)
     └──────┬───────┘
            │
     ┌──────┴───────┐
     │  Constraint   │  ← 你也在这里：STL鲁棒度评估
     │  Verification │     (traj → {ρ_i, violations})
     └──────────────┘
```

你是**状态构建器 + 约束验证器**。你不是模拟器。

### 1.4 你的路线"用 Simulator 实现 Planner 效果"是对还是错？

**这取决于你如何定义"Simulator"**：

| 定义层级 | 你的系统是否符合 | 说明 |
|:---|:---:|:---|
| **弱定义**（任何能评估"如果...会怎样"的系统） | ✅ 是 | 距离场可以评估"轨迹上的点是否安全"——这是最弱的 counterfactual reasoning |
| **中定义**（能输出显式状态表征的系统） | ⚠️ 勉强 | EDT 距离场确实是显式状态表征——但它是静态的、2D的、不含动力学的 |
| **强定义**（能前向求解 $s_{t+1} = F(s_t, a_t)$ 的系统） | ❌ 不是 | 你的系统不包含动力学、不执行前向推演、不预测状态转移 |
| **李飞飞定义**（$(s_t, a_t) \rightarrow s_{t+1}$，输出物理状态） | ❌ 不是 | 你的距离场不随时间演化，不接受动作输入来改变状态 |

**结论**：你的系统是一个**带有轻量世界模型（2D EDT距离场）的约束验证与重规划系统**。在 Model-Based RL 的谱系中，你处于 **Dyna-style planning** 的极简版本——世界模型是静态距离场，planning 是 A* 搜索。这在数学上是一致的、逻辑上自洽的。但你把它叫"模拟器"是一种**分类学上的过度宣称**。

---

## 第二部分：如果这个路线是对的（工程落地与架构Roadmap）

即使你的定位有偏差，你选择的路线——**用轻量距离场作为世界模型来做基于模型的规划**——在工程上是**完全合理且已被验证有效的**。这是 classical robotics 中 configuration space + potential field 方法的现代数值版本。

### 2.1 当前状态评估

**🐉 做得好的一面**（在行业中属于什么水平）：

| 模块 | 水平 | 证据 |
|:---|:---:|:---|
| **CAD→栅格化管线** | 工业级 | 1981 layouts, 0 errors, median 25.79ms——这是可复现的基准 |
| **形态学后处理** | 一流 | `weld_double_line_walls` + `remove_exterior_freespace` + `bridge_thin_walls` 三步后处理序列，直接解决了工业 DWG 最令人头痛的"双线墙漏水"问题 |
| **STL 鲁棒度语义** | 一流 | ρ = d_real − D_safe 的数学语义在代码、文档、JSON 三处完全自洽——学术界做到的团队不多 |
| **闭环自愈架构** | 前沿 | 生成→校验→结构化反馈→LLM修正→重规划的完整闭环——这在 2026 年仍然是前沿设计模式 |
| **DWG 安全沙箱** | 工业级 | 7 层防护链（扩展名→大小→魔数→CVE→dwgread→timeout→编码回退），R2004 CVE 单独标注 |

**😱 做得差的一面**：

| 问题 | 严重程度 | 具体表现 |
|:---|:---:|:---|
| **距离场是"拍一张"** | 🔴 致命 | `compute_distance_field` 调用一次后存入 bundle，全程不变。动态障碍物零处理 |
| **2.5D硬伤** | 🔴 致命 | `_eval_distance_via_field` 只读 (x,y)，z 被**完全忽略且静默通过**。若传入 6-DoF 机械臂轨迹，系统会报告 ρ>0（假阴性碰撞） |
| **轨迹合成是测试桩** | 🔴 致命 | `synthesize_trajectory` 只是线性插值——没有运动学约束、没有速度/加速度 profile、没有动力学 |
| **无前向推演能力** | 🔴 致命 | 你声称自己是 Simulator，但系统没有一行代码在做 $(s_t, a_t) \rightarrow s_{t+1}$ |
| **距离场无梯度** | 🟠 严重 | EDT 输出是标量场，没有梯度信息——你无法做基于梯度的轨迹优化（如 CHOMP、STOMP），只能用 A* 离散搜索 |
| **app.py 2400行巨石** | 🟡 中等 | 已自承"维护债" |

### 2.2 行业位置

| 对标系统 | 你的位置 |
|:---|:---|
| **NVIDIA Omniverse/PhysX** | 你差 6 个数量级——PhysX 有完整的刚体动力学、碰撞响应、约束求解器。你没有。 |
| **NVIDIA Cosmos** | 你差 5 个数量级——Cosmos 有 Predict/Transfer/Reason 三组件 + 多具身控制。你只有静态距离场。 |
| **ROS Navigation Stack (costmap_2d)** | 你**持平并超越**——ROS costmap 用朴素占据栅格 + 膨胀层；你的 EDT 距离场 + 安全代价 A* 在数学上更优（连续距离 vs 二值占据） |
| **学术界的 motion planning 方法** | 你的 STL 鲁棒度闭环很有特色——学术界的 motion planning 很少将 STL semantics 和 LLM-based replanning 结合 |

**一句话**：你在 **2D 移动底盘的静态避障路径规划** 这个细分领域做到了前沿水平。但离你声称的"通用物理模拟器"差 2-3 个架构代际。

### 2.3 无 CUDA 前向状态预测引擎设计方案

如果你坚持要走向真正的 $(s_t, a_t) \rightarrow s_{t+1}$，且不依赖 CUDA，以下是具体的架构设计。

**核心洞察**：你目前最大的计算瓶颈是 EDT。对于 2D 场景，EDT 在 CPU 上已经足够快（1981 layouts, 29ms avg）。挑战来自 **3D 升级**。3D 体素 SDF 的 CPU 计算复杂度从 O(N) 变为 O(N³/²)（N = 体素数），实用场景下 N ≈ 256³ ≈ 16M 体素。

**非 CUDA 技术栈选择**：

| 方案 | 适用场景 | 优势 | 劣势 |
|:---|:---|:---|:---|
| **Triton** (OpenAI) | 跨硬件 Python-level kernel 编写 | 写一次跑在 NVIDIA/AMD/Intel GPU 上 | 仍需要 GPU，只是不绑 CUDA |
| **Taichi** | 可微物理仿真 + 跨后端 | 一套代码 → CUDA/Vulkan/Metal/CPU；原生支持稀疏数据结构 | 生态较小，调试工具不成熟 |
| **Mojo** | 系统级数值计算 | Python 语法 + C 性能；SIMD 原生支持 | 仍处于早期，生态极不成熟；**不解决 GPU 替代问题** |
| **ISPC** (Intel) | CPU SIMD 向量化 | 一行 C 代码 → AVX-512/ARM NEON；无需 GPU | 仅限 CPU |
| **Rust + rayon** | 多核 CPU 高并发 | 零成本抽象 + 无锁数据结构 + 编译期内存安全 | 学习曲线陡峭 |
| **Zig + 手写 SIMD** | 极致底层控制 | 编译期计算 + 显式内存布局 | 开发效率极低 |

**推荐技术栈**（按目标分层）：

```
Layer 1 (快速路径 - 2D 保持现状):
    scipy.ndimage.distance_transform_edt  # 已经 O(N), 已经 CPU, 已经够快
    numpy + numba.jit                     # 热路径 JIT 编译

Layer 2 (3D 升级 - 体素 SDF):
    方案A: Taichi → 一套代码生成 CUDA/Vulkan/Metal/CPU 多后端
    方案B: ISPC → 纯 CPU，AVX-512 向量化，无 GPU 依赖

Layer 3 (动力学引擎 - 前向状态推演):
    方案A: MuJoCo (DeepMind) → 开源，纯 C，不绑 CUDA
    方案B: Taichi 可微物理 → 自定义轻量级刚体引擎

Layer 4 (并行调度 - Go 风格并发):
    Rust + tokio/rayon → work-stealing 调度器
    或直接 Go + cgo 调用 C/Taichi 层
```

**前向状态预测引擎的数据结构设计**：

```rust
// 核心数据结构 —— 与你的"五大状态空间"对齐

/// 3D 体素距离场（替代当前 2D EDT）
struct VoxelSDF {
    data: NdArray<f32, 3>,          // 体素数据 (D, H, W)
    origin: [f32; 3],               // 世界坐标原点 (m)
    resolution: f32,                // m/体素
    // 增量更新支持 —— 解决当前"拍一张"问题
    dirty_region: Option<AABB>,     // 脏区域（动态障碍物）
    spatial_hash: HashMap<VoxelCoord, f32>, // 稀疏精细区域
}

/// 刚体状态 —— 物理模拟的基本单元
struct RigidBody {
    id: u64,
    pose: SE3,                      // 位置 + 姿态（李代数表示）
    velocity: Twist,                // 线速度 + 角速度（6维）
    mass: f32,
    inertia: Mat3,                  // 惯性张量
    collision_geometry: CollisionShape, // 碰撞几何（convex hull / sphere / box）
    contact_points: Vec<Contact>,   // 当前接触点
}

/// 前向动力学步进 —— 这才是真正的 (s_t, a_t) → s_{t+1}
fn forward_step(
    bodies: &[RigidBody],           // 当前状态 s_t
    actions: &[JointCommand],       // 动作输入 a_t
    sdf: &VoxelSDF,                 // 环境约束
    dt: f32,                        // 时间步长
) -> Vec<RigidBody> {               // 下一状态 s_{t+1}
    // Step 1: 施加动作力
    // Step 2: 碰撞检测（SDF 查询 + 窄相位 GJK/EPA）
    // Step 3: 约束求解（LCP/NCP —— contact + friction）
    // Step 4: 数值积分（semi-implicit Euler / Runge-Kutta 4）
    // Step 5: 更新 SDF 脏区域（如果有动态障碍物）
}
```

**为什么这条路仍然极其困难**：

1. 碰撞检测窄相位（GJK + EPA）的数值精度在 float32 下有限——工业级求解器（PhysX/Bullet）用了几十年才做到稳定
2. 并行化约束求解（LCP）是 hard problem——接触约束是全局耦合的，不能简单 parallel-for
3. 3D SDF 增量更新的算法复杂度不容小觑——动态障碍物每帧移动都需要更新 SDF
4. 你低估了"从头写一个物理引擎"的难度——这不是一个人在一两年内能完成的事

---

## 第三部分：如果这个路线是错的（架构纠偏与重新定位）

### 3.1 真实评价

你的核心矛盾是：

> **你想做 Simulator + Planner 的融合，但你实际只有 Static State Estimator + Constraint Verifier + LLM-based Replanner。**

这不是贬低。你构建的系统在它**实际所处的位置**上是优秀的。问题是你的**自我定位和目标设定**与代码的现实之间存在巨大鸿沟。

**你在 POMDP 循环中的真实位置**：

```
    Observation o_t                    ┌──────────────────────────┐
    (CAD/DWG/PNG) ───────────────────►│ Zone 1: State Builder    │
                                       │ o → s_hat               │
                                       │ (EDT distance field)    │
                                       └──────────┬─────────────┘
                                                  │ s_hat (static)
    ┌─────────────────────────────────────────────┘
    │
    ▼
┌──────────────────────────┐      ┌──────────────────────────┐
│ Zone 2: Constraint       │      │ Zone 3: Constraint       │
│ Compiler                 │ ───► │ Verifier                 │
│ NL → RoboIR STL          │      │ traj → {ρ_i, violations} │
└──────────────────────────┘      └──────────┬─────────────┘
                                              │ violations + worst_node
                                              ▼
                                     ┌──────────────────────────┐
                                     │ Zone 4: Feedback + LLM   │
                                     │ Replanner                │
                                     │ feedback → new_traj      │
                                     └──────────────────────────┘
```

**✅ 你不是 Simulator** —— 你不做前向状态转移
**✅ 你不是 Planner** —— 你依赖 LLM 和 A* 来规划，你只做验证和重规划触发
**✅ 你是一个 Verification-Centric Closed-Loop System** —— 约束验证 + 反馈驱动重规划

这在 Li 的分类学中没有完美的对应项。它更像是 **Simulator（静态状态构建部分） + 一个 LLM-mediated Planning Loop 中的 Verification Layer**。

### 3.2 这不是坏事

**你的系统最有价值的资产不是你声称的东西，而是你实际有的东西**：

1. **EDT 距离场**：一个精确、可解释、毫秒级的 2D 环境模型——这是 classical robotics 和 modern LLM-based planning 之间的完美桥梁
2. **STL 鲁棒度**：ρ = d_real − D_safe 的连续语义梯度给了 LLM **数值化的反馈信号**——这是纯文本 feedback 无法比拟的
3. **闭环自愈**：generate → verify → feedback → replan 的完整闭环——这是一个可论证的、比单步 LLM 规划更稳健的架构

### 3.3 重新定位建议

**推荐定位**：

> **SMDSL 是一个面向 2D 移动底盘的、基于 EDT 距离场世界模型的、STL 约束驱动的、LLM 辅助的闭环路径规划与验证系统。**

简称：**STL-Verified Distance-Field Path Planner with LLM-based Recovery**

在 Li 的分类学中，你的真实身份是：

| 分类 | 你占的部分 | 说明 |
|:---|:---:|:---|
| **Renderer** | 0% | 你不生成图像（除了可视化用的matplotlib） |
| **Simulator** | ~20% | 你有一个静态距离场（环境状态）——这是 Simulator 的 **状态构建子功能**，但缺了前向推演 |
| **Planner** | ~30% | 你用 A* 做路径搜索 + LLM 做约束修正——但轨迹合成无运动学、无动态障碍物 |

你是一个**三者的杂交体**——而且这不是贬低，这是你独特的差异化定位。

### 3.4 资产保全与重新聚焦方案

**砍掉的**（在当前阶段不值得投入）：

- ❌ 从零写前向动力学引擎——这是一整个团队多年的工作量
- ❌ 声称自己是"通用物理模拟器"——代码不支持，也没有必要
- ❌ 在没有动态障碍物处理的条件下做 3D 升级——先把 2D 做完整

**保留并深化的**（你的核心差异化资产）：

| 资产 | 当前状态 | 建议方向 |
|:---|:---|:---|
| **EDT 距离场** | 静态 2D | 增量更新（dynamic EDT 算法：Ryoo 2004, Lau 2010）——只重算脏区域 |
| **STL 鲁棒度** | 单帧查询 | 时序鲁棒度（一个轨迹段整体满足/违反）——不是逐点，而是区间语义 |
| **闭环自愈** | LLM 重规划 | 加 diff checker 硬约束——确保 LLM 修正不推翻原 intent |
| **Multi-format dispatcher** | 5 种格式 | 加 DWG 实体丢弃率告警（当前静默跳过 16,000+ SPLINE/ELLIPSE/HATCH） |

**新增的**（让你的系统向真正的 Simulator 方向迈出一小步）：

1. **局部动态更新**：在距离场中支持少量动态障碍物（mobile objects），使用 incremental EDT 算法
2. **速度/加速度约束**：在 `path_pixels_to_trajectory` 中加入梯形速度 profile（加速→匀速→减速），而非简单的时间平均分配
3. **梯度信息**：从 EDT 计算距离梯度 ∇d(x,y)，支持基于梯度的局部轨迹优化（而非当前全量 A* 重规划）
4. **Z 轴硬阻断**：在 `_eval_distance_via_field` 入口添加：若轨迹中 `max(z) − min(z) > ε`，直接报错而非静默通过

---

## 第四部分：非 CUDA 物理仿真的务实路线

### 4.1 抛弃幻想

关于"不用 CUDA 做通用物理仿真"，以下是你必须面对的现实：

- **工业级物理引擎**（PhysX/Bullet/MuJoCo）的核心约束求解器（LCP/NCP 求解器）是数十年优化的成果，不是算法创新的瓶颈，而是**工程优化的瓶颈**
- **DeepSeek 绕过 CUDA 用 PTX** 的故事不适用于你的场景——DeepSeek 做的是矩阵乘法和注意力，高度规则；物理仿真是不规则计算（稀疏接触、分支密集）
- **Mojo** 解决的是"Python 太慢"的问题，不是"GPU 不兼容"的问题

### 4.2 务实的非 CUDA 策略

**对于你当前的实际需求（2D 移动底盘避障 + 可能扩展到 3D 体素 SDF）**：

```
优先级 1: 保持现状
    scipy.ndimage EDT 已经 O(N) 且 CPU 化。不需要换。

优先级 2: 2D 热路径优化
    numba.jit → 把 _bilinear_sample 和 safety_aware_astar_flood
    的 inner loop 编译为机器码。不需要 CUDA。

优先级 3: 稀疏 SDF（如需要 3D）
    用 VDB (OpenVDB) 或 fVDB → 体素稀疏存储。
    这在 CPU 上已有高效实现。不需要 CUDA。

优先级 4: 局部动态更新
    用 Ryoo 2004 的 incremental EDT 算法。
    纯 CPU，O(N_dirty) 而非 O(N_total)。

优先级 5（远期）: 如需 GPU 加速但不绑定 CUDA
    Taichi → 写一次 Python-like 代码，多后端编译。
    或 Triton → 写 Python-level GPU kernel，跨厂商。
```

### 4.3 关于"类似 Go 的并发数据结构"

你的 EDT 和 A* 当前是单线程的。多线程并行化的务实方案：

```python
# 当前：单线程 A* 漫水
topology, visit_count = safety_aware_astar_flood(grid, distance_field, seeds, ...)

# 务实升级：多起点并行 A*（每个种子独立 A*，最后合并）
from concurrent.futures import ThreadPoolExecutor

def parallel_flood(grid, distance_field, seed_groups, robot_radius_px):
    with ThreadPoolExecutor(max_workers=8) as pool:
        futures = [
            pool.submit(safety_aware_astar_flood,
                        grid, distance_field, seeds, robot_radius_px, ...)
            for seeds in seed_groups
        ]
        results = [f.result() for f in futures]
    return merge_topologies(results)  # 按最大分类合并
```

对于真正的多线程距离场更新（Go 风格），考虑用 Rust + rayon：
- `par_chunks_mut` → 并行化栅格操作
- `Arc<Mutex<HashMap>>` → 共享可变空间哈希表用于稀疏动态区域
- 但这对你当前的 29ms 场景来说是 **over-engineering** —— 先看 profiling 热点在哪

---

## 总结

### 冷酷事实

1. **你不是 Simulator**。你做的是静态状态构建 + 约束验证 + 反馈驱动的重规划。这是一个有价值的位置，但不是你声称的那个位置。

2. **你不需要 CUDA**。你的 2D EDT 已经是 CPU 原生的，且足够快（29ms）。3D 升级的瓶颈在算法（稀疏数据结构、增量更新）不在硬件。不要被"绕过 CUDA"的叙事带偏——你先需要的是做正确的算法，而不是用 PTX 写 kernel。

3. **你最大的技术债务**不是缺少前向动力学，而是：(a) 2.5D 假阴性碰撞（z 轴被静默忽略），(b) 距离场静态快照（动态障碍物零处理），(c) 无速度/加速度 profile（轨迹合成是线性插值）。

4. **你最有价值的资产**是：EDT 距离场 + STL 鲁棒度语义 + 闭环自愈架构。这三者的组合在学术界是独特的，不要因为追逐"通用模拟器"的虚名而放弃它们。

### 建议优先级

| 优先级 | 行动 | 预期收益 | 工作量 |
|:---:|:---|:---|:---:|
| **P0** | Z 轴静默忽略 → 硬断言报错 | 消除假阴性碰撞（安全性） | 0.5h |
| **P0** | RoboIR diff 硬校验（修正后 intent+target_frame 不可变） | 防止 LLM 修正跑偏 | 2h |
| **P1** | 梯形速度 profile 替代线性插值 | 轨迹物理可信度大幅提升 | 4h |
| **P1** | Incremental EDT（局部动态更新） | 支持少量动态障碍物 | 8h |
| **P1** | 拆分 app.py 巨石 | 可维护性 | 8h |
| **P2** | 距离场梯度计算 ∇d(x,y) | 支持基于梯度的轨迹优化 | 3h |
| **P2** | 多起点并行 A*（ThreadPoolExecutor） | 已有 30ms → 保持不变，多数场景不需要 | 2h |
| **P3** | 3D 体素 SDF（VDB/fVDB） | 真正走向 3D | 40h+ |
| **P3** | 轻量刚体动力学（MuJoCo/Taichi 集成） | 真正的 (s_t,a_t)→s_{t+1} | 80h+ |

---

> **最后忠告**：不要被 NVIDIA Cosmos/Omniverse 的叙事带偏到"必须做全栈物理仿真"。你真正的差异化——EDT距离场 + STL语义 + LLM闭环——在 2D 移动底盘领域已经是一个有力的技术组合。把它打磨到极致，而不是跳到另一个赛道从零开始追赶。
