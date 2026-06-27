# SMDSL 学术发表战略方案 v1

> 版本：v1 · 日期：2026-06-16
> 基于：战略评估报告 + 深度验证报告 + 决策者补充意见

---

## 一、战略定位

### 1.1 核心赛道（重新定义）

这篇论文**不是**纯机器人论文。它的核心赛道是：

> **形式化验证（Formal Methods）与大模型安全（LLM Safety）在空间规划中的交叉应用。**

这意味着：
- 不拼硬件、不拼底层控制理论
- 拼的是：用严谨的数学工具（STL）去修补具身智能领域最紧迫的漏洞（缺乏可验证的安全指标）
- 目标读者：不仅做机器人的，更包括做 AI Safety、形式化方法、可信 AI 的审稿人

### 1.2 核心贡献陈述

> **首次将连续距离场（EDT）与信号时序逻辑（STL）结合为统一的验证框架，在窄通道场景中相对于 ROS costmap 膨胀层提供统计显著的连续安全裕度提升，并填补了"空间几何表示 → 时序逻辑监控"之间的工具链空白。**

### 1.3 框架图命名

SMDSL 的学术品牌名建议沿用 **SMDSL**（Spatial-Motion Domain-Specific Language），但在论文中可以强调全称的面向——**Spatial-Motion Verification Framework**，突出"验证"（Verification）而非"语言"（Language），更符合论文定位。

---

## 二、战略决策（已确认）

| 决策项 | 选择 | 理由 |
|--------|------|------|
| **目标会议** | RA-L + IROS 2027（滚动截稿） | 审稿快（1-2 个月），更看重新颖方法和严谨性，不要求硬件验证。ICRA 竞争过于激烈且偏爱硬件 |
| **Baseline 策略** | 经典规划层（A*/Costmap）+ 约束解码（SafeDec/STLCG++）双轨 | 传统规划层做兜底对比，约束解码线证明"LLM 安全"这个核心叙事 |
| **硬件验证** | **不做** | 10,000+ 大规模程序化仿真 + 统计检验的力量远超低成本硬件验证。低成本硬件的误差反而稀释算法的严谨性 |
| **Diff SpaTiaL** | 正面区分 | 在 Related Work 中主动提及。SMDSL：离散轻量级后验证（post-validation），适合实时/资源受限边缘场景；Diff SpaTiaL：可微连续优化，计算复杂度高 |
| **实验室定位** | 独立研究能力展示 | 本项目作为个人"Masterpiece"作品，展示：发现学术真空、设计实验、完成完整框架验证的独立科研能力。适合作为申请港科广红鸟挑战班等顶尖研究型项目的敲门砖 |

---

## 三、论文核心叙事

### 3.1 八股结构

**问题**：
> 当前 VLA/LLM 输出的机器人规划缺乏可验证的物理安全性。——大量热门 baseline 都是 "Let me do"。这个数据从如下数据展示如何底层的不可靠。

**关键数据**（来自战略报告的安全基准）：
> - GPT-4 原始安全率仅 37.6%
> - Llama2-7b 原始安全率仅 3.0%
> - 990 篇 SafeBench 论文中，98% 停留在纯成功率（Success Rate）评估，无一提供连续安全度保障

**现有方案的不足**：
> - 传统方法（Costmap 膨胀层）：二值判定，无法提供连续安全梯度。求窄通道中堵塞通道；在窄通道中直接堵死通道，宽通道中贴边行走
> - 纯约束解码基线（SafeDec/T3 Planner）：工作在 logits 空间上，依赖器模型动力学模型进行黑箱干预，缺乏精确的底层几何反馈，对于几何空间的理解不够
> - 纯 STL 基线（STLCG++）：使用正确但缺乏完备的底层空间表示

**本文方法**：
> SMDSL：提出了一种双层的安全验证验证框架：
> - 底层：利用连续 EDT 获得光滑的距离场
> - 上层：将距离场编码至 STL 时序逻辑中作为统计反馈
> - 提出了一种双层的安全架构：底层（连续距离场安全性） + 上层（STL监控 + LLM修正）

**实验结果**：
> 基于程序生成的 10,000 个窄通道地图：
> - EDT+STL 相对于 Costmap+A* 的平均最小安全距离提升 xxx 毫米（p<0.001）
> - 窄通道通过率提升：EDT xxx% vs Costmap xxx%（p<0.001）
> - 传统经典方法极窄通道碰撞率：14.8% → SMDSL（仿真验证）碰撞率接近于 0
> - 对比基于 LLM 直接规划的安全率（37.6%）提升至 95%+（当结合安全距离场约束时）  

### 3.2 数据呈现策略

不追求 "真实机器人" 视频，通过以下几点更严谨地呈现：

| 呈现形式 | 具体方法 | 可产出的灵感/参考 |
|---------|---------|------------------|
| 统计图表 | 箱线图、配对 t-test、Mann-Whitney U | clearance_min / clearance_mean 分布对比 |
| 热力图 | seaborn 生成安全梯度热力图叠加在迷宫地图上 | 视觉冲击力强，学术严谨感十足 |
| 方法框架图 | 极简几何图形，突出"安全漏斗"（EDT+STL 过滤器） | 左侧 LLM 轨迹 → 中间安全漏斗 → 右侧安全纠正轨迹 |
| 统计学显著性 | 大规模 10,000+ 独立实验 | 样本库不是多就好，是需要少而精亮，高质量的证明比海量垃圾更有说服力 |

---

## 四、分阶段执行计划

### Phase 0：代码库整理（~3 天）

| 任务 | 内容 | 产出 |
|------|------|------|
| P0.1 | 修复 benchmark 路径分裂问题 | 统一的 `SMDSL/benchmark/` |
| P0.2 | 确认 v3 代码库状态稳定 | 全量 63 测试通过 |
| P0.3 | 论文专用目录结构搭建 | `papers/` 目录 + 实验脚本组织 |
| P0.4 | RA-L 格式模板准备 | IEEE 模板 + 页面限制确认 |

### Phase A1：论文实验（6-8 周）

#### A1.0：窄通道程序化地图生成器（Week 1，核心基建）

**文件**：新建 `benchmark/procedural_maps.py`

```python
def generate_narrow_passage_map(
    width: int, height: int,
    min_corridor_width: float,  # 最窄处宽度（米/像素）
    max_corridor_width: float,  # 最宽处宽度
    noise_level: float,         # 障碍物随机噪声
    n_obstacles: int,           # 额外随机障碍物数
) -> tuple[np.ndarray, dict]:
    """
    程序化生成含狭窄通道的测试地图。
    采用 Perlin 噪声 + 形态学操作生成自然的走廊结构。
    返回: (grid, metadata)
    metadata 包含:
      - narrowest_width: 最窄通道宽度（像素）
      - narrow_region_centers: 窄通道区域中心坐标列表
      - difficulty_score: 基于通道宽度/长度/曲率的综合难度评分
    """

def generate_batch(
    n_maps: int, output_dir: str,
    difficulty: str = "mixed",  # "easy" | "medium" | "hard" | "mixed"
    seed: int = 42,
) -> list[dict]:
    """
    批量生成地图，为大规模统计实验做准备。
    按难度分层采样，确保每层级有足够样本。
    """
```

**要求**：
- 可复现（固定 seed）
- 难度可量化（通道宽度、曲率、噪声水平）
- 输出格式兼容现有 benchmark 框架（与 `dispatch_cad` 的 grid 格式一致）
- 元数据完整（每条地图记录难度标签、实际最小通道宽度等）

#### A1.1：EDT vs Costmap 大规模对比（Week 2-3）

**文件**：`benchmark/experiment_setup.py` + 现有 `bench_edt_vs_costmap.py` 的增强

**实验设计**：

```
数据集：
  - 程序化生成: 9,000 张地图（easy/medium/hard 各 3,000）
  - FloorplanQA: 1,000 张精选布局（作为真实世界泛化验证）
  起终点对: 每张地图随机生成 3 对（一易一难）
  总计实验: 10,000 × 3 = 30,000 次运行 × 2 方法

指标：
  主要指标:
    - clearance_min (mm) — 路径上到障碍物的最小距离
    - ρ_min = clearance_min - D_safe (mm) — 最小 STL 鲁棒度
    - collision_rate (%) — 任意点 clearance < robot_radius 的比例
  次要指标:
    - clearance_mean (mm)
    - path_length (m)
    - planning_time (ms)
    - narrow_passage_success_rate (%) — 在窄通道成功通过的比率

统计方法：
  - 配对 t-test（同一起终点对下 EDT vs Costmap）
  - Mann-Whitney U test（非参数分布下的比较）
  - Cohen's d（效应量）
  - 按 difficulty 分层的 subgroup analysis
```

**输出**：
- 详细表格（按难度层级分组）
- 箱线图（clearance_min 分布）
- 显著性标注（*** p<0.001, ** p<0.01, * p<0.05）
- 所有原始数据保存为 JSON（可复现）

#### A1.2：扩展 Baseline 对比（Week 3-4）

**新增基线**：

| Baseline | 实现方式 | 用途 |
|----------|---------|------|
| A* + Costmap | 已有 | 传统规划层的代表 |
| PRM + Costmap | 新建（OMPL 风格） | 采样规划的代表 |
| STL-only（无 EDT） | 已有 `check_stl_constraint_violation` 但只用 trajectory 采样 | 消融：去掉底层距离场 |
| EDT-only（无 STL） | A* + safety_weight 但无时序约束 | 消融：去掉上层逻辑 |
| SafeDec 风格 | `constrain_decoding.py` 轻量 clone | 约束解码的代表 |
| LTL baseline | SELP 风格简化版 | 时序逻辑的代表 |

**统计目标**：
> 在 10,000 地图 × 6 方法 = 60,000 次运行后，输出汇总对比表

#### A1.3：消融实验（Week 4-5）

| 消融维度 | 参数范围 | 测量指标 |
|---------|---------|---------|
| 网格分辨率 | 0.02/0.05/0.10/0.20 m/px | clearance 精度 vs 计算时间 |
| 安全权重 | 0.1/0.3/0.5/0.7/1.0 | 路径保守度 vs 长度 |
| 机器人半径 | 0.10/0.15/0.20/0.30 m | 半径敏感度 |
| 窄通道宽度比 | 1.0×/1.2×/1.5×/2.0× robot_radius | 通不过临界点 |

#### A1.4：LLM 闭合回路实验（Week 5-6）

**文件**：`benchmark/experiment_closed_loop.py`（基于 `test_closed_loop_recovery.py` 扩展）

```
场景：故意给 LLM 不安全的指令（"穿墙"、"走最短路径不管碰撞"）
指标：
  - 初始安全率（%）：LLM 原始输出中安全的比例
  - 修正后安全率（%）：经过 EDT+STL 验证+LLM 修正后
  - 修正轮数分布
  - 成功率 vs 安全率的权衡曲线
```

#### A1.5：可视化 + 统计（Week 6-7）

```python
# 统一的可视化工具
def generate_paper_figures(results_db: str, output_dir: str):
    """
    自动生成论文所需的所有图表：
    1. Figure 1: 方法框架图（输出 TikZ/PDF）
    2. Figure 2: 对比箱线图
    3. Figure 3: 消融实验折线图（参数敏感性）
    4. Figure 4: 热力图案例（EDT vs Costmap 并排对比）
    5. Table 1: 总体对比表
    6. Table 2: 分层（按难度）对比表
    7. Table 3: 消融结果表
    """
```

**高质量可视化要求**：
- 所有图表使用统一的配色方案（Nature/Science 风格）
- 字体大小可读（不要小于 8pt）
- 箱线图带有统计显著性标注
- 热力图使用 perceptually uniform colormaps（viridis/mako）
- SVG/PDF 矢量格式输出

#### A1.6：论文初稿 + arXiv preprint（Week 7-8）

基于 A1.1-1.5 的全部数据，撰写论文初稿并挂 arXiv。

---

### Phase A2：论文投稿（4 周）

| 任务 | 内容 | 产出 |
|------|------|------|
| A2.1 | Introduction + Related Work | 定稿引言（800 字以内）+ 相关工作的定位 |
| A2.2 | Method | 方法框架图 + EDT/STL/LLM 回路描述 |
| A2.3 | Experiments | 全部表格 + 图表 + 统计检验 |
| A2.4 | 全稿内部审查 | 一致性检查 + 语言润色 |
| A2.5 | 挂 arXiv + 提交 RA-L | 跟踪号 + 初审 |

---

### Phase A3：审稿响应（接收后）

| 任务 | 内容 |
|------|------|
| A3.1 | 收到审稿意见后准备 rebuttal |
| A3.2 | 开源代码正式发布（论文接收后） |
| A3.3 | 根据审稿意见补充实验（如有需要） |

---

## 五、风险矩阵

| 风险 | 概率 | 影响 | 缓解措施 |
|------|------|------|---------|
| 被 scoop（其他组同时发表 EDT+STL） | 中 | 高 | ASAP 挂 arXiv 抢占时间戳；每周检查 Semantic Scholar + arXiv |
| 审稿人质疑无硬件验证 | 中 | 中 | 10,000+ 统计实验的严谨性可对冲；RA-L 本身对硬件要求低于 ICRA |
| 审稿人质疑没有真实机器人背景 | 低 | 中 | 将论文定位为 Formal Methods + AI Safety 而非纯机器人；审稿人可能是 CS/AI 背景 |
| 实验结果不够显著 | 低 | 高 | 在极端窄通道场景上做压力测试放大差异；如果差异小，可以增强样本量直到统计显著 |
| RA-L 拒稿 | 低-中 | 低 | 转到 IEEE Access（引用分 3.4+）或 IROS 2027 继续投 |

---

## 六、时间线总览

```
Week 1-2     ██░░░░░░░░░░░░  A1.0 程序化生成器 + A1.1 基准实验
Week 3-4     ████░░░░░░░░░░  A1.2 扩展 Baseline + A1.3 消融
Week 5-6     ██████░░░░░░░░  A1.4 闭合回路 + A1.5 可视化
Week 7-8     ████████░░░░░░  A1.6 论文初稿 + arXiv
Week 9-10    ██████████░░░░  A2.1-2.3 论文完整化
Week 11-12   ████████████░░  A2.4-2.5 提交 RA-L
Week 13+     ██████████████  Rebuffal / 补充实验 / 代码开源
```

---

## 七、命名约定

| 用途 | 名称 |
|------|------|
| 论文框架名 | SMDSL — Spatial-Motion Domain-Specific Language |
| 会议目标 | RA-L + IROS 2027 |
| 实验目录 | `papers/ral2027/` |
| 数据目录 | `benchmark/results/paper_experiments/` |

---

## 八、待讨论的细节问题

1. **作者排序**：你是一作 + 通讯？是否需要挂 BHE 实验室导师作为合作者？
2. **arXiv 账号**：是否有 arXiv 的认可作者身份（endorsement）？
3. **注册费**：RA-L 作为 IEEE 期刊，有无版面费？IROS 2027 的注册费和差旅预算？
4. **共同作者**：需要邀请外部合作者（比如有机器人背景的人）挂名增加可信度吗？
5. **RA-L 投稿经验**：是否需要先投一篇短文（RA-L 允许 6 页 + 1 页引用）积累经验？
