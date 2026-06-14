# SMDSL Demo — 自然语言 → 空间-动作中间表示

> 模块化解耦：VLM 仅输出 DSL，绝不直接输出底层机器人动作。

## 架构

```
自然语言指令
    │
    ▼
┌──────────────────┐
│ VLM (vlm_parser) │  ← 前端，仅生成 DSL
│  输出 RoboIR     │
└────────┬─────────┘
         │  SMDSL (RoboIR) 中间表示
         ▼
┌──────────────────┐
│ Rust 编译器      │  ← 后端，静态验证 + 编译
│ (rust_compiler)  │
└────────┬─────────┘
         │  动作序列
         ▼
┌──────────────────┐
│ 独立规划器       │  ← 验证可执行性
│ (后端仿真器)     │
└──────────────────┘
```

## 与 RoboInter 的对齐

- [RoboInter-VQA](https://huggingface.co/datasets/InternRobotics/RoboInter-VQA) — 自然语言与机器人中间表示的对齐 QA 基准
- [RoboInter-Data](https://huggingface.co/datasets/InternRobotics/RoboInter-Data) — 大规模机器人操作中间表示数据集

这两个数据集将作为 SMDSL RoboIR 的对齐基准。

## 五项量化指标

| 指标 | 含义 |
|------|------|
| `parsing_accuracy` | 语法与编译级——JSON Schema 及 Frame 坐标系校验通过率 |
| `precision_recall` | 代码精度与召回率——约束细节的完整性 (F1) |
| `stl_robustness` | 数学与物理边界——Signal Temporal Logic 安全冗余距离 |
| `end_to_end_latency` | 执行效率——LLM 推理 + 编译解析总耗时 |
| `path_efficiency` | 轨迹代价与理论最优比 |

## 启动

```bash
cd smdsl_demo
python -m pip install gradio
python app.py
```

---

## ⚠️ 系统局限性声明 (Limitations)

> 在向真实物理机械臂或更复杂 3D 空间迁移前，使用方必须阅读以下声明。

**本系统当前版本为 2.5D 架构，距离场未引入 Z 轴维度；仅适用于机器人底盘的 2D 避障导航与宏观任务规划。对于 6‑DoF 机械臂的三维空间避障，未来需将 2D EDT 升级为 3D 体素距离场或 fVDB。**

具体表现：

- `cad_parser/astar_topology.py` 的 `compute_distance_field` 调用 `scipy.ndimage.distance_transform_edt`，输入为 H×W 2D 栅格；
- `smdsl_demo/spatial_api_stub.py` 的 `_eval_distance_via_field` 在采样时仅使用轨迹点的 `(x, y)`，**轨迹点的 `z` 字段被完全忽略**；
- `path_pixels_to_trajectory`（`astar_topology.py`）默认 `z = 0.0`；
- 3D 可视化（`cad_parser/visualize.py` 的 `generate_3d_topology_preview`）只是把 2D 墙体的每个像素拉成一条垂直线段做白模预览，**不构成真正的 3D 碰撞体积**。

**适用场景**：
- ✅ 工厂 AGV / 移动底盘的全局避障路径规划
- ✅ FloorplanQA / 户型图级的语义拓扑分析
- ✅ 楼层级宏观任务规划（不涉及垂直方向碰撞）
- ❌ 6‑DoF 机械臂的桌面级抓取规划
- ❌ 多层空间（楼梯 / 上下料台 / 货架）的连续 Z 维避障
- ❌ 动态障碍物（移动行人 / 其他机器人）—— 距离场是一次性快照

**未来升级路径（建议）**：
1. 把 EDT 替换为 **3D 体素 SDF**（参考 fVDB、OctoMap、Voxblox）；
2. 给 `_eval_distance_via_field` 增加 z 维采样，保留 2D 回退作为楼层级导航的 fast path；
3. 距离场支持增量更新（动态障碍物）。

完整局限性分析见 `docs/EVALUATION_REPORT.md` 第四章。
