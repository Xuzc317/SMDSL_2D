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
