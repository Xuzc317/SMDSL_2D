# SMDSL_2D 测试手册

> 完整测试指南。覆盖范围：63 个单元测试 + MCP 工具测试 + 基准验证。
> 最后更新：2026-06-15

---

## 1. 测试概览

### 1.1 测试文件清单 (8 文件, 63 tests)

| 文件 | 测试数 | 覆盖范围 | Phase |
|------|--------|---------|-------|
| `tests/test_z_axis.py` | 5 | Z 轴硬断言 | 1.1 |
| `tests/test_entity_stats.py` | 9 | DWG 实体统计 + drop note | 1.3 |
| `tests/test_roboir_diff.py` | 6 | RoboIR intent/target_frame 不变性 | 1.2 |
| `tests/test_trajectory_smoother.py` | 11 | 梯形 Profile + 三次样条 | 2.1 |
| `tests/test_gradient_refine.py` | 7 | EDT 梯度 + 路径微调 | 2.2 |
| `tests/test_metrics.py` | 16 | stl_robustness, path_efficiency 等 | Next |
| `tests/test_rust_compiler.py` | 9 | RoboIR 编译器验证 (validate) | Next |
| `smdsl_demo/mcp_server.py` | 6 | MCP 工具端到端 | UX |

### 1.3 MCP Server 工具测试

MCP Server 提供 6 个工具，可通过 Claude Desktop 调用并验证:

| 工具 | 功能 | 验证状态 |
|------|------|---------|
| `cad_dispatch` | CAD 文件解析 → 栅格元数据 | ✅ |
| `compute_topology` | EDT 距离场 + 拓扑分析 | ✅ |
| `plan_path` | A* 安全路径规划 | ✅ |
| `validate_trajectory` | STL 鲁棒度验证 | ✅ |
| `smooth_trajectory` | 样条平滑轨迹合成 | ✅ |
| `analyze_scene` | 完整场景分析 + 安全候选点 | ✅ |

> \* 需要 DEEPSEEK_API_KEY 环境变量，不计入自动化测试计数

### 1.2 测试统计

```
单元测试: 63 (8 文件)
MCP 工具: 6 (端到端验证)
基准验证: 200 layouts × 5 pairs (EDT +36.9% vs Costmap)
状态:     全部通过 (100%)
执行时间: < 2s
框架:     pytest 9.1.0 + MCP stdio
```

---

## 2. 运行测试

### 2.1 全部单元测试

```bash
cd D:\Code\SMDSL_demo

# 运行全部 38 个测试
python -m pytest SMDSL/tests/ -v

# 静默模式（仅摘要）
python -m pytest SMDSL/tests/ -q

# 带覆盖率报告（需安装 pytest-cov）
python -m pytest SMDSL/tests/ --cov=SMDSL --cov-report=term-missing
```

### 2.2 按模块运行

```bash
# Z 轴断言
python -m pytest SMDSL/tests/test_z_axis.py -v

# DWG 实体统计
python -m pytest SMDSL/tests/test_entity_stats.py -v

# RoboIR diff 检查
python -m pytest SMDSL/tests/test_roboir_diff.py -v

# 轨迹合成
python -m pytest SMDSL/tests/test_trajectory_smoother.py -v

# 梯度微调
python -m pytest SMDSL/tests/test_gradient_refine.py -v
```

### 2.3 集成测试（需 API Key）

```bash
# 设置 API Key
$env:DEEPSEEK_API_KEY = "sk-..."

# 端到端闭环自愈测试
python SMDSL/tests/test_closed_loop_recovery.py

# Demo 3 管道测试
python -m pytest SMDSL/smdsl_demo/test_demo3_pipeline.py -v
```

### 2.4 基准验证

```bash
# 快速验证（合成数据 10 layouts）
python benchmark/bench_edt_vs_costmap.py

# 完整基准（200 layouts × 5 pairs）
python benchmark/bench_edt_vs_costmap.py \
    --data_dir data/datasets/synthetic_benchmark \
    --n_layouts 200 --n_pairs 5

# OSM 真实路网
python benchmark/bench_edt_vs_costmap.py \
    --data_dir data/datasets/osm_networks \
    --n_layouts 30 --n_pairs 5
```

---

## 3. 测试用例详解

### 3.1 test_z_axis.py — Z 轴硬断言

**被测函数**: `smdsl_demo.spatial_api_stub._eval_distance_via_field`

| 测试用例 | 输入 | 预期输出 |
|---------|------|---------|
| `test_z_axis_rejected_with_minus_inf` | z 从 0→0.5 变化的 10 点轨迹 | `robustness == -inf`, `source == "z_axis_not_supported"` |
| `test_z_axis_uniform_but_nonzero` | 全部 `z=0.5`（均匀非零） | 正常执行（`z_range=0 < 0.01`）|
| `test_z_axis_tiny_variation_not_triggered` | z_range = 0.005 < 0.01 | 不触发断言 |
| `test_pure_2d_trajectory_executes_normally` | 纯 2D 轨迹（z=0）| `source == "distance_field"`, details ≥ 10 条 |
| `test_empty_trajectory` | 空轨迹 `[]` | `robustness != -inf`（空守卫返回）|

### 3.2 test_entity_stats.py — DWG 实体统计

**被测函数**: `cad_parser.dispatcher._extract_dwg_geometry` + `_format_entity_drop_note`

| 测试用例 | 输入 | 预期输出 |
|---------|------|---------|
| `test_mixed_entities_count` | 6 实体（LINE/POLYLINE/CIRCLE/TEXT/INSERT/MTEXT）| processed=3, skipped: TEXT=1, INSERT=1, MTEXT=1 |
| `test_all_known_entities` | 5 已知类型实体 | processed=5, skipped=0 |
| `test_all_unknown_entities` | 2 未知类型 | processed=0, skipped=2 |
| `test_empty_entities` | `[]` | total=0, processed=0 |
| `test_entity_with_acdb_prefix` | AcDbLine, AcDbPolyline | 正确去除前缀后处理 |
| `test_no_drop_when_below_threshold` | 5% 丢弃率 | `_format_entity_drop_note` 返回空 |
| `test_no_drop_at_exactly_threshold` | 10% 丢弃率 | 返回空（不大于 10%）|
| `test_drop_above_threshold` | 30% 丢弃率 | 返回包含 "30.0%" 的非空字符串 |
| `test_empty_stats` | total=0 | 返回空 |

### 3.3 test_roboir_diff.py — RoboIR Diff 检查

**被测逻辑**: `test_closed_loop_recovery.run_correction_loop` 中的 diff 检查

| 测试用例 | 输入 | 预期输出 |
|---------|------|---------|
| `test_intact_roboir_passes` | 仅改 `stl_constraints` | `True`（通过）|
| `test_intent_modified_is_rejected` | intent: navigate→pick | `False`（拦截）|
| `test_target_frame_modified_is_rejected` | target_frame: room_A→room_B | `False`（拦截）|
| `test_both_modified_is_rejected` | 同时修改 intent + target_frame | `False`（拦截）|
| `test_none_fields_handled` | 双方 target_frame=None | `True`（通过）|
| `test_missing_fields_unequal` | original 缺 target_frame，corrected 有 | `False`（拦截）|

### 3.4 test_trajectory_smoother.py — 轨迹合成

**被测函数**: `smdsl_demo.trajectory_smoother`

| 测试用例 | 验证内容 |
|---------|---------|
| `test_normal_profile_has_three_phases` | 加速→匀速→减速三段特征 |
| `test_short_distance_degrades_to_triangle` | 短距离退化为三角形 Profile |
| `test_zero_distance_returns_zeros` | 零距离返回全零 |
| `test_spline_preserves_endpoints` | 样条保持起点终点 |
| `test_spline_single_point` | 单点处理 |
| `test_output_format` | 输出格式 `{t,x,y,z,roll,pitch,yaw}` |
| `test_time_is_monotonic` | 时间单调递增 |
| `test_endpoint_within_tolerance` | 起终点偏差 < 1 像素 |
| `test_empty_path_returns_empty` | 空路径返回 `[]` |
| `test_single_point_path_returns_empty` | 单点返回 `[]` |
| `test_path_length_preserved` | 轨迹长度变化 < 5% |

### 3.5 test_gradient_refine.py — 梯度微调

**被测函数**: `cad_parser.astar_topology.compute_field_gradient` + `refine_path_via_gradient`

| 测试用例 | 验证内容 |
|---------|---------|
| `test_gradient_shapes_match` | 梯度输出维度 = 输入维度 |
| `test_gradient_zero_in_uniform_region` | 均匀区域梯度为零 |
| `test_gradient_points_away_from_obstacle` | 梯度指向远离障碍物方向 |
| `test_start_end_preserved` | 微调后起终点不变 |
| `test_refined_path_stays_in_free_space` | 微调后 d ≥ robot_radius_px |
| `test_path_length_preserved` | 路径点数不膨胀 |
| `test_short_path_unchanged` | < 3 点路径直接返回 |

---

## 4. 添加新测试

### 4.1 测试模板

```python
"""
test_xxx.py — 简短描述 (Phase X.Y)
"""

import sys
from pathlib import Path
_PROJECT_ROOT = Path(__file__).resolve().parent.parent
if str(_PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(_PROJECT_ROOT))

import pytest


class TestXxx:
    def test_normal_case(self):
        """正常路径"""
        ...

    def test_edge_case(self):
        """边界/异常路径"""
        ...
```

### 4.2 命名规范

- 文件名: `test_{模块名}.py`
- 类名: `Test{功能名}`
- 方法名: `test_{场景}`

---

## 5. 持续集成建议

```yaml
# GitHub Actions .github/workflows/test.yml
name: SMDSL Tests
on: [push, pull_request]
jobs:
  test:
    runs-on: ubuntu-latest
    steps:
      - uses: actions/checkout@v4
      - uses: actions/setup-python@v5
        with:
          python-version: '3.11'
      - run: pip install -r requirements.txt
      - run: python -m pytest SMDSL/tests/ -v --tb=short
```

---

## 6. 故障排查

| 症状 | 可能原因 | 解决 |
|------|---------|------|
| `ModuleNotFoundError: cad_parser` | 不在项目根目录 | `cd D:\Code\SMDSL_demo` |
| `ModuleNotFoundError: scipy` | 依赖未安装 | `pip install -r requirements.txt` |
| 测试挂起不结束 | LLM API 超时 | 仅运行单元测试：`pytest SMDSL/tests/ -v --ignore=SMDSL/tests/test_closed_loop_recovery.py` |
| 基准退出 `KeyError` | 数据格式不匹配 | 确认使用合成的 FloorplanQA 格式 JSON |
