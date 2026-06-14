"""
test_closed_loop_recovery.py — End-to-End Closed-Loop Recovery Test
═══════════════════════════════════════════════════════════════════════

将 cad_parser（环境感知）、vlm_parser（语义编译）和 spatial_api_stub +
metrics（物理验证）串联，测试完整的"初始生成 → 物理报错 → LLM 反思 →
修正成功"闭环自愈流水线。

测试流程:
  1. SCENE INIT     — 加载 CAD 图纸，提取拓扑包 (distance_field + grid_transform)
  2. THE TRAP       — 故意给出会导致物理碰撞的自然语言指令
  3. INITIAL GEN    — VLM 生成初始 RoboIR
  4. VALIDATION     — 物理引擎校验 → 捕获结构化失败反馈
  5. SELF-CORRECTION — 将失败反馈回灌 LLM，最多重试 3 次
  6. METRICS        — 打印闭环诊断报告

用法::

    # 使用 FloorplanQA JSON（推荐，不需要 LibreDWG）
    python tests/test_closed_loop_recovery.py

    # 使用 DWG 工业图纸
    python tests/test_closed_loop_recovery.py --cad_path data/dwg_samples/.../xxx.dwg

    # 自定义参数
    python tests/test_closed_loop_recovery.py --resolution 0.02 --d_safe 0.30 --max_retries 5
"""

from __future__ import annotations

import argparse
import json
import math
import os
import sys
import time
import traceback
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, Dict, List, Optional, Tuple

import numpy as np

# ── 路径修复：确保项目根目录在 sys.path 中 ──
_PROJECT_ROOT = Path(__file__).resolve().parent.parent
if str(_PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(_PROJECT_ROOT))

# ── 终端编码修复 ──
if sys.stdout.encoding != "utf-8":
    try:
        sys.stdout.reconfigure(encoding="utf-8")
    except (AttributeError, OSError):
        pass
if sys.stderr.encoding != "utf-8":
    try:
        sys.stderr.reconfigure(encoding="utf-8")
    except (AttributeError, OSError):
        pass


# ══════════════════════════════════════════════════════════════════════
# 0. 常量
# ══════════════════════════════════════════════════════════════════════

DEFAULT_CAD_PATH = "data/cad_samples/floorplanqa/layouts/kitchen/room_0.json"
DEFAULT_RESOLUTION = 0.05       # m/px
DEFAULT_ROBOT_RADIUS = 0.30     # m
DEFAULT_D_SAFE = 0.30           # m — STL Distance 安全阈值
MAX_RETRIES = 3                 # LLM 自愈最大重试次数
N_TRAJECTORY_POINTS = 50        # 合成轨迹采样点数


# ══════════════════════════════════════════════════════════════════════
# 1. 辅助数据类
# ══════════════════════════════════════════════════════════════════════

@dataclass
class RecoveryMetrics:
    """闭环自愈指标记录器。"""
    t_scene_init_ms: float = 0.0
    t_initial_gen_ms: float = 0.0
    t_validation_ms: float = 0.0
    t_correction_ms: float = 0.0

    initial_robustness: float = float("-inf")
    final_robustness: float = float("-inf")
    n_llm_calls: int = 0
    n_retry_rounds: int = 0
    correction_success: bool = False
    failure_type: str = ""

    violation_nodes_initial: int = 0
    violation_nodes_final: int = 0

    errors: List[str] = field(default_factory=list)

    @property
    def total_time_ms(self) -> float:
        return (self.t_scene_init_ms + self.t_initial_gen_ms
                + self.t_validation_ms + self.t_correction_ms)


# ══════════════════════════════════════════════════════════════════════
# 2. 场景初始化与拓扑提取
# ══════════════════════════════════════════════════════════════════════

def init_scene(
    cad_path: str,
    resolution: float,
    robot_radius_m: float,
) -> Tuple[Dict[str, Any], Dict[str, Any], np.ndarray]:
    """
    加载 CAD 图纸 → 运行完整管线 → 返回 (topology_bundle, pipeline_result, distance_field)。
    """
    p = Path(cad_path)
    if not p.exists():
        raise FileNotFoundError(f"CAD 文件不存在: {cad_path}")

    ext = p.suffix.lower()
    if ext == ".dwg":
        # DWG 路径：通过 dispatcher → grid → distance_field
        from cad_parser.dispatcher import dispatch_cad
        from cad_parser.astar_topology import (
            compute_distance_field,
            remove_exterior_freespace,
            bridge_thin_walls,
            weld_double_line_walls,
        )
        result = dispatch_cad(str(p), resolution=resolution,
                              padding_m=0.5, wall_thickness_m=0.10)
        grid = result["grid"]
        transform = result["transform"]
        # 工业级后处理（与 dispatch_cad 内 DWG 分支一致）
        grid = weld_double_line_walls(
            grid, resolution=transform["resolution"], max_gap_m=0.30,
        )
        grid = remove_exterior_freespace(grid, close_gaps_px=2)
        grid = bridge_thin_walls(grid, kernel_px=2)
        distance_field = compute_distance_field(grid)
        bundle: Dict[str, Any] = {
            "distance_field": distance_field,
            "grid_transform": {
                "origin": transform["origin"],
                "resolution": transform["resolution"],
                "shape": transform["shape"],
            },
            "robot_radius_m": robot_radius_m,
            "robot_radius_px": robot_radius_m / transform["resolution"],
            "layout_id": p.stem,
            "room_type": "dwg_industrial",
        }
        pipeline_result: Dict[str, Any] = {
            "grid": grid, "transform": transform,
            "distance_field": distance_field,
            "cad_data": result.get("cad_data", {}),
        }
        return bundle, pipeline_result, distance_field
    else:
        # FloorplanQA JSON：走 run_pipeline
        from cad_parser.astar_topology import run_pipeline, to_topology_bundle
        pipe = run_pipeline(
            cad_path=str(p),
            resolution=resolution,
            robot_radius_m=robot_radius_m,
            padding_m=0.5,
            wall_thickness_m=0.10,
            safety_weight=0.5,
            max_steps=2_000_000,
            seeds_world=None,
            include_grid=True,
        )
        bundle = to_topology_bundle(pipe)
        return bundle, pipe, pipe["distance_field"]


def find_free_centroids(
    distance_field: np.ndarray,
    transform: Dict[str, Any],
    n: int = 4,
    min_clearance_m: float = 0.0,
) -> List[Tuple[float, float]]:
    """
    在距离场中找到 n 个彼此远离的安全自由空间中心点（世界坐标）。

    策略：
      1. 取距离场 ≥ ``min_clearance_m`` 且大于中位数的自由像素作为候选集
      2. 在候选集中找欧氏距离最远的两个点（作为起终点）
      3. 若 n > 2，继续贪心加入距离已选点最远的候选点

    Args:
        min_clearance_m: 候选点要求的最小物理 clearance（米）。设置为
            ``d_safe + headroom``（如 ``0.30 + 0.15 = 0.45``）可保证 LLM
            把 ``Distance > X`` 改严时仍有可行解。
    """
    df = distance_field.astype(np.float32)
    H, W = df.shape
    res = transform["resolution"]
    ox, oy = transform["origin"]

    free_mask = df > 0
    if not free_mask.any():
        idx = np.argmax(df)
        r, c = divmod(int(idx), W)
        return [(ox + c * res, oy + r * res)]

    median_df = np.median(df[free_mask])
    min_clearance_px = max(min_clearance_m / res, 1.0)
    threshold_px = max(median_df, min_clearance_px)
    candidate = df >= threshold_px
    rows, cols = np.where(candidate)

    # 若严格阈值下候选点不足，先松到中位数；仍不够则退到全部 free
    if len(rows) < 2:
        candidate = df >= max(median_df, 1.0)
        rows, cols = np.where(candidate)
    if len(rows) < 2:
        rows, cols = np.where(free_mask)

    pts_px = np.column_stack([rows, cols])  # (N, 2)

    # ── 贪心最远点选择 ──
    # 第 1 个点：距离场最大值
    idx_max = np.argmax(df[rows, cols])
    chosen_px = [pts_px[idx_max].astype(float)]
    chosen_idx = {idx_max}

    for _ in range(1, n):
        if len(chosen_idx) >= len(pts_px):
            break
        # 计算每个未选点到所有已选点的最小距离
        min_dists = np.full(len(pts_px), np.inf)
        for cp in chosen_px:
            dists = np.sqrt(np.sum((pts_px - cp) ** 2, axis=1))
            min_dists = np.minimum(min_dists, dists)
        # 排除已选点
        for ci in chosen_idx:
            min_dists[ci] = -1.0
        best_idx = int(np.argmax(min_dists))
        if min_dists[best_idx] <= 0:
            break
        chosen_px.append(pts_px[best_idx].astype(float))
        chosen_idx.add(best_idx)

    centroids: List[Tuple[float, float]] = []
    for r, c in chosen_px:
        x = ox + c * res
        y = oy + r * res
        centroids.append((x, y))
    return centroids


# ══════════════════════════════════════════════════════════════════════
# 3. 诱导失败的指令生成 (The Trap)
# ══════════════════════════════════════════════════════════════════════

def build_trap_instruction(centroids: List[Tuple[float, float]]) -> str:
    """
    构造一个"几何上必然碰撞"的自然语言指令。

    语义：要求机器人沿最短直线路径穿越建筑，
    但故意不提安全距离（让 LLM 生成过小的 Distance 约束或根本未意识到障碍物）。
    """
    if len(centroids) >= 2:
        x1, y1 = centroids[0]
        x2, y2 = centroids[-1]
        return (
            f"以最短直线距离将物料从位置 A ({x1:.1f}, {y1:.1f}) "
            f"快速移动到位置 B ({x2:.1f}, {y2:.1f})。"
            f"要求路径尽可能短、速度快，5 秒内完成。"
            f"不要绕路，直接穿过。"
        )
    # 回退：通用陷阱指令
    return (
        "以最短直线距离，将物料从房间 A 穿过狭窄缝隙移动到房间 B。"
        "要求路径最短、速度最快、不绕路。"
    )


# ══════════════════════════════════════════════════════════════════════
# 4. 合成测试轨迹（直线 → 必然穿墙）
# ══════════════════════════════════════════════════════════════════════

def synthesize_trajectory(
    centroids: List[Tuple[float, float]],
    n_points: int = N_TRAJECTORY_POINTS,
) -> List[Dict[str, float]]:
    """
    在两个安全自由空间中心之间生成一条**直线轨迹**。

    这条直线几乎必然穿越墙体（因为真实建筑中两点之间不可能全是自由空间），
    从而触发物理引擎的 COLLISION 检测。
    """
    if len(centroids) < 2:
        raise ValueError("至少需要 2 个自由空间中心点来合成轨迹。")

    (x1, y1), (x2, y2) = centroids[0], centroids[-1]
    trajectory: List[Dict[str, float]] = []
    for i in range(n_points):
        t_norm = i / (n_points - 1)
        trajectory.append({
            "t": t_norm * 5.0,                 # 总时长 5 秒
            "x": x1 + (x2 - x1) * t_norm,
            "y": y1 + (y2 - y1) * t_norm,
            "z": 0.0,
            "roll": 0.0,
            "pitch": 0.0,
            "yaw": 0.0,
        })
    return trajectory


# ══════════════════════════════════════════════════════════════════════
# 5. 物理校验与结构化反馈
# ══════════════════════════════════════════════════════════════════════

def validate_roboir(
    roboir: Dict[str, Any],
    trajectory: List[Dict[str, float]],
    topology_bundle: Dict[str, Any],
    d_safe: float = DEFAULT_D_SAFE,
) -> Dict[str, Any]:
    """
    将 RoboIR + trajectory 送入物理引擎校验。

    Returns:
        {
            "passed": bool,
            "robustness": float,
            "violation_report": List[Dict],   # check_stl_constraint_violation 输出
            "structured_feedback": Dict,      # generate_structured_feedback 输出
            "n_violation_nodes": int,
        }
    """
    from smdsl_demo.vlm_parser import normalize_stl_constraints
    from smdsl_demo.spatial_api_stub import check_stl_constraint_violation
    from smdsl_demo.metrics import generate_structured_feedback, FailureTaxonomy

    # 1. 标准化 STL 约束
    raw_constraints = roboir.get("stl_constraints", [])
    constraint_rules = normalize_stl_constraints(raw_constraints)

    # 2. 如果没有 Distance 约束，注入一个严格的（这是陷阱的核心）
    has_distance = any("Distance" in str(c.get("expr", "")) for c in constraint_rules)
    if not has_distance:
        constraint_rules.append({
            "type": "stl_constraint",
            "expr": f"Distance > {d_safe}",
            "unit": "m",
            "reference": "obstacle",
        })

    # 3. 物理引擎求解
    violation_report = check_stl_constraint_violation(
        trajectory, constraint_rules,
        topology_bundle=topology_bundle,
    )

    # 4. 提取鲁棒度
    robustness = float("inf")
    n_violation_nodes = 0
    for rpt in violation_report:
        rho = rpt.get("robustness", 0.0)
        if rho < robustness:
            robustness = rho
        n_violation_nodes += len(rpt.get("violation_nodes", []))

    passed = robustness > 0

    # 5. 结构化反馈
    if not passed:
        error_type = FailureTaxonomy.COLLISION if robustness < 0 else FailureTaxonomy.STL_VIOLATION
        feedback = generate_structured_feedback(
            error_type=error_type,
            robustness_score=robustness,
            context={
                "n_violation_nodes": n_violation_nodes,
                "constraint_rules": constraint_rules,
                "violation_report_summary": [
                    {
                        "rule_expr": r.get("rule_expr"),
                        "robustness": r.get("robustness"),
                        "violated": r.get("violated"),
                        "n_violation_nodes": len(r.get("violation_nodes", [])),
                    }
                    for r in violation_report
                ],
            },
        )
    else:
        feedback = generate_structured_feedback(
            error_type=FailureTaxonomy.STL_VIOLATION,
            robustness_score=robustness,
            context={"message": "All constraints satisfied."},
        )

    return {
        "passed": passed,
        "robustness": robustness,
        "violation_report": violation_report,
        "structured_feedback": feedback,
        "n_violation_nodes": n_violation_nodes,
        "constraint_rules": constraint_rules,
    }


# ══════════════════════════════════════════════════════════════════════
# 6. 闭环自愈 — Feedback Retry Loop
# ══════════════════════════════════════════════════════════════════════

CORRECTION_PROMPT_TEMPLATE = """\
你的上一次 RoboIR 输出在物理引擎校验中失败。以下是结构化错误报告：

{feedback_json}

【错误类型】: {error_type}
【鲁棒度 ρ】: {robustness}（ρ < 0 表示硬约束被破坏：发生了碰撞或越界）
【违规节点数】: {n_violation_nodes} 个轨迹点上的物理约束被破坏
【原始指令】: {original_instruction}

请执行**最小责任层恢复**（Minimal Responsibility Recovery）：
1. 不要改变整体的 Intent 和目标坐标系
2. 仅在违规的轨迹区间添加或增强规避约束：
   - 增大 Distance 安全阈值（如 Distance > {d_safe_higher}）
   - 或添加中间 Waypoint 绕开障碍物
3. 输出修正后的完整 RoboIR JSON（只含 intent, target_frame, grasp_type, stl_constraints）

直接输出 JSON，不要用 Markdown 代码块包裹。"""


def _extract_distance_threshold(
    roboir: Dict[str, Any], fallback: float,
) -> float:
    """
    从 RoboIR 的 stl_constraints 中提取 ``Distance > X`` 的最大阈值（米）。

    Returns:
        所有 ``Distance > X`` 中的最大 X；未声明 → ``fallback``。
    """
    import re
    rules = roboir.get("stl_constraints", []) or []
    best = float("-inf")
    pat = re.compile(r"Distance\s*>\s*([0-9]*\.?[0-9]+)")
    for rule in rules:
        expr = str(rule.get("expr", "") if isinstance(rule, dict) else rule)
        for m in pat.finditer(expr):
            try:
                v = float(m.group(1))
                if v > best:
                    best = v
            except (ValueError, TypeError):
                continue
    return best if best > 0 else fallback


def _replan_trajectory_with_clearance(
    start_xy: Tuple[float, float],
    goal_xy: Tuple[float, float],
    pipeline_result: Dict[str, Any],
    d_safe_required: float,
    total_time_s: float = 5.0,
    safety_margin_m: float = 0.05,
) -> Optional[List[Dict[str, float]]]:
    """
    用 A* 在距离场 ≥ ``d_safe_required + safety_margin_m`` 的自由空间里重规划轨迹。

    ``run_correction_loop`` 在每轮 LLM 修正后调用：LLM 把 ``Distance > X`` 改严
    时，此函数据新阈值（加 1 像素安全余量）规划满足该 clearance 的绕行路径。

    ``safety_margin_m`` 的工程意义：A* 网格离散化下，``robot_radius_px = X/res``
    会让路径恰好落在 ``df = X/res`` 像素上，双线性采样在整点处返回精确等于
    ``X``，导致 ``ρ = 0``（边界等于而非严格大于）。加 1 px 余量后 ``ρ > 0`` 成立。

    Returns:
        新轨迹（[{t,x,y,z,roll,pitch,yaw}, ...]）；规划失败 → None。
    """
    from cad_parser.astar_topology import (
        astar_shortest_path, path_pixels_to_trajectory,
    )

    grid = pipeline_result.get("grid")
    distance_field = pipeline_result.get("distance_field")
    transform = pipeline_result.get("transform")
    if grid is None or distance_field is None or transform is None:
        return None

    res = float(transform["resolution"])
    ox, oy = transform["origin"]

    robot_radius_px = max(1.0, (d_safe_required + safety_margin_m) / res)

    sr = int(round((start_xy[1] - oy) / res))
    sc = int(round((start_xy[0] - ox) / res))
    gr = int(round((goal_xy[1] - oy) / res))
    gc = int(round((goal_xy[0] - ox) / res))

    path_rc = astar_shortest_path(
        grid=grid,
        distance_field=distance_field,
        start_rc=(sr, sc),
        goal_rc=(gr, gc),
        robot_radius_px=robot_radius_px,
        safety_weight=0.5,
    )
    if not path_rc:
        return None

    new_traj = path_pixels_to_trajectory(
        path_rc=path_rc,
        transform=transform,
        total_time_s=total_time_s,
        z=0.0, yaw_rad=0.0, sample_step=2,
    )
    return new_traj if new_traj else None


def run_correction_loop(
    roboir: Dict[str, Any],
    validation_result: Dict[str, Any],
    trajectory: List[Dict[str, float]],
    topology_bundle: Dict[str, Any],
    original_instruction: str,
    d_safe: float = DEFAULT_D_SAFE,
    max_retries: int = MAX_RETRIES,
    pipeline_result: Optional[Dict[str, Any]] = None,
    start_xy: Optional[Tuple[float, float]] = None,
    goal_xy: Optional[Tuple[float, float]] = None,
) -> Tuple[Dict[str, Any], Dict[str, Any], RecoveryMetrics]:
    """
    闭环自愈主循环：将失败反馈回灌 LLM，重试直到通过或耗尽重试次数。

    Returns:
        (final_roboir, final_validation, metrics)
    """
    metrics = RecoveryMetrics()
    metrics.initial_robustness = validation_result["robustness"]
    metrics.violation_nodes_initial = validation_result["n_violation_nodes"]
    metrics.failure_type = validation_result["structured_feedback"].get(
        "error", {},
    ).get("type", "unknown")

    current_roboir = roboir
    current_validation = validation_result

    # 如果初始就通过，不需要重试
    if validation_result["passed"]:
        metrics.correction_success = True
        metrics.final_robustness = validation_result["robustness"]
        metrics.violation_nodes_final = validation_result["n_violation_nodes"]
        metrics.n_llm_calls = 1
        return current_roboir, current_validation, metrics

    # ── 自愈循环 ──
    t0 = time.perf_counter()
    for attempt in range(1, max_retries + 1):
        metrics.n_retry_rounds = attempt
        print(f"\n  [{attempt}/{max_retries}] 正在请求 LLM 修正...")

        # 构建修正 prompt
        feedback_json = json.dumps(
            current_validation["structured_feedback"],
            ensure_ascii=False, indent=2,
        )
        correction_prompt = CORRECTION_PROMPT_TEMPLATE.format(
            feedback_json=feedback_json,
            error_type=metrics.failure_type,
            robustness=current_validation["robustness"],
            n_violation_nodes=current_validation["n_violation_nodes"],
            original_instruction=original_instruction,
            d_safe_higher=d_safe + 0.10,
        )

        # 调用 LLM 修正
        try:
            from smdsl_demo.vlm_parser import parse_instruction_to_roboir
            corrected_roboir = parse_instruction_to_roboir(
                correction_prompt,
                local_context=None,
                max_retries=2,
            )
            metrics.n_llm_calls += 1
        except Exception as e:
            print(f"    LLM 调用失败: {e}")
            metrics.errors.append(f"LLM correction round {attempt}: {e}")
            continue

        # 据修正后的 d_safe 重规划轨迹（真闭环关键步骤）：LLM 把 Distance
        # 改严时，物理层用 A* 据新的 clearance 重新规划轨迹；否则约束改了
        # 轨迹没变，ρ 反而更差。
        current_trajectory = trajectory
        if pipeline_result is not None and start_xy is not None and goal_xy is not None:
            d_safe_new = _extract_distance_threshold(
                corrected_roboir, fallback=d_safe,
            )
            if d_safe_new > 0:
                # total_time_s=4.9 留 0.1 s 缓冲，避免 Time < 5.0 约束 ρ = 0 边界
                new_traj = _replan_trajectory_with_clearance(
                    start_xy=start_xy,
                    goal_xy=goal_xy,
                    pipeline_result=pipeline_result,
                    d_safe_required=d_safe_new,
                    total_time_s=4.9,
                )
                # 物理可行性兜底：严苛 d_safe 不可达 → 退回原 d_safe 重规划。
                # 这保证 LLM 把约束改严后，物理层至少能给出一条「比原直线更安全」的可行轨迹。
                d_safe_used = d_safe_new
                if (new_traj is None or len(new_traj) < 2) and d_safe_new > d_safe:
                    new_traj = _replan_trajectory_with_clearance(
                        start_xy=start_xy,
                        goal_xy=goal_xy,
                        pipeline_result=pipeline_result,
                        d_safe_required=d_safe,
                        total_time_s=4.9,
                    )
                    d_safe_used = d_safe
                if new_traj is not None and len(new_traj) >= 2:
                    current_trajectory = new_traj
                    fallback_note = " (fallback to baseline d_safe)" if d_safe_used != d_safe_new else ""
                    print(
                        f"    [REPLAN] A* 据 d_safe={d_safe_used:.2f}m 重规划成功："
                        f"{len(new_traj)} 点 (原 {len(trajectory)} 点){fallback_note}"
                    )
                else:
                    print(
                        f"    [REPLAN-FAIL] d_safe={d_safe_new:.2f}m 下 A* 不可达，"
                        f"沿用上一条轨迹。"
                    )

        # 校验修正后的 RoboIR + 新轨迹
        corrected_validation = validate_roboir(
            corrected_roboir, current_trajectory, topology_bundle, d_safe=d_safe,
        )

        print(
            f"    修正后 ρ = {corrected_validation['robustness']:.4f} "
            f"(初始 ρ = {metrics.initial_robustness:.4f}), "
            f"违规节点: {corrected_validation['n_violation_nodes']}"
        )

        current_roboir = corrected_roboir
        current_validation = corrected_validation

        if corrected_validation["passed"]:
            print(f"    [OK] 第 {attempt} 轮修正成功！ρ > 0")
            metrics.correction_success = True
            break
        else:
            print(f"    [RETRY] 第 {attempt} 轮仍未通过，ρ = {corrected_validation['robustness']:.4f}")

            # 如果 robustness 有改善，更新 failure_type
            if corrected_validation["robustness"] > current_validation.get("robustness", float("-inf")):
                metrics.failure_type = corrected_validation["structured_feedback"].get(
                    "error", {},
                ).get("type", metrics.failure_type)

    metrics.t_correction_ms = (time.perf_counter() - t0) * 1000.0
    metrics.final_robustness = current_validation["robustness"]
    metrics.violation_nodes_final = current_validation["n_violation_nodes"]

    return current_roboir, current_validation, metrics


# ══════════════════════════════════════════════════════════════════════
# 7. 指标统计与诊断报告
# ══════════════════════════════════════════════════════════════════════

def print_diagnostic_report(
    roboir_initial: Dict[str, Any],
    roboir_final: Dict[str, Any],
    validation_initial: Dict[str, Any],
    validation_final: Dict[str, Any],
    metrics: RecoveryMetrics,
    cad_path: str,
    d_safe: float,
) -> None:
    """打印完整的闭环诊断报告。"""
    sep = "=" * 62
    print(f"\n{sep}")
    print("  SMDSL 闭环自愈诊断报告 (Closed-Loop Recovery Report)")
    print(f"{sep}")

    # ── 场景信息 ──
    print(f"\n  CAD 图纸  : {cad_path}")
    print(f"  D_safe    : {d_safe} m")

    # ── 耗时统计 ──
    print(f"\n  ┌─ 耗时统计 ───────────────────────────────────────┐")
    print(f"  │ 场景初始化    : {metrics.t_scene_init_ms:7.0f} ms")
    print(f"  │ 初始 RoboIR 生成: {metrics.t_initial_gen_ms:7.0f} ms")
    print(f"  │ 物理校验      : {metrics.t_validation_ms:7.0f} ms")
    print(f"  │ 自愈循环      : {metrics.t_correction_ms:7.0f} ms")
    print(f"  │ 总耗时         : {metrics.total_time_ms:7.0f} ms")
    print(f"  └──────────────────────────────────────────────────┘")

    # ── 鲁棒度演化 ──
    print(f"\n  ┌─ 鲁棒度 (ρ) 演化 ───────────────────────────────┐")
    delta_rho = metrics.final_robustness - metrics.initial_robustness
    print(f"  │ ρ_initial = {metrics.initial_robustness:+.4f}")
    print(f"  │ ρ_final   = {metrics.final_robustness:+.4f}")
    print(f"  │ Δρ        = {delta_rho:+.4f}")
    status_icon = "[OK]" if metrics.correction_success else "[FAIL]"
    print(f"  │ 判定      : ρ > 0 → {status_icon}")
    print(f"  └──────────────────────────────────────────────────┘")

    # ── LLM 调用统计 ──
    print(f"\n  ┌─ LLM 调用统计 ──────────────────────────────────┐")
    print(f"  │ 初始生成             : 1 次")
    print(f"  │ 自愈修正             : {metrics.n_retry_rounds} 轮 ({metrics.n_llm_calls} 次 API 调用)")
    print(f"  │ 总 LLM 对话轮数       : {1 + metrics.n_llm_calls}")
    print(f"  └──────────────────────────────────────────────────┘")

    # ── 修正计划成功率 ──
    print(f"\n  ┌─ 修正计划成功率 ────────────────────────────────┐")
    print(f"  │ 最大重试次数         : {MAX_RETRIES}")
    print(f"  │ 实际重试轮数         : {metrics.n_retry_rounds}")
    print(f"  │ 修正成功             : {metrics.correction_success}")
    success_rate = "100%" if metrics.correction_success else "0%"
    print(f"  │ 修正成功率           : {success_rate}")
    print(f"  └──────────────────────────────────────────────────┘")

    # ── 违规节点改善 ──
    print(f"\n  ┌─ 违规节点改善 ──────────────────────────────────┐")
    print(f"  │ 初始违规节点数       : {metrics.violation_nodes_initial}")
    print(f"  │ 最终违规节点数       : {metrics.violation_nodes_final}")
    reduction = metrics.violation_nodes_initial - metrics.violation_nodes_final
    print(f"  │ 减少                 : {reduction} ({reduction/max(1,metrics.violation_nodes_initial)*100:.0f}%)")
    print(f"  └──────────────────────────────────────────────────┘")

    # ── RoboIR 对比 ──
    print(f"\n  ┌─ RoboIR 演化 ───────────────────────────────────┐")
    print(f"  │ 初始 intent : {roboir_initial.get('intent', '?')}")
    print(f"  │ 最终 intent : {roboir_final.get('intent', '?')}")
    init_constraints = roboir_initial.get("stl_constraints", [])
    final_constraints = roboir_final.get("stl_constraints", [])
    print(f"  │ 初始约束数  : {len(init_constraints)}")
    print(f"  │ 最终约束数  : {len(final_constraints)}")
    print(f"  └──────────────────────────────────────────────────┘")

    # ── 初始约束 vs 最终约束 ──
    if init_constraints != final_constraints:
        print(f"\n  ┌─ 约束变化详情 ──────────────────────────────────┐")
        for i, c in enumerate(final_constraints):
            c_str = json.dumps(c, ensure_ascii=False)
            is_new = c not in init_constraints
            marker = " [NEW]" if is_new else ""
            print(f"  │ [{i}]{marker} {c_str}")
        print(f"  └──────────────────────────────────────────────────┘")

    # ── 错误日志 ──
    if metrics.errors:
        print(f"\n  ┌─ 错误日志 ──────────────────────────────────────┐")
        for err in metrics.errors:
            print(f"  │ {err[:80]}")
        print(f"  └──────────────────────────────────────────────────┘")

    print(f"\n{sep}")
    verdict = "PASS" if metrics.correction_success else "FAIL (重试耗尽)"
    print(f"  最终判定: {verdict}")
    print(f"{sep}\n")


# ══════════════════════════════════════════════════════════════════════
# 8. 主入口
# ══════════════════════════════════════════════════════════════════════

def main(argv: Optional[List[str]] = None) -> int:
    parser = argparse.ArgumentParser(
        description="SMDSL 闭环自愈测试 — 端到端: CAD → RoboIR → 物理校验 → LLM 自愈",
    )
    parser.add_argument(
        "--cad_path", type=str, default=DEFAULT_CAD_PATH,
        help=f"CAD 图纸路径（JSON 或 DWG），默认 {DEFAULT_CAD_PATH}",
    )
    parser.add_argument(
        "--resolution", type=float, default=DEFAULT_RESOLUTION,
        help=f"栅格分辨率 m/px（默认 {DEFAULT_RESOLUTION}）",
    )
    parser.add_argument(
        "--robot_radius", type=float, default=DEFAULT_ROBOT_RADIUS,
        help=f"机器人物理半径 m（默认 {DEFAULT_ROBOT_RADIUS}）",
    )
    parser.add_argument(
        "--d_safe", type=float, default=DEFAULT_D_SAFE,
        help=f"STL Distance 安全阈值 m（默认 {DEFAULT_D_SAFE}）",
    )
    parser.add_argument(
        "--max_retries", type=int, default=MAX_RETRIES,
        help=f"LLM 自愈最大重试次数（默认 {MAX_RETRIES}）",
    )
    parser.add_argument(
        "--skip_llm", action="store_true",
        help="跳过 LLM 调用（仅测试场景初始化和轨迹合成）",
    )
    parser.add_argument(
        "--verbose", action="store_true",
        help="打印详细的违规节点信息",
    )

    args = parser.parse_args(argv)

    # ── 检查 DeepSeek API Key ──
    api_key = os.environ.get("DEEPSEEK_API_KEY", "").strip()
    if not api_key and not args.skip_llm:
        print("[WARNING] 环境变量 DEEPSEEK_API_KEY 未设置。")
        print("  闭环自愈需要 LLM 调用。将使用 --skip_llm 模式继续。")
        print("  设置方法: $env:DEEPSEEK_API_KEY = '<your-key>'")
        args.skip_llm = True

    print("=" * 62)
    print("  SMDSL 闭环自愈测试 — 端到端大闭环")
    print("=" * 62)
    print(f"  CAD: {args.cad_path}")
    print(f"  Resolution: {args.resolution} m/px")
    print(f"  Robot Radius: {args.robot_radius} m")
    print(f"  D_safe: {args.d_safe} m")
    print(f"  Max Retries: {args.max_retries}")
    print(f"  LLM Mode: {'SKIP' if args.skip_llm else 'DeepSeek-V4'}")
    print("=" * 62)

    metrics = RecoveryMetrics()

    # ─── Phase 1: 场景初始化 ───────────────────────────────────────
    t0 = time.perf_counter()
    print("\n[Phase 1/5] 场景初始化: 加载 CAD → 拓扑提取...")
    try:
        topology_bundle, pipeline_result, distance_field = init_scene(
            args.cad_path, args.resolution, args.robot_radius,
        )
        print(f"  距离场形状: {distance_field.shape}")
        print(f"  分辨率: {topology_bundle['grid_transform']['resolution']} m/px")
    except Exception as e:
        print(f"\n  [FATAL] 场景初始化失败: {e}")
        traceback.print_exc()
        return 1
    metrics.t_scene_init_ms = (time.perf_counter() - t0) * 1000.0

    # ─── Phase 2: 寻找自由空间中心 ──────────────────────────────────
    print("\n[Phase 2/5] 寻找自由空间中心点...")
    # 给 LLM 修正留 0.15 m 头部余量：若 LLM 把 Distance > 0.30 改严到 0.45,
    # 候选 centroids 仍能满足新的 clearance 要求，闭环才有真正的「成功」可能。
    centroids = find_free_centroids(
        distance_field, topology_bundle["grid_transform"], n=4,
        min_clearance_m=args.d_safe + 0.15,
    )
    if len(centroids) < 2:
        print("  [FATAL] 距离场中未找到至少 2 个自由空间中心点。")
        return 1
    print(f"  找到 {len(centroids)} 个自由空间中心点:")
    for i, (cx, cy) in enumerate(centroids):
        print(f"    [{i}] ({cx:.2f}, {cy:.2f})")

    # ─── Phase 3: 陷阱指令 + 初始 RoboIR 生成 ────────────────────────
    trap_instruction = build_trap_instruction(centroids)
    print(f"\n[Phase 3/5] 陷阱指令 + 初始 RoboIR 生成...")
    print(f"  陷阱指令: \"{trap_instruction}\"")

    # 合成必然穿墙的直线轨迹
    trajectory = synthesize_trajectory(centroids, n_points=N_TRAJECTORY_POINTS)
    print(f"  合成轨迹: {len(trajectory)} 点, "
          f"起点({trajectory[0]['x']:.2f}, {trajectory[0]['y']:.2f}) → "
          f"终点({trajectory[-1]['x']:.2f}, {trajectory[-1]['y']:.2f})")

    t0 = time.perf_counter()
    if args.skip_llm:
        # 无 LLM 模式：手动构造一个故意脆弱的 RoboIR
        roboir_initial: Dict[str, Any] = {
            "intent": "shortest_path_transport",
            "target_frame": "obstacle",
            "grasp_type": "none",
            "stl_constraints": [
                {"expr": "Time < 5.0"},
                # 故意不设置 Distance 约束 — 底层会注入 d_safe 导致碰撞
            ],
        }
        print("  [SKIP LLM] 使用手动构造的脆弱 RoboIR（无 Distance 约束）")
    else:
        try:
            from smdsl_demo.vlm_parser import parse_instruction_to_roboir
            roboir_initial = parse_instruction_to_roboir(
                trap_instruction,
                local_context={"nearest_objects": ["obstacle"]},
                max_retries=2,
            )
            metrics.n_llm_calls = 1
            print(f"  LLM 初始生成成功:")
            print(f"    intent: {roboir_initial.get('intent', '?')}")
            print(f"    constraints: {json.dumps(roboir_initial.get('stl_constraints', []), ensure_ascii=False)}")
        except Exception as e:
            print(f"  [WARNING] LLM 初始生成失败: {e}")
            print("  回退到手动构造的脆弱 RoboIR。")
            roboir_initial = {
                "intent": "shortest_path_transport",
                "target_frame": "obstacle",
                "grasp_type": "none",
                "stl_constraints": [
                    {"expr": "Time < 5.0"},
                ],
            }
    metrics.t_initial_gen_ms = (time.perf_counter() - t0) * 1000.0

    # ─── Phase 4: 物理校验 + 闭环自愈 ─────────────────────────────
    print(f"\n[Phase 4/5] 物理校验 + 闭环自愈...")

    # 4a. 初始校验
    t0 = time.perf_counter()
    validation_initial = validate_roboir(
        roboir_initial, trajectory, topology_bundle, d_safe=args.d_safe,
    )
    metrics.t_validation_ms = (time.perf_counter() - t0) * 1000.0

    print(f"  初始物理校验:")
    print(f"    ρ = {validation_initial['robustness']:+.4f}")
    print(f"    违规节点数: {validation_initial['n_violation_nodes']}")
    print(f"    通过: {validation_initial['passed']}")

    if args.verbose and validation_initial["n_violation_nodes"] > 0:
        for rpt in validation_initial["violation_report"]:
            viol_nodes = rpt.get("violation_nodes", [])
            if viol_nodes:
                worst = min(viol_nodes, key=lambda n: n.get("rho", 0.0))
                print(f"    最差违规节点: t={worst.get('t', '?'):.2f}s, "
                      f"pos=({worst.get('x', '?'):.2f}, {worst.get('y', '?'):.2f}), "
                      f"d_real={worst.get('d_real_m', '?'):.3f}m, ρ={worst.get('rho', '?'):.4f}")

    # 4b. 闭环自愈
    if args.skip_llm:
        print("\n  [SKIP LLM] 跳过闭环自愈循环（--skip_llm 模式或 API Key 未设置）")
        print("  展示初始校验结果作为参考。")
        roboir_final = roboir_initial
        validation_final = validation_initial
        metrics.initial_robustness = validation_initial["robustness"]
        metrics.final_robustness = validation_initial["robustness"]
        metrics.violation_nodes_initial = validation_initial["n_violation_nodes"]
        metrics.violation_nodes_final = validation_initial["n_violation_nodes"]
        metrics.correction_success = validation_initial["passed"]
        metrics.n_llm_calls = metrics.n_llm_calls  # 保留初始生成调用计数
    else:
        roboir_final, validation_final, metrics = run_correction_loop(
            roboir=roboir_initial,
            validation_result=validation_initial,
            trajectory=trajectory,
            topology_bundle=topology_bundle,
            original_instruction=trap_instruction,
            d_safe=args.d_safe,
            max_retries=args.max_retries,
            pipeline_result=pipeline_result,
            start_xy=centroids[0],
            goal_xy=centroids[-1],
        )

    # ─── Phase 5: 诊断报告 ────────────────────────────────────────
    print(f"\n[Phase 5/5] 生成诊断报告...")
    print_diagnostic_report(
        roboir_initial=roboir_initial,
        roboir_final=roboir_final,
        validation_initial=validation_initial,
        validation_final=validation_final,
        metrics=metrics,
        cad_path=args.cad_path,
        d_safe=args.d_safe,
    )

    return 0 if metrics.correction_success else 2


if __name__ == "__main__":
    sys.exit(main())
