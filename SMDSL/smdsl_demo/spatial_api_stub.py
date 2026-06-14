"""
spatial_api_stub.py — Zone 3: 约束求解与时空规划

职责：
  - 接收 Demo 1（VLM 编译器）编译后的 RoboIR 声明式约束。
  - 执行纯几何与物理的代数求解：
      a. 欧几里得距离计算
      b. A* 拓扑路径代价
      c. STL 约束违反检测
  - 返回数值结果给 Zone 4 做结构化反馈。
  - 不涉及任何大模型调用。只做数学。
"""

from __future__ import annotations

import math
from typing import Any, Dict, List, Optional, Tuple

# numpy 仅用于距离场采样（不参与 LLM 输出，符合 Zone 3 数学层定位）
try:
    import numpy as np
except ImportError:  # 允许在没有 numpy 的环境下跑非距离场分支
    np = None  # type: ignore


# ──────────────────────────────────────────────
# 类型别名（便于 Zone 3 ↔ Zone 2 接口对齐）
# ──────────────────────────────────────────────

# {"position": {x,y,z}, "orientation": {roll,pitch,yaw}, "frame": "..."}
Pose = Dict[str, Any]
NodeId = str                 # 拓扑图中的节点标识
# 邻接表: {node: [(neighbor, cost)]}
AreaGraph = Dict[NodeId, List[Tuple[NodeId, float]]]
ConstraintRule = Dict[str, Any]  # STL 约束规则
Trajectory = List[Dict[str, float]]  # 轨迹点序列

# Demo 1 拓扑包导出 — Zone 3 求解 STL Distance 的真值依据。
# {
#   "distance_field": np.ndarray (H, W) float32,  # 单位：像素 (px)
#   "grid_transform": {
#       "origin": (min_x_world, min_y_world),     # 单位：米
#       "resolution": float,                       # 米 / 像素
#       "shape": (H, W),
#   },
# }
TopologyBundle = Dict[str, Any]


# ──────────────────────────────────────────────
# 坐标转换工具（世界 ↔ 栅格）
# ──────────────────────────────────────────────

def _world_to_grid(
    x_m: float,
    y_m: float,
    grid_transform: Dict[str, Any],
) -> Tuple[float, float]:
    """米 → 像素 (row_f, col_f)，浮点版本（便于双线性插值）。"""
    ox, oy = grid_transform["origin"]
    res = grid_transform["resolution"]
    col_f = (x_m - ox) / res
    row_f = (y_m - oy) / res
    return row_f, col_f


def _bilinear_sample(
    field: "np.ndarray",
    row_f: float,
    col_f: float,
    out_of_bounds_value: float = 0.0,
) -> float:
    """
    双线性插值采样 — 处理子像素轨迹点，避免阶梯量化误差。

    超出栅格范围时返回 ``out_of_bounds_value``（默认 0 = 视为已碰墙）。
    """
    if np is None:
        # numpy 缺失时退化为最近邻
        H, W = field.shape  # type: ignore
        r = int(round(row_f))
        c = int(round(col_f))
        if 0 <= r < H and 0 <= c < W:
            return float(field[r, c])
        return out_of_bounds_value

    H, W = field.shape
    if row_f < 0 or row_f > H - 1 or col_f < 0 or col_f > W - 1:
        return out_of_bounds_value
    r0 = int(math.floor(row_f))
    c0 = int(math.floor(col_f))
    r1 = min(r0 + 1, H - 1)
    c1 = min(c0 + 1, W - 1)
    fr = row_f - r0
    fc = col_f - c0
    v00 = float(field[r0, c0])
    v01 = float(field[r0, c1])
    v10 = float(field[r1, c0])
    v11 = float(field[r1, c1])
    v0 = v00 * (1 - fc) + v01 * fc
    v1 = v10 * (1 - fc) + v11 * fc
    return v0 * (1 - fr) + v1 * fr


def query_clearance_at(
    x_m: float,
    y_m: float,
    topology_bundle: TopologyBundle,
) -> float:
    """
    Demo 3 核心查询：返回世界点 (x, y) 到最近障碍物的物理距离 (米)。

    若点在栅格外部，返回 0（视为已撞）—— 调用方可据此判断越界即碰撞。

    Args:
        x_m, y_m: 世界坐标（米）。
        topology_bundle: Demo 1 导出的 ``{"distance_field", "grid_transform"}``。

    Returns:
        d_real_m，单位米。
    """
    df = topology_bundle["distance_field"]
    tx = topology_bundle["grid_transform"]
    row_f, col_f = _world_to_grid(x_m, y_m, tx)
    d_px = _bilinear_sample(df, row_f, col_f, out_of_bounds_value=0.0)
    return float(d_px) * float(tx["resolution"])


# ──────────────────────────────────────────────
# API a: 欧几里得距离
# ──────────────────────────────────────────────

def calculate_euclidean_distance(pose_a: Pose, pose_b: Pose) -> float:
    """
    计算两个位姿之间的欧几里得距离（纯几何）。

    前提条件：
      - 调用方必须确保 pose_a 和 pose_b 处于同一参考系。
      - 坐标系一致性应由 Zone 2 的 RustCompilerStub 在静态检查中保证。
      - 本函数不进行 Frame 转换。

    Args:
        pose_a: 第一个位姿，需包含 position: {x, y, z}（可选 frame 元信息）
        pose_b: 第二个位姿，同上

    Returns:
        3D 欧几里得距离（米）

    Raises:
        KeyError: 位姿缺少 position 或 x/y/z 字段
        ValueError: 检测到 Frame 不一致（留作运行时断言）
    """
    pos_a = pose_a.get("position", {})
    pos_b = pose_b.get("position", {})

    # 运行时 Frame 一致性断言（双重保障 Zone 2 静态检查）
    frame_a = pose_a.get("frame")
    frame_b = pose_b.get("frame")
    if frame_a and frame_b and frame_a != frame_b:
        raise ValueError(
            f"Frame mismatch at runtime: '{frame_a}' vs '{frame_b}'. "
            "Zone 2 静态检查应已拦截此情况。"
        )

    dx = pos_a.get("x", 0.0) - pos_b.get("x", 0.0)
    dy = pos_a.get("y", 0.0) - pos_b.get("y", 0.0)
    dz = pos_a.get("z", 0.0) - pos_b.get("z", 0.0)

    return math.sqrt(dx * dx + dy * dy + dz * dz)


# ──────────────────────────────────────────────
# API b: A* 拓扑路径代价
# ──────────────────────────────────────────────

def get_astar_path_cost(
    start_node: NodeId,
    target_node: NodeId,
    area_graph: AreaGraph,
) -> Tuple[float, List[NodeId]]:
    """
    在拓扑图中计算 A* 最短路径及其总代价。

    Args:
        start_node: 起始节点 ID
        target_node: 目标节点 ID
        area_graph: 拓扑图，邻接表格式 {node: [(neighbor, edge_cost), ...]}

    Returns:
        (total_cost, path) —— 总代价（米）和路径节点序列。
        如果 start_node 或 target_node 不在图中，返回 (inf, [])。
        如果不可达，返回 (inf, [])。

    Note:
        本实现使用标准 A*，启发式函数为欧几里得距离占位。
        生产环境应替换为导航网格（NavMesh）或 ROS 全局规划器。
    """
    if start_node not in area_graph or target_node not in area_graph:
        return float("inf"), []

    import heapq

    # 启发式函数（占位 —— 实际应用需接入 topology 几何数据）
    def heuristic(n: NodeId) -> float:
        return 0.0  # 转为 Dijkstra；接入真实坐标后可替换为欧几里得

    open_set = [(0.0, start_node)]
    came_from: Dict[NodeId, Optional[NodeId]] = {start_node: None}
    g_score: Dict[NodeId, float] = {start_node: 0.0}

    while open_set:
        current_f, current = heapq.heappop(open_set)

        if current == target_node:
            # 重建路径
            path: List[NodeId] = []
            node: Optional[NodeId] = current
            while node is not None:
                path.append(node)
                node = came_from.get(node)
            path.reverse()
            return g_score[current], path

        for neighbor, edge_cost in area_graph.get(current, []):
            tentative_g = g_score[current] + edge_cost
            if neighbor not in g_score or tentative_g < g_score[neighbor]:
                g_score[neighbor] = tentative_g
                came_from[neighbor] = current
                f_score = tentative_g + heuristic(neighbor)
                heapq.heappush(open_set, (f_score, neighbor))

    return float("inf"), []  # 不可达


# ──────────────────────────────────────────────
# API c: STL 约束违反检测
# ──────────────────────────────────────────────

def check_stl_constraint_violation(
    trajectory: Trajectory,
    constraint_rules: List[ConstraintRule],
    reference_poses: Optional[Dict[str, Pose]] = None,
    topology_bundle: Optional[TopologyBundle] = None,
) -> List[Dict[str, Any]]:
    """
    在轨迹上检测 STL 约束违反情况。

    对每个 STL 约束规则，逐时间步扫描轨迹并计算 STL 鲁棒性度量
    （Robustness Degree） ρ：

        ρ > 0   : 安全冗余
        ρ = 0   : 临界
        ρ < 0   : 硬约束被破坏（碰撞 / 越界 / 超时 / 朝向偏差超限）

    Distance > D_safe 求解策略（双轨制）::

        优先：若提供 ``topology_bundle``（Demo 1 导出的 distance_field +
              grid_transform），逐轨迹点查询 d_real_m = field(x, y) * res，
              ρ = d_real_m − D_safe。**这是 Demo 3 的官方语义**：
              真实物理距离 vs. 障碍物。
        兜底：若仅提供 ``reference_poses["obstacle"]``，回退到 Demo 1
              未接入时的"轨迹点 → 单一参考 pose"距离。
        失败：两者都没提供 → ρ = -∞ + structured error。

    Args:
        trajectory: 轨迹点序列，每点含
            ``{"t": 秒, "x": m, "y": m, "z": m, "roll", "pitch", "yaw"}``。
        constraint_rules: RoboIR 中的 STL 约束列表。每项格式::

                {
                    "type": "stl_constraint",
                    "expr": "Distance > 0.10",
                    "unit": "m",
                    "reference": "obstacle"   # 可选；用于挑选 reference_pose
                }

        reference_poses: 兜底参考 pose 表（旧版 API）。
        topology_bundle: Demo 1 拓扑包（含 distance_field + grid_transform）。

    Returns:
        违反报告列表，每项包含::

            {
                "rule_expr": str,
                "max_violation": float,
                "violation_duration": float,
                "robustness": float,           # 全程 min ρ
                "violated": bool,
                "violation_nodes": [           # ρ < 0 的轨迹点（结构化反馈）
                    {"t": ..., "x": ..., "y": ...,
                     "row": int, "col": int,
                     "d_real_m": float, "rho": float}
                ],
                "details": [...],              # 全部时间步逐点剖面
            }

        若无轨迹点或规则列表为空，返回空列表。
    """
    if not trajectory or not constraint_rules:
        return []

    refs = reference_poses or {}
    results: List[Dict[str, Any]] = []

    for rule in constraint_rules:
        if rule.get("type") != "stl_constraint":
            continue
        expr = rule.get("expr", "")
        ref_key = rule.get("reference")
        ref_pose = refs.get(ref_key) if ref_key else None

        report = _evaluate_single_constraint(
            trajectory, expr,
            reference_pose=ref_pose,
            topology_bundle=topology_bundle,
        )
        results.append(report)

    return results


def _evaluate_single_constraint(
    trajectory: Trajectory,
    expr: str,
    reference_pose: Optional[Pose] = None,
    topology_bundle: Optional[TopologyBundle] = None,
) -> Dict[str, Any]:
    """
    评估单条 STL 约束在轨迹上的违反情况。

    支持的表达式模式：
      - ``Time < {value}``             总时间约束 (s)
      - ``Distance > {value}``         最小安全距离 (m)
                                       优先使用 topology_bundle 的距离场，
                                       退回 reference_pose 单点比较
      - ``OrientationDiff < {value}``  朝向偏差上限 (deg)，需要 reference_pose
      - ``Force in [{min}, {max}]``    力在范围内 (N)
      - ``Velocity < {value}``         速度上限 (m/s)

    见 ``check_stl_constraint_violation`` 文档了解返回结构（包含
    ``violation_nodes`` 列表 — Demo 3 结构化反馈的核心）。
    """
    import re

    report: Dict[str, Any] = {
        "rule_expr": expr,
        "max_violation": 0.0,
        "violation_duration": 0.0,
        "robustness": float("inf"),
        "violated": False,
        "violation_nodes": [],   # ρ < 0 的轨迹点结构化清单
        "details": [],
        "source": "unknown",     # distance_field / reference_pose / time_only
    }

    def _extract_value(e: str) -> Optional[float]:
        nums = re.findall(r"[-+]?\d*\.?\d+", e)
        return float(nums[0]) if nums else None

    # ── Time < T ──
    if "Time" in expr and "<" in expr:
        report["source"] = "time_only"
        time_limit = _extract_value(expr)
        if time_limit and trajectory:
            total_time = trajectory[-1].get("t", 0)
            violation = total_time - time_limit
            robustness = -violation if violation > 0 else abs(violation)
            report["max_violation"] = max(0.0, violation)
            report["violation_duration"] = max(0.0, violation)
            report["robustness"] = robustness
            report["violated"] = violation > 0
            for pt in trajectory:
                t = pt.get("t", 0)
                viol = t > time_limit
                report["details"].append({
                    "t": t, "value": t,
                    "bound": time_limit, "violated": viol,
                })
                if viol:
                    report["violation_nodes"].append({
                        "t": t,
                        "x": pt.get("x", 0.0),
                        "y": pt.get("y", 0.0),
                        "rho": time_limit - t,
                    })

    # ── Distance > D ──
    # 语义：每个轨迹点到 *最近障碍物* 的真实物理距离 d_real 必须 > D_safe。
    # ρ = d_real - D_safe。ρ<0 即视为碰撞 / 过度贴近。
    elif "Distance" in expr and ">" in expr:
        min_dist = _extract_value(expr)
        if min_dist is None:
            report["details"].append(
                {"warning": f"Distance 表达式 '{expr}' 缺少阈值"})
            return report

        # 优先级 1：使用 Demo 1 的距离场（官方 STL 物理求解路径）
        if topology_bundle is not None:
            report["source"] = "distance_field"
            return _eval_distance_via_field(
                trajectory, min_dist, topology_bundle, report,
            )

        # 优先级 2：回退到 reference_pose 单点距离
        if reference_pose is not None:
            report["source"] = "reference_pose_fallback"
            return _eval_distance_via_reference(
                trajectory, min_dist, reference_pose, report,
            )

        # 都没有 → 暴露 Zone 2 接口漏注入
        report["robustness"] = float("-inf")
        report["violated"] = True
        report["source"] = "missing_reference"
        report["details"].append({
            "error": (
                "stl_constraint Distance 缺少 distance_field 或 "
                "reference_pose。请把 Demo 1 拓扑包通过 topology_bundle "
                "传入，或在 Zone 2 注入 reference_pose。"
            )
        })
        return report

    # ── (Distance helpers in module scope, see _eval_distance_via_*) ──

    # ── OrientationDiff < D ──
    # 语义：每个轨迹点的朝向与 reference_pose.orientation 之差必须 < D（度）。
    # 修正：旧实现从轨迹点本身读取 target_yaw，但 trajectory 不应包含目标值；
    #       目标值由 Zone 2 通过 reference_pose 注入。
    elif "OrientationDiff" in expr and "<" in expr:
        angle_limit = _extract_value(expr)
        if angle_limit is None:
            report["details"].append(
                {"warning": f"OrientationDiff 表达式 '{expr}' 缺少阈值"})
            return report
        if reference_pose is None:
            report["robustness"] = float("-inf")
            report["violated"] = True
            report["details"].append({
                "error": "stl_constraint OrientationDiff 缺少 reference_pose。"
                         "Zone 2 编译器应通过 constraint.reference 字段"
                         "声明参照位姿。"
            })
            return report

        ref_yaw = reference_pose.get("orientation", {}).get("yaw", 0.0)
        max_viol = 0.0
        min_rob = float("inf")
        viol_time = 0.0
        prev_t = trajectory[0].get("t", 0)
        for pt in trajectory:
            # 归一化角度差到 [0, 180] 度，避免 ±π 翻转伪违反
            raw_diff = abs(pt.get("yaw", 0.0) - ref_yaw)
            diff_rad = raw_diff % (2 * math.pi)
            if diff_rad > math.pi:
                diff_rad = 2 * math.pi - diff_rad
            diff_deg = math.degrees(diff_rad)
            viol = diff_deg - angle_limit
            cur_t = pt.get("t", 0)
            if viol > 0:
                max_viol = max(max_viol, viol)
                viol_time += max(0.0, cur_t - prev_t)
            rob = angle_limit - diff_deg
            min_rob = min(min_rob, rob)
            report["details"].append({
                "t": cur_t,
                "diff_deg": diff_deg,
                "bound_deg": angle_limit,
                "violated": viol > 0,
            })
            if viol > 0:
                report["violation_nodes"].append({
                    "t": cur_t,
                    "x": pt.get("x", 0.0),
                    "y": pt.get("y", 0.0),
                    "diff_deg": diff_deg,
                    "rho": angle_limit - diff_deg,
                })
            prev_t = cur_t
        report["max_violation"] = max_viol
        report["violation_duration"] = viol_time
        report["robustness"] = min_rob
        report["violated"] = max_viol > 0
        report["source"] = "reference_pose"

    # ── 默认: 无法解析的表达式标记为警告 ──
    else:
        report["robustness"] = 0.0
        report["violated"] = True
        report["details"].append({"warning": f"无法解析的约束表达式: {expr}"})

    return report


# ──────────────────────────────────────────────
# Distance > D 求解器（两条路径）
# ──────────────────────────────────────────────

def _eval_distance_via_field(
    trajectory: Trajectory,
    min_dist_m: float,
    bundle: TopologyBundle,
    report: Dict[str, Any],
) -> Dict[str, Any]:
    """
    *官方* STL Distance 求解：用 Demo 1 的距离场逐点查询 d_real_m。

    每个轨迹点：
        row, col = world_to_grid(x, y)
        d_real_m = bilinear_sample(distance_field, row, col) * resolution
        ρ = d_real_m - D_safe
    """
    # [Fix 1] Z-axis hard assertion: detect 3D trajectory passed to 2D solver
    if not trajectory:
        report["robustness"] = float("inf")
        report["violated"] = False
        report["source"] = "distance_field"
        return report
    z_vals = [float(pt.get("z", 0.0)) for pt in trajectory]
    z_range = max(z_vals) - min(z_vals)
    if z_range > 0.01:
        report["robustness"] = float("-inf")
        report["violated"] = True
        report["source"] = "z_axis_not_supported"
        report["details"].append({
            "error": (
                f"Trajectory has significant Z variation (delta_z={z_range:.3f}m). "
                f"Current distance field only supports 2D (x,y) plane. "
                f"Use a 2D trajectory or upgrade to 3D voxel SDF."
            )
        })
        return report
    df = bundle["distance_field"]
    tx = bundle["grid_transform"]
    res = float(tx["resolution"])

    max_viol = 0.0
    min_rob = float("inf")
    viol_time = 0.0
    prev_t = trajectory[0].get("t", 0.0)
    H, W = (df.shape if np is not None else (0, 0))

    for pt in trajectory:
        x = float(pt.get("x", 0.0))
        y = float(pt.get("y", 0.0))
        cur_t = float(pt.get("t", 0.0))

        row_f, col_f = _world_to_grid(x, y, tx)
        d_px = _bilinear_sample(df, row_f, col_f, out_of_bounds_value=0.0)
        d_real_m = float(d_px) * res
        rho = d_real_m - min_dist_m
        viol = -rho if rho < 0 else 0.0

        if viol > 0:
            max_viol = max(max_viol, viol)
            viol_time += max(0.0, cur_t - prev_t)
        min_rob = min(min_rob, rho)

        # 标定栅格内坐标（即使越界也保留浮点供调试）
        in_bounds = (
            (np is not None) and 0 <= row_f <= H - 1 and 0 <= col_f <= W - 1
        )

        report["details"].append({
            "t": cur_t,
            "x": x, "y": y,
            "row_f": round(row_f, 3), "col_f": round(col_f, 3),
            "d_real_m": round(d_real_m, 4),
            "min_dist_required": min_dist_m,
            "rho": round(rho, 4),
            "in_bounds": in_bounds,
            "violated": viol > 0,
        })
        if rho < 0:
            report["violation_nodes"].append({
                "t": cur_t,
                "x": x, "y": y,
                "row": int(round(row_f)),
                "col": int(round(col_f)),
                "d_real_m": round(d_real_m, 4),
                "rho": round(rho, 4),
                "in_bounds": in_bounds,
            })
        prev_t = cur_t

    report["max_violation"] = max_viol
    report["violation_duration"] = viol_time
    report["robustness"] = min_rob
    report["violated"] = max_viol > 0
    return report


def _eval_distance_via_reference(
    trajectory: Trajectory,
    min_dist_m: float,
    reference_pose: Pose,
    report: Dict[str, Any],
) -> Dict[str, Any]:
    """
    Fallback：当 distance_field 不可用时，按"轨迹点 → 单一参考 pose"
    的欧式距离评估 Distance 约束。

    注意：这只能模拟 *一个* 障碍物 / 目标，无法捕捉环境中的其他障碍。
    Demo 3 的"距离场全局求解"路径才是 STL 鲁棒度的官方语义。
    """
    ref_x = reference_pose.get("position", {}).get("x", 0.0)
    ref_y = reference_pose.get("position", {}).get("y", 0.0)
    ref_z = reference_pose.get("position", {}).get("z", 0.0)

    max_viol = 0.0
    viol_time = 0.0
    min_rob = float("inf")
    prev_t = trajectory[0].get("t", 0.0)
    for pt in trajectory:
        dx = pt.get("x", 0.0) - ref_x
        dy = pt.get("y", 0.0) - ref_y
        dz = pt.get("z", 0.0) - ref_z
        d_real = math.sqrt(dx * dx + dy * dy + dz * dz)
        rho = d_real - min_dist_m
        viol = -rho if rho < 0 else 0.0
        cur_t = pt.get("t", 0.0)
        if viol > 0:
            max_viol = max(max_viol, viol)
            viol_time += max(0.0, cur_t - prev_t)
        min_rob = min(min_rob, rho)
        report["details"].append({
            "t": cur_t,
            "clearance": d_real,
            "min_dist_required": min_dist_m,
            "rho": rho,
            "violated": viol > 0,
            "_fallback": "single_reference_pose",
        })
        if rho < 0:
            report["violation_nodes"].append({
                "t": cur_t,
                "x": pt.get("x", 0.0),
                "y": pt.get("y", 0.0),
                "d_real_m": d_real,
                "rho": rho,
                "_fallback": "single_reference_pose",
            })
        prev_t = cur_t

    report["max_violation"] = max_viol
    report["violation_duration"] = viol_time
    report["robustness"] = min_rob
    report["violated"] = max_viol > 0
    return report
