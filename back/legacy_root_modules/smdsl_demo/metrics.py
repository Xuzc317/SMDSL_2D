"""
metrics.py — Zone 4: 闭环执行与结构化反馈

聚焦两个核心职责：
  1. 量化评估指标（parsing_accuracy, precision_recall, stl_robustness, ...）
  2. 结构化诊断反馈 — FailureTaxonomy + generate_structured_feedback()
     向 VLM 返回带有具体错误类型的诊断报告，驱动大模型自我修正。
"""

from __future__ import annotations

from typing import Dict, List, Any, Optional, Tuple
import json
import math
from enum import Enum

# 双重导入兜底：既支持 ``python smdsl_demo/app.py``（裸名），
# 也支持 ``python -m smdsl_demo.metrics``（包名）。
try:
    from .spatial_api_stub import check_stl_constraint_violation as _check_stl
    from .rust_compiler_stub import RustCompilerStub as _RustCompilerStub
except ImportError:  # 兜底：裸名（CWD 在 smdsl_demo/）
    from spatial_api_stub import check_stl_constraint_violation as _check_stl  # type: ignore
    from rust_compiler_stub import RustCompilerStub as _RustCompilerStub  # type: ignore


# ══════════════════════════════════════════════
# FailureTaxonomy — 结构化错误类型枚举
# ══════════════════════════════════════════════

class FailureTaxonomy(str, Enum):
    """
    结构化错误类型分类 —— 用于 Zone 4 诊断报告。

    每一类对应一种可被 VLM 理解并修正的语义级错误。
    """
    FRAME_MISMATCH = "frame_mismatch"
    """异构坐标系直接相加。例如 tool0 下的位移加到 map 坐标上。"""

    STL_VIOLATION = "stl_violation"
    """违反时空约束。例如轨迹执行过程中超过了 Time / Distance / 
    Velocity 等 STL 约束的边界。"""

    COLLISION_WALL = "collision_wall"
    """碰撞检测（规划期）—— 规划轨迹与已知墙面 / 静态家具几何相交。"""

    COLLISION = "collision"
    """碰撞检测（运行期，距离场驱动）—— 由 Demo 1 的全局 EDT 距离场
    判定：轨迹点到最近障碍物的真实物理距离 d_real_m < D_safe，
    即 STL 鲁棒度 ρ = d_real_m − D_safe < 0。
    与 COLLISION_WALL 的区别：
      - COLLISION_WALL：只能在编译期/规划期发现的几何相交。
      - COLLISION：执行轨迹相对真实环境的连续鲁棒度违反，
        含定量 ρ 值与每个违规栅格点的位置（违规节点列表）。"""

    GROUNDING_FAILED = "grounding_failed"
    """实体锚定失败 —— 自然语言中的指代对象未在 CAD 节点中找到。"""

    POSE_UNDEFINED = "pose_undefined"
    """引用的位姿标签未在 pose_declarations 中声明。"""

    GRASP_TYPE_INVALID = "grasp_type_invalid"
    """抓取类型不合法或与目标物不兼容。"""

    STL_SYNTAX_ERROR = "stl_syntax_error"
    """STL 约束表达式语法错误。"""

    TIMEOUT = "timeout"
    """执行超时 —— 任务未在指定时间窗口内完成。"""


# ══════════════════════════════════════════════
# 结构化反馈函数
# ══════════════════════════════════════════════

def generate_structured_feedback(
    error_type: FailureTaxonomy,
    robustness_score: float,
    context: Optional[Dict[str, Any]] = None,
) -> Dict[str, Any]:
    """
    生成向 VLM 返回的可解释结构化错误日志。

    该日志包含：
      - 错误分类（枚举）
      - 定量诊断（鲁棒性分值、违反程度）
      - 错误位置（action ID、时间戳、相关坐标系）
      - 建议修正方向

    Args:
        error_type: FailureTaxonomy 枚举成员，标明错误大类
        robustness_score: STL 鲁棒性度量值。
                          正值 = 安全冗余；0 = 临界；负值 = 违反。
        context: 可选的附加上下文信息，可包含：
                 - "action_id": 出错的 action 编号
                 - "frame_a"/"frame_b": 涉及的坐标系
                 - "expected"/"actual": 期望值与实际值
                 - "trajectory_segment": 出错轨迹片段
                 - "entity": 锚定失败的实体名称

    Returns:
        结构化反馈字典，可直接序列化为 JSON 供 VLM 阅读。

    Example:
        >>> generate_structured_feedback(
        ...     FailureTaxonomy.FRAME_MISMATCH, -0.3,
        ...     {"action_id": 2, "frame_a": "tool0", "frame_b": "map"}
        ... )
        {
            "error": {
                "type": "frame_mismatch",
                "severity": "error",
                "robustness_score": -0.3
            },
            "diagnosis": "...
        }
    """
    # ── 严重级别判定 ──
    if robustness_score < -0.5:
        severity = "critical"
    elif robustness_score < 0:
        severity = "error"
    elif robustness_score < 0.1:
        severity = "warning"
    else:
        severity = "info"

    # ── 错误信息模板 ──
    error_messages = {
        FailureTaxonomy.FRAME_MISMATCH: {
            "summary": "异构坐标系直接相加",
            "suggestion": "检查涉及的坐标系族是否一致，"
                          "或在运算前插入 Frame 变换节点。",
            "hint": "考虑在 RoboIR 的 frame_declarations 中添加 "
                    "Transform 声明来显式转换坐标系。",
        },
        FailureTaxonomy.STL_VIOLATION: {
            "summary": "违反 STL 时空约束",
            "suggestion": "调整轨迹参数（速度/路径）或放宽约束边界。",
            "hint": "检查 trajectory 是否在约束边界附近振荡；"
                    "可增加安全余量。",
        },
        FailureTaxonomy.COLLISION_WALL: {
            "summary": "轨迹与已知墙面 / 家具几何相交（规划期）",
            "suggestion": "重新规划避障路径，或添加中间途经点。",
            "hint": "考虑将障碍物区域标记为拓扑图中的不可通行节点。",
        },
        FailureTaxonomy.COLLISION: {
            "summary": "轨迹相对真实环境的鲁棒度违反（距离场驱动）",
            "suggestion": "降低任务安全余量、增加避障路径点，"
                          "或在 STL 中放宽 Distance 阈值。",
            "hint": "查看 context.violation_nodes 获取每个违规栅格点的 "
                    "(t, x, y, d_real_m, ρ)，针对最深违规点重规划。",
        },
        FailureTaxonomy.GROUNDING_FAILED: {
            "summary": "实体锚定失败",
            "suggestion": "检查指令中的实体名称是否在 CAD 节点库中。",
            "hint": "使用同义词扩展或提供更精确的实体描述。",
        },
        FailureTaxonomy.POSE_UNDEFINED: {
            "summary": "引用了未声明的 Pose 标签",
            "suggestion": "在 pose_declarations 中添加缺失的 Pose 声明。",
            "hint": "确保 actions 中所有 target_pose 已在声明区定义。",
        },
        FailureTaxonomy.GRASP_TYPE_INVALID: {
            "summary": "抓取类型与目标物不兼容",
            "suggestion": "更换抓取类型或检查目标物几何属性。",
            "hint": "参考 CAD 节点中目标物的 material/weight 属性。",
        },
        FailureTaxonomy.STL_SYNTAX_ERROR: {
            "summary": "STL 约束表达式语法错误",
            "suggestion": "修正 STL 表达式格式，使用合法操作符。",
            "hint": "合法操作符: Time, Distance, OrientationDiff, "
                    "Velocity, Force, Torque。",
        },
        FailureTaxonomy.TIMEOUT: {
            "summary": "执行超时",
            "suggestion": "缩短路径或提高执行速度。",
            "hint": "检查 STL 约束 'Time < T' 的边界值是否合理。",
        },
    }

    msg = error_messages.get(
        error_type,
        {"summary": "未知错误", "suggestion": "请人工介入检查", "hint": ""},
    )

    # ── 组装反馈 ──
    feedback: Dict[str, Any] = {
        "error": {
            "type": error_type.value,
            "severity": severity,
            "robustness_score": round(robustness_score, 4),
        },
        "diagnosis": {
            "summary": msg["summary"],
            "suggestion": msg["suggestion"],
            "hint": msg["hint"],
        },
        "context": context or {},
    }

    # 注入 worst_node 精确几何证据（如果存在）
    worst = (context or {}).get("worst_node")
    if isinstance(worst, dict) and worst:
        t = float(worst.get("t", 0))
        x = float(worst.get("x", 0))
        y = float(worst.get("y", 0))
        d_real = float(worst.get("d_real_m", 0))
        rho = float(worst.get("rho", 0))
        feedback["diagnosis"]["semantic_hint"] = (
            f"违规发生在时间 t={t:.2f}s，"
            f"具体坐标为 (x={x:.2f}, y={y:.2f})。"
            f"该点的实际距离 d_real_m={d_real:.3f}m，"
            f"低于安全阈值，导致鲁棒度 rho={rho:.3f}。"
        )

    # 根据严重级别补充额外诊断信息
    if severity in ("critical", "error") and robustness_score < 0:
        feedback["diagnosis"]["violation_magnitude"] = round(
            abs(robustness_score), 4
        )
        feedback["recommended_action"] = (
            "requires_recompile" if severity == "critical" else "adjust_params"
        )

    return feedback


# ══════════════════════════════════════════════
# 原有指标函数（保持向后兼容 + Zone 4 增强）
# ══════════════════════════════════════════════

def parsing_accuracy(
    robo_ir_str: str,
    schema: Optional[Dict[str, Any]] = None,
) -> float:
    """
    语法与编译级 —— JSON Schema 及 Frame 坐标系校验通过率。

    增强：集成 RustCompilerStub 的 ValidationError 枚举，
    返回细粒度错误计数而非单一的布尔值。

    Args:
        robo_ir_str: RoboIR 字符串
        schema: JSON Schema 定义文件

    Returns:
        0.0 ~ 1.0 的校验通过率
    """
    stubs = _RustCompilerStub()
    valid, errors = stubs.validate(robo_ir_str)
    if not valid:
        # 带权衰减：每种错误类型减 0.15
        error_types = set(e.error_type.value for e in errors)
        return max(0.0, 1.0 - len(error_types) * 0.15)
    return 1.0


def precision_recall(
    ground_truth: Dict[str, Any],
    generated: Dict[str, Any],
) -> float:
    """
    代码精度与召回率 —— 约束细节的完整性。

    Args:
        ground_truth: 人工标注的标准 SMDSL 约束
        generated: VLM 生成的 SMDSL 约束

    Returns:
        F1 分数（0.0 ~ 1.0）
    """
    gt_actions = ground_truth.get("actions", [])
    gen_actions = generated.get("actions", [])

    def _extract_constraint_sigs(actions: List[Dict]) -> set:
        """提取约束签名集合用于匹配。"""
        sigs = set()
        for act in actions:
            for c in act.get("constraints", []):
                sigs.add((act.get("type"), c.get("type", ""), c.get("expr", "")))
        return sigs

    gt_sigs = _extract_constraint_sigs(gt_actions)
    gen_sigs = _extract_constraint_sigs(gen_actions)

    if not gt_sigs:
        return 1.0 if not gen_sigs else 0.0

    true_positives = len(gt_sigs & gen_sigs)
    false_positives = len(gen_sigs - gt_sigs)
    false_negatives = len(gt_sigs - gen_sigs)

    precision = true_positives / (true_positives + false_positives) \
        if (true_positives + false_positives) > 0 else 0.0
    recall = true_positives / (true_positives + false_negatives) \
        if (true_positives + false_negatives) > 0 else 0.0

    if precision + recall == 0:
        return 0.0
    return 2 * precision * recall / (precision + recall)


def stl_robustness(
    trajectory: List[Dict[str, float]],
    constraints: List[Dict[str, Any]],
    timestep: float = 0.1,
    reference_poses: Optional[Dict[str, Dict[str, Any]]] = None,
    topology_bundle: Optional[Dict[str, Any]] = None,
) -> float:
    """
    Signal Temporal Logic 定量鲁棒性度量 ρ = min over (constraints, t)。

    Args:
        trajectory: 机器人轨迹点序列
        constraints: SMDSL STL 约束列表
        timestep: 轨迹采样间隔（秒）
        reference_poses: 兜底参考 Pose 表（Distance fallback / OrientationDiff）
        topology_bundle: Demo 1 距离场拓扑包 — Distance 约束的官方求解依据。
                         参见 spatial_api_stub.check_stl_constraint_violation。

    Returns:
        ρ 鲁棒性值（最小值取遍所有约束和时间步）。
        无轨迹或无约束时返回 0.0（中性，无信息）。
    """
    if not trajectory or not constraints:
        return 0.0

    violation_reports = _check_stl(
        trajectory, constraints,
        reference_poses=reference_poses,
        topology_bundle=topology_bundle,
    )
    if not violation_reports:
        return float("inf")

    min_robustness = min(
        r.get("robustness", float("inf")) for r in violation_reports
    )
    return min_robustness


def end_to_end_latency(
    llm_start: float,
    llm_end: float,
    compile_start: float,
    compile_end: float,
) -> float:
    """
    执行效率 —— LLM 推理 + 编译解析总耗时。

    Args:
        llm_start: LLM 调用开始时间戳
        llm_end: LLM 调用结束时间戳
        compile_start: Rust 编译开始时间戳
        compile_end: Rust 编译结束时间戳

    Returns:
        总耗时（秒）
    """
    llm_latency = llm_end - llm_start
    compile_latency = compile_end - compile_start
    return llm_latency + compile_latency


def path_efficiency(
    planned_trajectory: List[Dict[str, float]],
    optimal_cost: float,
) -> float:
    """
    轨迹代价与理论最优比。

    Args:
        planned_trajectory: 规划器输出的轨迹点序列
        optimal_cost: 理论最优轨迹的代价

    Returns:
        代价比率 efficiency = optimal_cost / actual_cost（0.0 ~ 1.0）
    """
    actual_cost = 0.0
    for i in range(1, len(planned_trajectory)):
        dx = planned_trajectory[i].get("x", 0) - planned_trajectory[i - 1].get("x", 0)
        dy = planned_trajectory[i].get("y", 0) - planned_trajectory[i - 1].get("y", 0)
        actual_cost += math.sqrt(dx**2 + dy**2)

    if actual_cost <= 0:
        return 0.0
    return min(1.0, optimal_cost / actual_cost)


# ══════════════════════════════════════════════
# 便捷入口 & 结构化反馈演示
# ══════════════════════════════════════════════

def compute_all_metrics(
    robo_ir_str: str,
    nl_instruction: str = "",
) -> Dict[str, float]:
    """
    计算所有指标（兼容原接口）。

    Args:
        robo_ir_str: RoboIR 字符串
        nl_instruction: 原始自然语言指令

    Returns:
        {指标名: 值} 字典
    """
    return {
        "parsing_accuracy": parsing_accuracy(robo_ir_str),
        "stl_robustness": stl_robustness([], []),
        "end_to_end_latency_ms": end_to_end_latency(0, 0, 0, 0) * 1000,
        "path_efficiency": path_efficiency([], 1.0),
        "precision_recall_f1": precision_recall({}, {}),
    }


def _classify_violation(report: Dict[str, Any]) -> FailureTaxonomy:
    """
    根据 STL 违规报告内容选择最具体的 FailureTaxonomy 分类。

    规则：
      - Distance > D_safe 且使用了距离场 → COLLISION（运行期碰撞）
      - Distance > D_safe 但只用 reference_pose → STL_VIOLATION（弱）
      - Time < T → TIMEOUT
      - 其他 → STL_VIOLATION
    """
    expr = report.get("rule_expr", "")
    source = report.get("source", "")

    if "Distance" in expr and ">" in expr:
        if source == "distance_field":
            return FailureTaxonomy.COLLISION
        return FailureTaxonomy.STL_VIOLATION
    if "Time" in expr and "<" in expr:
        return FailureTaxonomy.TIMEOUT
    return FailureTaxonomy.STL_VIOLATION


def run_diagnostic_pipeline(
    robo_ir_str: str,
    trajectory: List[Dict[str, float]],
    constraints: List[Dict[str, Any]],
    reference_poses: Optional[Dict[str, Dict[str, Any]]] = None,
    topology_bundle: Optional[Dict[str, Any]] = None,
) -> List[Dict[str, Any]]:
    """
    完整诊断流水线：执行 STL 鲁棒性检测 → 映射到 FailureTaxonomy
    → 生成结构化反馈（含每个违规栅格点的定量 ρ 偏差）。

    Args:
        robo_ir_str: RoboIR 字符串
        trajectory: 执行轨迹
        constraints: STL 约束列表
        reference_poses: 兜底参考 Pose 表
        topology_bundle: Demo 1 拓扑包（Distance 约束官方求解路径）

    Returns:
        结构化反馈列表，每条对应一个被违反的约束。
        每条 feedback["context"] 含 ``violation_nodes`` 字段（违规节点详情）。
    """
    feedback_list: List[Dict[str, Any]] = []

    # 1. 编译期检查
    compiler = _RustCompilerStub()
    valid, errors = compiler.validate(robo_ir_str)
    if not valid:
        for err in errors:
            error_type = {
                "frame_mismatch": FailureTaxonomy.FRAME_MISMATCH,
                "stl_syntax_error": FailureTaxonomy.STL_SYNTAX_ERROR,
                "invalid_pose_reference": FailureTaxonomy.POSE_UNDEFINED,
                "invalid_grasp_type": FailureTaxonomy.GRASP_TYPE_INVALID,
            }.get(err.error_type.value, FailureTaxonomy.STL_VIOLATION)
            feedback_list.append(generate_structured_feedback(
                error_type, -1.0,
                {"compiler_error": err.message, "location": err.location},
            ))

    # 2. 运行时 STL 违反检测
    violation_reports = _check_stl(
        trajectory, constraints,
        reference_poses=reference_poses,
        topology_bundle=topology_bundle,
    )
    for report in violation_reports:
        if not report.get("violated"):
            continue
        taxonomy = _classify_violation(report)
        nodes = report.get("violation_nodes", [])
        # 选择最深违规点作为代表（min ρ）
        worst_node = min(nodes, key=lambda n: n.get("rho", 0.0)) \
            if nodes else None

        feedback_list.append(generate_structured_feedback(
            taxonomy,
            report.get("robustness", 0.0),
            {
                "rule": report.get("rule_expr"),
                "source": report.get("source"),
                "max_violation": report.get("max_violation"),
                "violation_duration": report.get("violation_duration"),
                "violation_nodes": nodes,
                "worst_node": worst_node,
                "n_violation_nodes": len(nodes),
            },
        ))

    return feedback_list
