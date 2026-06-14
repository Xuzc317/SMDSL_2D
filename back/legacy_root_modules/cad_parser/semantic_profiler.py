"""
semantic_profiler.py — 环境语义画像层 (Semantic Scene Profiler)

位于 Demo 1（拓扑解析）和 Demo 2（语义编译）之间。
从纯数学拓扑数据中蒸馏高维统计特征，喂给 LLM 生成环境描述和物理约束，
作为 VLM 动作编译的全局上下文。

核心算法：
  - 狭窄通道检测：骨架化 (medial axis) + EDT 局部极小值 → GVD 瓶颈检测
  - 复杂度评分：拓扑分支数 × 死区比例 × 瓶颈数加权
  - LLM 语义生成：蒸馏特征 + 文本标签 → DeepSeek → 结构化 JSON

依赖：
  - numpy, scipy (必须)
  - scikit-image (首选) 或 opencv-python (Fallback 骨架化)
  - openai (LLM 调用)
"""

from __future__ import annotations

import json
import os
import re
import time
from typing import Any, Dict, List, Optional, Tuple

import numpy as np

# ── 骨架化库：skimage 优先，cv2 兜底 ──────────────────────
_SKELETONIZE = None
_SKELETONIZE_SOURCE = "none"

try:
    from skimage.morphology import skeletonize as _sk_skel
    _SKELETONIZE = _sk_skel
    _SKELETONIZE_SOURCE = "skimage"
except ImportError:
    pass

if _SKELETONIZE is None:
    try:
        import cv2
        def _cv2_thinning(binary: np.ndarray) -> np.ndarray:
            """Zhang-Suen 细化算法（纯 numpy，不依赖 ximgproc）。"""
            bin_img = binary.astype(np.uint8) * 255
            thinning = bin_img // 255
            prev = np.zeros_like(thinning)
            while not np.array_equal(thinning, prev):
                prev = thinning.copy()
                # 子迭代 1
                p2 = np.roll(thinning, 1, axis=1)
                p4 = np.roll(thinning, 1, axis=0)
                p6 = np.roll(thinning, -1, axis=1)
                p8 = np.roll(thinning, -1, axis=0)
                p3 = np.roll(np.roll(thinning, 1, axis=0), 1, axis=1)
                p5 = np.roll(np.roll(thinning, 1, axis=0), -1, axis=1)
                p7 = np.roll(np.roll(thinning, -1, axis=0), -1, axis=1)
                p9 = np.roll(np.roll(thinning, -1, axis=0), 1, axis=1)
                A = ((p2 == 0) & (p3 == 1)).astype(int) + \
                    ((p3 == 0) & (p4 == 1)).astype(int) + \
                    ((p4 == 0) & (p5 == 1)).astype(int) + \
                    ((p5 == 0) & (p6 == 1)).astype(int) + \
                    ((p6 == 0) & (p7 == 1)).astype(int) + \
                    ((p7 == 0) & (p8 == 1)).astype(int) + \
                    ((p8 == 0) & (p9 == 1)).astype(int) + \
                    ((p9 == 0) & (p2 == 1)).astype(int)
                B = p2 + p3 + p4 + p5 + p6 + p7 + p8 + p9
                m1 = (thinning == 1) & (B >= 2) & (B <= 6) & (A == 1) & \
                     (p2 * p4 * p6 == 0) & (p4 * p6 * p8 == 0)
                thinning[m1] = 0
                # 子迭代 2
                p2 = np.roll(thinning, 1, axis=1)
                p4 = np.roll(thinning, 1, axis=0)
                p6 = np.roll(thinning, -1, axis=1)
                p8 = np.roll(thinning, -1, axis=0)
                p3 = np.roll(np.roll(thinning, 1, axis=0), 1, axis=1)
                p5 = np.roll(np.roll(thinning, 1, axis=0), -1, axis=1)
                p7 = np.roll(np.roll(thinning, -1, axis=0), -1, axis=1)
                p9 = np.roll(np.roll(thinning, -1, axis=0), 1, axis=1)
                A = ((p2 == 0) & (p3 == 1)).astype(int) + \
                    ((p3 == 0) & (p4 == 1)).astype(int) + \
                    ((p4 == 0) & (p5 == 1)).astype(int) + \
                    ((p5 == 0) & (p6 == 1)).astype(int) + \
                    ((p6 == 0) & (p7 == 1)).astype(int) + \
                    ((p7 == 0) & (p8 == 1)).astype(int) + \
                    ((p8 == 0) & (p9 == 1)).astype(int) + \
                    ((p9 == 0) & (p2 == 1)).astype(int)
                B = p2 + p3 + p4 + p5 + p6 + p7 + p8 + p9
                m2 = (thinning == 1) & (B >= 2) & (B <= 6) & (A == 1) & \
                     (p2 * p4 * p8 == 0) & (p2 * p6 * p8 == 0)
                thinning[m2] = 0
            return thinning.astype(bool)
        _SKELETONIZE = _cv2_thinning
        _SKELETONIZE_SOURCE = "cv2-zhangsuen"
    except ImportError:
        pass


# ── LLM 配置（复用项目常量） ──────────────────────────────

DEEPSEEK_API_KEY_ENV = "DEEPSEEK_API_KEY"
DEEPSEEK_API_KEY_ENV_FALLBACK = "DEEPSEEK_KEY"
DEEPSEEK_BASE_URL = "https://api.deepseek.com"
DEEPSEEK_DEFAULT_MODEL = "deepseek-v4-flash"

# ── 拓扑类别常量（与 astar_topology.py 保持一致） ──────────

CLASS_PATH = 2
CLASS_OBSTACLE = 3
CLASS_INFLATED = 4


# ══════════════════════════════════════════════════════════════════════
# Step 1: 几何特征蒸馏
# ══════════════════════════════════════════════════════════════════════

def extract_topology_features(
    pipeline_result: Dict[str, Any],
) -> Dict[str, Any]:
    """
    从 Demo 1 流水线产物中提取高维统计特征，供 LLM 消费。

    输入 pipeline_result 预期包含：
      - grid: np.ndarray (H,W) uint8, 0=占据 1=自由
      - distance_field: np.ndarray (H,W) float32
      - topology: np.ndarray (H,W) int (CLASS_PATH=2 / OBSTACLE=3 / INFLATED=4)
      - transform: {"origin": (x,y), "resolution": float, "shape": (H,W)}
      - robot_radius_px: float
      - topo_stats: {"n_components": int, "largest_component_frac": float,
                     "component_sizes": [...]}
      - cad_data: 原始 CAD 语义（可选）

    Returns:
        {
            "navigable_area_sqm": float,
            "free_area_sqm": float,
            "obstacle_area_sqm": float,
            "inflated_area_sqm": float,
            "max_clearance_m": float,
            "mean_clearance_m": float,
            "bottlenecks": [{"width_m": float, "position_xy": [x,y],
                             "severity": "high|medium|low"}, ...],
            "complexity_score": float (0~1),
            "n_disconnected_regions": int,
            "deadzone_fraction": float,
        }
    """
    grid = pipeline_result.get("grid")
    distance_field = pipeline_result.get("distance_field")
    topology = pipeline_result.get("topology")
    transform = pipeline_result.get("transform", {})
    robot_radius_px = pipeline_result.get("robot_radius_px", 0.0)
    topo_stats = pipeline_result.get("topo_stats", {})
    cad_data = pipeline_result.get("cad_data") or {}

    if grid is None or distance_field is None or topology is None:
        return _empty_features()

    resolution = float(transform.get("resolution", 0.05))
    origin = transform.get("origin", (0.0, 0.0))

    # ── 面积统计 ─────────────────────────────────────
    px_to_sqm = resolution * resolution
    free_mask = grid == 1
    path_mask = topology == CLASS_PATH
    obstacle_mask = topology == CLASS_OBSTACLE
    inflated_mask = topology == CLASS_INFLATED

    navigable_area_sqm = float(np.sum(path_mask)) * px_to_sqm
    free_area_sqm = float(np.sum(free_mask)) * px_to_sqm
    obstacle_area_sqm = float(np.sum(obstacle_mask)) * px_to_sqm
    inflated_area_sqm = float(np.sum(inflated_mask)) * px_to_sqm
    deadzone_fraction = float(
        1.0 - np.sum(path_mask) / max(1, np.sum(free_mask))
    )

    # ── clearance 统计 ───────────────────────────────
    # 仅在可通行区域 (PATH) 内采样，避免边界噪声
    path_df = distance_field[path_mask] if np.any(path_mask) else np.array([0])
    max_clearance_m = float(np.max(path_df)) * resolution * 2.0  # 直径
    mean_clearance_m = float(np.mean(path_df)) * resolution * 2.0

    # ── 瓶颈检测：骨架化 + EDT 局部极小值 ─────────────
    bottlenecks = _detect_bottlenecks(
        path_mask=path_mask,
        distance_field=distance_field,
        resolution=resolution,
        origin=origin,
        robot_radius_px=robot_radius_px,
    )

    # ── 复杂度评分 ───────────────────────────────────
    n_components = topo_stats.get("n_components", 1)
    n_bottlenecks = len([b for b in bottlenecks if b["severity"] != "low"])
    complexity_raw = (
        min(1.0, (n_components - 1) * 0.15) +     # 连通域越多越复杂
        min(0.5, n_bottlenecks * 0.10) +           # 瓶颈越多越复杂
        min(0.5, deadzone_fraction * 2.0)           # 死区多→复杂
    )
    complexity_score = round(min(1.0, complexity_raw / 2.0), 4)

    return {
        "navigable_area_sqm": round(navigable_area_sqm, 3),
        "free_area_sqm": round(free_area_sqm, 3),
        "obstacle_area_sqm": round(obstacle_area_sqm, 3),
        "inflated_area_sqm": round(inflated_area_sqm, 3),
        "max_clearance_m": round(max_clearance_m, 4),
        "mean_clearance_m": round(mean_clearance_m, 4),
        "bottlenecks": bottlenecks,
        "complexity_score": complexity_score,
        "n_disconnected_regions": n_components,
        "deadzone_fraction": round(deadzone_fraction, 4),
        "_skeleton_source": _SKELETONIZE_SOURCE,
    }


def _detect_bottlenecks(
    path_mask: np.ndarray,
    distance_field: np.ndarray,
    resolution: float,
    origin: Tuple[float, float],
    robot_radius_px: float,
    min_path_area: int = 50,
) -> List[Dict[str, Any]]:
    """
    基于骨架化 (medial axis) + EDT 局部极小值检测狭窄通道。

    算法：
      1. 对 PATH mask 做骨架化 → 中轴线
      2. 在骨架点上采样 distance_field
      3. 找骨架上的局部极小值（邻域窗口内 df 最小的点）
      4. 按 clearance 阈值分类严重程度
    """
    if _SKELETONIZE is None:
        return [{
            "width_m": -1.0,
            "position_xy": [0.0, 0.0],
            "severity": "unknown",
            "note": f"骨架化库不可用 (skimage 或 cv2)，请 pip install scikit-image",
        }]

    if np.sum(path_mask) < min_path_area:
        return []

    H, W = path_mask.shape

    # 1. 骨架化
    skeleton = _SKELETONIZE(path_mask)
    skel_indices = np.argwhere(skeleton)
    if len(skel_indices) < 3:
        return []

    # 2. 骨架上的 df 值
    skel_df = distance_field[skeleton]
    robot_diameter_px = robot_radius_px * 2.0

    # 3. 找局部极小值
    #    对每个骨架点，检查其 8 邻域是否全是更大的 df → 局部极小
    local_min_mask = np.zeros_like(skeleton, dtype=bool)
    for r, c in skel_indices:
        r0, r1 = max(0, r - 1), min(H, r + 2)
        c0, c1 = max(0, c - 1), min(W, c + 2)
        neighborhood = distance_field[r0:r1, c0:c1]
        skeleton_neighborhood = skeleton[r0:r1, c0:c1]
        skel_vals = neighborhood[skeleton_neighborhood]
        if len(skel_vals) > 0 and distance_field[r, c] <= np.min(skel_vals):
            local_min_mask[r, c] = True

    # 4. 收集瓶颈（仅关心 clearance < 3 × robot_diameter）
    min_indices = np.argwhere(local_min_mask)
    bottlenecks: List[Dict[str, Any]] = []
    seen_positions: set = set()

    for r, c in min_indices:
        df_val = float(distance_field[r, c])
        width_m = df_val * resolution * 2.0  # 通道直径（米）

        # 空间去重：5px 范围内只保留最窄的
        pos_key = (r // 5, c // 5)
        if pos_key in seen_positions:
            continue
        seen_positions.add(pos_key)

        # 严重程度
        if robot_radius_px > 0:
            if width_m < robot_radius_px * resolution * 2.5:
                severity = "high"
            elif width_m < robot_radius_px * resolution * 5.0:
                severity = "medium"
            else:
                severity = "low"
        else:
            severity = "low" if width_m > 0.6 else "medium"

        # 世界坐标
        ox, oy = origin
        wx = ox + c * resolution
        wy = oy + r * resolution

        bottlenecks.append({
            "width_m": round(width_m, 4),
            "position_xy": [round(wx, 3), round(wy, 3)],
            "severity": severity,
            "grid_rc": [int(r), int(c)],
        })

    # 按严重程度排序：high → medium → low
    sev_order = {"high": 0, "medium": 1, "low": 2}
    bottlenecks.sort(key=lambda b: (sev_order.get(b["severity"], 9), b["width_m"]))

    return bottlenecks[:20]


def _empty_features() -> Dict[str, Any]:
    return {
        "navigable_area_sqm": 0.0,
        "free_area_sqm": 0.0,
        "obstacle_area_sqm": 0.0,
        "inflated_area_sqm": 0.0,
        "max_clearance_m": 0.0,
        "mean_clearance_m": 0.0,
        "bottlenecks": [],
        "complexity_score": 0.0,
        "n_disconnected_regions": 0,
        "deadzone_fraction": 0.0,
    }


# ══════════════════════════════════════════════════════════════════════
# Step 2: LLM 语义生成
# ══════════════════════════════════════════════════════════════════════

SCENE_PROFILER_SYSTEM_PROMPT = r"""
你是一个具身智能空间分析专家（Semantic Scene Profiler）。
你的任务不是直接控制机器人，而是**理解空间的物理约束**，
为下游的动作编译器（RoboIR）提供可信的全局上下文。

【输入说明】
你会收到一份从 CAD 平面图数学拓扑数据中蒸馏出的结构化特征，包括：
- 可通行面积、障碍物占比、死区比例
- 最大/平均 clearance（可通行直径）
- 狭窄通道（瓶颈）的位置和宽度
- 拓扑复杂度评分
- 原始 CAD 中的文本标签（如有）

【你的任务】
基于以上特征，输出结构化 JSON，包含：

1. **scene_type**: 环境的整体定性标签。
   候选：open_workshop / cluttered_room / multi_room_complex /
          narrow_corridor / large_hall / industrial_cell / unknown
2. **spatial_layout_summary**: 1-2 句中文，描述空间布局和连通性。
3. **recommended_global_constraints**: 基于物理特征推荐的全局约束：
   - safety_distance_m: 推荐的最小安全距离（米）
     · clearance < 0.3m  → 0.03~0.05
     · 0.3~0.6m          → 0.10~0.15
     · 0.6~1.5m          → 0.20~0.30
     · > 1.5m            → 0.30~0.50
   - max_velocity_ms: 推荐最大移动速度（米/秒）
     · 复杂度高(>0.6) + 瓶颈多 → 0.1~0.3 m/s
     · 复杂度低(<0.3) + 开阔  → 0.5~1.0 m/s
   - requires_precise_grasp: bool，是否有窄空间需要精细操作
   - high_risk_zones: 需要特别小心的区域描述列表
4. **scene_description**: 一段给机器人操作员的自然语言概述（中文，约100字）。

【约束规则】
- 不要在 JSON 中输出具体的坐标数值（坐标留给底层求解器）。
- 如果数据不足以判断某个字段，使用 null 或合理默认值。
- 你的建议必须是**物理可实现**的。不要推荐安全距离超过最大 clearance。
- safety_distance_m 绝对不能超过 max_clearance_m。
"""


def generate_scene_description(
    features: Dict[str, Any],
    text_labels: Optional[List[str]] = None,
    room_type: Optional[str] = None,
    api_key: Optional[str] = None,
    model: str = DEEPSEEK_DEFAULT_MODEL,
    max_retries: int = 2,
) -> Dict[str, Any]:
    """
    将拓扑蒸馏特征 + 文本标签喂给 LLM，生成结构化场景语义画像。

    Args:
        features: extract_topology_features() 的输出。
        text_labels: DWG/JSON 中的文本标注列表。
        room_type: JSON 模式下的房间类型（如 kitchen, bedroom）。
        api_key: DeepSeek API key，默认从环境变量读取。
        model: 模型名称。
        max_retries: API 失败重试次数。

    Returns:
        {"status": "ok"|"error",
         "profile": {"scene_type":..., "recommended_global_constraints":...},
         "raw_response": "..."}
    """
    resolved_key = api_key or os.environ.get(DEEPSEEK_API_KEY_ENV, "") or os.environ.get(DEEPSEEK_API_KEY_ENV_FALLBACK, "")
    if not resolved_key:
        return {
            "status": "error",
            "profile": None,
            "message": f"未设置 {DEEPSEEK_API_KEY_ENV}，跳过语义画像生成。",
        }

    try:
        from openai import OpenAI
    except ImportError:
        return {"status": "error", "profile": None, "message": "需要 pip install openai"}

    # ── 组装 User Prompt ─────────────────────────────
    parts: List[str] = [
        "请基于以下 CAD 拓扑蒸馏特征，生成环境语义画像 JSON。",
        "",
        "## 空间统计",
        f"- 可通行面积: {features.get('navigable_area_sqm', 0):.2f} m²",
        f"- 总自由面积: {features.get('free_area_sqm', 0):.2f} m²",
        f"- 障碍物面积: {features.get('obstacle_area_sqm', 0):.2f} m²",
        f"- 膨胀禁入区: {features.get('inflated_area_sqm', 0):.2f} m²",
        f"- 最大 clearance: {features.get('max_clearance_m', 0):.3f} m",
        f"- 平均 clearance: {features.get('mean_clearance_m', 0):.3f} m",
        f"- 死区比例: {features.get('deadzone_fraction', 0):.1%}",
        f"- 不连通区域数: {features.get('n_disconnected_regions', 0)}",
        f"- 复杂度评分: {features.get('complexity_score', 0):.3f} (0=简单 1=极复杂)",
    ]

    bottlenecks = features.get("bottlenecks", [])
    if bottlenecks:
        parts.append("")
        parts.append("## 狭窄通道（瓶颈）")
        for i, b in enumerate(bottlenecks[:10]):
            note = b.get("note", "")
            parts.append(
                f"- 瓶颈 #{i+1}: 宽度={b['width_m']:.3f}m "
                f"严重程度={b['severity']}"
                + (f" ({note})" if note else "")
            )
    else:
        parts.append("")
        parts.append("## 狭窄通道（瓶颈）")
        parts.append("- 未检测到显著瓶颈。")

    if room_type:
        parts.insert(1, f"## 房间类型\n- {room_type}")
    if text_labels:
        parts.append("")
        parts.append("## CAD 文本标签")
        parts.append("- " + ", ".join(t for t in text_labels[:30] if t.strip()))

    parts.append("")
    parts.append("请输出 JSON，只输出 JSON 不要任何解释。")

    user_prompt = "\n".join(parts)

    # ── 调用 API ─────────────────────────────────────
    client = OpenAI(api_key=resolved_key, base_url=DEEPSEEK_BASE_URL)
    last_err: Optional[Exception] = None

    for attempt in range(1, max_retries + 1):
        try:
            kwargs: Dict[str, Any] = dict(
                model=model,
                messages=[
                    {"role": "system", "content": SCENE_PROFILER_SYSTEM_PROMPT},
                    {"role": "user", "content": user_prompt},
                ],
                temperature=0.15,
                max_tokens=1024,
            )
            try:
                kwargs["response_format"] = {"type": "json_object"}
                resp = client.chat.completions.create(**kwargs)
            except TypeError:
                kwargs.pop("response_format", None)
                resp = client.chat.completions.create(**kwargs)

            content = resp.choices[0].message.content or ""
            profile = _safe_parse_json(content)
            return {
                "status": "ok",
                "profile": profile,
                "raw_response": content,
                "model_used": model,
            }
        except (json.JSONDecodeError, Exception) as e:
            last_err = e
            if attempt < max_retries:
                time.sleep(attempt * 1.0)

    return {
        "status": "error",
        "profile": None,
        "message": f"LLM 调用失败（{max_retries} 次重试）: {last_err}",
    }


def _safe_parse_json(raw: str) -> Dict[str, Any]:
    """容错 JSON 解析。"""
    cleaned = re.sub(r"^```(?:json)?\s*\n?", "", raw.strip())
    cleaned = re.sub(r"\n?```\s*$", "", cleaned)
    cleaned = re.sub(r",\s*([}\]])", r"\1", cleaned)
    return json.loads(cleaned)


# ══════════════════════════════════════════════════════════════════════
# 便捷函数：一键生成语义画像
# ══════════════════════════════════════════════════════════════════════

def profile_scene(
    pipeline_result: Dict[str, Any],
    text_labels: Optional[List[str]] = None,
    room_type: Optional[str] = None,
    api_key: Optional[str] = None,
) -> Dict[str, Any]:
    """
    一键流程：特征蒸馏 → LLM 语义画像。

    Args:
        pipeline_result: Demo 1 流水线产物（含 grid, distance_field,
                        topology, transform, robot_radius_px, topo_stats, cad_data）。
        text_labels: 额外文本标签。
        room_type: 房间类型。
        api_key: API key。

    Returns:
        {"status": "ok"|"error",
         "features": {...},
         "profile": {"scene_type":..., "recommended_global_constraints":...}}
    """
    features = extract_topology_features(pipeline_result)

    # 收集文本标签
    all_labels = list(text_labels or [])
    cad_data = pipeline_result.get("cad_data") or {}
    if isinstance(cad_data, dict):
        for door in cad_data.get("doors", []) or []:
            all_labels.append(door.get("label", ""))
        for obj in cad_data.get("objects", []) or []:
            all_labels.append(obj.get("label", ""))
    # DWG 语义
    semantics = pipeline_result.get("semantics") or {}
    for t in semantics.get("texts", []) or []:
        all_labels.append(t.get("value", ""))

    rt = room_type or (
        cad_data.get("room_type", None)
        if isinstance(cad_data, dict)
        else None
    )

    llm_result = generate_scene_description(
        features=features,
        text_labels=all_labels,
        room_type=rt,
        api_key=api_key,
    )

    return {
        "status": llm_result.get("status", "error"),
        "features": features,
        "profile": llm_result.get("profile"),
        "raw_response": llm_result.get("raw_response", ""),
        "message": llm_result.get("message", ""),
    }
