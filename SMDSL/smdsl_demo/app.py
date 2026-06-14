"""
app.py — SMDSL 模块化调试面板（Gradio）

三个独立选项卡，分离调试每一层；通过 gr.State 在 Tab 之间贯通数据：

  Tab 1: Demo 1 — 环境感知 (CAD 拓扑)
         JSON / SVG / PNG 三路 dispatcher → 距离场 → 5 联可视化
         → 起终点选择 → 两点最短路径 → 轨迹 (state 同步到 Tab 3)

  Tab 2: Demo 2 — 语义编译 (VLM → RoboIR)
         自然语言 + 局部环境 → DeepSeek (STL 护栏 prompt) → RoboIR
         → 引用一致性校验 + 可选 STL 风格中文摘要 (state 同步到 Tab 3)

  Tab 3: Demo 3 — 物理求解与反馈 (STL 验证)
         RoboIR + 轨迹 → 距离场 ρ 求解 → 双图 + Pose 表 + 结构化反馈

启动::

    $env:NO_PROXY = "localhost,127.0.0.1"
    $env:DEEPSEEK_API_KEY = "<key>"
    python -m smdsl_demo.app
"""

from __future__ import annotations

import datetime as _dt
import json
import os
import sys
import traceback
from pathlib import Path
from typing import Any, Dict, List, Optional, Tuple

import gradio as gr
import numpy as np

_THIS_DIR = Path(__file__).resolve().parent
_REPO_ROOT = _THIS_DIR.parent
if str(_REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(_REPO_ROOT))

from smdsl_demo.metrics import (  # noqa: E402
    FailureTaxonomy,
    generate_structured_feedback,
)
from smdsl_demo.spatial_api_stub import (  # noqa: E402
    check_stl_constraint_violation,
)
from smdsl_demo.vlm_parser import (  # noqa: E402
    DEEPSEEK_API_KEY_ENV,
    DEEPSEEK_DEFAULT_MODEL,
    VlmParser,
    VlmParserError,
    generate_instruction_examples as _gen_instruction_examples,
    normalize_stl_constraints,
    validate_roboir_references,
)
from smdsl_demo.visualize_demo3 import (  # noqa: E402
    generate_3d_dashboard,
    render_robustness_curve,
    render_trajectory_overlay,
)

import tempfile as _tempfile
_OUT_DIR = Path(_tempfile.gettempdir()) / "smdsl_ui_cache"
_OUT_DIR.mkdir(parents=True, exist_ok=True)
_DATA_ROOT = str(_REPO_ROOT / "data" / "cad_samples")


# ══════════════════════════════════════════════════════════════════════
# Tab 1 — 环境感知 (dispatcher 三路 + 起终点 + 路径)
# ══════════════════════════════════════════════════════════════════════

# 默认预设：换更复杂的 bedroom 样本（如不存在则回退 kitchen）
_PRESET_CANDIDATES = [
    "data/cad_samples/floorplanqa/layouts/bedroom/room_5.json",
    "data/cad_samples/floorplanqa/layouts/kitchen/room_24.json",
]
_DEFAULT_SAMPLE_PATH = next(
    (str(_REPO_ROOT / p) for p in _PRESET_CANDIDATES
     if (_REPO_ROOT / p).exists()),
    str(_REPO_ROOT / _PRESET_CANDIDATES[-1]),
)


from smdsl_demo.ui_common import format_seed_label as _format_seed_label


def _gather_seed_candidates(
    parse_result: Dict[str, Any],
    distance_field,
    robot_radius_px: float,
    topology_world,
) -> List[Dict[str, Any]]:
    """
    生成起终点候选清单（供下拉选择）。

    候选来源（按优先级）：
      1. doors（仅 JSON 模式）
      2. objects 旁的安全空间（仅 JSON 模式）
      3. 距离场 top-K 安全自由空间中心
      4. 房间几何中心
    """
    import numpy as _np  # noqa: PLC0415

    candidates: List[Dict[str, Any]] = []
    cad_data = parse_result.get("cad_data") or {}
    transform = parse_result["transform"]
    res = transform["resolution"]
    ox, oy = transform["origin"]
    H, W = transform["shape"]

    seen: set = set()

    def _push(kind: str, label: str, x: float, y: float, extra: str = "") -> None:
        key = (round(x, 2), round(y, 2))
        if key in seen:
            return
        seen.add(key)
        idx = len(candidates) + 1
        candidates.append({
            "label": _format_seed_label(idx, kind, x, y, extra),
            "kind": kind,
            "name": label,
            "x": round(x, 4),
            "y": round(y, 4),
        })

    def _pt_xy(p: Any) -> Optional[Tuple[float, float]]:
        """兼容 tuple[(x,y)] 与 dict{"x":, "y":} 两种 schema。"""
        if isinstance(p, dict):
            try:
                return float(p["x"]), float(p["y"])
            except (KeyError, TypeError, ValueError):
                return None
        try:
            return float(p[0]), float(p[1])
        except (IndexError, TypeError, ValueError):
            return None

    def _centroid(pts: List[Any]) -> Optional[Tuple[float, float]]:
        coords = [c for c in (_pt_xy(p) for p in pts) if c is not None]
        if not coords:
            return None
        cx = sum(c[0] for c in coords) / len(coords)
        cy = sum(c[1] for c in coords) / len(coords)
        return cx, cy

    # Pass 1: 距离场 top-K 空间分散 — 这些点是"真正在屋子里、最舒服走路"的位置，
    # 应该排在候选清单最前面，UI 上更直观。
    if distance_field is not None:
        flat = distance_field.ravel()
        top = _np.argsort(flat)[-200:][::-1]
        nms_radius_px = max(robot_radius_px * 3.0, min(H, W) * 0.15)
        picked: List[Tuple[int, int]] = []
        for flat_idx in top:
            d_val = distance_field.flat[flat_idx]
            if d_val < max(robot_radius_px, 1.0):
                continue
            r, c = divmod(int(flat_idx), W)
            too_close = any(
                ((r - pr) ** 2 + (c - pc) ** 2) ** 0.5 < nms_radius_px
                for pr, pc in picked
            )
            if too_close:
                continue
            picked.append((r, c))
            x = ox + c * res
            y = oy + r * res
            _push("safe_center", f"safe_{r}_{c}", x, y,
                  f"d={d_val * res:.2f}m · 屋内最舒服")
            if len(candidates) >= 6 or len(picked) >= 6:
                break

    # Pass 2: JSON 才有的语义候选（门 + 物体邻域）
    if parse_result.get("mode") == "json" and isinstance(cad_data, dict):
        for door in cad_data.get("doors", []) or []:
            ctr = _centroid(door.get("points", []) or [])
            if ctr is None:
                continue
            _push("door", door.get("label", "door"), ctr[0], ctr[1])
        for obj in cad_data.get("objects", []) or []:
            ctr = _centroid(obj.get("points", []) or [])
            if ctr is None:
                continue
            cx, cy = ctr
            for dx, dy in [(0.6, 0.0), (-0.6, 0.0), (0.0, 0.6), (0.0, -0.6),
                           (0.4, 0.4), (-0.4, -0.4)]:
                nx, ny = cx + dx, cy + dy
                col = int((nx - ox) / res)
                row = int((ny - oy) / res)
                if 0 <= row < H and 0 <= col < W:
                    if distance_field[row, col] >= robot_radius_px:
                        _push("near_obj", obj.get("label", "object"), nx, ny,
                              f"~{obj.get('label', 'object')}")
                        break

    # Pass 3: OSM 模式 — 用 passage 门洞中点作为语义候选
    if parse_result.get("mode") == "osm":
        for edge in parse_result.get("osm_edges", []) or []:
            mid = edge.get("midpoint_m", [])
            if len(mid) >= 2:
                _push("door", edge.get("name", "passage"),
                      float(mid[0]), float(mid[1]),
                      f"{edge.get('from_room','')}→{edge.get('to_room','')}")
        # 也把各房间质心加进去
        for rm in parse_result.get("osm_rooms", []) or []:
            c = rm.get("centroid_m", [])
            if len(c) >= 2:
                _push("room_center", rm.get("name", "room"),
                      float(c[0]), float(c[1]),
                      f"{rm.get('area_m2', 0):.0f}m²")

    # 兜底：几何中心
    if not candidates:
        _push("center", "geom_center", ox + W * res / 2, oy + H * res / 2)

    return candidates


def demo1_run(layout_path_text: str, png_invert: bool = False,
              robot_radius_m_ui: float = 0.15):
    """
    Tab 1 主流水线：dispatch → distance_field → A*flood → 渲染图 + 候选种子。

    Returns:
        json_text, image_path, status_md, dropdown_choices,
        topology_state, candidates_state
    """
    empty_choices = gr.update(choices=[], value=None)
    _empty_pick_state = {"start_xy": None, "goal_xy": None, "mode": "start"}

    def _err_return(payload: Dict[str, Any], status: str):
        return (
            json.dumps(payload, indent=2, ensure_ascii=False),
            None, None, None, status,
            empty_choices, empty_choices, None, [],
            _empty_pick_state, None, None,  # semantics_json, semantics_md
        )

    path_str = (layout_path_text or "").strip()
    if not path_str:
        return _err_return({"error": "path_empty"},
                           "❌ 请先选择或输入 CAD 文件路径")

    try:
        from cad_parser.dispatcher import dispatch_cad  # noqa: PLC0415
        from cad_parser.astar_topology import (  # noqa: PLC0415
            CLASS_INFLATED, CLASS_PATH, compute_distance_field,
            classify_topology_global, to_topology_bundle,
            extract_door_seeds,
        )
        from cad_parser.dispatcher import (  # noqa: PLC0415
            get_optional_seeds_world,
        )
        from cad_parser.visualize import render_pipeline_quad  # noqa: PLC0415
    except ImportError as e:
        return _err_return({"error": "import_failed", "message": str(e)},
                           f"❌ 模块导入失败：{e}")

    # OSM 建筑尺度大（通常 100~200m），0.05m/px 会生成超大 grid；
    # 自动切换到 0.15m/px，精度足够，速度快 9×。
    _osm_mode = Path(path_str).suffix.lower() == ".osm"
    _resolution = 0.15 if _osm_mode else 0.05
    try:
        parsed = dispatch_cad(path_str, resolution=_resolution, padding_m=0.5,
                              wall_thickness_m=0.10, png_invert=png_invert)
    except Exception as e:  # noqa: BLE001
        return _err_return(
            {"error": "dispatch_failed", "message": str(e),
             "hint": "支持 .json/.dwg/.osm/.svg/.png/.jpg；详见 Tab 1 顶部说明"},
            f"❌ Dispatcher 失败：{e}",
        )

    grid = parsed["grid"]
    transform = parsed["transform"]
    distance_field = compute_distance_field(grid)
    robot_radius_m = max(0.0, float(robot_radius_m_ui))
    robot_radius_px = robot_radius_m / transform["resolution"] if robot_radius_m > 0 else 0.0

    # 用于可视化的种子（不再用于拓扑漫水）——给用户看门洞所在位置
    if parsed["mode"] == "json":
        seeds = extract_door_seeds(
            parsed["cad_data"], transform,
            distance_field=distance_field,
            robot_radius_px=robot_radius_px,
        )
    else:
        seeds_world = get_optional_seeds_world(
            parsed, distance_field, robot_radius_px=robot_radius_px,
        )
        seeds = []
        for x, y in seeds_world:
            row = int((y - transform["origin"][1]) / transform["resolution"])
            col = int((x - transform["origin"][0]) / transform["resolution"])
            seeds.append((row, col))
    if not seeds:
        H, W = grid.shape
        seeds = [(H // 2, W // 2)]

    # 全局矩阵化拓扑分类（替代 BFS 漫水）
    topology, topo_stats = classify_topology_global(
        grid, distance_field, robot_radius_px=robot_radius_px,
    )
    # visit_count 已不再来自漫水；用 path mask 提供等价信息以兼容旧可视化
    visit_count = (topology == CLASS_PATH).astype(np.uint32)

    pipeline_result = {
        "cad_data": parsed["cad_data"] or {},
        "grid": grid, "distance_field": distance_field,
        "topology": topology, "visit_count": visit_count,
        "transform": transform, "seeds": seeds,
        "robot_radius_px": robot_radius_px,
        "topo_stats": topo_stats,
    }

    candidates = _gather_seed_candidates(
        parsed, distance_field, robot_radius_px,
        topology_world=topology,
    )

    from cad_parser.visualize import (  # noqa: PLC0415
        render_source_cad, render_clickable_topology,
    )
    stem = Path(path_str).stem
    out_quad = _OUT_DIR / f"demo1_{stem}_quad.png"
    out_src = _OUT_DIR / f"demo1_{stem}_source.png"
    out_click = _OUT_DIR / f"demo1_{stem}_clickable.png"
    try:
        render_pipeline_quad(
            pipeline_result, str(out_quad),
            title_prefix=f"Demo 1 [{parsed['mode']}] — {Path(path_str).name}",
        )
        png_path: Optional[str] = str(out_quad)
    except Exception:
        png_path = None
    try:
        source_png: Optional[str] = render_source_cad(parsed, str(out_src))
    except Exception:
        source_png = None
    try:
        clickable_png: Optional[str] = render_clickable_topology(
            pipeline_result, str(out_click), marker_world_xy=[],
            candidates=candidates,
        )
    except Exception:
        clickable_png = None

    n_path = int((topology == CLASS_PATH).sum())
    n_inflated = int((topology == CLASS_INFLATED).sum())
    total_free = int((grid == 1).sum())
    path_frac = n_path / max(1, total_free)
    info = {
        "_pipeline_ok": True,
        "_source_file": path_str,
        "_mode": parsed["mode"],
        "_has_semantics": parsed["has_semantics"],
        "_dataset_note": parsed["note"],
        "_grid_shape": list(grid.shape),
        "_resolution_m_per_px": transform["resolution"],
        "_n_path_pixels": n_path,
        "_n_inflated_pixels": n_inflated,
        "_path_fraction_of_free": round(path_frac, 4),
        "_n_seed_candidates": len(candidates),
        "_topology_stats": {
            "n_components": topo_stats["n_components"],
            "largest_component_frac": round(
                topo_stats["largest_component_frac"], 4,
            ),
            "top_component_sizes": topo_stats["component_sizes"][:5],
        },
    }

    bundle = to_topology_bundle({
        **pipeline_result,
        "robot_radius_m": robot_radius_m,
        "cad_data": pipeline_result["cad_data"]
                    if isinstance(pipeline_result["cad_data"], dict)
                    else {},
    })
    topology_state_value = {
        "bundle": bundle,
        "grid": grid,
        "distance_field": distance_field,
        "topology": topology,
        "transform": transform,
        "grid_shape": list(grid.shape),
        "robot_radius_m": robot_radius_m,
        "robot_radius_px": robot_radius_px,
        "mode": parsed["mode"],
        "has_semantics": parsed["has_semantics"],
        "source_path": path_str,
        "clickable_png_path": clickable_png,
        "cad_data": parsed.get("cad_data", {}),
        "semantics": parsed.get("semantics", {}),
        "topo_stats": topo_stats,
        # OSM 专属字段（供 Demo 3 generate_osm_3d_scene 使用）
        "osm_rooms": parsed.get("osm_rooms", []),
        "osm_edges": parsed.get("osm_edges", []),
        "adjacency_graph": parsed.get("adjacency_graph", {}),
    }

    choice_labels = [c["label"] for c in candidates]
    start_choice = gr.update(
        choices=choice_labels,
        value=choice_labels[0] if choice_labels else None,
    )
    goal_choice = gr.update(
        choices=choice_labels,
        value=(choice_labels[1] if len(choice_labels) > 1
               else (choice_labels[0] if choice_labels else None)),
    )

    # 最大自由空间："距离场峰值 × 2"≈"最大内切圆直径"，物理意义直观
    res = float(transform["resolution"])
    max_clearance_m = float(distance_field.max()) * res
    largest_space_m = max_clearance_m * 2.0
    n_nodes = int((topology == CLASS_PATH).sum())

    # ── 语义展示（DWG 轨迹 B / OSM 原生拓扑） ────────────────────
    semantics_json: Any = None
    semantics_md: Optional[str] = None
    _mode = parsed.get("mode", "")

    if _mode == "dwg" and parsed.get("semantics"):
        sem = parsed["semantics"]
        semantics_json = {
            "layers": sem.get("layers", []),
            "texts": [t for t in sem.get("texts", [])[:50]],
            "blocks": [b for b in sem.get("blocks", [])[:50]],
            "entity_summary": sem.get("entity_summary", {}),
        }
        lines_md = [
            "## 图纸语义信息（轨迹 B — AI 语义流）",
            "",
            f"**图层 ({len(sem.get('layers', []))} 个)**: "
            + ", ".join(f"`{l}`" for l in sem.get("layers", [])[:20]),
            "",
            f"**文本标注**: {sem.get('entity_summary', {}).get('texts', 0)} 条",
            f"**块参照**: {sem.get('entity_summary', {}).get('blocks', 0)} 个",
            f"**线段**: {sem.get('entity_summary', {}).get('lines', 0)} 条",
            f"**多段线**: {sem.get('entity_summary', {}).get('polylines', 0)} 条",
            "",
            "> 这些数据可喂给 `semantic_extractor`，"
            "由大模型识别上料口/下料口/设备等业务节点。",
        ]
        semantics_md = "\n".join(lines_md)

    elif _mode == "osm" and parsed.get("semantics"):
        sem = parsed["semantics"]
        rooms_list = sem.get("rooms", [])
        passages_list = sem.get("passages", [])
        adj = sem.get("adjacency_graph", {})
        semantics_json = {
            "rooms": rooms_list,
            "passages": passages_list,
            "adjacency_graph": adj,
            "origin_latlon": sem.get("origin_latlon", []),
        }
        # 房间列表（最多显示 15 行）
        room_rows = "\n".join(
            f"| `{r['name']}` | {r.get('area_type','room')} "
            f"| {r.get('area_m2', 0):.1f} | {r.get('height_m', 0):.1f} |"
            for r in rooms_list[:15]
        )
        room_suffix = (
            f"\n*…共 {len(rooms_list)} 个房间（仅显示前 15）*"
            if len(rooms_list) > 15 else ""
        )
        passage_rows = "\n".join(
            f"| `{p['name']}` | `{p.get('from','')}` | `{p.get('to','')}` |"
            for p in passages_list[:10]
        )
        passage_suffix = (
            f"\n*…共 {len(passages_list)} 条通路（仅显示前 10）*"
            if len(passages_list) > 10 else ""
        )
        isolated = len(rooms_list) - len(adj)
        lines_md = [
            "## 图纸语义信息（osmAG 原生拓扑）",
            "",
            f"**房间数**: {len(rooms_list)}　"
            f"**通路/门数**: {len(passages_list)}　"
            f"**邻接图节点**: {len(adj)}　"
            f"**孤立房间（无门）**: {max(0, isolated)}",
            "",
            "### 房间列表",
            "| 名称 | 类型 | 面积 m² | 高度 m |",
            "|------|------|---------|--------|",
            room_rows + room_suffix,
            "",
            "### 通路/门列表",
            "| 名称 | from | to |",
            "|------|------|----|",
            passage_rows + passage_suffix,
            "",
            "> osmAG 模式已**跳过** rasterize_to_grid + A\\* flood-fill，"
            "直接利用 OSM 多边形填充生成 occupancy grid。",
            "> 邻接图可直接用于 Demo 2 空间查询和 Demo 3 场景图渲染。",
        ]
        semantics_md = "\n".join(lines_md)

    # ── 保存流水线产物给语义画像按钮使用 ──────────────────
    # (不在这里调 LLM，由独立按钮触发，避免拖慢解析)
    semantic_icon = "SEMANTIC" if parsed["has_semantics"] else "GEOMETRY"
    nc = topo_stats["n_components"]
    nc_icon = "🟢" if nc <= 2 else ("🟡" if nc <= 5 else "🔴")
    nc_hint = (
        f"{nc_icon} 连通域 {nc}"
        + (f"（主区占 {topo_stats['largest_component_frac']:.0%}）"
           if nc > 0 else "")
    )
    status_prefix = ""
    note = parsed.get("note", "")
    if "实体丢弃率" in str(note):
        status_prefix = (
            "<div style='background:#fff3cd;border:1px solid #ffc107;"
            "padding:8px 12px;margin-bottom:8px;border-radius:4px;"
            "font-size:14px;color:#664d03'>"
            "⚠️ 实体丢弃率偏高："
            + str(note).replace("\n", "<br>")
            + "</div>\n\n"
        )
    status = (
        f"{status_prefix}"
        f"✅ 解析完成 · 模式 = **{parsed['mode'].upper()}** {semantic_icon} · "
        f"格 {grid.shape[0]}×{grid.shape[1]} · "
        f"节点数 = **{n_nodes}** · 可走比例 = **{path_frac:.1%}** · "
        f"最大空间 ≈ **{largest_space_m:.2f} m** · "
        f"候选起终点 {len(candidates)} · {nc_hint}"
    )
    return (
        json.dumps(info, indent=2, ensure_ascii=False, default=str),
        source_png, png_path, clickable_png, status,
        start_choice, goal_choice,
        topology_state_value, candidates,
        # 重置 click_pick_state（每次解析后清空已选点）
        {"start_xy": None, "goal_xy": None, "mode": "start"},
        json.dumps(semantics_json, indent=2, ensure_ascii=False)
            if semantics_json else None,
        semantics_md,
    )


# ── 可点击地图选起终点 ────────────────────────────────────────

def _world_to_disp_pixel(
    x_world: float, y_world: float,
    transform: Dict[str, Any],
    img_h: int, img_w: int,
) -> Tuple[int, int]:
    """世界 → matplotlib 显示像素的近似（用于标注，非求解，无需精确）。"""
    ox, oy = transform["origin"]
    res = transform["resolution"]
    H, W = transform["shape"]
    col_frac = (x_world - ox) / (W * res)
    row_frac = (y_world - oy) / (H * res)
    return int(col_frac * img_w), int(row_frac * img_h)


def _redraw_clickable_with_markers(
    topology_state: Dict[str, Any],
    pick_state: Dict[str, Any],
    candidates: Optional[List[Dict[str, Any]]] = None,
) -> Optional[str]:
    """重渲染可点击拓扑图，叠加用户已选的 A/B 标记。"""
    try:
        from cad_parser.dispatcher import dispatch_cad  # noqa: PLC0415
        from cad_parser.astar_topology import (  # noqa: PLC0415
            compute_distance_field, safety_aware_astar_flood,
            extract_door_seeds,
        )
        from cad_parser.visualize import render_clickable_topology  # noqa: PLC0415
        from cad_parser.dispatcher import (  # noqa: PLC0415
            get_optional_seeds_world,
        )
    except ImportError:
        return None

    try:
        parsed = dispatch_cad(topology_state["source_path"])
        grid = parsed["grid"]
        df = compute_distance_field(grid)
        rpx = topology_state["robot_radius_px"]
        if parsed["mode"] == "json":
            seeds = extract_door_seeds(
                parsed["cad_data"], parsed["transform"],
                distance_field=df, robot_radius_px=rpx,
            )
        else:
            seeds_w = get_optional_seeds_world(parsed, df, robot_radius_px=rpx)
            seeds = []
            for x, y in seeds_w:
                row = int((y - parsed["transform"]["origin"][1])
                          / parsed["transform"]["resolution"])
                col = int((x - parsed["transform"]["origin"][0])
                          / parsed["transform"]["resolution"])
                seeds.append((row, col))
        if not seeds:
            seeds = [(grid.shape[0] // 2, grid.shape[1] // 2)]
        topology, _ = safety_aware_astar_flood(
            grid, df, seeds, robot_radius_px=rpx, safety_weight=0.5,
        )
    except Exception:
        return None

    markers: List[Dict[str, Any]] = []
    if pick_state.get("start_xy"):
        sx, sy = pick_state["start_xy"]
        markers.append({"x": sx, "y": sy, "color": "#1f77b4", "label": "A"})
    if pick_state.get("goal_xy"):
        gx, gy = pick_state["goal_xy"]
        markers.append({"x": gx, "y": gy, "color": "#d62728", "label": "B"})

    pipeline_result = {
        "topology": topology,
        "transform": parsed["transform"],
        "seeds": seeds,
    }
    out_path = _OUT_DIR / "demo1_clickable_marked.png"
    return render_clickable_topology(
        pipeline_result, str(out_path), marker_world_xy=markers,
        candidates=candidates,
    )


def demo1_image_click(
    evt: gr.SelectData,
    topology_state: Optional[Dict[str, Any]],
    pick_state: Optional[Dict[str, Any]],
    candidates: Optional[List[Dict[str, Any]]] = None,
):
    """
    用户在拓扑图上点击：把屏幕像素坐标换算为世界坐标，按当前 mode 写入 A 或 B。
    满 2 个点后下游自动调 demo1_plan_path（由 .then() 链触发）。
    """
    if topology_state is None:
        return (
            None, gr.update(),
            "❌ 请先点 [⚙️ 解析 CAD] 生成距离场",
            pick_state or {"start_xy": None, "goal_xy": None, "mode": "start"},
        )

    # gr.SelectData.index for Image: tuple (x_pixel, y_pixel) in original image
    try:
        x_pix, y_pix = evt.index
    except Exception:
        return (
            None, gr.update(),
            "❌ 无法读取点击坐标（请重试或换用下拉选）",
            pick_state,
        )

    # 用户看到的 PNG 是 matplotlib 保存的图，它的像素坐标 ≠ 距离场矩阵索引。
    # 真正稳的换算是：matplotlib extent 已经把 axes 对齐到世界坐标，但
    # 保存为 PNG 后会有"bbox_inches='tight'"裁掉的边距 — 这里使用透视近似：
    # 直接按图像比例线性映射到 transform["shape"] 像素索引。
    # 经验上，对中等尺寸图（≤ 1200px）误差 < 1 个 cell（5cm），可接受。
    img_path = topology_state.get("clickable_png_path")
    if not img_path or not Path(img_path).exists():
        return (
            None, gr.update(), "❌ 拓扑图丢失，请重新解析", pick_state,
        )
    try:
        from PIL import Image as _PILImage  # noqa: PLC0415
        with _PILImage.open(img_path) as im:
            disp_w, disp_h = im.size
    except Exception:
        disp_w, disp_h = 1000, 800

    transform = topology_state["transform"]
    ox, oy = transform["origin"]
    res = transform["resolution"]
    H, W = transform["shape"]

    # matplotlib tight_layout 通常留 ~12% 边距 — 启发式映射
    x_frac = max(0.0, min(1.0, (x_pix / max(1, disp_w) - 0.10) / 0.80))
    y_frac = max(0.0, min(1.0, (y_pix / max(1, disp_h) - 0.07) / 0.88))
    x_world = ox + x_frac * W * res
    y_world = oy + y_frac * H * res

    # Snap to nearest safe pixel: 落在墙/膨胀区时分两级搜索：
    #   1) 局部半径 1.2m 找距离场最大点；
    #   2) 若局部全是膨胀区，退化为"全局所有 safe 像素中欧氏最近的一个"。
    snapped = False
    try:
        import numpy as _np  # noqa: PLC0415
        from cad_parser.dispatcher import dispatch_cad  # noqa: PLC0415
        from cad_parser.astar_topology import (  # noqa: PLC0415
            compute_distance_field,
        )
        parsed = dispatch_cad(topology_state["source_path"])
        df = compute_distance_field(parsed["grid"])
        rpx = topology_state["robot_radius_px"]
        col = int(round((x_world - ox) / res))
        row = int(round((y_world - oy) / res))
        col = max(0, min(W - 1, col))
        row = max(0, min(H - 1, row))
        if df[row, col] < rpx:
            search_r = max(8, int(1.2 / res))
            r0, r1 = max(0, row - search_r), min(H, row + search_r + 1)
            c0, c1 = max(0, col - search_r), min(W, col + search_r + 1)
            patch = df[r0:r1, c0:c1].copy()
            patch[patch < rpx] = -1
            if patch.max() >= rpx:
                idx = int(patch.argmax())
                pr, pc = divmod(idx, patch.shape[1])
                row = r0 + pr
                col = c0 + pc
                snapped = True
            else:
                # 全局退化：所有 safe 像素中欧氏最近
                safe_mask = df >= rpx
                if safe_mask.any():
                    rows, cols = _np.where(safe_mask)
                    dists = (rows - row) ** 2 + (cols - col) ** 2
                    k = int(dists.argmin())
                    row, col = int(rows[k]), int(cols[k])
                    snapped = True
            if snapped:
                x_world = ox + col * res
                y_world = oy + row * res
    except Exception:
        pass

    new_state = dict(pick_state or {})
    mode = new_state.get("mode", "start")
    if mode == "start" or new_state.get("start_xy") is None:
        new_state["start_xy"] = (round(x_world, 3), round(y_world, 3))
        new_state["mode"] = "goal"
        status = (
            f"✅ 起点 A 已选 ({x_world:.2f}, {y_world:.2f}) m · "
            f"请再点一下选终点 B"
        )
    elif mode == "goal" or new_state.get("goal_xy") is None:
        new_state["goal_xy"] = (round(x_world, 3), round(y_world, 3))
        new_state["mode"] = "done"
        status = (
            f"✅ 终点 B 已选 ({x_world:.2f}, {y_world:.2f}) m · "
            f"自动求 A* 路径..."
        )
    else:
        # 已完成 — 第三次点击当作"重选起点"
        new_state = {"start_xy": (round(x_world, 3), round(y_world, 3)),
                     "goal_xy": None, "mode": "goal"}
        status = (
            f"🔄 重选起点 A ({x_world:.2f}, {y_world:.2f}) m · "
            f"请再点一下选终点 B"
        )

    new_img = _redraw_clickable_with_markers(
        topology_state, new_state, candidates=candidates,
    )
    return new_img, gr.update(), status, new_state


def demo1_reset_picks(
    topology_state: Optional[Dict[str, Any]],
    candidates: Optional[List[Dict[str, Any]]] = None,
):
    empty = {"start_xy": None, "goal_xy": None, "mode": "start"}
    if topology_state is None:
        return None, "🔄 已清空（尚未解析 CAD）", empty
    new_img = _redraw_clickable_with_markers(
        topology_state, empty, candidates=candidates,
    )
    return new_img, "🔄 已清空选点 · 请重新在地图上点击起点 A", empty


def demo1_plan_path_from_click(
    pick_state: Optional[Dict[str, Any]],
    topology_state: Optional[Dict[str, Any]],
    total_time_s: float,
):
    """从点击的两个世界坐标直接求 A* 路径。"""
    pick = pick_state or {}
    if not (pick.get("start_xy") and pick.get("goal_xy")):
        return (
            "", None,
            "ℹ️ 还差一个点（再点一次地图即可）" if pick.get("start_xy")
            else "ℹ️ 请先点击地图选起点 A",
            "",
        )
    sx, sy = pick["start_xy"]
    gx, gy = pick["goal_xy"]
    return _plan_path_core(sx, sy, gx, gy, topology_state, total_time_s)


def _plan_path_core(
    sx: float, sy: float, gx: float, gy: float,
    topology_state: Optional[Dict[str, Any]],
    total_time_s: float,
):
    """求 A* 路径 + 渲染叠加图 + 生成 Demo 3 用 trajectory JSON。"""
    if topology_state is None:
        return (
            "", None,
            "❌ 请先在上方点 [⚙️ 解析 CAD] 生成距离场后再选起终点。",
            "",
        )

    # 直接复用 topology_state 中的 grid 和 distance_field（不再重新解析）
    grid = topology_state.get("grid")
    df = topology_state.get("distance_field")
    if grid is None or df is None:
        return ("", None, "❌ topology_state 缺少 grid/distance_field，请重新解析 CAD。", "")

    try:
        from cad_parser.astar_topology import (  # noqa: PLC0415
            astar_shortest_path, compute_field_gradient, refine_path_via_gradient,
        )
        from smdsl_demo.trajectory_smoother import smooth_path_to_trajectory
    except ImportError as e:
        return ("", None, f"❌ 模块导入失败：{e}", "")

    transform = topology_state["transform"]
    res = transform["resolution"]
    ox, oy = transform["origin"]
    robot_radius_px = topology_state.get("robot_radius_px", 3.0)

    def _w2p(x: float, y: float) -> Tuple[int, int]:
        return int(round((y - oy) / res)), int(round((x - ox) / res))

    start_rc = _w2p(sx, sy)
    goal_rc = _w2p(gx, gy)

    # Snap to nearest safe pixel if clicked point is on wall/inflated
    def _snap_to_safe(rc: Tuple[int, int], max_search: int = 30) -> Tuple[int, int]:
        r, c = rc
        H, W = grid.shape
        if (0 <= r < H and 0 <= c < W and grid[r, c] == 1
                and df[r, c] >= robot_radius_px):
            return rc
        best_rc = rc
        best_d = -1.0
        for dr in range(-max_search, max_search + 1, 2):
            for dc in range(-max_search, max_search + 1, 2):
                nr, nc = r + dr, c + dc
                if 0 <= nr < H and 0 <= nc < W and grid[nr, nc] == 1:
                    if df[nr, nc] > best_d:
                        best_d = df[nr, nc]
                        best_rc = (nr, nc)
        return best_rc

    start_rc = _snap_to_safe(start_rc)
    goal_rc = _snap_to_safe(goal_rc)

    # A* with 1.3x safety buffer — keeps path well clear of inflated zones
    safe_radius_px = robot_radius_px * 1.3
    path = astar_shortest_path(
        grid, df, start_rc, goal_rc,
        robot_radius_px=safe_radius_px,
        safety_weight=1.0, max_steps=2_000_000,
    )
    if path is None:
        return (
            "", None,
            f"❌ 从 ({sx:.2f},{sy:.2f}) → ({gx:.2f},{gy:.2f}) 不可达。"
            f"可能两点之一在膨胀缓冲区里，或被障碍物分隔。",
            "",
        )

    # Phase 2.2: 距离场梯度路径微调
    ref_gx, ref_gy = compute_field_gradient(df)
    path = refine_path_via_gradient(
        path_rc=path,
        distance_field=df,
        gradient_field=(ref_gx, ref_gy),
        robot_radius_px=safe_radius_px,
    )

    traj = smooth_path_to_trajectory(
        path_rc=path,
        resolution=float(res),
        origin_xy=(float(ox), float(oy)),
        total_time_s=float(total_time_s),
    )

    # 用实际出发的像素位置（而非用户原始点击坐标）记录世界坐标
    # 这样路径叠加图与轨迹数据的 A/B 完全一致，消除"漂移"感
    actual_start_x = ox + start_rc[1] * res
    actual_start_y = oy + start_rc[0] * res
    actual_goal_x = ox + goal_rc[1] * res
    actual_goal_y = oy + goal_rc[0] * res

    # Render path as Plotly figure overlay on distance field
    try:
        import plotly.graph_objects as go
        fig_path = go.Figure()
        # Distance field heatmap
        df_m = df * res
        H, W = df.shape
        ds = max(1, max(H, W) // 150)
        df_ds = df_m[::ds, ::ds]
        Hd, Wd = df_ds.shape
        fig_path.add_trace(go.Heatmap(
            z=df_ds,
            x=np.linspace(ox, ox + W * res, Wd),
            y=np.linspace(oy, oy + H * res, Hd),
            colorscale="Viridis", name="Clearance (m)",
            colorbar=dict(title="m"), zmin=0,
        ))
        # Path line
        px = [ox + c * res for _, c in path]
        py = [oy + r * res for r, _ in path]
        fig_path.add_trace(go.Scatter(
            x=px, y=py, mode="lines+markers",
            line=dict(color="cyan", width=4),
            marker=dict(size=6, color="cyan"),
            name="A* Path",
        ))
        # Start/end
        fig_path.add_trace(go.Scatter(
            x=[ox + start_rc[1] * res], y=[oy + start_rc[0] * res],
            mode="markers", marker=dict(size=12, color="lime", symbol="diamond"),
            name="Start A",
        ))
        fig_path.add_trace(go.Scatter(
            x=[ox + goal_rc[1] * res], y=[oy + goal_rc[0] * res],
            mode="markers", marker=dict(size=14, color="yellow", symbol="cross"),
            name="Goal B",
        ))
        fig_path.update_layout(
            title=f"A* Path A->B (clearance >= {robot_radius_px*res:.2f}m)",
            xaxis_title="X (m)", yaxis_title="Y (m)",
            yaxis=dict(scaleanchor="x", scaleratio=1),
            margin=dict(l=20, r=20, t=40, b=20), height=450,
        )
        path_plotly = fig_path
    except Exception:
        path_plotly = None

    traj_text = json.dumps(traj, indent=2, ensure_ascii=False)
    info = {
        "start_world": {"x": actual_start_x, "y": actual_start_y},
        "goal_world": {"x": actual_goal_x, "y": actual_goal_y},
        "n_path_pixels": len(path),
        "n_trajectory_waypoints": len(traj),
        "total_time_s": float(total_time_s),
    }
    status = (
        f"✅ A→B 路径已求解 · 路径像素 {len(path)} · 轨迹路点 {len(traj)} · "
        f"已自动同步至 Tab 3"
    )
    return (
        json.dumps(info, indent=2, ensure_ascii=False),
        path_plotly, status, traj_text,
    )


def demo1_plan_path(
    start_label: str,
    goal_label: str,
    topology_state: Optional[Dict[str, Any]],
    candidates: List[Dict[str, Any]],
    total_time_s: float,
):
    """从两个候选标签求 A* 最短路径（与点击模式互补）。"""
    if not start_label or not goal_label:
        return ("", None, "❌ 请先选择起点 A 与终点 B。", "")
    label_to_pt = {c["label"]: c for c in (candidates or [])}
    if start_label not in label_to_pt or goal_label not in label_to_pt:
        return ("", None, "❌ 选项不在候选清单中（可能换样本后未刷新）", "")
    sx, sy = label_to_pt[start_label]["x"], label_to_pt[start_label]["y"]
    gx, gy = label_to_pt[goal_label]["x"], label_to_pt[goal_label]["y"]
    return _plan_path_core(sx, sy, gx, gy, topology_state, total_time_s)


# ══════════════════════════════════════════════════════════════════════
# Tab 2 — DeepSeek 语义编译
# ══════════════════════════════════════════════════════════════════════

_VLM_SINGLETON: Optional[VlmParser] = None


def _get_vlm(api_key_override: str = "") -> VlmParser:
    global _VLM_SINGLETON
    if api_key_override:
        # 用户提供了 UI 中的 key → 新建实例（不缓存）
        return VlmParser(api_key=api_key_override)
    if _VLM_SINGLETON is None:
        _VLM_SINGLETON = VlmParser()
    return _VLM_SINGLETON


def demo2_test_api_key(api_key_text: str) -> str:
    """测试 DeepSeek API key 是否有效。"""
    resolved = api_key_text.strip() if api_key_text else os.environ.get("DEEPSEEK_KEY", "") or os.environ.get("DEEPSEEK_API_KEY", "")
    if not resolved:
        return "MISSING: 未提供 Key（密码框为空且环境变量也未设置）"
    try:
        from openai import OpenAI
        client = OpenAI(api_key=resolved, base_url="https://api.deepseek.com")
        resp = client.models.list()
        return f"OK: Key valid (models: {len(resp.data)})"
    except Exception as e:
        err = str(e)
        if "401" in err or "Authentication" in err:
            return "FAIL: 401 鉴权失败，Key 无效或已过期"
        return f"FAIL: {err[:100]}"


def demo2_infer_context(instruction: str, api_key_override: str = ""):
    if not instruction or not instruction.strip():
        return "", "❌ 请先在上面输入自然语言指令。"
    try:
        vlm = _get_vlm(api_key_override=api_key_override.strip() if api_key_override else "")
        ctx = vlm.infer_local_context(instruction)
    except VlmParserError as e:
        return "", f"❌ 推断失败：{e}"
    return (
        json.dumps(ctx, indent=2, ensure_ascii=False),
        f"✅ 已抽取 {len(ctx.get('nearest_objects', []))} 个实体；"
        f"如有不准可手动调整后再编译。",
    )


def demo2_summarize(roboir_json_text: str, api_key_override: str = ""):
    if not roboir_json_text or not roboir_json_text.strip():
        return "", "❌ 请先编译生成 RoboIR。"
    try:
        roboir = json.loads(roboir_json_text)
    except json.JSONDecodeError:
        return "", "❌ RoboIR 不是合法 JSON。"
    if "error" in roboir:
        return "", "❌ 当前 RoboIR 是错误对象，无法摘要。"
    try:
        vlm = _get_vlm(api_key_override=api_key_override.strip() if api_key_override else "")
        text = vlm.summarize_roboir(roboir)
    except VlmParserError as e:
        return "", f"❌ 摘要失败：{e}"
    return (
        f"**STL 风格声明式描述：**\n\n> {text}\n\n"
        f"*⚠️ 此摘要仅供阅读检查 LLM 是否理解对了你的意图；"
        f"机器人实际执行依据是 RoboIR JSON，不是这段文字。*"
    ), "✅ STL 摘要生成完毕"


def demo2_generate_examples(
    scene_profile: Dict[str, Any],
    topology_state: Optional[Dict[str, Any]],
    api_key_override: str = "",
):
    """根据场景画像 + 文件格式，调用 LLM 生成 3 个情境化指令示例。

    Returns: (display_md, examples_json_str, status_md)
    """
    # scene_profile_state 应为 dict；早期版本可能误传 JSON 字符串
    if isinstance(scene_profile, str):
        try:
            scene_profile = json.loads(scene_profile)
        except Exception:
            scene_profile = {}
    if not scene_profile:
        return (
            "*⚠️ 请先在 Tab 1 解析 CAD 并点击 [🧠 生成环境语义画像]，再生成示例。*",
            "[]",
            "⚠️ 场景画像未生成 — 请先完成 Tab 1 解析 + 语义画像步骤",
        )

    file_format = "unknown"
    if topology_state:
        file_format = topology_state.get("mode", "unknown")

    try:
        vlm = _get_vlm(api_key_override=api_key_override.strip() if api_key_override else "")
        examples = vlm.generate_instruction_examples(
            scene_profile=scene_profile,
            file_format=file_format,
            n_examples=3,
        )
    except VlmParserError as e:
        return (
            f"*❌ 生成失败：{e}*",
            "[]",
            f"❌ LLM 示例生成失败：{e}",
        )

    # 补足 3 条
    while len(examples) < 3:
        examples.append({"label": f"示例 {len(examples)+1}", "instruction": "", "context": ""})
    examples = examples[:3]

    # 构建展示 Markdown
    lines = [f"**已生成 {len(examples)} 个情境示例（{file_format} 格式场景）：**\n"]
    for i, ex in enumerate(examples, 1):
        lines.append(f"**{i}. {ex['label']}**  ")
        lines.append(f"> {ex['instruction']}\n")
    display_md = "\n".join(lines)
    examples_json = json.dumps(examples, ensure_ascii=False)

    return (
        display_md,
        examples_json,
        f"✅ 已生成 {len(examples)} 个情境示例 · 点 [📋 填入示例 N] 按钮使用",
    )


def demo3_diagnostic_report(
    roboir_json_text: str,
    trajectory_json_text: str,
    demo3_output_json_text: str,
    api_key_override: str = "",
):
    """
    让 DeepSeek 把 Demo 3 的求解输出翻译成 4 节中文诊断报告。
    """
    if not roboir_json_text or not roboir_json_text.strip():
        return "", "❌ RoboIR 为空（请先在 Demo 2 编译，或手填）"
    if not trajectory_json_text or not trajectory_json_text.strip():
        return "", "❌ 轨迹为空"
    if not demo3_output_json_text or not demo3_output_json_text.strip():
        return "", "❌ 还没跑 STL 求解 — 请先点 [🔬 执行 STL 物理求解]"

    try:
        roboir = json.loads(roboir_json_text)
        traj = json.loads(trajectory_json_text)
        demo3_out = json.loads(demo3_output_json_text)
    except json.JSONDecodeError as e:
        return "", f"❌ JSON 解析失败：{e}"

    if "error" in demo3_out:
        return "", "❌ Demo 3 输出含错误，无法诊断"

    try:
        vlm = _get_vlm(api_key_override=api_key_override.strip() if api_key_override else "")
        report = vlm.summarize_diagnostic(roboir, traj, demo3_out)
    except VlmParserError as e:
        return "", f"❌ DeepSeek 失败：{e}"
    return (
        f"**🩺 DeepSeek 诊断报告**\n\n{report}\n\n"
        f"*基于 RoboIR + 实际轨迹 + 结构化反馈生成。仅供参考，"
        f"机器人执行依据仍是 RoboIR JSON。*"
    ), f"✅ 诊断报告已生成 @ {_ts()}"


# ── Spatial-RAG: 环境语义画像按钮 ───────────────────

def demo1_3d_preview(topology_state: Dict[str, Any]) -> Tuple[Any, str]:
    """生成 3D 拓扑白模预览。"""
    if not topology_state or topology_state.get("grid") is None:
        return None, "*请先运行 Demo 1 解析 CAD。*"
    try:
        from cad_parser.visualize import generate_3d_topology_preview  # noqa: PLC0415
        pipeline = {
            "grid": topology_state.get("grid"),
            "distance_field": topology_state.get("distance_field"),
            "topology": topology_state.get("topology"),
            "transform": topology_state.get("transform", {}),
            "seeds": [],  # seeds are in pixel coords inside demo1_run
            "semantics": topology_state.get("semantics", {}),
            "cad_data": topology_state.get("cad_data", {}),
        }
        fig = generate_3d_topology_preview(pipeline)
        return fig, "3D topology preview generated (drag to rotate/zoom)"
    except Exception as e:
        return None, f"3D preview failed: {e}"


def demo1_scene_graph_3d(topology_state: Dict[str, Any]) -> Tuple[Any, str]:
    """
    生成 3D 空间场景图（浏览器内 Plotly，无需下载）。

    - OSM 模式：房间多边形 Mesh3d + 拓扑通路图层 + Hover 语义
    - 其他模式：距离场地板热力图 + 墙体线框 + 连通区域着色
    """
    if not topology_state or topology_state.get("grid") is None:
        return None, "❌ 请先在 Tab 1 解析 CAD 文件。"

    mode = topology_state.get("mode", "")

    try:
        if mode == "osm" and topology_state.get("osm_rooms"):
            # OSM 专属：精确多边形场景图
            from smdsl_demo.visualize_demo3 import generate_osm_3d_scene  # noqa: PLC0415
            parse_result = {
                "mode": "osm",
                "osm_rooms": topology_state.get("osm_rooms", []),
                "osm_edges": topology_state.get("osm_edges", []),
                "adjacency_graph": topology_state.get("adjacency_graph", {}),
            }
            src = Path(topology_state.get("source_path", "")).name
            fig = generate_osm_3d_scene(
                parse_result,
                wall_height=3.2,
                title=f"3D Space Graph — {src}",
                show_topology_edges=True,
            )
            n_rooms = len(topology_state.get("osm_rooms", []))
            n_edges = len(topology_state.get("osm_edges", []))
            status = (
                f"✅ 3D 场景图生成完成 · OSM 模式 · "
                f"{n_rooms} 个房间多边形 · {n_edges} 条拓扑通路 · "
                "可拖转旋转/滚轮缩放/悬停查看房间语义"
            )
        else:
            # 其他模式：grid 场景图（距离场地板 + 等高线幕墙）
            from smdsl_demo.visualize_demo3 import generate_grid_3d_scene  # noqa: PLC0415
            pipeline = {
                "grid": topology_state.get("grid"),
                "distance_field": topology_state.get("distance_field"),
                "topology": topology_state.get("topology"),
                "transform": topology_state.get("transform", {}),
                "seeds": topology_state.get("seeds", []),
                "semantics": topology_state.get("semantics", {}),
                "cad_data": topology_state.get("cad_data", {}),
            }
            src = Path(topology_state.get("source_path", "")).name
            fig = generate_grid_3d_scene(
                pipeline,
                wall_height=3.0,
                title=f"3D Space Graph — {src} [{mode.upper()}]",
            )
            status = (
                f"✅ 3D 场景图生成完成 · {mode.upper()} 模式 · "
                "距离场地板 + 3D 墙体幕墙 · 可拖转/缩放/Hover"
            )
        return fig, status
    except Exception as e:  # noqa: BLE001
        import traceback  # noqa: PLC0415
        return None, f"❌ 3D 场景图生成失败：{e}\n{traceback.format_exc()[:400]}"


def demo1_profile(topology_state: Dict[str, Any],
                  api_key_override: str = "") -> Tuple[Any, Any, Any]:
    """
    独立按钮：基于 Demo 1 解析结果，调用 LLM 生成环境语义画像。
    同时返回推荐约束 JSON 供 Tab 2 使用。
    """
    if not topology_state:
        return (
            json.dumps({"error": "no_topology"}, indent=2, ensure_ascii=False),
            "*请先运行 Demo 1 解析 CAD，再生成环境语义画像。*",
            {},
        )

    # ── 键值校验：打印缺失的 key 帮助排查数据流 ──────────
    required = ["grid", "distance_field", "topology", "transform"]
    missing = [k for k in required if k not in topology_state or topology_state.get(k) is None]
    if missing:
        available = list(topology_state.keys())
        print(f"[Spatial-RAG] topology_state 缺失键: {missing}，当前键: {available}")
        return (
            json.dumps({"error": "topology_incomplete", "missing_keys": missing,
                        "available_keys": available}, indent=2, ensure_ascii=False),
            f"*环境画像无法生成：topology_state 缺少 {missing}。请重新运行 Demo 1 解析。*",
            {},
        )

    try:
        from cad_parser.semantic_profiler import profile_scene  # noqa: PLC0415

        grid = topology_state["grid"]
        distance_field = topology_state["distance_field"]
        topology = topology_state["topology"]
        transform = topology_state.get("transform", {})
        robot_radius_px = topology_state.get("robot_radius_px", 0.0)
        topo_stats = topology_state.get("topo_stats", {})
        cad_data = topology_state.get("cad_data") or {}
        semantics = topology_state.get("semantics") or {}

        pipeline = {
            "grid": grid,
            "distance_field": distance_field,
            "topology": topology,
            "transform": transform,
            "robot_radius_px": robot_radius_px,
            "topo_stats": topo_stats,
            "cad_data": cad_data,
            "semantics": semantics,
        }

        result = profile_scene(pipeline, api_key=(
            api_key_override.strip() if api_key_override else None))

        if result["status"] != "ok":
            return (
                json.dumps(result, indent=2, ensure_ascii=False, default=str),
                f"*语义画像生成失败：{result.get('message', '未知错误')}*",
                {},
            )

        profile = result["profile"] or {}
        features = result["features"] or {}

        # 构建 Markdown 展示
        md_lines = [
            "## Spatial-RAG 环境语义画像",
            "",
            f"**场景类型**: {profile.get('scene_type', 'unknown')}",
            f"**复杂度评分**: {features.get('complexity_score', 0):.2f} (0=简单 1=极复杂)",
            "",
            f"**{profile.get('spatial_layout_summary', '')}**",
            "",
            "### 物理特征",
            f"- 可通行面积: **{features.get('navigable_area_sqm', 0):.1f} m²**",
            f"- 最大 clearance: **{features.get('max_clearance_m', 0):.2f} m**",
            f"- 死区比例: **{features.get('deadzone_fraction', 0):.1%}**",
            f"- 不连通区域: **{features.get('n_disconnected_regions', 0)}**",
        ]

        bottlenecks = features.get("bottlenecks", [])
        if bottlenecks:
            md_lines.append("")
            md_lines.append("### 狭窄通道（瓶颈）")
            for b in bottlenecks[:6]:
                sev_icon = {"high": "🔴", "medium": "🟡", "low": "🟢"}.get(b.get("severity", ""), "⚪")
                md_lines.append(f"- {sev_icon} 宽度 {b['width_m']:.2f}m [{b.get('severity', '?')}]")

        rc = profile.get("recommended_global_constraints", {})
        if rc:
            md_lines.append("")
            md_lines.append("### 推荐全局约束（已同步到 Tab 2）")
            md_lines.append(f"- 安全距离: **{rc.get('safety_distance_m', 'N/A')} m**")
            md_lines.append(f"- 最大速度: **{rc.get('max_velocity_ms', 'N/A')} m/s**")
            md_lines.append(f"- 精细抓取: **{rc.get('requires_precise_grasp', False)}**")
            hr = rc.get("high_risk_zones", [])
            if hr:
                md_lines.append(f"- 高风险区域: {hr}")

        md_lines.append("")
        md_lines.append(f"> 场景描述：{profile.get('scene_description', '')}")

        # 约束 JSON（给 Tab 2 的隐藏 state）
        constraints_for_tab2 = {
            "scene_type": profile.get("scene_type", "unknown"),
            "recommended_global_constraints": rc,
            "spatial_layout_summary": profile.get("spatial_layout_summary", ""),
            "complexity_score": features.get("complexity_score", 0),
        }

        return (
            json.dumps(result, indent=2, ensure_ascii=False, default=str),
            "\n".join(md_lines),
            constraints_for_tab2,   # dict，非 JSON 字符串
        )
    except Exception as e:
        import traceback
        return (
            json.dumps({"error": str(e), "trace": traceback.format_exc()}, indent=2, ensure_ascii=False),
            f"*语义画像生成异常：{e}*",
            {},
        )


def demo2_run(instruction: str, local_context_text: str,
              scene_profile: Optional[Dict[str, Any]] = None,
              api_key_override: str = ""):
    """
    Returns:
        roboir_json_text, validation_md, status_md
    """
    if not instruction or not instruction.strip():
        return (
            json.dumps({"error": "instruction_empty"},
                       indent=2, ensure_ascii=False),
            "", "❌ 自然语言指令为空。",
        )

    if local_context_text and local_context_text.strip():
        try:
            local_context = json.loads(local_context_text)
        except json.JSONDecodeError as e:
            return (
                json.dumps({"error": "local_context_invalid_json",
                            "message": str(e)}, indent=2, ensure_ascii=False),
                "", f"❌ 局部环境 JSON 解析失败：{e}",
            )
    else:
        local_context = {}

    # 注入 Spatial-RAG 全局约束（如果 Tab 1 已生成）
    if scene_profile and isinstance(scene_profile, dict):
        constraints = scene_profile.get("recommended_global_constraints", {})
        if constraints:
            local_context["_global_scene_profile"] = {
                "scene_type": scene_profile.get("scene_type", "unknown"),
                "spatial_layout": scene_profile.get("spatial_layout_summary", ""),
                "constraints": constraints,
            }

    try:
        vlm = _get_vlm(api_key_override=api_key_override.strip() if api_key_override else "")
        result = vlm.parse_instruction_to_roboir(instruction, local_context)
    except VlmParserError as e:
        err_msg = str(e)
        is_auth_err = "401" in err_msg or "authentication" in err_msg.lower() or "invalid" in err_msg.lower()
        if is_auth_err:
            status = (
                "❌ API 鉴权失败，请检查你的 API Key 是否有效或账户是否欠费。"
            )
        else:
            status = f"❌ DeepSeek 失败：{err_msg}"
        return (
            json.dumps({"error": "vlm_failed", "message": err_msg},
                       indent=2, ensure_ascii=False),
            "", status,
        )
    except Exception:  # noqa: BLE001
        return (
            json.dumps({"error": "unexpected",
                        "traceback": traceback.format_exc().splitlines()[-6:]},
                       indent=2, ensure_ascii=False),
            "", "❌ 非预期异常",
        )

    warnings = validate_roboir_references(result, local_context)
    if warnings:
        lines = ["**⚠️ 引用一致性校验：**"]
        for w in warnings:
            lines.append(
                f"- `{w['field']}` = `{w['value']}` — {w['message']}  \n"
                f"  *Hint*: {w['hint']}"
            )
        validation_md = "\n".join(lines)
        status = (
            f"⚠️ 编译完成但有 {len(warnings)} 条引用警告 "
            f"（Tab 3 可能用回退策略求解）"
        )
    else:
        validation_md = "✅ **引用一致性校验通过** — 所有 ref 都在 nearest_objects 中。"
        status = "✅ 编译完成 + 校验通过 · 已自动同步至 Tab 3"

    return (
        json.dumps(result, indent=2, ensure_ascii=False),
        validation_md, status,
    )


_DEMO2_EXAMPLES = [
    {
        "label": "① 搬马克杯（厨房，避障）",
        "instruction": "避开中间的设备，把绿色的马克杯放到桌子中间，杯子不能倒。",
        "context": json.dumps(
            {"nearest_objects": ["green_mug", "center_table",
                                 "obstacle_machine"]},
            ensure_ascii=False, indent=2,
        ),
    },
    {
        "label": "② 取金属板（仓库，限时）",
        "instruction": "请在 4 秒内用磁力抓手把金属板从货架 A 搬到传送带 B，"
                       "全程保持板子水平，不要碰到旁边的玻璃瓶。",
        "context": json.dumps(
            {"nearest_objects": ["metal_plate", "shelf_A", "conveyor_B",
                                 "glass_bottle"]},
            ensure_ascii=False, indent=2,
        ),
    },
    {
        "label": "③ 巡检（无抓取，时序约束）",
        "instruction": "巡检走一圈，全程距离任何障碍物至少 0.5 米，不要碰到任何东西。",
        "context": json.dumps(
            {"nearest_objects": ["patrol_zone_center"]},
            ensure_ascii=False, indent=2,
        ),
    },
]


# ══════════════════════════════════════════════════════════════════════
# Tab 3 — STL 物理求解
# ══════════════════════════════════════════════════════════════════════

def _classify(rep: Dict[str, Any]) -> FailureTaxonomy:
    expr = rep.get("rule_expr", "")
    src = rep.get("source", "")
    if "Distance" in expr and ">" in expr:
        if src == "distance_field":
            return FailureTaxonomy.COLLISION
        return FailureTaxonomy.STL_VIOLATION
    if "Time" in expr and "<" in expr:
        return FailureTaxonomy.TIMEOUT
    return FailureTaxonomy.STL_VIOLATION


def _extract_min_distance(constraints: List[Dict[str, Any]]) -> Optional[float]:
    import re as _re  # noqa: PLC0415
    best: Optional[float] = None
    for c in constraints:
        expr = str(c.get("expr", ""))
        m = _re.search(r"Distance\s*>\s*([\d.]+)", expr)
        if m:
            v = float(m.group(1))
            best = v if best is None else max(best, v)
    return best


def _build_pose_table(
    trajectory: List[Dict[str, float]],
    roboir: Dict[str, Any],
    bundle: Optional[Dict[str, Any]],
    min_distance_m: Optional[float],
) -> List[List[Any]]:
    """
    起点 / Goal (in: RoboIR 声明) / 终点（实际轨迹末尾）三栏 Pose 表。
    """
    if not trajectory:
        return []

    start = trajectory[0]
    end = trajectory[-1]

    target_frame = roboir.get("target_frame", "?")
    goal_in = {"x": "?", "y": "?", "z": "?",
               "roll": "?", "pitch": "?", "yaw": "?"}

    rho_start = "—"
    rho_end = "—"
    rho_label = "ρ at this pose"
    if bundle is not None:
        try:
            from smdsl_demo.visualize_demo3 import (  # noqa: PLC0415
                _sample_distance_at,
            )
            d_s = _sample_distance_at(start.get("x", 0), start.get("y", 0),
                                      bundle)
            d_e = _sample_distance_at(end.get("x", 0), end.get("y", 0), bundle)
            if min_distance_m is not None:
                rho_label = f"ρ = clearance − {min_distance_m:.2f}"
                rho_start = f"{d_s - min_distance_m:+.3f} m"
                rho_end = f"{d_e - min_distance_m:+.3f} m"
            else:
                rho_label = "clearance (无 Distance 约束)"
                rho_start = f"{d_s:.3f} m"
                rho_end = f"{d_e:.3f} m"
            if d_s == 0.0:
                rho_start += "  ⚠️ 起点超出距离场"
            if d_e == 0.0:
                rho_end += "  ⚠️ 终点超出距离场"
        except Exception as e:  # noqa: BLE001
            rho_start = f"<err: {type(e).__name__}>"
            rho_end = "—"

    rows = [
        ["x (m)", start.get("x", "?"), goal_in["x"], end.get("x", "?")],
        ["y (m)", start.get("y", "?"), goal_in["y"], end.get("y", "?")],
        ["z (m)", start.get("z", "?"), goal_in["z"], end.get("z", "?")],
        ["roll (rad)", start.get("roll", "?"), goal_in["roll"],
         end.get("roll", "?")],
        ["pitch (rad)", start.get("pitch", "?"), goal_in["pitch"],
         end.get("pitch", "?")],
        ["yaw (rad)", start.get("yaw", "?"), goal_in["yaw"],
         end.get("yaw", "?")],
        ["t (s)", start.get("t", "?"), "—", end.get("t", "?")],
        ["target_frame", "—", target_frame, "—"],
        [rho_label, rho_start, "—", rho_end],
    ]
    return rows


def demo3_run(
    roboir_json_text: str,
    trajectory_json_text: str,
    use_distance_field: bool,
    topology_state: Optional[Dict[str, Any]],
):
    """
    Returns:
        json_text, traj_png, rho_png, pose_table_data, status_md
    """
    err_quad = lambda payload, status: (  # noqa: E731
        json.dumps(payload, indent=2, ensure_ascii=False),
        None, None, [], status,
    )
    empty_plotly = None  # type: ignore
    if not roboir_json_text or not roboir_json_text.strip():
        return err_quad({"error": "roboir_empty"}, "❌ RoboIR 为空")
    if not trajectory_json_text or not trajectory_json_text.strip():
        return err_quad({"error": "trajectory_empty"}, "❌ 轨迹为空")

    try:
        roboir = json.loads(roboir_json_text)
    except json.JSONDecodeError as e:
        return err_quad({"error": "roboir_invalid_json", "message": str(e)},
                        "❌ RoboIR JSON 解析失败")

    try:
        trajectory = json.loads(trajectory_json_text)
    except json.JSONDecodeError as e:
        return err_quad({"error": "trajectory_invalid_json", "message": str(e)},
                        "❌ 轨迹 JSON 解析失败")

    if not isinstance(trajectory, list) or not trajectory:
        return err_quad({"error": "trajectory_must_be_nonempty_list"},
                        "❌ 轨迹必须是非空 list")

    raw_constraints = roboir.get("stl_constraints", [])
    if not isinstance(raw_constraints, list) or not raw_constraints:
        return err_quad({"error": "stl_constraints_empty"},
                        "❌ RoboIR.stl_constraints 为空")

    constraints = normalize_stl_constraints(raw_constraints)

    if use_distance_field and topology_state is not None:
        bundle = topology_state.get("bundle")
        bundle_status = (
            f"loaded from Tab 1 ({topology_state.get('mode')}, "
            f"{Path(topology_state.get('source_path', '')).name})"
        )
    elif use_distance_field:
        bundle = _load_default_bundle()
        bundle_status = (
            "loaded default kitchen_room_24 (Tab 1 未先行解析)"
            if bundle is not None else "load_failed"
        )
    else:
        bundle = None
        bundle_status = "disabled"

    try:
        reports = check_stl_constraint_violation(
            trajectory, constraints, topology_bundle=bundle,
        )
    except Exception:  # noqa: BLE001
        return err_quad(
            {"error": "stl_solver_exception",
             "traceback": traceback.format_exc().splitlines()[-8:]},
            "❌ STL 求解异常",
        )

    feedbacks: List[Dict[str, Any]] = []
    n_violations = 0
    for rep in reports:
        if rep.get("violated"):
            n_violations += 1
            taxonomy = _classify(rep)
            nodes = rep.get("violation_nodes", [])
            worst = (
                min(nodes, key=lambda n: n.get("rho", 0.0))
                if nodes else None
            )
            fb = generate_structured_feedback(
                taxonomy, rep.get("robustness", 0.0),
                {
                    "rule": rep.get("rule_expr"),
                    "source": rep.get("source"),
                    "max_violation": rep.get("max_violation"),
                    "violation_duration": rep.get("violation_duration"),
                    "n_violation_nodes": len(nodes),
                    "worst_node": worst,
                    "violation_nodes_preview": nodes[:5],
                },
            )
        else:
            fb = {
                "error": {
                    "type": "none", "severity": "info",
                    "robustness_score": round(
                        float(rep.get("robustness", 0.0)), 4),
                },
                "diagnosis": {
                    "summary": "约束满足",
                    "rule": rep.get("rule_expr"),
                    "source": rep.get("source"),
                },
            }
        feedbacks.append(fb)

    output = {
        "intent": roboir.get("intent"),
        "target_frame": roboir.get("target_frame"),
        "n_constraints": len(constraints),
        "n_violations": n_violations,
        "n_total_constraints": len(reports),
        "topology_bundle": bundle_status,
        "feedbacks": feedbacks,
        "raw_reports_summary": [
            {
                "rule_expr": r.get("rule_expr"),
                "source": r.get("source"),
                "robustness": (None if r.get("robustness") is None
                               else round(float(r["robustness"]), 4)),
                "violated": r.get("violated"),
                "n_violation_nodes": len(r.get("violation_nodes", [])),
            }
            for r in reports
        ],
    }
    json_text = json.dumps(output, indent=2, ensure_ascii=False, default=str)

    # ── 生成 Plotly 3D 可视化 ──────────────────────────────
    min_d = _extract_min_distance(constraints)

    # 收集所有 violation_nodes
    all_violations: List[Dict[str, Any]] = []
    for rep in reports:
        if rep.get("violated"):
            all_violations.extend(rep.get("violation_nodes", []))

    try:
        from smdsl_demo.visualize_demo3 import generate_3d_dashboard  # noqa: PLC0415
        topo = topology_state.get("topology") if topology_state else None
        fig_3d, fig_rho = generate_3d_dashboard(
            trajectory=trajectory,
            bundle=bundle,
            topology=topo,
            violation_nodes=all_violations if all_violations else None,
            min_distance_m=min_d,
        )
    except Exception:
        fig_3d, fig_rho = None, None

    pose_table = _build_pose_table(trajectory, roboir, bundle, min_d)

    icon = "OK" if n_violations == 0 else "WARN"
    status = (
        f"{icon} {n_violations} / {len(reports)} constraints violated · "
        f"distance field: {bundle_status}"
    )
    return json_text, fig_3d, fig_rho, pose_table, status


_DEFAULT_BUNDLE_CACHE: Optional[Dict[str, Any]] = None


def _load_default_bundle() -> Optional[Dict[str, Any]]:
    """无 Tab 1 状态时的兜底 bundle：kitchen room_24。"""
    global _DEFAULT_BUNDLE_CACHE
    if _DEFAULT_BUNDLE_CACHE is not None:
        return _DEFAULT_BUNDLE_CACHE
    try:
        from cad_parser.astar_topology import (  # noqa: PLC0415
            run_pipeline, to_topology_bundle,
        )
        layout_path = (
            _REPO_ROOT
            / "data/cad_samples/floorplanqa/layouts/kitchen/room_24.json"
        )
        if not layout_path.exists():
            return None
        pipe = run_pipeline(
            str(layout_path), resolution=0.05, robot_radius_m=0.25,
            include_grid=False,
        )
        _DEFAULT_BUNDLE_CACHE = to_topology_bundle(pipe)
        return _DEFAULT_BUNDLE_CACHE
    except Exception:
        return None


# ══════════════════════════════════════════════════════════════════════
# UI defaults
# ══════════════════════════════════════════════════════════════════════

DEFAULT_INSTRUCTION = _DEMO2_EXAMPLES[0]["instruction"]
DEFAULT_LOCAL_CTX = _DEMO2_EXAMPLES[0]["context"]

DEFAULT_DEMO3_ROBOIR = json.dumps(
    {
        "intent": "patrol_loop_demo",
        "target_frame": "patrol_zone_center",
        "grasp_type": "none",
        "stl_constraints": [
            {"expr": "Distance > 0.30", "ref": "obstacle"},
            {"expr": "Time < 5.0"},
        ],
    },
    indent=2, ensure_ascii=False,
)

_DEFAULT_TRAJECTORY = [
    {"t": 0.0, "x": 4.10, "y": 2.10, "z": 0.0,
     "roll": 0.0, "pitch": 0.0, "yaw": 0.0},
    {"t": 0.5, "x": 4.30, "y": 2.50, "z": 0.0,
     "roll": 0.0, "pitch": 0.0, "yaw": 0.0},
    {"t": 1.0, "x": 4.30, "y": 3.00, "z": 0.0,
     "roll": 0.0, "pitch": 0.0, "yaw": 0.0},
    {"t": 1.5, "x": 4.30, "y": 3.50, "z": 0.0,
     "roll": 0.0, "pitch": 0.0, "yaw": 0.0},
    {"t": 2.0, "x": 4.30, "y": 3.95, "z": 0.0,
     "roll": 0.0, "pitch": 0.0, "yaw": 0.0},
]
DEFAULT_TRAJECTORY_TEXT = json.dumps(_DEFAULT_TRAJECTORY, indent=2)


def _api_key_status_md() -> str:
    from smdsl_demo.vlm_parser import DEEPSEEK_API_KEY_ENV_FALLBACK  # noqa: PLC0415
    has_key1 = bool(os.environ.get(DEEPSEEK_API_KEY_ENV, "").strip())
    has_key2 = bool(os.environ.get(DEEPSEEK_API_KEY_ENV_FALLBACK, "").strip())
    has_key = has_key1 or has_key2
    icon = "OK" if has_key else "MISSING"
    source = f"DEEPSEEK_API_KEY" if has_key1 else (f"DEEPSEEK_KEY" if has_key2 else "无")
    return (
        f"**API Key 状态**: `{icon}` 来源=`{source}`  ·  "
        f"**模型**: `{DEEPSEEK_DEFAULT_MODEL}`  ·  "
        f"**最大重试**: 3 次（指数回退）"
    )


def _flow_nav_md(active: int) -> str:
    """返回当前 Tab 的进度条 HTML（顶部紧凑版，节点可点击跳转）。

    Args:
        active: 当前 Demo 序号 (1/2/3)，0 表示全局头部（无激活节点）。
    """
    # (idx, 标题, 副标题, 对应 tab 索引 — None 表示输入节点不跳转)
    nodes = [
        (0, "自然语言 + CAD", "用户输入",      None),
        (1, "● 环境感知",    "Demo 1 · 距离场 / 拓扑", 0),
        (2, "语义编译",      "Demo 2 · STL RoboIR",   1),
        (3, "物理求解",      "Demo 3 · ρ + 反馈",     2),
    ]
    cells = []
    for idx, name, sub, tab_idx in nodes:
        if idx == active and idx != 0:
            style = (
                "background:#fdf3ee;color:#94472f;font-weight:600;"
                "border:1px solid #cc785c;"
            )
        elif idx == 0:
            style = (
                "background:#ffffff;color:#6c6a65;font-weight:500;"
                "border:1px dashed #d9d6d0;"
            )
        elif idx < active:
            style = (
                "background:#f7f5f1;color:#2a2a2a;"
                "border:1px solid #ece9e3;font-weight:500;"
            )
        else:
            style = (
                "background:#ffffff;color:#a8a59f;font-weight:400;"
                "border:1px solid #ece9e3;"
            )

        # Gradio 4.x 用 role="tab" 渲染 tab 按钮，文字匹配最稳健
        _demo_labels = ["Demo 1", "Demo 2", "Demo 3"]
        if tab_idx is not None:
            _kw = _demo_labels[tab_idx]
            onclick = (
                f"onclick=\"(function(){{"
                f"var tabs=Array.from(document.querySelectorAll('[role=tab]'));"
                f"var t=tabs.find(function(el){{return el.textContent.indexOf('{_kw}')>-1;}});"
                f"if(t)t.click();"
                f"}})()\" "
            )
            extra_style = "cursor:pointer;transition:box-shadow .15s;"
            hover_title = f"title='点击跳转到 {_kw}'"
        else:
            onclick = ""
            extra_style = ""
            hover_title = ""

        cells.append(
            f"<div {onclick}{hover_title} style='flex:1;text-align:center;"
            f"padding:10px 12px;border-radius:10px;{style}{extra_style}"
            f"font-size:13px;line-height:1.4'>"
            f"<div>{name}</div>"
            f"<div style='font-size:11px;opacity:.7;margin-top:3px;"
            f"font-weight:400'>{sub}</div>"
            "</div>"
        )
    arrow = (
        "<div style='align-self:center;color:#cfcbc2;font-size:14px;"
        "padding:0 6px'>→</div>"
    )
    inner = arrow.join(cells)
    return (
        "<div class='smdsl-flow-nav' style='display:flex;"
        "align-items:stretch;gap:0;margin:6px 0 16px 0;'>" + inner + "</div>"
    )


def _ts() -> str:
    return _dt.datetime.now().strftime("%H:%M:%S")


# ══════════════════════════════════════════════════════════════════════
# 文档站风格 CSS（致敬 https://code.claude.com/docs/en/overview）
# ══════════════════════════════════════════════════════════════════════

_CLAUDE_DOCS_CSS = """
/* ── 强制亮色主题：覆盖 Gradio dark-mode CSS 变量 ────────────── */
:root, .dark, body, html {
    --body-background-fill:           #fbfaf8 !important;
    --body-background-fill-secondary: #f5f4f1 !important;
    --panel-background-fill:          #ffffff !important;
    --block-background-fill:          #ffffff !important;
    --input-background-fill:          #ffffff !important;
    --input-background-fill-focus:    #ffffff !important;
    --chatbot-background-fill:        #ffffff !important;
    --border-color-primary:           #ece9e3 !important;
    --border-color-accent:            #cc785c !important;
    --color-accent:                   #cc785c !important;
    --color-accent-soft:              #fdf3ee !important;
    --body-text-color:                #1a1a1a !important;
    --body-text-color-subdued:        #6c6a65 !important;
    --block-label-text-color:         #2a2a2a !important;
    --block-title-text-color:         #181818 !important;
    --input-placeholder-color:        #a8a59f !important;
    --button-primary-background-fill: #1a1a1a !important;
    --button-primary-text-color:      #ffffff !important;
    --button-secondary-background-fill: #ffffff !important;
    --button-secondary-text-color:    #1a1a1a !important;
    --button-secondary-border-color:  #d9d6d0 !important;
    --table-even-background-fill:     #f7f5f1 !important;
    --table-odd-background-fill:      #ffffff !important;
    --code-background-fill:           #f7f5f1 !important;
    --shadow-drop:                    none !important;
    --shadow-drop-lg:                 none !important;
    --color-grey-100:                 #fbfaf8 !important;
    --color-grey-200:                 #f5f4f1 !important;
    --neutral-950:                    #1a1a1a !important;
    --neutral-900:                    #2a2a2a !important;
    --neutral-800:                    #404040 !important;
}

/* ── 全局字体 + 米白底 ─────────────────────────────────────────── */
.gradio-container, .gradio-container * {
    color-scheme: light !important;
}
.gradio-container {
    font-family: 'Inter', 'ui-sans-serif', -apple-system, BlinkMacSystemFont,
                 'Segoe UI', 'PingFang SC', 'Hiragino Sans GB',
                 'Microsoft YaHei', system-ui, sans-serif !important;
    background: #fbfaf8 !important;
    color: #1a1a1a !important;
    max-width: 1180px !important;
    margin: 0 auto !important;
    padding: 28px 36px !important;
    line-height: 1.6 !important;
}

/* ── 标题层级（致敬 docs.anthropic 文章页） ────────────────────── */
.gradio-container h1 {
    font-size: 30px !important;
    font-weight: 600 !important;
    letter-spacing: -0.02em !important;
    color: #181818 !important;
    border-bottom: none !important;
    margin: 6px 0 4px 0 !important;
}
.gradio-container h2 {
    font-size: 19px !important;
    font-weight: 600 !important;
    letter-spacing: -0.01em !important;
    color: #2a2a2a !important;
    margin: 22px 0 10px 0 !important;
}
.gradio-container h3 {
    font-size: 15px !important;
    font-weight: 600 !important;
    color: #404040 !important;
    margin-top: 18px !important;
}

/* ── 正文段落 ────────────────────────────────────────────────── */
.gradio-container p, .gradio-container li {
    color: #3a3a3a !important;
    font-size: 14.5px !important;
}

/* ── Tab 标签栏 ──────────────────────────────────────────────── */
.tab-nav button {
    background: transparent !important;
    border: none !important;
    border-bottom: 2px solid transparent !important;
    border-radius: 0 !important;
    padding: 12px 18px !important;
    font-weight: 500 !important;
    color: #6a6a6a !important;
    font-size: 14px !important;
    transition: all 0.15s ease !important;
}
.tab-nav button:hover {
    color: #cc785c !important;
}
.tab-nav button.selected {
    color: #cc785c !important;
    border-bottom: 2px solid #cc785c !important;
    background: transparent !important;
}

/* ── 按钮：克制 + 单一强调色 ─────────────────────────────────── */
button.primary, button[variant="primary"] {
    background: #1a1a1a !important;
    color: #ffffff !important;
    border: 1px solid #1a1a1a !important;
    border-radius: 8px !important;
    font-weight: 500 !important;
    box-shadow: none !important;
}
button.primary:hover {
    background: #2d2d2d !important;
    border-color: #2d2d2d !important;
}
button.secondary, button[variant="secondary"] {
    background: #ffffff !important;
    color: #1a1a1a !important;
    border: 1px solid #d9d6d0 !important;
    border-radius: 8px !important;
    font-weight: 500 !important;
    box-shadow: none !important;
}
button.secondary:hover {
    background: #f5f4f1 !important;
    border-color: #cc785c !important;
}

/* ── 输入框 / 文本域 ─────────────────────────────────────────── */
.gradio-container textarea,
.gradio-container input[type="text"],
.gradio-container input[type="number"] {
    background: #ffffff !important;
    border: 1px solid #e6e3dd !important;
    border-radius: 8px !important;
    font-family: 'JetBrains Mono', 'Menlo', 'Consolas', monospace !important;
    font-size: 13px !important;
}
.gradio-container textarea:focus,
.gradio-container input:focus {
    border-color: #cc785c !important;
    box-shadow: 0 0 0 3px rgba(204, 120, 92, 0.12) !important;
}

/* ── 容器卡片：薄边 + 圆角 ───────────────────────────────────── */
.block, .form, .panel {
    background: #ffffff !important;
    border: 1px solid #ece9e3 !important;
    border-radius: 12px !important;
    box-shadow: none !important;
}

/* ── 数据表 ─────────────────────────────────────────────────── */
.gradio-container table {
    border-collapse: collapse !important;
    font-size: 13.5px !important;
}
.gradio-container th {
    background: #f5f4f1 !important;
    font-weight: 600 !important;
    color: #2a2a2a !important;
    border-bottom: 2px solid #d9d6d0 !important;
}
.gradio-container td {
    border-bottom: 1px solid #ece9e3 !important;
    padding: 8px 12px !important;
}

/* ── 代码块 ─────────────────────────────────────────────────── */
.gradio-container pre, .gradio-container code {
    background: #f7f5f1 !important;
    border-radius: 6px !important;
    font-family: 'JetBrains Mono', 'Menlo', 'Consolas', monospace !important;
}

/* ── 分割线 ─────────────────────────────────────────────────── */
.gradio-container hr {
    border: none !important;
    border-top: 1px solid #ece9e3 !important;
    margin: 28px 0 !important;
}

/* ── 自定义流程导航卡片 ──────────────────────────────────────── */
.smdsl-flow-nav { font-family: inherit !important; }

/* ── Markdown 内 inline code ─────────────────────────────────── */
.gradio-container :not(pre) > code {
    background: #f5f0eb !important;
    color: #b25a3f !important;
    padding: 1px 6px !important;
    border-radius: 4px !important;
    font-size: 13px !important;
}

/* ── 强制 body / 页面背景为米白，消灭 dark 模式残留 ───────────── */
body, html { background: #f5f4f1 !important; }

/* ── Code 块亮色覆盖 ─────────────────────────────────────────── */
.codemirror-wrapper, .cm-editor, .cm-scroller,
.codemirror-wrapper *, .cm-editor * {
    background: #f7f5f1 !important;
    color: #1a1a1a !important;
}
.cm-gutters { background: #ece9e3 !important; color: #6c6a65 !important; }

/* ── Plot 背景（Plotly 图表） ──────────────────────────────────── */
.js-plotly-plot, .plotly, .plot-container {
    background: #ffffff !important;
}

/* ── Accordion / Tabs 背景 ─────────────────────────────────────── */
details, details > div, .tabitem, .tabs { background: transparent !important; }
"""


# ══════════════════════════════════════════════════════════════════════
# Build UI
# ══════════════════════════════════════════════════════════════════════

def _build_theme() -> "gr.themes.Soft":
    # 注：font 列表的首项必须是 gr.themes.Font 或 GoogleFont 对象；
    # 其余可为字符串 fallback。我们已经用 CSS !important 完全接管字体，
    # 这里给一个最小合规的主题对象，仅用于配色。
    return gr.themes.Soft(
        primary_hue=gr.themes.Color(
            c50="#fcf5f1", c100="#f9ebe4", c200="#f0d4c5", c300="#e6b89f",
            c400="#dc9d7a", c500="#cc785c", c600="#b85e44", c700="#94472f",
            c800="#6f3322", c900="#4a2117", c950="#2a120c",
        ),
        neutral_hue=gr.themes.Color(
            c50="#fbfaf8", c100="#f5f4f1", c200="#ece9e3", c300="#d9d6d0",
            c400="#a8a59f", c500="#6c6a65", c600="#494844", c700="#2f2e2c",
            c800="#1f1e1d", c900="#141413", c950="#0a0a0a",
        ),
        font=[
            gr.themes.GoogleFont("Inter"),
            "ui-sans-serif", "system-ui", "sans-serif",
        ],
        font_mono=[
            gr.themes.GoogleFont("JetBrains Mono"),
            "ui-monospace", "Menlo", "Consolas", "monospace",
        ],
    )


def build_ui() -> gr.Blocks:
    with gr.Blocks(title="SMDSL — 模块化调试面板") as demo:
        # gr.State 必须在 with gr.Blocks 内
        topology_state = gr.State(value=None)
        candidates_state = gr.State(value=[])
        click_pick_state = gr.State(
            value={"start_xy": None, "goal_xy": None, "mode": "start"},
        )
        shared_roboir_state = gr.State(value="")
        shared_traj_state = gr.State(value="")
        shared_demo3_out_state = gr.State(value="")

        gr.Markdown("# SMDSL (RoboIR) — 模块化调试面板")
        gr.Markdown(
            "**端到端流程**：自然语言 + CAD →（① 环境感知 + ② 语义编译）→ "
            "③ 物理求解 → 结构化反馈给 LLM。每个 Tab 之间通过状态自动贯通。"
        )

        # ══════════════════════════════════════════════════════════════
        # Tab 1
        # ══════════════════════════════════════════════════════════════
        with gr.Tab("Demo 1 — 环境感知 (CAD 拓扑)"):
            gr.HTML(_flow_nav_md(1))
            gr.Markdown("## 这一步在做什么？")
            gr.Markdown(
                "**输入**：CAD 平面图。**支持三种格式 → 不同语义能力**：\n\n"
                "| 格式 | 解析方式 | Demo 2 词汇能力 | Demo 3 路径规划 |\n"
                "|------|----------|----------------|------------------|\n"
                "| **`.json`** (FloorplanQA) | 矢量 + 完整语义 | ✅ 自动获得 doors/objects | ✅ |\n"
                "| **`.svg`** | 抽 line/polyline / path / rect | ⚠️ 无对象语义 (需手填) | ✅ |\n"
                "| **`.png/.jpg`** | 二值化为 occupancy | ⚠️ 无对象语义 (需手填) | ✅ |\n\n"
                "**处理**：栅格化 → 外部剔除 → 形态学破壁 → 距离场 (EDT) → "
                "全局矩阵化拓扑分类。\n\n"
                "**输出**：① 4 联可视化；② 候选起终点清单；③ 选 A→B 后求最短路径，"
                "**自动同步轨迹至 Tab 3**。"
            )
            # 📌 数据来源 & 零幻觉护栏（文档站风格 callout）
            gr.HTML(
                "<div style='background:#fbfaf8;border:1px solid #ece9e3;"
                "border-left:3px solid #cc785c;padding:14px 18px;"
                "border-radius:8px;margin:14px 0;color:#2a2a2a;"
                "font-size:13.5px;line-height:1.65'>"
                "<div style='font-weight:600;color:#94472f;margin-bottom:6px;"
                "font-size:13px;letter-spacing:.02em;text-transform:uppercase'>"
                "Note · 数据来源与算法声明</div>"
                "支持多格式输入：<b>.json</b>（FloorplanQA 矢量，完整语义）、"
                "<b>.svg</b>（线段几何，无对象语义）、<b>.png/.jpg</b>（二值化栅格，手绘图效果有限）。"
                "<br>JSON 样本来自 "
                "<a href='https://github.com/yale-nlp/FloorPlanQA' "
                "target='_blank' style='color:#cc785c;text-decoration:none;"
                "font-weight:500;border-bottom:1px solid #e6b89f'>"
                "FloorplanQA</a>（耶鲁 NLP，BSD-3 / 仅研究用）；"
                "PNG 迷宫来自 <b>MAPF 基准</b>；其余为自定义测试图。"
                "<br>空间解析管线采用 <b>100% 确定性算法</b>"
                "（EDT 距离场 + A* 最短路径 + 形态学 + 连通分量），"
                "<b>不含</b>深度学习视觉推理，几何层零幻觉保证。"
                "</div>"
            )
            gr.Markdown("---")
            gr.Markdown("## 1️⃣ 选择 / 上传 CAD 文件")

            # 6 个本地测试快速预设（路径+说明）
            _TEST_PRESETS = [
                {
                    "path": r"D:\code_backup_dwg\data\cad_samples\mapf_all_png\Paris_1_256.png",
                    "label": "① PNG 迷宫 Paris_1_256",
                    "hint": "mapf_all_png/Paris_1_256.png — 迷宫栅格图",
                    "is_dir": False,
                },
                {
                    "path": r"C:\Users\admin\AppData\Local\Temp\gradio\30a4521c8e4d8f5c4f95598a6a40791a46a3e56b40a8c3cbd4f2e69563202c41\room_48.json",
                    "label": "② JSON 房间 room_48",
                    "hint": "FloorplanQA JSON 矢量 — 完整语义",
                    "is_dir": False,
                },
                {
                    "path": r"C:\Users\admin\AppData\Local\Temp\gradio\bad2222842025f7089570da795492aa5fc3e96c0bb179eac202fdcae80233d03\bedroom_example.svg",
                    "label": "③ SVG 卧室",
                    "hint": "bedroom_example.svg — 矢量，无对象语义",
                    "is_dir": False,
                },
                {
                    "path": r"C:\Users\admin\AppData\Local\Temp\gradio\c9e3b858134c87e2ed9f39c9d63e2a11423e2b5fda953b47bec28e35ea496103\factory_layout_003.jpg",
                    "label": "④ JPG 工厂手绘（边界弱）",
                    "hint": "factory_layout_003.jpg — 手绘，识别效果有限",
                    "is_dir": False,
                },
                {
                    "path": r"C:\Users\admin\AppData\Local\Temp\gradio\0785c08215e4b01d2eb2a2dee0dc241b7af536f983af4618e2834217f0895ffb\factory_layout_002.jpg",
                    "label": "⑤ JPG 工厂手绘（边缘较清）",
                    "hint": "factory_layout_002.jpg — 内边缘较清晰",
                    "is_dir": False,
                },
                {
                    "path": r"C:\Users\admin\AppData\Local\Temp\gradio\69672a7ff67c55741f97224ec6a3bf41b4bc79c217e45d799674c4c7fcbca204\0006-0013.png",
                    "label": "⑥ PNG CAD 纯黑线图",
                    "hint": "0006-0013.png — 白底黑线，建议勾选「PNG 反色」",
                    "is_dir": False,
                },
            ]

            with gr.Tabs():
                with gr.TabItem("📂 浏览本地数据集"):
                    # Python glob 不支持 brace 扩展 {a,b,c}，
                    # 这里用 **/* 全列出，再用 ignore_glob 排除二进制/缓存目录。
                    d1_explorer = gr.FileExplorer(
                        glob="**/*",
                        ignore_glob="**/{__pycache__,_archive,*.npy,*.npz,*.zip,*.tar,*.gz,*.parquet,*.h5}/**",
                        root_dir=_DATA_ROOT,
                        file_count="single",
                        label=(f"根目录: {_DATA_ROOT}  "
                               f"(展开后可看到 .json / .dwg / .osm / .svg / .png 文件)"),
                        height=320,
                    )
                    d1_use_explorer_btn = gr.Button(
                        "✅ 使用所选文件",
                    )
                    gr.Markdown(
                        "*提示：FileExplorer 默认折叠子目录，先点 ▶ 展开 "
                        "`floorplanqa/layouts/<room_type>/` 即可看到 JSON 文件。"
                        "选中后 **必须**点上方 [✅ 使用所选文件] 才会写入路径。*"
                    )
                with gr.TabItem("📤 上传外部文件"):
                    d1_upload = gr.File(
                        label="支持 .json / .dwg / .osm / .svg / .png / .jpg / .jpeg",
                        file_count="single",
                        file_types=[".json", ".dwg", ".osm", ".svg", ".png",
                                    ".jpg", ".jpeg", ".bmp", ".tif", ".tiff"],
                    )
                with gr.TabItem("⚡ 快速预设（6 个测试用例）"):
                    gr.Markdown("点击对应按钮直接填入测试文件路径：")
                    # 生成 6 个预设按钮（3 行 × 2 列）
                    _preset_btns: list[gr.Button] = []
                    for i in range(0, len(_TEST_PRESETS), 2):
                        with gr.Row():
                            for p in _TEST_PRESETS[i:i+2]:
                                btn = gr.Button(p["label"], size="sm")
                                _preset_btns.append(btn)
                    d1_preset_btn = gr.Button(
                        f"🔄 加载默认预设：{Path(_DEFAULT_SAMPLE_PATH).name}",
                        size="sm",
                    )

            d1_path = gr.Textbox(
                label="当前 CAD 文件路径（可手动编辑）",
                placeholder="选 / 上传 / 预设按钮会自动填入",
                value="",
            )
            with gr.Row():
                d1_png_invert = gr.Checkbox(
                    label="PNG 反色（黑底白线时勾选）",
                    value=False,
                )
                d1_robot_radius = gr.Slider(
                    label="机器人半径 (m)",
                    info="默认 0.15m。越大约束越严格（0.25-0.5m 适合大型机器人）",
                    minimum=0.0, maximum=0.5, value=0.15, step=0.05,
                )

            d1_btn = gr.Button("⚙️ 解析 CAD（生成距离场 + 拓扑）",
                               variant="primary")
            d1_status = gr.Markdown("*等待选择 CAD 文件...*")

            gr.Markdown("### 📷 第 1 视图：CAD 原图（这是你的输入长什么样）")
            d1_source_image = gr.Image(
                label="原图回显：JSON→矢量重渲 / SVG→线段重渲 / PNG→原图",
                type="filepath", interactive=False, height=380,
            )
            gr.Markdown("### 📊 第 2 视图：算法过程（栅格 / 距离场 / A* 热力 / 拓扑）")
            d1_image = gr.Image(
                label="4 联图：(a) 栅格 / (b) 距离场 / (c) A* 热力 / (d) 拓扑标签",
                type="filepath", interactive=False, height=420,
            )
            with gr.Accordion("🌐 3D 拓扑白模预览", open=True):
                d1_3d_btn = gr.Button("🏗 生成 3D 白模预览", variant="primary", size="lg")
                d1_3d_plot = gr.Plot(label="3D Topology Preview", min_width=600)

            with gr.Accordion("🌆 3D 空间场景图", open=False):
                gr.Markdown(
                    "在浏览器内生成交互式 3D 空间场景图，无需下载任何文件。\n\n"
                    "- **OSM 文件**：精确多边形房间板块 + 拓扑通路连线 + Hover 语义（房间名/面积/高度）\n"
                    "- **其他格式**：距离场地板热力图 + 墙体 3D 线框\n\n"
                    "支持：**拖转旋转 · 滚轮缩放 · 双击复位 · 悬停查看房间信息**"
                )
                d1_scene3d_btn = gr.Button(
                    "🏙 生成 3D 空间场景图",
                    variant="primary", size="lg",
                )
                d1_scene3d_status = gr.Markdown("*解析 CAD 后点击上方按钮。*")
                d1_scene3d_plot = gr.Plot(
                    label="3D Space Graph",
                    min_width=700,
                )

            d1_out = gr.Code(
                label="解析摘要 JSON", language="json", lines=10,
            )

            with gr.Accordion("🤖 DWG 图纸语义信息（仅 .dwg 格式有输出）", open=False):
                d1_semantics_json = gr.Code(
                    label="语义数据 JSON（图层 / 文本 / 块参照）",
                    language="json", lines=8, interactive=False,
                )
                d1_semantics_md = gr.Markdown(
                    "*上传 .dwg 文件并解析后，此处将展示提取到的图层、文本标注、块参照等语义数据。*"
                )

            # Spatial-RAG 画像
            d1_profile_btn = gr.Button(
                "🧠 生成环境语义画像 (Spatial-RAG)",
                variant="secondary", size="sm",
            )
            # 隐藏 state：桥接 Tab 1 画像 → Tab 2 全局约束
            scene_profile_state = gr.State({})

            with gr.Accordion("🧠 LLM 环境语义画像 (Spatial-RAG)", open=False):
                d1_env_md = gr.Markdown(
                    "*解析 CAD 后点击上方按钮，LLM 将分析环境生成语义画像，"
                    "并自动同步推荐约束到 Tab 2。*"
                )
                d1_show_json_btn = gr.Button(
                    "📄 展开查看原始语义 JSON", size="sm", variant="secondary",
                )
                d1_env_json = gr.Code(
                    label="语义画像 JSON（完整数据）",
                    language="json", lines=10, interactive=False,
                    visible=False,
                )

            gr.Markdown("---")
            gr.Markdown(
                "## 2️⃣ 选起点 A 与终点 B 🖱\n"
                "### 在下面这张『拓扑标签图』上直接**点两次**：第 1 次 = 起点 A（蓝点），"
                "第 2 次 = 终点 B（红点）— 点完自动求 A* 最短路径，并同步到 Tab 3。"
            )
            gr.Markdown(
                "**说明** ：\n"
                "- 图上**灰色小圆点带 `#1` `#2` 标号**是系统挑出来的『推荐候选位置』"
                "（屋内最舒服、远离墙）——你不必非要点在这些点上，**点哪都行**\n"
                "- 如果你点的位置在**墙上 / 膨胀禁入区（橙色）**，会自动 snap 到最近的安全像素\n"
                "- 想精确选某扇门或某个家具旁的位置 → 展开下方 [🔽 备选下拉清单]\n"
                "- 想换位置：直接点第 3 次自动重选起点；或点 [🔄 清空选点]"
            )
            d1_click_status = gr.Markdown("*等待解析 CAD...*")
            d1_clickable = gr.Image(
                label="🗺 拓扑图（点击选起终点） — 绿=可走 / 橙=膨胀禁入 / 黑=墙",
                type="filepath", interactive=False, height=520,
            )
            with gr.Row():
                d1_reset_btn = gr.Button("🔄 清空选点", size="sm")
                d1_total_time = gr.Slider(
                    label="轨迹总时长 T (秒)",
                    info="A* 给出的是几何路径；这里把它均匀拉伸到 T 秒生成时间戳，"
                         "供 Demo 3 检查『Time < X』之类的时序约束",
                    minimum=1.0, maximum=20.0, value=5.0, step=0.5,
                )

            with gr.Accordion("🔽 备选：用下拉清单精确选点（doors / 物体邻域 / 安全中心）",
                              open=False):
                with gr.Row():
                    d1_start = gr.Dropdown(
                        label="起点 A", choices=[], value=None,
                        interactive=True,
                    )
                    d1_goal = gr.Dropdown(
                        label="终点 B", choices=[], value=None,
                        interactive=True,
                    )
                d1_plan_btn = gr.Button(
                    "▶ 用下拉值求 A → B 最短路径", variant="secondary",
                )

            d1_plan_status = gr.Markdown("*等待求解...*")
            d1_path_image = gr.Plot(
                label="A* Path Overlay (cyan line, green=start, yellow=goal)",
            )
            d1_path_info = gr.Code(
                label="路径信息", language="json", lines=6,
            )

            # ── 文件源切换 ─────────────────────────────────
            d1_use_explorer_btn.click(
                fn=lambda p: (p or "", "📂 已选择浏览文件"),
                inputs=d1_explorer, outputs=[d1_path, d1_status],
            )
            d1_upload.change(
                fn=lambda f: (
                    (f if isinstance(f, str) else (f.name if hasattr(f, 'name') else "")) if f is not None else "",
                    "📤 已上传外部文件",
                ),
                inputs=d1_upload, outputs=[d1_path, d1_status],
            )
            d1_preset_btn.click(
                fn=lambda: (_DEFAULT_SAMPLE_PATH, "⚡ 预设已加载"),
                inputs=None, outputs=[d1_path, d1_status],
            )
            # 6 个快速预设按钮
            for _preset_btn, _preset in zip(_preset_btns, _TEST_PRESETS):
                _p_path = _preset["path"]
                _p_hint = _preset["hint"]
                _preset_btn.click(
                    fn=lambda p=_p_path, h=_p_hint: (p, f"⚡ 预设: {h}"),
                    inputs=None, outputs=[d1_path, d1_status],
                )

            # 语义 JSON 展开/折叠 toggle
            d1_json_vis_state = gr.State(value=False)

            def _toggle_json_vis(is_visible: bool):
                new_vis = not is_visible
                lbl = "🙈 隐藏语义 JSON" if new_vis else "📄 展开查看原始语义 JSON"
                return gr.update(visible=new_vis), gr.update(value=lbl), new_vis

            d1_show_json_btn.click(
                fn=_toggle_json_vis,
                inputs=d1_json_vis_state,
                outputs=[d1_env_json, d1_show_json_btn, d1_json_vis_state],
            )

            # ── 解析 ─────────────────────────────────────
            d1_btn.click(
                fn=lambda: "⏳ 正在解析（dispatcher → 距离场 → A* 漫水 → 渲染）...",
                inputs=None, outputs=d1_status,
            ).then(
                fn=demo1_run,
                inputs=[d1_path, d1_png_invert, d1_robot_radius],
                outputs=[d1_out, d1_source_image, d1_image, d1_clickable,
                         d1_status, d1_start, d1_goal,
                         topology_state, candidates_state, click_pick_state,
                         d1_semantics_json, d1_semantics_md],
            ).then(
                fn=lambda: "✅ 解析完成 · 点击地图选起终点 / 或展开备选下拉",
                inputs=None, outputs=d1_click_status,
            )

            # 3D 拓扑预览按钮
            d1_3d_btn.click(
                fn=demo1_3d_preview,
                inputs=[topology_state],
                outputs=[d1_3d_plot, d1_status],
            )

            # 3D 空间场景图
            d1_scene3d_btn.click(
                fn=lambda: (None, "⏳ 正在渲染 3D 空间场景图..."),
                inputs=None,
                outputs=[d1_scene3d_plot, d1_scene3d_status],
            ).then(
                fn=demo1_scene_graph_3d,
                inputs=[topology_state],
                outputs=[d1_scene3d_plot, d1_scene3d_status],
            )

            # ── 点击图选 A/B ──────────────────────────────
            d1_clickable.select(
                fn=demo1_image_click,
                inputs=[topology_state, click_pick_state, candidates_state],
                outputs=[d1_clickable, d1_clickable,
                         d1_click_status, click_pick_state],
            ).then(
                fn=demo1_plan_path_from_click,
                inputs=[click_pick_state, topology_state, d1_total_time],
                outputs=[d1_path_info, d1_path_image,
                         d1_plan_status, shared_traj_state],
            )

            d1_reset_btn.click(
                fn=demo1_reset_picks,
                inputs=[topology_state, candidates_state],
                outputs=[d1_clickable, d1_click_status, click_pick_state],
            )

            # ── 备选：下拉求路径 ────────────────────────────
            d1_plan_btn.click(
                fn=lambda: "⏳ 正在求 A* 最短路径...",
                inputs=None, outputs=d1_plan_status,
            ).then(
                fn=demo1_plan_path,
                inputs=[d1_start, d1_goal, topology_state,
                        candidates_state, d1_total_time],
                outputs=[d1_path_info, d1_path_image,
                         d1_plan_status, shared_traj_state],
            )

        # ══════════════════════════════════════════════════════════════
        # Tab 2
        # ══════════════════════════════════════════════════════════════
        with gr.Tab("Demo 2 — 语义编译 (VLM → RoboIR)"):
            gr.HTML(_flow_nav_md(2))
            gr.Markdown("## 这一步在做什么？")
            gr.Markdown(
                "**输入**：自然语言指令 + 局部环境（场景实体白名单）。\n\n"
                "**处理**：DeepSeek 翻译为 **声明式 STL 约束**（不是动作脚本！）：\n"
                "- ❌ 错误：『先到 A 再夹取』（命令式）\n"
                "- ✅ 正确：『∃t* ∈ [0, T]: end_effector at A, ∀t: dist > 0.10』（声明式）\n\n"
                "**输出**：RoboIR JSON + 引用一致性校验 + 可选 STL 风格中文摘要。\n\n"
                "**编译完成自动同步至 Tab 3**，无需手动按钮。"
            )
            gr.Markdown("---")
            gr.Markdown(_api_key_status_md())
            with gr.Row():
                d2_api_key = gr.Textbox(
                    label="DeepSeek API Key",
                    placeholder="sk-...（留空则使用环境变量）",
                    type="password",
                    value=os.environ.get("DEEPSEEK_KEY", "")
                         or os.environ.get("DEEPSEEK_API_KEY", ""),
                    scale=4,
                )
                d2_test_key_btn = gr.Button("Test Key", size="sm", scale=1)
                d2_key_status = gr.Markdown("")
            gr.Markdown("## 操作步骤")
            gr.Markdown(
                "**A. 用预设示例**：点 ① / ② / ③ 一键填入 → 点 [🤖 编译]\n\n"
                "**B. 完全自由编辑**：写指令 → 点 [🧠 智能推断局部环境] → 点 [🤖 编译] →"
                "（可选）点 [📖 STL 摘要] 检查 LLM 理解\n\n"
                "*校验结果会显示在 RoboIR 输出下方。引用了未声明物体（如指令说"
                "『搬纸巾』但 nearest_objects 里没有 tissue）会黄色警告。*"
            )

            gr.Markdown("**📋 一键填入预设示例（固定）：**")
            with gr.Row():
                ex_btns = [
                    gr.Button(ex["label"], size="sm")
                    for ex in _DEMO2_EXAMPLES
                ]

            gr.Markdown("**🪄 LLM 情境示例（基于当前场景画像自动生成）：**")
            d2_gen_ex_btn = gr.Button(
                "🪄 根据当前地图生成情境示例（需先完成 Tab 1 解析 + 语义画像）",
                variant="secondary", size="sm",
            )
            # 存储生成的示例列表（JSON str）
            _gen_ex_state = gr.State(value="[]")
            # 展示生成结果的 Markdown
            d2_gen_ex_display = gr.Markdown(
                "*点击上方按钮后，生成的示例将显示在这里...*"
            )
            with gr.Row():
                gen_fill_btns = [
                    gr.Button(f"📋 填入示例 {i+1}", size="sm", interactive=False)
                    for i in range(3)
                ]

            d2_instr = gr.Textbox(
                label="① 自然语言指令（自由编辑）",
                lines=3, value=DEFAULT_INSTRUCTION,
            )
            d2_infer_btn = gr.Button(
                "🧠 智能推断局部环境（让 LLM 抽实体）", size="sm",
            )
            d2_ctx = gr.Code(
                label="② 局部环境（含 nearest_objects 的 JSON）",
                language="json", value=DEFAULT_LOCAL_CTX, lines=6,
            )

            with gr.Row():
                d2_btn = gr.Button("🤖 调用 DeepSeek 编译", variant="primary")
                d2_summary_btn = gr.Button("📖 翻译为 STL 风格中文摘要",
                                           variant="secondary")

            d2_status = gr.Markdown("*等待操作...*")
            d2_out = gr.Code(
                label="③ RoboIR (JSON) — 机器人执行依据",
                language="json", lines=18,
            )
            d2_validation = gr.Markdown(
                "*引用一致性校验结果会出现在这里...*"
            )

            # 护栏 callout — 中文摘要"仅供人阅读"（文档站风格 warning）
            gr.HTML(
                "<div style='background:#fbfaf8;border:1px solid #ece9e3;"
                "border-left:3px solid #d9534f;padding:14px 18px;"
                "border-radius:8px;margin:14px 0 6px 0;color:#2a2a2a;"
                "font-size:13.5px;line-height:1.65'>"
                "<div style='font-weight:600;color:#a83227;margin-bottom:6px;"
                "font-size:13px;letter-spacing:.02em;text-transform:uppercase'>"
                "Warning · 护栏提示</div>"
                "下方中文摘要<b>仅供人类阅读参考</b>，<u>不影响实际执行</u>。"
                "机器人底层的绝对执行依据是 <b>上方的 RoboIR JSON 强类型约束代码</b>。"
                "若摘要与 JSON 不一致，<b>以 JSON 为准</b>。"
                "</div>"
            )
            d2_summary_text = gr.Markdown(
                "*点 [📖 翻译为 STL 风格中文摘要] 后这里会出现声明式 STL 描述...*"
            )

            for btn, ex in zip(ex_btns, _DEMO2_EXAMPLES):
                _i = ex["instruction"]
                _c = ex["context"]
                btn.click(
                    fn=lambda i=_i, c=_c: (
                        i, c, "✅ 已填入示例，可点 [🤖 编译]",
                    ),
                    inputs=None,
                    outputs=[d2_instr, d2_ctx, d2_status],
                )

            # 🪄 LLM 情境示例生成
            d2_gen_ex_btn.click(
                fn=lambda: "⏳ 正在调用 LLM 生成情境示例（约 5~10 秒）...",
                inputs=None, outputs=d2_status,
            ).then(
                fn=demo2_generate_examples,
                inputs=[scene_profile_state, topology_state, d2_api_key],
                outputs=[d2_gen_ex_display, _gen_ex_state, d2_status],
            ).then(
                fn=lambda _: (
                    gr.update(interactive=True),
                    gr.update(interactive=True),
                    gr.update(interactive=True),
                ),
                inputs=[_gen_ex_state],
                outputs=gen_fill_btns,
            )

            # 📋 填入示例 N 按钮 → 从 _gen_ex_state 中取对应示例
            def _make_fill_fn(idx: int):
                def _fill(examples_json: str):
                    try:
                        exs = json.loads(examples_json)
                        if idx < len(exs):
                            ex = exs[idx]
                            return (
                                ex.get("instruction", ""),
                                ex.get("context", ""),
                                f"✅ 已填入示例 {idx+1}：{ex.get('label', '')}",
                            )
                    except Exception:
                        pass
                    return gr.update(), gr.update(), "⚠️ 示例数据无效，请重新生成"
                return _fill

            for _k in range(3):
                gen_fill_btns[_k].click(
                    fn=_make_fill_fn(_k),
                    inputs=[_gen_ex_state],
                    outputs=[d2_instr, d2_ctx, d2_status],
                )

            d2_test_key_btn.click(
                fn=demo2_test_api_key,
                inputs=[d2_api_key],
                outputs=[d2_key_status],
            )

            d2_infer_btn.click(
                fn=lambda: "⏳ 正在让 LLM 抽取实体...",
                inputs=None, outputs=d2_status,
            ).then(
                fn=demo2_infer_context,
                inputs=[d2_instr, d2_api_key],
                outputs=[d2_ctx, d2_status],
            )

            d2_btn.click(
                fn=lambda: "⏳ 正在调用 DeepSeek API（约 5~15 秒）...",
                inputs=None, outputs=d2_status,
            ).then(
                fn=demo2_run,
                inputs=[d2_instr, d2_ctx, scene_profile_state, d2_api_key],
                outputs=[d2_out, d2_validation, d2_status],
            ).then(
                fn=lambda text: text,
                inputs=d2_out, outputs=shared_roboir_state,
            )

            # Spatial-RAG 画像按钮（在 d2_api_key 定义之后绑定）
            d1_profile_btn.click(
                fn=lambda: "Generating scene profile via LLM...",
                inputs=None, outputs=d1_status,
            ).then(
                fn=demo1_profile,
                inputs=[topology_state, d2_api_key],
                outputs=[d1_env_json, d1_env_md, scene_profile_state],
            ).then(
                fn=lambda: "Scene profile ready. Constraints synced to Tab 2.",
                inputs=None, outputs=d1_status,
            )

            d2_summary_btn.click(
                fn=lambda: "⏳ 正在生成 STL 摘要...",
                inputs=None, outputs=d2_status,
            ).then(
                fn=demo2_summarize,
                inputs=[d2_out, d2_api_key],
                outputs=[d2_summary_text, d2_status],
            )

        # ══════════════════════════════════════════════════════════════
        # Tab 3
        # ══════════════════════════════════════════════════════════════
        with gr.Tab("Demo 3 — 物理求解与反馈 (STL 验证)"):
            gr.HTML(_flow_nav_md(3))
            gr.Markdown("## 这一步在做什么？")
            gr.Markdown(
                "**输入**：① RoboIR (Tab 2 自动同步)；② 模拟轨迹 (Tab 1 自动同步 / 手动)。\n\n"
                "**处理**：调出 Tab 1 算好的距离场，逐帧检查轨迹是否满足 RoboIR 约束。"
                "对 `Distance > D` 求 ρ = clearance − D，对 `Time < T` 检查总时长。\n\n"
                "**输出**：📊 轨迹叠加图  📈 ρ(t) 曲线  📋 Pose 表  📝 结构化反馈。"
            )
            gr.Markdown("---")

            with gr.Row():
                d3_load_roboir_btn = gr.Button("📥 从 Tab 2 加载 RoboIR")
                d3_load_traj_btn = gr.Button("📥 从 Tab 1 加载轨迹")
                d3_state_indicator = gr.Markdown(
                    "*待 Tab 1 / Tab 2 推送数据...*"
                )

            with gr.Row():
                d3_roboir = gr.Code(
                    label="RoboIR (来自 Tab 2 / 手编)",
                    language="json", value=DEFAULT_DEMO3_ROBOIR,
                    lines=16, max_lines=16,
                )
                d3_traj = gr.Code(
                    label="模拟轨迹 List[{t,x,y,z,roll,pitch,yaw}]"
                          " (来自 Tab 1 / 手编)",
                    language="json", value=DEFAULT_TRAJECTORY_TEXT,
                    lines=16, max_lines=16,
                )

            d3_use_field = gr.Checkbox(
                label="使用距离场（推荐勾选；勾掉只验时间约束）",
                value=True,
            )
            d3_btn = gr.Button("🔬 执行 STL 物理求解", variant="primary")
            d3_status = gr.Markdown("*等待求解...*")

            # Pose 摘要表放在执行按钮正下方（最先看到结果）
            d3_pose_table = gr.Dataframe(
                headers=["量", "起点 (实际)", "终点 (RoboIR 声明)",
                         "终点 (实际轨迹)"],
                datatype=["str", "str", "str", "str"],
                label="📋 Pose 摘要：起点 / RoboIR 声明的目标 / 实际终点",
                value=[],
                wrap=True,
            )

            # 3D 轨迹图独占一行（全宽，不被压缩）
            d3_traj_image = gr.Plot(
                label="📊 3D Trajectory Sandbox",
            )

            # ρ(t) 曲线独占一行
            d3_rho_image = gr.Plot(
                label="📈 rho(t) Robustness Curve",
            )

            d3_out = gr.Code(
                label="📝 结构化反馈：FailureTaxonomy + ρ + worst_node",
                language="json", lines=20,
            )

            gr.Markdown("---")
            gr.Markdown(
                "## 🩺 用 DeepSeek 写一份可读的诊断报告\n"
                "*结构化 JSON 适合机器/LLM 自纠错，但人不好读。"
                "下面这个按钮会把上面的 RoboIR + 轨迹 + 反馈喂给 DeepSeek，"
                "让它输出 **4 节中文诊断报告**（任务理解 / 执行结果 / 失败原因 / 改进建议）。*"
            )
            d3_diag_btn = gr.Button(
                "🩺 让 DeepSeek 写诊断报告", variant="secondary",
            )
            d3_diag_status = gr.Markdown("*等待求解完成后点此按钮...*")
            d3_diag_report = gr.Markdown(
                "*诊断报告将出现在这里...*",
            )

            def _pull_roboir(text):
                if text and text.strip():
                    return (text,
                            f"✅ 已从 Tab 2 加载 RoboIR @ {_ts()}（共 "
                            f"{len(text)} 字符）")
                return (gr.update(),
                        f"⚠️ Tab 2 还没编译过 — 请先去 Tab 2 点 [🤖 编译]，"
                        f"或继续用当前已有的 RoboIR @ {_ts()}")

            def _pull_traj(text):
                if text and text.strip():
                    return (text,
                            f"✅ 已从 Tab 1 加载轨迹 @ {_ts()}（共 "
                            f"{len(text)} 字符）")
                return (gr.update(),
                        f"⚠️ Tab 1 还没求过路径 — 请先去 Tab 1 点 "
                        f"[▶ 求 A → B 最短路径]，或继续用当前已有轨迹 @ {_ts()}")

            d3_load_roboir_btn.click(
                fn=_pull_roboir,
                inputs=shared_roboir_state,
                outputs=[d3_roboir, d3_state_indicator],
            )
            d3_load_traj_btn.click(
                fn=_pull_traj,
                inputs=shared_traj_state,
                outputs=[d3_traj, d3_state_indicator],
            )

            d3_btn.click(
                fn=lambda: "⏳ 正在求解 + 渲染...",
                inputs=None, outputs=d3_status,
            ).then(
                fn=demo3_run,
                inputs=[d3_roboir, d3_traj, d3_use_field, topology_state],
                outputs=[d3_out, d3_traj_image, d3_rho_image,
                         d3_pose_table, d3_status],
            ).then(
                fn=lambda text: text,
                inputs=d3_out, outputs=shared_demo3_out_state,
            )

            d3_diag_btn.click(
                fn=lambda: "⏳ 正在让 DeepSeek 撰写诊断报告...",
                inputs=None, outputs=d3_diag_status,
            ).then(
                fn=demo3_diagnostic_report,
                inputs=[d3_roboir, d3_traj, shared_demo3_out_state, d2_api_key],
                outputs=[d3_diag_report, d3_diag_status],
            )

    return demo


def main() -> None:
    demo = build_ui()
    demo.launch(theme=_build_theme(), css=_CLAUDE_DOCS_CSS)


if __name__ == "__main__":
    main()
