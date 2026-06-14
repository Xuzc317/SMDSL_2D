"""
visualize.py — Demo 1 拓扑解析结果可视化

输出 2x2 四联图：
  (a) 二值栅格（墙体黑 / 自由白 / 门洞淡蓝）
  (b) 距离场（连续 heatmap）
  (c) A* 漫水访问热力（log 压缩）
  (d) 拓扑标签（CLASS_PATH 绿 / CLASS_OBSTACLE 黑 / CLASS_INFLATED 橙）
"""

from __future__ import annotations

from pathlib import Path
from typing import Any, Dict, List, Optional

import numpy as np

# matplotlib 在无 GUI 时需要 Agg 后端
import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.colors import ListedColormap

# 中文字体 fallback — Windows 一般装了 Microsoft YaHei / SimHei，
# macOS 用 PingFang，其它平台尽量退到 sans-serif，避免 glyph 缺失警告。
plt.rcParams["font.sans-serif"] = [
    "Microsoft YaHei", "SimHei", "PingFang SC", "Noto Sans CJK SC",
    "Arial Unicode MS", "DejaVu Sans",
]
plt.rcParams["axes.unicode_minus"] = False

try:
    from cad_parser.astar_topology import (
        CLASS_INFLATED,
        CLASS_LOADING,
        CLASS_OBSTACLE,
        CLASS_PATH,
        CLASS_UNKNOWN,
    )
except ImportError:
    from astar_topology import (  # type: ignore
        CLASS_INFLATED,
        CLASS_LOADING,
        CLASS_OBSTACLE,
        CLASS_PATH,
        CLASS_UNKNOWN,
    )


# 拓扑标签调色板（5 类）— 高对比版
_TOPO_PALETTE = [
    "#b8cfe8",  # 0 unknown  (淡钢蓝，而非透明灰)
    "#d62728",  # 1 loading  (红色 — 上料)
    "#4cb94c",  # 2 path     (鲜绿，加深可见度)
    "#222222",  # 3 obstacle (近黑)
    "#ff8c1a",  # 4 inflated (深橙，高对比)
]


def render_pipeline_quad(
    pipeline_result: Dict[str, Any],
    output_path: str,
    dpi: int = 140,
    title_prefix: str = "Demo 1",
) -> None:
    """
    将 ``astar_topology.run_pipeline`` 的全部中间产物渲染为四联图。

    Args:
        pipeline_result: ``run_pipeline`` 的返回字典。
        output_path: PNG 输出路径。
        dpi: 输出分辨率。
        title_prefix: 总标题前缀。
    """
    grid: np.ndarray = pipeline_result["grid"]
    distance_field: np.ndarray = pipeline_result["distance_field"]
    topology: np.ndarray = pipeline_result["topology"]
    visit_count: np.ndarray = pipeline_result["visit_count"]
    seeds = pipeline_result.get("seeds", [])
    cad_data = pipeline_result.get("cad_data", {})
    transform = pipeline_result.get("transform", {})
    robot_radius_px = pipeline_result.get("robot_radius_px", 0.0)

    room_type = cad_data.get("room_type", "?")
    layout_id = cad_data.get("layout_id", "?")
    resolution = transform.get("resolution", 1.0)

    H, W = grid.shape
    aspect = max(0.5, min(2.0, W / max(1, H)))
    fig, axes = plt.subplots(2, 2, figsize=(10 * aspect, 9), dpi=dpi)
    fig.patch.set_facecolor("white")
    (ax_grid, ax_dist), (ax_visit, ax_topo) = axes
    for _ax in axes.flat:
        _ax.set_facecolor("#f4f6f8")  # 未覆盖像素显示为淡蓝灰，而非白色

    # (a) 二值栅格 — 自由空间白，墙体深灰，门洞淡蓝叠加
    grid_display = np.where(grid == 1, 0.96, 0.18)  # free=近白, wall=深灰
    grid_rgb = np.stack([grid_display] * 3, axis=-1).astype(np.float32)
    ax_grid.imshow(grid_rgb, origin="upper", vmin=0, vmax=1)
    ax_grid.set_title("(a) Rasterized grid (walls=0, free=1)")
    _draw_doors(ax_grid, cad_data, transform, color="#1f77b4", alpha=0.45)
    _draw_seeds(ax_grid, seeds, robot_radius_px)
    ax_grid.set_xlabel(f"resolution = {resolution:.3f} m/px")
    ax_grid.axis("off")

    # (b) 距离场
    im = ax_dist.imshow(distance_field, cmap="viridis", origin="upper")
    ax_dist.set_title(
        f"(b) Distance field (max={distance_field.max():.1f} px)"
    )
    plt.colorbar(im, ax=ax_dist, fraction=0.046, pad=0.04, label="px to wall")
    # 标出 R_robot 等高线
    if robot_radius_px > 0:
        ax_dist.contour(
            distance_field, levels=[robot_radius_px],
            colors="red", linewidths=1.0, linestyles="--",
        )
    ax_dist.axis("off")

    # (c) 可走区域热力图 — visit_count 是 PATH 二值 mask 时用 YlGn，
    #     原始 BFS 计数时用 magma（两种场景自动适配）
    vc = visit_count.astype(np.float64)
    is_binary = (vc.max() <= 1.0)
    if is_binary:
        # 二值 PATH mask → 绿色高亮可走区
        cmap_c = "YlGn"
        heat = vc
        vmax_c = 1.0
        cb_label = "可走区域 (1=PATH)"
    else:
        heat = np.sqrt(vc)
        vmax_c = np.percentile(heat[heat > 0], 99) if heat.max() > 0 else 1.0
        cmap_c = "magma"
        cb_label = "√visits (99 分位)"
    im2 = ax_visit.imshow(
        heat, cmap=cmap_c, origin="upper", vmin=0, vmax=max(0.01, vmax_c),
    )
    n_reached = int((visit_count > 0).sum())
    n_free = int((grid == 1).sum())
    ax_visit.set_title(
        f"(c) 路径可达区  PATH={n_reached}/{n_free} "
        f"({n_reached / max(1, n_free):.1%})"
    )
    plt.colorbar(im2, ax=ax_visit, fraction=0.046, pad=0.04, label=cb_label)
    _draw_seeds(ax_visit, seeds, robot_radius_px)
    ax_visit.axis("off")

    # (d) 拓扑标签
    cmap = ListedColormap(_TOPO_PALETTE)
    ax_topo.imshow(topology, cmap=cmap, vmin=0, vmax=4, origin="upper")
    legend_patches = [
        plt.Rectangle((0, 0), 1, 1, color=_TOPO_PALETTE[CLASS_PATH],
                      label="path"),
        plt.Rectangle((0, 0), 1, 1, color=_TOPO_PALETTE[CLASS_OBSTACLE],
                      label="obstacle"),
        plt.Rectangle((0, 0), 1, 1, color=_TOPO_PALETTE[CLASS_INFLATED],
                      label="inflated (R<R_robot)"),
        plt.Rectangle((0, 0), 1, 1, color=_TOPO_PALETTE[CLASS_UNKNOWN],
                      label="unknown"),
    ]
    if (topology == CLASS_LOADING).any():
        legend_patches.insert(
            0,
            plt.Rectangle((0, 0), 1, 1, color=_TOPO_PALETTE[CLASS_LOADING],
                          label="loading"),
        )
    ax_topo.legend(handles=legend_patches, loc="lower right", fontsize=8,
                   framealpha=0.85)
    n_path = int((topology == CLASS_PATH).sum())
    n_inflated = int((topology == CLASS_INFLATED).sum())
    n_free_d = int((grid == 1).sum())
    ax_topo.set_title(
        f"(d) 拓扑标签  路径={n_path / max(1, n_free_d):.1%} / "
        f"膨胀禁入={n_inflated / max(1, n_free_d):.1%}  "
        f"(机器人 R={robot_radius_px * resolution:.2f}m)"
    )
    _draw_seeds(ax_topo, seeds, robot_radius_px)
    ax_topo.axis("off")

    fig.suptitle(
        f"{title_prefix} — room_type={room_type}, layout_id={layout_id}, "
        f"grid={H}×{W}, R_robot={robot_radius_px:.1f}px",
        fontsize=12,
    )
    fig.tight_layout(rect=(0, 0, 1, 0.96))
    out = Path(output_path)
    out.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(out, dpi=dpi, bbox_inches="tight")
    plt.close(fig)


# ── 工具：画门洞 / 种子 ──────────────────────────────

def _draw_doors(ax, cad_data, transform, color="#1f77b4", alpha=0.4) -> None:
    if not cad_data or not transform:
        return
    origin = transform.get("origin", (0.0, 0.0))
    resolution = transform.get("resolution", 1.0)
    for door in cad_data.get("doors", []):
        pts = door.get("points", [])
        if len(pts) < 3:
            continue
        cols = [(p[0] - origin[0]) / resolution for p in pts]
        rows = [(p[1] - origin[1]) / resolution for p in pts]
        ax.fill(cols, rows, color=color, alpha=alpha, zorder=2)


def _draw_seeds(ax, seeds, robot_radius_px: float = 0.0) -> None:
    for r, c in seeds:
        ax.plot(c, r, "x", color="#e377c2", markersize=10,
                markeredgewidth=2, zorder=4)
        if robot_radius_px > 0:
            circ = plt.Circle(
                (c, r), robot_radius_px,
                fill=False, color="#e377c2", linewidth=1, linestyle=":",
                zorder=4,
            )
            ax.add_patch(circ)


# ══════════════════════════════════════════════════════════════════════
# 原图回显：JSON 矢量 → 渲染 / SVG 矢量 → 渲染 / PNG → 直接 copy
# ══════════════════════════════════════════════════════════════════════

def _pt_xy_compat(p):
    """同时兼容 dict {x,y} 与 tuple (x,y) 两种 schema。"""
    if isinstance(p, dict):
        try:
            return float(p["x"]), float(p["y"])
        except (KeyError, TypeError, ValueError):
            return None
    try:
        return float(p[0]), float(p[1])
    except (IndexError, TypeError, ValueError):
        return None


def render_source_cad(
    parse_result: Dict[str, Any],
    output_path: str,
    dpi: int = 140,
) -> str:
    """
    把 CAD 输入"还原"成一张可读的原图，供用户与下游 4 联图对照。

    - JSON: 画 boundary + walls + objects (含 label) + doors + windows
    - SVG : 画抽取到的所有 line/polyline/polygon
    - PNG : 直接 PIL 复制原图

    Returns:
        实际写出的 PNG 路径（与 output_path 一致）。
    """
    mode = parse_result.get("mode", "json")
    out = Path(output_path)
    out.parent.mkdir(parents=True, exist_ok=True)

    if mode == "png":
        # PNG 直接回显原图
        from PIL import Image  # noqa: PLC0415
        Image.open(parse_result["source_path"]).save(out)
        return str(out)

    if mode in ("svg", "dwg"):
        # SVG / DWG：显示二值化后的栅格，让用户看到"算法看到的样子"
        grid = parse_result.get("grid")
        if grid is not None:
            transform = parse_result.get("transform", {})
            res = transform.get("resolution", 1.0)
            origin = transform.get("origin", (0.0, 0.0))
            H, W = grid.shape
            fig_s, ax_s = plt.subplots(
                figsize=(max(5, W * res * 1.2), max(4, H * res * 1.2)), dpi=dpi
            )
            fig_s.patch.set_facecolor("white")
            ax_s.set_facecolor("#f4f6f8")
            disp = np.where(grid == 1, 0.95, 0.18).astype(np.float32)
            ax_s.imshow(
                np.stack([disp] * 3, axis=-1),
                origin="upper",
                extent=(origin[0], origin[0] + W * res,
                        origin[1] + H * res, origin[1]),
            )
            ax_s.set_title(
                f"CAD 原图（{mode.upper()} → 栅格）  "
                f"{H}×{W} px @ {res:.3f} m/px",
                fontsize=10,
            )
            ax_s.set_xlabel("x (m)")
            ax_s.set_ylabel("y (m)")
            ax_s.set_aspect("equal")
            ax_s.grid(True, alpha=0.2)
            fig_s.tight_layout()
            fig_s.savefig(out, dpi=dpi, bbox_inches="tight")
            plt.close(fig_s)
            return str(out)

    cad_data = parse_result.get("cad_data") or {}
    fig, ax = plt.subplots(figsize=(8, 7), dpi=dpi)
    fig.patch.set_facecolor("white")
    ax.set_facecolor("#f9f9f7")

    # 边界（外墙轮廓）
    boundary = cad_data.get("boundary", [])
    if boundary:
        coords = [_pt_xy_compat(p) for p in boundary]
        coords = [c for c in coords if c is not None]
        if coords:
            xs = [c[0] for c in coords] + [coords[0][0]]
            ys = [c[1] for c in coords] + [coords[0][1]]
            ax.fill(xs, ys, color="#e8ecf0", edgecolor="#222",
                    linewidth=2.5, zorder=1)

    # 墙体（用粗线段画）
    walls = cad_data.get("walls", [])
    for w in walls:
        if isinstance(w, dict) and "start" in w and "end" in w:
            s = _pt_xy_compat(w["start"])
            e = _pt_xy_compat(w["end"])
        elif isinstance(w, dict) and "points" in w:
            pts = [_pt_xy_compat(p) for p in w["points"]]
            pts = [p for p in pts if p is not None]
            if len(pts) >= 2:
                s, e = pts[0], pts[-1]
            else:
                continue
        else:
            continue
        if s and e:
            ax.plot([s[0], e[0]], [s[1], e[1]],
                    color="#111", linewidth=3.0, zorder=2)

    # 家具/物体（淡灰填充 + label）
    objects = cad_data.get("objects", [])
    for obj in objects:
        pts = obj.get("points", []) if isinstance(obj, dict) else []
        coords = [_pt_xy_compat(p) for p in pts]
        coords = [c for c in coords if c is not None]
        if len(coords) >= 3:
            xs = [c[0] for c in coords] + [coords[0][0]]
            ys = [c[1] for c in coords] + [coords[0][1]]
            ax.fill(xs, ys, color="#9ecae1", edgecolor="#3182bd",
                    linewidth=1.0, alpha=0.7, zorder=3)
            cx = sum(c[0] for c in coords) / len(coords)
            cy = sum(c[1] for c in coords) / len(coords)
            ax.text(cx, cy, obj.get("label", "?"),
                    ha="center", va="center", fontsize=7,
                    color="#08306b", zorder=4)

    # 门洞（绿色淡填充）
    for door in cad_data.get("doors", []):
        pts = door.get("points", []) if isinstance(door, dict) else []
        coords = [_pt_xy_compat(p) for p in pts]
        coords = [c for c in coords if c is not None]
        if len(coords) >= 3:
            xs = [c[0] for c in coords] + [coords[0][0]]
            ys = [c[1] for c in coords] + [coords[0][1]]
            ax.fill(xs, ys, color="#a1d99b", edgecolor="#31a354",
                    linewidth=1.2, alpha=0.8, zorder=4)

    # 窗（黄色淡填充）
    for win in cad_data.get("windows", []):
        pts = win.get("points", []) if isinstance(win, dict) else []
        coords = [_pt_xy_compat(p) for p in pts]
        coords = [c for c in coords if c is not None]
        if len(coords) >= 3:
            xs = [c[0] for c in coords] + [coords[0][0]]
            ys = [c[1] for c in coords] + [coords[0][1]]
            ax.fill(xs, ys, color="#fec44f", edgecolor="#cc4c02",
                    linewidth=1.0, alpha=0.7, zorder=4)

    # 图例
    from matplotlib.patches import Patch  # noqa: PLC0415
    legend = [
        Patch(facecolor="#e8ecf0", edgecolor="#222", label="boundary"),
        Patch(facecolor="#111", label="wall"),
        Patch(facecolor="#9ecae1", edgecolor="#3182bd", label="object"),
        Patch(facecolor="#a1d99b", edgecolor="#31a354", label="door"),
        Patch(facecolor="#fec44f", edgecolor="#cc4c02", label="window"),
    ]
    ax.legend(handles=legend, loc="upper right", fontsize=8, framealpha=0.9)

    src_name = Path(parse_result.get("source_path", "")).name
    n_obj = len(objects)
    ax.set_title(
        f"CAD 原图（{mode.upper()}, {src_name}） — "
        f"walls={len(walls)}, objects={n_obj}, "
        f"doors={len(cad_data.get('doors', []))}",
        fontsize=10,
    )
    ax.set_xlabel("x (m)")
    ax.set_ylabel("y (m)")
    ax.set_aspect("equal")
    ax.grid(True, alpha=0.25)
    fig.tight_layout()
    fig.savefig(out, dpi=dpi, bbox_inches="tight")
    plt.close(fig)
    return str(out)


# ══════════════════════════════════════════════════════════════════════
# 可点击拓扑图：单图、高对比、米单位坐标轴
# ══════════════════════════════════════════════════════════════════════

def render_clickable_topology(
    pipeline_result: Dict[str, Any],
    output_path: str,
    dpi: int = 140,
    marker_world_xy: Optional[List[Dict[str, Any]]] = None,
    show_grid: bool = True,
    candidates: Optional[List[Dict[str, Any]]] = None,
) -> str:
    """
    渲染一张**用户可在上面用米坐标读位置**的高对比拓扑图。

    颜色编码（与 4 联图的拓扑面板一致但更鲜明）：
      - 墙体 / 边界 = 黑
      - 膨胀缓冲（机器人禁入）= 橙
      - 可走区域 = 浅绿
      - 未到达 = 浅灰

    用 imshow 的 ``extent`` 把坐标轴标成米；用户在 Gradio 上点击图时，
    gr.SelectData.index 给出的是显示像素，下游需配合 transform 自己换算。

    Args:
        pipeline_result: ``astar_topology.run_pipeline`` 的返回字典。
        output_path: 输出 PNG 路径。
        marker_world_xy: 形如 ``[{"x": .., "y": .., "color": ".", "label": ".."}]``
            的标记列表，会被画成大圆点+文字（用于显示已选的起终点）。
    Returns:
        实际写出的 PNG 路径。
    """
    topology = pipeline_result["topology"]
    transform = pipeline_result.get("transform", {})
    seeds = pipeline_result.get("seeds", [])

    res = transform.get("resolution", 1.0)
    origin = transform.get("origin", (0.0, 0.0))
    H, W = topology.shape
    x_min, y_min = origin
    x_max = x_min + W * res
    y_max = y_min + H * res

    # 简化为 4 类：path=2 / obstacle=3 / inflated=4 / unknown=0
    cmap = ListedColormap([
        "#b8cfe8",  # 0 unknown  (淡钢蓝 — 与四联图一致)
        "#b8cfe8",  # 1 loading  (当作 unknown 处理)
        "#5ccc5c",  # 2 path     (鲜绿，比 #7fd17f 更饱和)
        "#1a1a1a",  # 3 obstacle (近黑)
        "#ff8c1a",  # 4 inflated (深橙)
    ])

    aspect = max(0.6, min(1.8, W / max(1, H)))
    fig, ax = plt.subplots(figsize=(8 * aspect, 7), dpi=dpi)
    fig.patch.set_facecolor("white")
    ax.set_facecolor("#b8cfe8")  # 超出地图范围的区域填淡蓝
    ax.imshow(
        topology, cmap=cmap, vmin=0, vmax=4,
        extent=(x_min, x_max, y_max, y_min),  # y 翻转保持图像方向
        origin="upper", interpolation="nearest",
    )

    # 候选点（编号 + 小灰圆点）— 让用户能看见"备选下拉里那些点"实际位置
    if candidates:
        for i, c in enumerate(candidates, start=1):
            x_w = float(c.get("x", 0.0))
            y_w = float(c.get("y", 0.0))
            ax.plot(x_w, y_w, "o", color="#555555", markersize=6,
                    markeredgecolor="white", markeredgewidth=1, zorder=4)
            ax.annotate(
                f"#{i}",
                xy=(x_w, y_w),
                xytext=(4, 4), textcoords="offset points",
                fontsize=7, color="#222",
                bbox=dict(boxstyle="round,pad=0.15", facecolor="white",
                          edgecolor="#888", alpha=0.7),
                zorder=4,
            )

    # 种子点（粉色 ×，门洞）
    for r, c in seeds:
        x_w = origin[0] + c * res
        y_w = origin[1] + r * res
        ax.plot(x_w, y_w, "x", color="#d62728", markersize=14,
                markeredgewidth=2.5, zorder=5)

    # 用户已选的 A / B
    if marker_world_xy:
        for m in marker_world_xy:
            ax.plot(m["x"], m["y"], "o", color=m.get("color", "#1f77b4"),
                    markersize=16, markeredgecolor="white",
                    markeredgewidth=2, zorder=6)
            ax.annotate(
                m.get("label", "?"),
                xy=(m["x"], m["y"]),
                xytext=(8, 8), textcoords="offset points",
                fontsize=12, fontweight="bold",
                color=m.get("color", "#1f77b4"),
                bbox=dict(boxstyle="round,pad=0.3", facecolor="white",
                          edgecolor=m.get("color", "#1f77b4"), alpha=0.85),
                zorder=7,
            )

    from matplotlib.patches import Patch  # noqa: PLC0415
    legend = [
        Patch(facecolor="#5ccc5c", edgecolor="#2a8a2a", linewidth=1.2,
              label="[绿] 可走区 — 可在此点击选起/终点"),
        Patch(facecolor="#ff8c1a", edgecolor="#b06010", linewidth=1.0,
              label="[橙] 膨胀缓冲 — 机器人重心禁入区"),
        Patch(facecolor="#1a1a1a", edgecolor="#ffffff", linewidth=0.5,
              label="[黑] 墙 / 障碍物"),
        Patch(facecolor="#b8cfe8", edgecolor="#7a9abf", linewidth=0.5,
              label="[蓝] 未到达区（孤立空间）"),
    ]
    ax.legend(handles=legend, loc="lower right", fontsize=8.5,
              framealpha=0.92, edgecolor="#888")
    ax.set_title(
        "Topology Labels — click green regions for A/B",
        fontsize=11, fontweight="bold",
    )
    ax.set_xlabel("x (m)", fontsize=9)
    ax.set_ylabel("y (m)", fontsize=9)
    ax.set_aspect("equal")
    if show_grid:
        ax.grid(True, alpha=0.20, color="#aaaaaa", linewidth=0.4)

    out = Path(output_path)
    out.parent.mkdir(parents=True, exist_ok=True)
    fig.tight_layout()
    fig.savefig(out, dpi=dpi, bbox_inches="tight")
    plt.close(fig)
    return str(out)


# ══════════════════════════════════════════════════════════════════════
# 3D 拓扑预览 (Plotly) — 交互式白模
# ══════════════════════════════════════════════════════════════════════

try:
    import plotly.graph_objects as _go
    HAS_PLOTLY_VIZ = True
except ImportError:
    HAS_PLOTLY_VIZ = False
    _go = None  # type: ignore


def generate_3d_topology_preview(
    pipeline_result: Dict[str, Any],
    title: str = "SMDSL 3D Topology Preview",
) -> Any:
    """
    生成 3D 拓扑白模预览——墙体拉伸为 3D 线框，地面着色距离场。

    Args:
        pipeline_result: 含 grid, distance_field, topology, transform, seeds, semantics
        title: 图表标题
    Returns:
        plotly.graph_objects.Figure
    """
    if not HAS_PLOTLY_VIZ:
        raise ImportError("plotly not installed. pip install plotly")

    from scipy import ndimage

    CLASS_PATH = 2
    CLASS_OBSTACLE = 3
    CLASS_INFLATED = 4

    distance_field = pipeline_result.get("distance_field")
    topology = pipeline_result.get("topology")
    transform = pipeline_result.get("transform", {})
    seeds = pipeline_result.get("seeds", [])
    semantics = pipeline_result.get("semantics") or {}

    res = float(transform.get("resolution", 0.05))
    ox = float(transform.get("origin", (0, 0))[0])
    oy = float(transform.get("origin", (0, 0))[1])
    H0, W0 = (distance_field.shape if distance_field is not None
              else transform.get("shape", (10, 10)))

    fig = _go.Figure()

    # Layer 1: Distance field floor
    if distance_field is not None:
        ds = max(1, max(H0, W0) // 200)
        df_ds = distance_field[::ds, ::ds]
        Hd, Wd = df_ds.shape
        df_m = df_ds * res
        fig.add_trace(_go.Surface(
            x=np.linspace(ox, ox + W0 * res, Wd),
            y=np.linspace(oy, oy + H0 * res, Hd),
            z=np.zeros_like(df_m),
            surfacecolor=df_m, colorscale="Viridis",
            colorbar=dict(title="Clearance (m)", x=1.02),
            name="Distance Field", showscale=True, hoverinfo="skip",
        ))

    # Layer 2: Wall wireframe as vertical line segments (not dots)
    if topology is not None:
        wall = (topology == CLASS_OBSTACLE) | (topology == CLASS_INFLATED)
        dsw = max(1, max(H0, W0) // 200)
        if dsw > 1:
            Hw, Ww = H0 // dsw, W0 // dsw
            wall = wall[:Hw*dsw, :Ww*dsw].reshape(Hw, dsw, Ww, dsw).max(axis=(1, 3))
        else:
            Hw, Ww = H0, W0
        edges = ndimage.sobel(wall.astype(float)) > 0.1 if np.any(wall) and not np.all(wall) else wall
        epts = np.argwhere(edges)
        if len(epts) > 0:
            max_pts = 4000
            if len(epts) > max_pts:
                epts = epts[np.random.choice(len(epts), max_pts, replace=False)]
            ex = ox + epts[:, 1] * (res * dsw)
            ey = oy + epts[:, 0] * (res * dsw)
            # Vertical line segments: connect each wall point from Z=0 to Z=2.5
            wall_x, wall_y, wall_z = [], [], []
            for i in range(len(ex)):
                wall_x.extend([ex[i], ex[i], None])
                wall_y.extend([ey[i], ey[i], None])
                wall_z.extend([0.0, 2.5, None])
            fig.add_trace(_go.Scatter3d(
                x=wall_x, y=wall_y, z=wall_z, mode="lines",
                line=dict(color="gray", width=1.5),
                name="Walls", hoverinfo="skip",
            ))

    # Layer 3: Path region highlights
    if topology is not None:
        path = topology == CLASS_PATH
        dsp = max(1, max(H0, W0) // 200)
        if dsp > 1:
            Hp, Wp = H0 // dsp, W0 // dsp
            path = path[:Hp*dsp, :Wp*dsp].reshape(Hp, dsp, Wp, dsp).max(axis=(1, 3))
        ppts = np.argwhere(path)
        if len(ppts) > 0:
            if len(ppts) > 5000:
                ppts = ppts[np.random.choice(len(ppts), 5000, replace=False)]
            fig.add_trace(_go.Scatter3d(
                x=ox + ppts[:, 1] * (res * dsp),
                y=oy + ppts[:, 0] * (res * dsp),
                z=[0.05]*len(ppts), mode="markers",
                marker=dict(size=3, color="limegreen", opacity=0.6),
                name="Navigable (PATH)", hoverinfo="skip",
            ))

    # Layer 4: Seed points (doors)
    if seeds:
        sr = np.array(seeds)
        fig.add_trace(_go.Scatter3d(
            x=ox + sr[:, 1] * res, y=oy + sr[:, 0] * res,
            z=[1.0]*len(sr), mode="markers",
            marker=dict(size=8, color="cyan", symbol="diamond"),
            name="Doors / Seeds", hoverinfo="skip",
        ))

    # Layer 5: DWG text labels
    texts = semantics.get("texts", [])
    if texts:
        tv, tx, ty = [], [], []
        for t in texts[:20]:
            v = t.get("value", "").strip()
            if v and len(v) < 30:
                p = t.get("position", [0, 0])
                tv.append(v[:20]); tx.append(p[0]); ty.append(p[1])
        if tv:
            fig.add_trace(_go.Scatter3d(
                x=tx, y=ty, z=[2.5]*len(tx), mode="text",
                text=tv, textfont=dict(size=10, color="white"),
                name="Text Labels", hoverinfo="text",
            ))

    fig.update_layout(
        title=dict(text=title, font=dict(size=16)),
        scene=dict(
            xaxis=dict(title="X (m)", range=[ox, ox + W0 * res]),
            yaxis=dict(title="Y (m)", range=[oy, oy + H0 * res]),
            zaxis=dict(title="Z (m)", range=[0, 3.5]),
            aspectmode="data",
            camera=dict(eye=dict(x=1.5, y=1.5, z=1.0)),
        ),
        margin=dict(l=0, r=0, b=0, t=40), height=550,
        legend=dict(yanchor="top", y=0.99, xanchor="left", x=0.01),
    )
    return fig
