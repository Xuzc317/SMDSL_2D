"""
osm_ingestion.py — osmAG .osm 直接接入拓扑管道

架构合规性说明
──────────────
- 对于 .osm 文件，**绕过** rasterize_to_grid + A* flood-fill 两步。
- OSM 已原生携带：
    ① 房间多边形坐标（area_way）→ 直接多边形填充生成 occupancy grid
    ② 房间邻接图（passage_way from/to）→ 直接构建 adjacency_graph

- 输出的 ParseResult 字典完全兼容 dispatcher.dispatch_cad() 返回格式，
  同时通过额外字段（osm_rooms / osm_edges / adjacency_graph）
  为 Demo 2 语义分析和 Demo 3 3D 可视化提供原生语义数据。

主入口：parse_osm_to_topology(file_path, resolution, padding_m)
"""

from __future__ import annotations

import math
import xml.etree.ElementTree as ET
from pathlib import Path
from typing import Any, Dict, List, Optional, Tuple

import numpy as np

try:
    from cad_parser.osm_evaluator import (
        _parse_nodes,
        _parse_ways,
        _compute_origin,
        _to_metric,
        _latlon_to_xy,
        _polygon_area,
        _centroid,
    )
except ImportError:
    from osm_evaluator import (  # type: ignore
        _parse_nodes,
        _parse_ways,
        _compute_origin,
        _to_metric,
        _latlon_to_xy,
        _polygon_area,
        _centroid,
    )


# ══════════════════════════════════════════════════════════════════════
# 多边形填充 → occupancy grid
# ══════════════════════════════════════════════════════════════════════

def _build_grid_from_polygons(
    room_polys_m: List[List[Tuple[float, float]]],
    bbox: Tuple[float, float, float, float],
    resolution: float,
    padding_m: float,
) -> Tuple[np.ndarray, Dict[str, Any]]:
    """
    将房间多边形直接光栅化为 occupancy grid。

    约定：grid[r, c] = 1 → 自由空间；= 0 → 墙/障碍。
    房间内部填充为 1，房间边界及背景为 0。

    Args:
        room_polys_m: 每个房间的 [(x_m, y_m), ...] 顶点列表（未闭合或闭合均可）
        bbox: (x_min, y_min, x_max, y_max)（米）
        resolution: 米/像素
        padding_m: 在 bbox 四周增加的额外余量

    Returns:
        (grid, transform)
    """
    x_min, y_min, x_max, y_max = bbox
    ox = x_min - padding_m
    oy = y_min - padding_m
    W = max(2, int(math.ceil((x_max - x_min + 2 * padding_m) / resolution)))
    H = max(2, int(math.ceil((y_max - y_min + 2 * padding_m) / resolution)))

    grid = np.zeros((H, W), dtype=np.uint8)

    def to_pix(x_m: float, y_m: float) -> Tuple[int, int]:
        col = int(round((x_m - ox) / resolution))
        row = int(round((y_m - oy) / resolution))
        return np.clip(row, 0, H - 1), np.clip(col, 0, W - 1)

    try:
        from skimage.draw import polygon as skimage_polygon  # noqa: PLC0415
        _has_skimage = True
    except ImportError:
        _has_skimage = False

    for poly in room_polys_m:
        if len(poly) < 3:
            continue
        # 去掉闭合重复点
        pts = poly if poly[0] != poly[-1] else poly[:-1]
        if len(pts) < 3:
            continue

        if _has_skimage:
            rows_px = np.array([to_pix(x, y)[0] for x, y in pts])
            cols_px = np.array([to_pix(x, y)[1] for x, y in pts])
            rr, cc = skimage_polygon(rows_px, cols_px, shape=(H, W))
            grid[rr, cc] = 1
        else:
            # 后备：扫描线光栅化（纯 numpy，无 skimage 依赖）
            rows_px = [to_pix(x, y)[0] for x, y in pts]
            cols_px = [to_pix(x, y)[1] for x, y in pts]
            r_min, r_max = max(0, min(rows_px)), min(H - 1, max(rows_px))
            for r in range(r_min, r_max + 1):
                # 求扫描线 r 与多边形各边的交点
                intersect_cols: List[float] = []
                n = len(pts)
                for i in range(n):
                    j = (i + 1) % n
                    ry0, cy0 = rows_px[i], cols_px[i]
                    ry1, cy1 = rows_px[j], cols_px[j]
                    if ry0 == ry1:
                        continue
                    if not (min(ry0, ry1) <= r < max(ry0, ry1)):
                        continue
                    t = (r - ry0) / (ry1 - ry0)
                    intersect_cols.append(cy0 + t * (cy1 - cy0))
                intersect_cols.sort()
                for k in range(0, len(intersect_cols) - 1, 2):
                    c0 = max(0, int(math.ceil(intersect_cols[k])))
                    c1 = min(W - 1, int(math.floor(intersect_cols[k + 1])))
                    if c0 <= c1:
                        grid[r, c0: c1 + 1] = 1

    transform: Dict[str, Any] = {
        "origin": (ox, oy),
        "resolution": resolution,
        "shape": (H, W),
    }
    return grid, transform


# ══════════════════════════════════════════════════════════════════════
# 邻接图构建
# ══════════════════════════════════════════════════════════════════════

def _build_adjacency_graph(
    passage_ways: List[Dict[str, Any]],
) -> Dict[str, List[str]]:
    """
    从 passage_way 列表构建无向邻接图。

    Returns:
        {room_name: [neighbor_room_name, ...]}
    """
    graph: Dict[str, List[str]] = {}
    for p in passage_ways:
        src = p.get("from_room", "")
        dst = p.get("to_room", "")
        if not src or not dst:
            continue
        graph.setdefault(src, [])
        graph.setdefault(dst, [])
        if dst not in graph[src]:
            graph[src].append(dst)
        if src not in graph[dst]:
            graph[dst].append(src)
    return graph


# ══════════════════════════════════════════════════════════════════════
# 主解析函数
# ══════════════════════════════════════════════════════════════════════

def parse_osm_to_topology(
    file_path: str,
    resolution: float = 0.1,
    padding_m: float = 1.0,
    wall_thickness_m: float = 0.15,
) -> Dict[str, Any]:
    """
    将 osmAG .osm 文件直接解析为兼容 dispatcher.dispatch_cad() 的 ParseResult。

    与其他格式解析器的核心差异：
      - 不调用 rasterize_to_grid（OSM 已有精确多边形）
      - 不调用 A* flood-fill（OSM 已有 passage 邻接关系）
      - grid 由多边形直接填充，resolution 对 Demo 3 距离场有意义

    Returns ParseResult dict：
      mode:           "osm"
      grid:           np.ndarray (H, W) uint8
      transform:      {origin, resolution, shape}
      cad_data:       兼容 JSON 模式的结构（含 boundary/walls/objects/doors）
      semantics:      {rooms, passages, adjacency_graph, origin_latlon}
      osm_rooms:      原生房间列表（带 coords_m / area_m2 / centroid / tags）
      osm_edges:      原生通路列表（带 from_room / to_room / midpoint_m）
      adjacency_graph: {room_name: [neighbors]}
      source_path:    str
      has_semantics:  True
      note:           str

    Raises:
        CadDispatchError（兼容 dispatcher 的异常类型）
    """
    try:
        from cad_parser.dispatcher import CadDispatchError  # noqa: PLC0415
    except ImportError:
        class CadDispatchError(RuntimeError):  # type: ignore
            pass

    p = Path(file_path)
    if not p.exists():
        raise CadDispatchError(f"文件不存在：{file_path}")

    try:
        tree = ET.parse(p)
    except ET.ParseError as e:
        raise CadDispatchError(f"OSM XML 解析失败：{e}") from e

    root = tree.getroot()
    nodes_latlon = _parse_nodes(root)
    area_ways, passage_ways = _parse_ways(root, nodes_latlon)

    if not area_ways:
        raise CadDispatchError(
            f"OSM 文件中未找到任何 osmAG:type=area 的 way 元素：{file_path}"
        )

    # 质心作为局部坐标系原点
    lat0, lon0 = _compute_origin(nodes_latlon)

    # 节点 lat/lon → 局部米制 XY
    nodes_m: Dict[str, Tuple[float, float]] = {
        nid: _latlon_to_xy(lat, lon, lat0, lon0)
        for nid, (lat, lon) in nodes_latlon.items()
    }

    # ── 房间数据 ────────────────────────────────────────────
    osm_rooms: List[Dict[str, Any]] = []
    room_polys_m: List[List[Tuple[float, float]]] = []

    for w in area_ways:
        pts_m: List[Tuple[float, float]] = [
            nodes_m[r] for r in w["node_ids"] if r in nodes_m
        ]
        area_m2 = _polygon_area(pts_m)
        cx, cy = _centroid(pts_m)
        osm_rooms.append({
            "name": w["name"],
            "osm_id": w["id"],
            "area_type": w.get("area_type", "room"),
            "area_m2": round(area_m2, 2),
            "centroid_m": [round(cx, 3), round(cy, 3)],
            "height_m": float(w.get("height", "3.0") or "3.0"),
            "level": w.get("level", "1"),
            "indoor": w.get("indoor", "room"),
            "coords_m": [[round(x, 4), round(y, 4)] for x, y in pts_m],
            "tags": w.get("tags", {}),
        })
        room_polys_m.append(pts_m)

    # ── 通路数据 ────────────────────────────────────────────
    osm_edges: List[Dict[str, Any]] = []
    for w in passage_ways:
        pts_m = [nodes_m[r] for r in w["node_ids"] if r in nodes_m]
        cx, cy = _centroid(pts_m)
        osm_edges.append({
            "name": w["name"],
            "osm_id": w["id"],
            "from_room": w.get("from_room", ""),
            "to_room": w.get("to_room", ""),
            "midpoint_m": [round(cx, 4), round(cy, 4)],
            "door": w.get("door", ""),
            "coords_m": [[round(x, 4), round(y, 4)] for x, y in pts_m],
        })

    # ── bbox（所有节点） ─────────────────────────────────────
    all_x = [v[0] for v in nodes_m.values()]
    all_y = [v[1] for v in nodes_m.values()]
    x_min, x_max = min(all_x), max(all_x)
    y_min, y_max = min(all_y), max(all_y)
    bbox = (x_min, y_min, x_max, y_max)

    # ── 自动降采样保护（> 4000px 时放大 resolution）────────────
    effective_res = resolution
    max_dim = max(
        (x_max - x_min + 2 * padding_m) / resolution,
        (y_max - y_min + 2 * padding_m) / resolution,
    )
    if max_dim > 4000:
        effective_res = resolution * (max_dim / 2000.0)

    # ── 多边形填充 → occupancy grid ──────────────────────────
    grid, transform = _build_grid_from_polygons(
        room_polys_m, bbox, effective_res, padding_m
    )

    # ── 邻接图 ───────────────────────────────────────────────
    adjacency_graph = _build_adjacency_graph(passage_ways)

    # ── 构建兼容 JSON 模式的 cad_data ────────────────────────
    boundary_poly = [
        (x_min - padding_m, y_min - padding_m),
        (x_max + padding_m, y_min - padding_m),
        (x_max + padding_m, y_max + padding_m),
        (x_min - padding_m, y_max + padding_m),
    ]

    cad_data: Dict[str, Any] = {
        "layout_id": p.stem,
        "room_type": "osm_indoor",
        "units": "meters",
        "boundary": boundary_poly,
        "walls": [],             # OSM 无显式墙线；边界由多边形边缘隐式定义
        "doors": [
            {"position": e["midpoint_m"], "name": e["name"],
             "from": e["from_room"], "to": e["to_room"]}
            for e in osm_edges
        ],
        "windows": [],
        "objects": [
            {"label": r["name"], "points": r["coords_m"],
             "layer": r["area_type"], "area_m2": r["area_m2"]}
            for r in osm_rooms
        ],
        "bbox": [x_min, y_min, x_max, y_max],
        "_osm_origin_latlon": [lat0, lon0],
        "_osm_node_count": len(nodes_latlon),
        "_osm_room_count": len(osm_rooms),
        "_osm_passage_count": len(osm_edges),
    }

    semantics = {
        "rooms": [
            {"name": r["name"], "area_type": r["area_type"],
             "area_m2": r["area_m2"], "height_m": r["height_m"]}
            for r in osm_rooms
        ],
        "passages": [
            {"name": e["name"], "from": e["from_room"], "to": e["to_room"]}
            for e in osm_edges
        ],
        "adjacency_graph": adjacency_graph,
        "origin_latlon": [lat0, lon0],
    }

    note = (
        f"osmAG {p.name}：{len(osm_rooms)} 个房间多边形，"
        f"{len(osm_edges)} 条通路边，"
        f"bbox {x_max - x_min:.1f}m × {y_max - y_min:.1f}m，"
        f"grid {grid.shape[1]}×{grid.shape[0]}px @ {effective_res:.3f}m/px。"
        "已绕过 rasterize_to_grid 和 A* flood-fill，直接利用 OSM 拓扑。"
    )

    return {
        "mode": "osm",
        "grid": grid,
        "transform": transform,
        "cad_data": cad_data,
        "semantics": semantics,
        "osm_rooms": osm_rooms,
        "osm_edges": osm_edges,
        "adjacency_graph": adjacency_graph,
        "source_path": str(p),
        "has_semantics": True,
        "note": note,
    }
