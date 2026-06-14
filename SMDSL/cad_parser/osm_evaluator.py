"""
osm_evaluator.py — osmAG .osm XML 格式探针

职责：
  1. 解析 OSM XML，提取 node / way，并按 osmAG: 标签归类
  2. 输出 JSON 摘要（可直接阅读），评估与 topology_bundle 的对齐方案
  3. 可作为独立脚本运行，也可被 osm_ingestion 导入

运行：
  python cad_parser/osm_evaluator.py <path/to/file.osm>
"""

from __future__ import annotations

import json
import math
import sys
import xml.etree.ElementTree as ET
from pathlib import Path
from typing import Any, Dict, List, Optional, Tuple


# ══════════════════════════════════════════════════════════════════════
# 核心解析
# ══════════════════════════════════════════════════════════════════════

def _parse_nodes(root: ET.Element) -> Dict[str, Tuple[float, float]]:
    """id → (lat, lon)"""
    nodes: Dict[str, Tuple[float, float]] = {}
    for el in root.iter("node"):
        nid = el.get("id", "")
        try:
            lat = float(el.get("lat", "0"))
            lon = float(el.get("lon", "0"))
        except ValueError:
            continue
        nodes[nid] = (lat, lon)
    return nodes


def _parse_ways(
    root: ET.Element,
    nodes: Dict[str, Tuple[float, float]],
) -> Tuple[List[Dict[str, Any]], List[Dict[str, Any]]]:
    """
    返回 (area_ways, passage_ways)。

    area_way 结构：
      {id, name, area_type, indoor, level, height, tags, node_ids, coords_latlon}

    passage_way 结构：
      {id, name, from_room, to_room, indoor, level, height, tags, node_ids, coords_latlon}
    """
    area_ways: List[Dict[str, Any]] = []
    passage_ways: List[Dict[str, Any]] = []

    for el in root.iter("way"):
        wid = el.get("id", "")
        tags: Dict[str, str] = {
            t.get("k", ""): t.get("v", "") for t in el.findall("tag")
        }
        nd_refs = [nd.get("ref", "") for nd in el.findall("nd")]
        coords = [nodes[r] for r in nd_refs if r in nodes]

        osm_type = tags.get("osmAG:type", "")
        name = tags.get("name", wid)
        level = tags.get("level", "1")
        height = tags.get("height", "")
        indoor = tags.get("indoor", "")

        base = {
            "id": wid,
            "name": name,
            "indoor": indoor,
            "level": level,
            "height": height,
            "tags": tags,
            "node_ids": nd_refs,
            "coords_latlon": coords,
        }

        if osm_type == "area":
            base["area_type"] = tags.get("osmAG:areaType", "room")
            area_ways.append(base)
        elif osm_type == "passage":
            base["from_room"] = tags.get("osmAG:from", "")
            base["to_room"] = tags.get("osmAG:to", "")
            base["door"] = tags.get("door", "")
            passage_ways.append(base)
        # 未知类型的 way 忽略（如 root node way）

    return area_ways, passage_ways


def _latlon_to_xy(
    lat: float, lon: float, lat0: float, lon0: float
) -> Tuple[float, float]:
    """
    将 lat/lon 投影到以 (lat0, lon0) 为原点的局部平面坐标系（米）。
    使用等距柱面近似（equirectangular projection），适合小范围建筑图纸。
    """
    R = 6_371_000.0  # 地球平均半径（米）
    x = R * math.radians(lon - lon0) * math.cos(math.radians(lat0))
    y = R * math.radians(lat - lat0)
    return x, y


def _compute_origin(
    nodes: Dict[str, Tuple[float, float]],
) -> Tuple[float, float]:
    """取所有节点的 lat/lon 质心作为局部坐标系原点。"""
    if not nodes:
        return 0.0, 0.0
    lats = [v[0] for v in nodes.values()]
    lons = [v[1] for v in nodes.values()]
    return sum(lats) / len(lats), sum(lons) / len(lons)


def _to_metric(
    coords_latlon: List[Tuple[float, float]],
    origin: Tuple[float, float],
) -> List[Tuple[float, float]]:
    """将 (lat, lon) 列表转换为局部 (x_m, y_m) 列表。"""
    lat0, lon0 = origin
    return [_latlon_to_xy(lat, lon, lat0, lon0) for lat, lon in coords_latlon]


# ══════════════════════════════════════════════════════════════════════
# 摘要生成
# ══════════════════════════════════════════════════════════════════════

def _polygon_area(pts: List[Tuple[float, float]]) -> float:
    """Shoelace 公式计算多边形有符号面积（m²）。"""
    n = len(pts)
    if n < 3:
        return 0.0
    area = 0.0
    for i in range(n):
        j = (i + 1) % n
        area += pts[i][0] * pts[j][1]
        area -= pts[j][0] * pts[i][1]
    return abs(area) / 2.0


def _centroid(pts: List[Tuple[float, float]]) -> Tuple[float, float]:
    if not pts:
        return 0.0, 0.0
    return sum(p[0] for p in pts) / len(pts), sum(p[1] for p in pts) / len(pts)


def evaluate_osm_file(file_path: str) -> Dict[str, Any]:
    """
    主探针函数：解析 .osm，返回完整 JSON 摘要。

    返回结构::

        {
            "file": str,
            "node_count": int,
            "way_count": int,
            "area_count": int,
            "passage_count": int,
            "coord_origin_latlon": [lat0, lon0],
            "bbox_m": {"x_min", "x_max", "y_min", "y_max", "w_m", "h_m"},
            "rooms": [{id, name, area_type, area_m2, centroid_m, vertex_count, height, tags}],
            "passages": [{id, name, from_room, to_room, midpoint_m, door}],
            "topology_graph": {"nodes": [...], "edges": [...]},
            "alignment_notes": [str],
        }
    """
    p = Path(file_path)
    try:
        tree = ET.parse(p)
    except ET.ParseError as e:
        return {"error": f"XML 解析失败: {e}", "file": str(p)}

    root = tree.getroot()
    nodes = _parse_nodes(root)
    area_ways, passage_ways = _parse_ways(root, nodes)

    lat0, lon0 = _compute_origin(nodes)

    # 所有节点转米制
    nodes_m: Dict[str, Tuple[float, float]] = {
        nid: _latlon_to_xy(lat, lon, lat0, lon0)
        for nid, (lat, lon) in nodes.items()
    }

    # 收集 bbox
    if nodes_m:
        all_x = [v[0] for v in nodes_m.values()]
        all_y = [v[1] for v in nodes_m.values()]
        bbox = {
            "x_min": round(min(all_x), 3),
            "x_max": round(max(all_x), 3),
            "y_min": round(min(all_y), 3),
            "y_max": round(max(all_y), 3),
            "w_m": round(max(all_x) - min(all_x), 3),
            "h_m": round(max(all_y) - min(all_y), 3),
        }
    else:
        bbox = {}

    # 房间摘要
    rooms_summary: List[Dict[str, Any]] = []
    for w in area_ways:
        pts_m = [nodes_m[r] for r in w["node_ids"] if r in nodes_m]
        area_m2 = _polygon_area(pts_m)
        cx, cy = _centroid(pts_m)
        rooms_summary.append({
            "id": w["id"],
            "name": w["name"],
            "area_type": w.get("area_type", "room"),
            "area_m2": round(area_m2, 2),
            "centroid_m": [round(cx, 3), round(cy, 3)],
            "vertex_count": len(pts_m),
            "height": w.get("height", ""),
            "level": w.get("level", ""),
            "indoor": w.get("indoor", ""),
        })

    # 通路摘要
    passages_summary: List[Dict[str, Any]] = []
    for w in passage_ways:
        pts_m = [nodes_m[r] for r in w["node_ids"] if r in nodes_m]
        cx, cy = _centroid(pts_m)
        passages_summary.append({
            "id": w["id"],
            "name": w["name"],
            "from_room": w.get("from_room", ""),
            "to_room": w.get("to_room", ""),
            "midpoint_m": [round(cx, 3), round(cy, 3)],
            "door": w.get("door", ""),
            "node_count": len(pts_m),
        })

    # 拓扑图（抽象表示，供对齐分析）
    topo_nodes = [{"id": r["name"], "centroid_m": r["centroid_m"], "area_m2": r["area_m2"]}
                  for r in rooms_summary]
    topo_edges = [{"id": p["name"], "from": p["from_room"], "to": p["to_room"],
                   "via": p["midpoint_m"]}
                  for p in passages_summary]

    # 与 topology_bundle 的对齐说明
    alignment_notes = [
        "area_way → topology_bundle.cad_data['objects'] 中的房间多边形（coords_m）",
        "passage_way(from/to) → A* 拓扑图中的邻接边（adjacency_graph），无需 flood-fill 推断",
        "osmAG:areaType → cad_data['room_type'] 及 semantic_extractor 的区域分类",
        "height → 3D 可视化的墙体高度（visualize_demo3.py wall_height）",
        "level → 多层建筑时的 Z 偏移（当前 Demo 1~3 按单层处理，level 字段保留备用）",
        "coords_m（等距柱面投影）→ grid 坐标系原点 (x_min, y_min)，resolution 可按 bbox 自动推导",
        "OSM 模式绕过 rasterize_to_grid 和 A* flood-fill；直接多边形填充生成 occupancy grid",
    ]

    return {
        "file": str(p),
        "node_count": len(nodes),
        "way_count": len(area_ways) + len(passage_ways),
        "area_count": len(area_ways),
        "passage_count": len(passage_ways),
        "coord_origin_latlon": [round(lat0, 8), round(lon0, 8)],
        "bbox_m": bbox,
        "rooms": rooms_summary,
        "passages": passages_summary,
        "topology_graph": {
            "nodes": topo_nodes,
            "edges": topo_edges,
        },
        "alignment_notes": alignment_notes,
    }


# ══════════════════════════════════════════════════════════════════════
# CLI 入口
# ══════════════════════════════════════════════════════════════════════

if __name__ == "__main__":
    if len(sys.argv) < 2:
        # 自动选第一个 osm 文件
        data_root = Path(__file__).resolve().parents[2] / "data" / "osmAG-from-cad"
        osm_files = list(data_root.rglob("*.osm"))
        if not osm_files:
            print("用法: python osm_evaluator.py <file.osm>", file=sys.stderr)
            sys.exit(1)
        target = osm_files[0]
        print(f"[auto] 使用: {target}", file=sys.stderr)
    else:
        target = Path(sys.argv[1])

    summary = evaluate_osm_file(str(target))
    # 去掉 rooms/passages 列表以保持终端可读，最多显示 3 条
    display = dict(summary)
    display["rooms"] = summary["rooms"][:3]
    display["passages"] = summary["passages"][:3]
    display["_note"] = (
        f"（仅显示前 3 条；完整数量：rooms={summary['area_count']}，"
        f"passages={summary['passage_count']}）"
    )
    print(json.dumps(display, ensure_ascii=False, indent=2))
