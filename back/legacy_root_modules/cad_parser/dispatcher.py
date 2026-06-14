"""
dispatcher.py — Demo 1 多格式 CAD 输入派发器

按文件扩展名走不同 parser，统一产出可被下游消费的 occupancy grid：

  ┌────────────┬──────────────────────────┬───────────────────────────┐
  │ 输入格式    │ 处理方式                  │ 语义能力                   │
  ├────────────┼──────────────────────────┼───────────────────────────┤
  │ .json      │ FloorplanQA Schema 解析   │ ✅ 完整（door/object）      │
  │ .dwg       │ LibreDWG → 双轨提取       │ ✅ AI 语义流（layer/text）  │
  │ .svg       │ <line>/<polyline> 抽取    │ ⚠️ 仅边界（无对象）        │
  │ .png/.jpg  │ 二值化为 occupancy        │ ⚠️ 仅占据（无标签）        │
  └────────────┴──────────────────────────┴───────────────────────────┘

双轨分发（仅 DWG 模式）：
  轨迹 A（数学几何流）：AcDbLine / AcDbPolyline / AcDbCircle →
    墙体线段 → rasterize_to_grid → A* 拓扑。原有 30ms 级算法不受影响。
  轨迹 B（AI 语义流）：图层名 / Text / MText / Block References →
    供 semantic_extractor 消费，识别上料/下料位置等业务节点。

注意：PNG 路径不是『图像识别 / ML 分类』。它把整张图看作用户已经
栅格化好的占据图（黑像素=墙，白像素=自由空间），跳过本项目的
``rasterize_to_grid`` 一步。这与 PROJECT_CONTEXT 的"零训练 / 可解释"
原则完全一致。

返回的 ``ParseResult`` 字典字段：
  - mode:        "json" / "dwg" / "svg" / "png"
  - grid:        np.ndarray (H, W) uint8，0=占据 / 1=自由
  - transform:   {"origin": (x_m, y_m), "resolution": float, "shape": (H, W)}
  - cad_data:    语义信息（JSON/DWG 模式完整；其他模式受限）
  - semantics:   DWG 模式下的 Track B 语义数据（layers/texts/blocks）
  - source_path: 原始输入路径
"""

from __future__ import annotations

import json
import math
import xml.etree.ElementTree as ET
from pathlib import Path
from typing import Any, Dict, List, Optional, Tuple

import numpy as np

try:
    from cad_parser.astar_topology import (
        load_cad_vector,
        rasterize_to_grid,
        remove_exterior_freespace,
        weld_double_line_walls,
        bridge_thin_walls,
    )
except ImportError:
    from astar_topology import (  # type: ignore
        load_cad_vector,
        rasterize_to_grid,
        remove_exterior_freespace,
        weld_double_line_walls,
        bridge_thin_walls,
    )


JSON_EXTS = {".json"}
DWG_EXTS = {".dwg"}
SVG_EXTS = {".svg"}
RASTER_EXTS = {".png", ".jpg", ".jpeg", ".bmp", ".tif", ".tiff"}


class CadDispatchError(RuntimeError):
    """Dispatcher 通用异常。"""


# ══════════════════════════════════════════════════════════════════════
# JSON：FloorplanQA Schema → 走原栅格化路径，保留全部语义
# ══════════════════════════════════════════════════════════════════════

def _parse_json(
    path: Path,
    resolution: float,
    padding_m: float,
    wall_thickness_m: float,
) -> Dict[str, Any]:
    cad_data = load_cad_vector(str(path))
    grid, transform = rasterize_to_grid(
        cad_data,
        resolution=resolution,
        padding_m=padding_m,
        wall_thickness_m=wall_thickness_m,
    )
    return {
        "mode": "json",
        "grid": grid,
        "transform": transform,
        "cad_data": cad_data,
        "source_path": str(path),
        "has_semantics": True,
        "note": "完整语义：含 doors / objects / windows，可作 Demo 2 词汇表。",
    }


# ══════════════════════════════════════════════════════════════════════
# SVG：抽 <line> / <polyline> / <polygon> 当墙；不区分对象语义
# ══════════════════════════════════════════════════════════════════════

def _parse_svg(
    path: Path,
    resolution: float,
    padding_m: float,
    wall_thickness_m: float,
) -> Dict[str, Any]:
    """简化 SVG 解析：仅识别 line/polyline/polygon。

    若 SVG 含 viewBox 或 width/height（带 mm/m 单位），按比例换算到米；
    否则把每个 SVG 单位当作 1 米（用户可在 UI 上手调 resolution）。
    """
    try:
        tree = ET.parse(path)
    except ET.ParseError as e:
        raise CadDispatchError(f"SVG 解析失败：{e}") from e
    root = tree.getroot()
    ns = {"svg": "http://www.w3.org/2000/svg"}

    walls_xy: List[Tuple[float, float, float, float]] = []

    def add_segments(coords: List[Tuple[float, float]]) -> None:
        for i in range(len(coords) - 1):
            x1, y1 = coords[i]
            x2, y2 = coords[i + 1]
            walls_xy.append((x1, y1, x2, y2))

    def parse_path_d(d: str) -> List[Tuple[float, float]]:
        """轻量 SVG path 解析：仅支持 M/m/L/l/H/h/V/v/Z/z 命令。
        这些命令覆盖了绝大多数 CAD 工具导出的墙体路径。
        Bezier (C/c/Q/q) 等曲线命令被忽略（floor plan 几乎不用）。
        """
        coords: List[Tuple[float, float]] = []
        if not d:
            return coords
        import re  # noqa: PLC0415
        # 按命令字母切：M/m/L/l/H/h/V/v/Z/z
        tokens = re.findall(r"[MmLlHhVvZz]|[-+]?\d*\.?\d+(?:[eE][-+]?\d+)?", d)
        i, cur_x, cur_y, start_x, start_y = 0, 0.0, 0.0, 0.0, 0.0
        cmd = None
        while i < len(tokens):
            t = tokens[i]
            if t in "MmLlHhVvZz":
                cmd = t
                i += 1
                if cmd in "Zz":
                    coords.append((start_x, start_y))
                    cur_x, cur_y = start_x, start_y
                continue
            try:
                if cmd in ("M", "L"):
                    x, y = float(tokens[i]), float(tokens[i + 1])
                    i += 2
                    if cmd == "M":
                        start_x, start_y = x, y
                        cmd = "L"
                    cur_x, cur_y = x, y
                    coords.append((cur_x, cur_y))
                elif cmd in ("m", "l"):
                    dx, dy = float(tokens[i]), float(tokens[i + 1])
                    i += 2
                    if cmd == "m":
                        start_x, start_y = cur_x + dx, cur_y + dy
                        cmd = "l"
                    cur_x, cur_y = cur_x + dx, cur_y + dy
                    coords.append((cur_x, cur_y))
                elif cmd == "H":
                    cur_x = float(tokens[i]); i += 1
                    coords.append((cur_x, cur_y))
                elif cmd == "h":
                    cur_x += float(tokens[i]); i += 1
                    coords.append((cur_x, cur_y))
                elif cmd == "V":
                    cur_y = float(tokens[i]); i += 1
                    coords.append((cur_x, cur_y))
                elif cmd == "v":
                    cur_y += float(tokens[i]); i += 1
                    coords.append((cur_x, cur_y))
                else:
                    i += 1   # 未知命令的数字跳过
            except (ValueError, IndexError):
                break
        return coords

    for el in root.iter():
        tag = el.tag.split("}")[-1].lower()
        if tag == "line":
            try:
                x1 = float(el.get("x1", "0"))
                y1 = float(el.get("y1", "0"))
                x2 = float(el.get("x2", "0"))
                y2 = float(el.get("y2", "0"))
                walls_xy.append((x1, y1, x2, y2))
            except ValueError:
                continue
        elif tag in ("polyline", "polygon"):
            pts_str = el.get("points", "").replace(",", " ")
            try:
                vals = [float(t) for t in pts_str.split() if t]
            except ValueError:
                continue
            if len(vals) < 4 or len(vals) % 2 != 0:
                continue
            coords = [(vals[i], vals[i + 1]) for i in range(0, len(vals), 2)]
            if tag == "polygon" and coords[0] != coords[-1]:
                coords.append(coords[0])
            add_segments(coords)
        elif tag == "path":
            d_str = el.get("d", "")
            coords = parse_path_d(d_str)
            if len(coords) >= 2:
                add_segments(coords)
        elif tag == "rect":
            try:
                x = float(el.get("x", "0"))
                y = float(el.get("y", "0"))
                w = float(el.get("width", "0"))
                h = float(el.get("height", "0"))
                if w > 0 and h > 0:
                    add_segments([
                        (x, y), (x + w, y), (x + w, y + h),
                        (x, y + h), (x, y),
                    ])
            except ValueError:
                continue

    if not walls_xy:
        raise CadDispatchError(
            "SVG 中没找到任何 <line>/<polyline>/<polygon>/<path>/<rect>。"
            "本解析器支持基本几何元素与 M/L/H/V/Z 路径命令；"
            "若你的 SVG 仅含曲线 (C/Q/A)，请改用 PNG 模式上传。"
        )

    # SVG y 轴向下；我们项目世界坐标 y 向上。这里直接保留 SVG 坐标系，
    # 只是将原点 (0,0) 视作米——用户对绝对方向无要求时无影响。
    xs = [v for w in walls_xy for v in (w[0], w[2])]
    ys = [v for w in walls_xy for v in (w[1], w[3])]
    min_x, max_x = min(xs), max(xs)
    min_y, max_y = min(ys), max(ys)

    # 自动归一化：若图很大（> 50），按比例缩到 ~10m 量级
    raw_w = max_x - min_x
    raw_h = max_y - min_y
    if max(raw_w, raw_h) > 50:
        scale = 10.0 / max(raw_w, raw_h)
    else:
        scale = 1.0

    # 把所有点平移到正象限并应用 scale → 得到米单位的墙线
    walls_m = [
        (
            (x1 - min_x) * scale,
            (y1 - min_y) * scale,
            (x2 - min_x) * scale,
            (y2 - min_y) * scale,
        )
        for x1, y1, x2, y2 in walls_xy
    ]

    width_m = (max_x - min_x) * scale
    height_m = (max_y - min_y) * scale
    pad_m = padding_m
    H = max(2, int(np.ceil((height_m + 2 * pad_m) / resolution)))
    W = max(2, int(np.ceil((width_m + 2 * pad_m) / resolution)))
    grid = np.ones((H, W), dtype=np.uint8)
    origin = (-pad_m, -pad_m)

    from skimage.draw import line as bresenham_line  # noqa: PLC0415

    half_thk_px = max(1, int(wall_thickness_m / (2 * resolution)))

    def to_pix(x_m: float, y_m: float) -> Tuple[int, int]:
        col = int(round((x_m - origin[0]) / resolution))
        row = int(round((y_m - origin[1]) / resolution))
        return row, col

    for x1, y1, x2, y2 in walls_m:
        r1, c1 = to_pix(x1, y1)
        r2, c2 = to_pix(x2, y2)
        rr, cc = bresenham_line(r1, c1, r2, c2)
        for dr in range(-half_thk_px, half_thk_px + 1):
            for dc in range(-half_thk_px, half_thk_px + 1):
                rrs = np.clip(rr + dr, 0, H - 1)
                ccs = np.clip(cc + dc, 0, W - 1)
                grid[rrs, ccs] = 0

    return {
        "mode": "svg",
        "grid": grid,
        "transform": {
            "origin": origin,
            "resolution": resolution,
            "shape": (H, W),
        },
        "cad_data": {
            "_mode": "svg",
            "_n_segments": len(walls_xy),
            "_scale_factor": scale,
        },
        "source_path": str(path),
        "has_semantics": False,
        "note": (
            f"SVG 抽取了 {len(walls_xy)} 条墙段。"
            "无对象/门/窗语义，Demo 2 无 nearest_objects 词汇可参考。"
        ),
    }


# ══════════════════════════════════════════════════════════════════════
# PNG / JPG：二值化当作占据图；不识别任何语义
# ══════════════════════════════════════════════════════════════════════

def _parse_raster(
    path: Path,
    resolution: float,
    padding_m: float,
    invert: bool = False,
    threshold: int = 128,
) -> Dict[str, Any]:
    """
    PNG → occupancy grid。

    约定：
      - 默认深色像素 (< threshold) 视为墙/障碍 (grid=0)，浅色为自由空间 (grid=1)。
      - 若你的 CAD 图反色（白底黑墙），保持 invert=False；
        若是黑底白墙，传 invert=True。

    注意：PIL 已是 numpy 项目的标配传递依赖；本函数仅用其 ``open`` + ``convert("L")``。
    """
    try:
        from PIL import Image  # noqa: PLC0415
    except ImportError as e:
        raise CadDispatchError(
            "PNG 解析需要 Pillow，请运行 `pip install pillow`。"
        ) from e

    try:
        img = Image.open(path)
    except Exception as e:
        raise CadDispatchError(f"PNG 读取失败：{e}") from e

    # ① RGBA → 在白底上 alpha-composite，再灰度化
    # 否则透明像素 (alpha=0) 会被 .convert("L") 强转成 0，
    # 让 FloorplanCAD 等"线稿+透明背景"的 PNG 99% 像素全黑。
    if img.mode in ("RGBA", "LA") or (
        img.mode == "P" and "transparency" in img.info
    ):
        bg = Image.new("RGB", img.size, (255, 255, 255))
        bg.paste(img.convert("RGBA"), mask=img.convert("RGBA").split()[-1])
        img = bg
    img = img.convert("L")
    arr = np.asarray(img, dtype=np.uint8)

    # ② 鲁棒二值化：用户传入 threshold=128 是固定阈值，
    # 但若图像主色严重偏向一侧（e.g. 极淡灰墙），固定阈值会全黑/全白。
    # 这里加一个 Otsu 兜底：仅在固定阈值失败时启用。
    if invert:
        binary = (arr < threshold).astype(np.uint8)
    else:
        binary = (arr >= threshold).astype(np.uint8)

    free_ratio = binary.mean()
    if free_ratio < 0.001 or free_ratio > 0.999:
        # 固定阈值崩了 → 尝试 Otsu
        try:
            import cv2  # noqa: PLC0415
            _, otsu_bin = cv2.threshold(
                arr, 0, 1, cv2.THRESH_BINARY + cv2.THRESH_OTSU,
            )
            binary = otsu_bin.astype(np.uint8)
            if invert:
                binary = 1 - binary
            free_ratio = binary.mean()
        except ImportError:
            pass

    if free_ratio < 0.001 or free_ratio > 0.999:
        raise CadDispatchError(
            f"二值化后图像几乎全黑或全白（threshold={threshold}, "
            f"free_ratio={free_ratio:.4f}）。"
            f"图像 min={arr.min()} max={arr.max()} mean={arr.mean():.1f}。"
            "请勾选 PNG 反色 或上传线条更清晰的 CAD 图。"
        )

    H, W = binary.shape
    # PNG 模式不加 padding：用户上传的图片就是完整地图，
    # 再加 padding 会让距离场在边界外的 padding 区取到最大值，
    # 导致候选起终点全部偏向四角。
    origin = (0.0, 0.0)

    return {
        "mode": "png",
        "grid": binary,
        "transform": {
            "origin": origin,
            "resolution": resolution,
            "shape": (H, W),
        },
        "cad_data": {
            "_mode": "png",
            "_pixel_shape": list(arr.shape),
            "_assumed_resolution": resolution,
        },
        "source_path": str(path),
        "has_semantics": False,
        "note": (
            "PNG 经二值化得到 occupancy grid（这不是图像识别）。"
            "无对象/门/窗语义；Demo 2 仅可手填 nearest_objects。"
            f"假设 resolution={resolution} m/px（如尺度不对请调整）。"
        ),
    }


# ══════════════════════════════════════════════════════════════════════
# DWG：LibreDWG 工业解析 → 双轨分发（几何 + 语义）
# ══════════════════════════════════════════════════════════════════════

# ── 图层白名单：仅保留墙体/结构相关图层参与栅格化 ──
# 工业 DWG 含大量家具/标注/管线图层，不加过滤会导致 OOM 和 A* 连通性崩溃。
# 关键词匹配忽略大小写，命中任一关键词即视为墙体图层。
WALL_LAYER_KEYWORDS = [
    # ── 英文 ──
    "wall", "struc", "col", "beam", "slab",
    "door", "window",
    # ── 中文（建筑/结构常用图层命名） ──
    "墙", "建", "构", "柱", "承重",
    "门", "窗",
]

# 当 JSON 超过此阈值（字节数）时，强制开启严格图层过滤模式
LARGE_JSON_THRESHOLD_BYTES = 50 * 1024 * 1024  # 50 MB


def _is_wall_layer(layer_name: str) -> bool:
    """判断图层名是否匹配墙体/结构关键词（忽略大小写）。"""
    if not layer_name:
        return False
    lower = layer_name.lower()
    return any(kw in lower for kw in WALL_LAYER_KEYWORDS)


_DWG_UNIT_TO_M: Dict[int, float] = {
    1: 0.0254, 2: 0.3048, 4: 0.001, 5: 0.01, 6: 1.0, 8: 0.0254,
}

# LibreDWG type_code → entity name
_DWG_TYPE_MAP: Dict[int, str] = {
    # ── 几何实体（参与栅格化） ──
    18: "CIRCLE",  19: "LINE",      20: "SPLINE",
    21: "LWPOLYLINE", 22: "POLYLINE2D", 23: "POLYLINE3D",
    41: "ELLIPSE", 45: "ARC",
    # ── 语义实体（Track B 消费） ──
    1: "TEXT",     2: "ATTDEF",     3: "ATTRIB",
    7: "INSERT",  13: "POINT",
    44: "MTEXT",
    # ── 可能含边界 Loop 的实体（可选几何提取） ──
    50: "HATCH",
}
# 显式标记为非图形/簿记类型：遇到时静默跳过，不触发 "unknown entity" warning
_DWG_SKIP_TYPES: set[int] = {
    4, 5, 6,          # BLOCK_HEADER / ENDBLK / SEQEND
    8, 9, 10, 11, 12, # VERTEX 子实体（附属于 POLYLINE，不独立处理）
    14, 15, 16, 17,   # DIMENSION 族（标注线，非墙体）
    24, 25, 26,       # DIMENSION_RADIUS / DIAMETER / LEADER
    34, 35, 36,       # 更多 DIMENSION 变体
    42,               # 字典/样式表
    46, 47, 48, 49,   # ARCALIGNEDTEXT / HELIX / TABLE / GEOMAPIMAGE
    51, 52,           # MLEADER / MLEADERSTYLE 相关
    76, 77, 78, 79,   # 管理对象 / Section / 自定义
    507, 509,         # 应用程序自定义对象
}


def _find_entities(obj: Any, depth: int = 0) -> List[Dict[str, Any]]:
    """遍历 LibreDWG JSON OBJECTS 列表收集图形实体。

    LibreDWG 实体有 ``entity`` 字段（如 LINE/CIRCLE/TEXT/INSERT），
    跳过 VX_CONTROL / BLOCK_HEADER / END_BLK 等簿记对象。

    对无法识别的数字 type_code：静默跳过（标记为 UNKNOWN_OR_SKIP），
    不阻断解析流水线。
    """
    entities: List[Dict[str, Any]] = []
    if depth > 5:
        return entities
    object_list: Any = None
    if isinstance(obj, dict):
        object_list = obj.get("OBJECTS", obj.get("objects"))
    if object_list is None and isinstance(obj, list):
        object_list = obj
    if isinstance(object_list, list):
        for item in object_list:
            if not isinstance(item, dict):
                continue
            ent_name = item.get("entity", "")
            if ent_name and ent_name not in ("VX_CONTROL", "BLOCK_HEADER", "END_BLK"):
                entities.append(item)
            elif not ent_name:
                tc = item.get("type")
                if isinstance(tc, int):
                    if tc in _DWG_TYPE_MAP:
                        item = dict(item)
                        item["entity"] = _DWG_TYPE_MAP[tc]
                        entities.append(item)
                    elif tc in _DWG_SKIP_TYPES:
                        # 已知的非图形类型（DIMENSION / TABLE / 自定义对象）— 静默跳过
                        continue
                    else:
                        # 未识别类型码 — 标记后静默跳过，保证流水线不中断
                        pass
    return entities


def _dwg_point_to_xy(pt: Any, unit_scale: float = 1.0) -> Tuple[float, float]:
    if isinstance(pt, dict):
        return float(pt.get("x", 0)) * unit_scale, float(pt.get("y", 0)) * unit_scale
    if isinstance(pt, (list, tuple)) and len(pt) >= 2:
        return float(pt[0]) * unit_scale, float(pt[1]) * unit_scale
    return 0.0, 0.0


def _extract_dwg_geometry(
    entities: List[Dict[str, Any]],
    unit_scale: float = 1.0,
    wall_layer_filter: bool = False,
) -> Tuple[List[Tuple[Tuple[float, float], Tuple[float, float]]], List[Dict[str, Any]], int]:
    """从 DWG 实体列表提取墙体线段和多边形。

    当 ``wall_layer_filter=True`` 时，仅保留图层名匹配
    ``WALL_LAYER_KEYWORDS`` 的实体，家具/标注/管线等冗余图层直接丢弃。
    这能将 320MB 级工业图纸的参与数据量压缩到几 MB。

    Returns:
        (walls, polygons, skipped_count)
    """
    walls: List[Tuple[Tuple[float, float], Tuple[float, float]]] = []
    polygons: List[Dict[str, Any]] = []
    skipped: int = 0

    for ent in entities:
        # LibreDWG 用 entity 字段（字符串）标识类型
        etype = str(ent.get("entity", ent.get("type", ""))).upper()
        # 去掉可能的 AcDb 前缀（如 AcDbLine → LINE）
        if etype.startswith("ACDB"):
            etype = etype[4:]

        # ── 图层过滤 ──
        layer = str(ent.get("layer", ""))
        if isinstance(ent.get("layer"), list):
            layer = "layer_" + "_".join(str(x) for x in ent["layer"])
        if wall_layer_filter and not _is_wall_layer(layer):
            skipped += 1
            continue

        if etype in ("LINE", "19"):
            s = _dwg_point_to_xy(ent.get("start"), unit_scale)
            e = _dwg_point_to_xy(ent.get("end"), unit_scale)
            if s != e:
                walls.append((s, e))

        elif etype in ("LWPOLYLINE", "POLYLINE", "POLYLINE2D", "POLYLINE3D", "21", "22", "23"):
            vertices = ent.get("vertices", ent.get("points", []))
            if isinstance(vertices, list) and len(vertices) >= 2:
                pts = [_dwg_point_to_xy(v, unit_scale) for v in vertices]
                is_closed = ent.get("is_closed", ent.get("closed", False))
                if isinstance(is_closed, str):
                    is_closed = is_closed.lower() in ("true", "1", "yes")
                for i in range(len(pts) - 1):
                    if pts[i] != pts[i + 1]:
                        walls.append((pts[i], pts[i + 1]))
                if is_closed and pts[0] != pts[-1]:
                    walls.append((pts[-1], pts[0]))
                polygons.append({"label": f"polyline_{layer}" if layer else "polyline",
                                 "points": pts, "layer": layer})

        elif etype in ("CIRCLE", "18"):
            cx, cy = _dwg_point_to_xy(ent.get("center"), unit_scale)
            r = float(ent.get("radius", 0)) * unit_scale
            if r > 0:
                n_seg = 24
                circle_pts: List[Tuple[float, float]] = []
                for i in range(n_seg):
                    angle = 2 * math.pi * i / n_seg
                    circle_pts.append((cx + r * math.cos(angle), cy + r * math.sin(angle)))
                for i in range(n_seg):
                    j = (i + 1) % n_seg
                    walls.append((circle_pts[i], circle_pts[j]))
                polygons.append({"label": f"circle_{layer}" if layer else "circle",
                                 "points": circle_pts, "layer": layer})

        elif etype in ("ARC", "45"):
            cx, cy = _dwg_point_to_xy(ent.get("center"), unit_scale)
            r = float(ent.get("radius", 0)) * unit_scale
            start_a = float(ent.get("start_angle", 0))
            end_a = float(ent.get("end_angle", math.pi * 2))
            if r > 0:
                n_seg = max(4, int(abs(end_a - start_a) / (math.pi / 6)))
                arc_pts: List[Tuple[float, float]] = []
                for i in range(n_seg + 1):
                    a = start_a + (end_a - start_a) * i / n_seg
                    arc_pts.append((cx + r * math.cos(a), cy + r * math.sin(a)))
                for i in range(len(arc_pts) - 1):
                    walls.append((arc_pts[i], arc_pts[i + 1]))

        elif etype in ("ELLIPSE", "41"):
            # ── 椭圆离散化 ──
            # DWG 椭圆定义：中心点 C + 长轴向量 major + 短轴比例 radius_ratio
            # 参数方程 P(t) = C + A*cos(t) + B*sin(t), t ∈ [0, 2π]
            # 其中 A = major_axis 向量, B = 垂直于 A 且长度 = |A| * radius_ratio
            cx, cy = _dwg_point_to_xy(ent.get("center"), unit_scale)
            # major_axis 是从中心到长轴端点的相对向量
            major = ent.get("major_axis", ent.get("major", {}))
            ax, ay = _dwg_point_to_xy(major, unit_scale)
            # 将 major_axis 的绝对坐标转为相对于中心的向量
            ax -= cx
            ay -= cy
            major_len = math.hypot(ax, ay)
            if major_len <= 0:
                continue
            # 短轴比例：minor_radius / major_radius
            ratio = float(ent.get("radius_ratio", ent.get("minor_ratio", 1.0)))
            # 短轴向量 = 长轴向量逆时针旋转 90° 后乘以 ratio
            bx = -ay * ratio
            by = ax * ratio
            # 36 段采样（与 CIRCLE 的 24 段相比更精细以匹配椭圆曲率变化）
            n_seg = 36
            ellipse_pts: List[Tuple[float, float]] = []
            for i in range(n_seg):
                t = 2.0 * math.pi * i / n_seg
                ct, st = math.cos(t), math.sin(t)
                px = cx + ax * ct + bx * st
                py = cy + ay * ct + by * st
                ellipse_pts.append((px, py))
            for i in range(n_seg):
                j = (i + 1) % n_seg
                walls.append((ellipse_pts[i], ellipse_pts[j]))
            polygons.append({"label": f"ellipse_{layer}" if layer else "ellipse",
                             "points": ellipse_pts, "layer": layer})

        elif etype in ("SPLINE", "20"):
            # ── 样条曲线离散化（控制点直线连接降级方案） ──
            # 真实 SPLINE 由控制点 + 节点向量 + 阶数定义。
            # 为计算效率，暂用"控制点顺序直线插值"将其降维为 POLYLINE。
            # 当控制点密度 ≥ 原曲线采样率时，直线近似精度可接受。
            ctrl_pts_raw = ent.get("control_points",
                           ent.get("ctrl_pts",
                           ent.get("fit_points",
                           ent.get("points", []))))
            if isinstance(ctrl_pts_raw, list) and len(ctrl_pts_raw) >= 2:
                pts = [_dwg_point_to_xy(p, unit_scale) for p in ctrl_pts_raw]
                is_closed = ent.get("is_closed", ent.get("closed", False))
                if isinstance(is_closed, str):
                    is_closed = is_closed.lower() in ("true", "1", "yes")
                for i in range(len(pts) - 1):
                    if pts[i] != pts[i + 1]:
                        walls.append((pts[i], pts[i + 1]))
                if is_closed and pts[0] != pts[-1]:
                    walls.append((pts[-1], pts[0]))
                polygons.append({"label": f"spline_{layer}" if layer else "spline",
                                 "points": pts, "layer": layer})

    return walls, polygons, skipped


def _extract_dwg_semantics(entities: List[Dict[str, Any]]) -> Dict[str, Any]:
    layers: set = set()
    texts: List[Dict[str, Any]] = []
    blocks: List[Dict[str, Any]] = []
    summary = {"lines": 0, "polylines": 0, "circles": 0, "arcs": 0,
               "ellipses": 0, "splines": 0, "texts": 0, "blocks": 0}

    for ent in entities:
        # LibreDWG 用 entity 字段标识类型
        etype = str(ent.get("entity", ent.get("type", ""))).upper()
        # 去掉可能的 AcDb 前缀（如 AcDbLine → LINE）
        if etype.startswith("ACDB"):
            etype = etype[4:]
        layer = str(ent.get("layer", "0"))
        # layer 可能是 handle 引用数组，简单字符串化
        if isinstance(ent.get("layer"), list):
            layer = "layer_" + "_".join(str(x) for x in ent["layer"])
        if layer:
            layers.add(layer)

        if etype in ("LINE", "19"):
            summary["lines"] += 1
        elif etype in ("LWPOLYLINE", "POLYLINE", "POLYLINE2D", "POLYLINE3D", "21", "22", "23"):
            summary["polylines"] += 1
        elif etype in ("CIRCLE", "18"):
            summary["circles"] += 1
        elif etype in ("ARC", "45"):
            summary["arcs"] += 1
        elif etype in ("ELLIPSE", "41"):
            summary["ellipses"] += 1
        elif etype in ("SPLINE", "20"):
            summary["splines"] += 1
        elif etype in ("TEXT", "MTEXT", "ATTRIB", "ATTDEF", "1", "44"):
            summary["texts"] += 1
            text_val = str(ent.get("text_value", ent.get("text", ent.get("contents", ""))))
            if text_val.strip():
                pos = _dwg_point_to_xy(
                    ent.get("ins_pt", ent.get("insertion_point", ent.get("position"))), 1.0,
                )
                texts.append({
                    "value": text_val.strip(),
                    "position": list(pos),
                    "layer": layer,
                    "height": float(ent.get("height", 0)),
                })
        elif etype in ("INSERT", "7"):
            summary["blocks"] += 1
            pos = _dwg_point_to_xy(
                ent.get("ins_pt", ent.get("insertion_point", ent.get("position"))), 1.0,
            )
            blocks.append({
                "name": str(ent.get("name", ent.get("block_name", ""))
                           or f"block_ref_{ent.get('block_header', '?')}"),
                "position": list(pos),
                "layer": layer,
            })

    return {
        "layers": sorted(layers),
        "texts": texts,
        "blocks": blocks,
        "entity_summary": summary,
    }


def _parse_dwg(
    path: Path,
    resolution: float,
    padding_m: float,
    wall_thickness_m: float,
) -> Dict[str, Any]:
    from cad_parser.dwg_ingestion import parse_dwg_to_json  # noqa: PLC0415

    ingestion = parse_dwg_to_json(str(path))
    if ingestion["status"] != "ok":
        raise CadDispatchError(f"DWG 解析失败：{ingestion['message']}")

    dwg_json = ingestion["json"]
    dwg_version = ingestion.get("dwg_version", "unknown")

    entities = _find_entities(dwg_json.get("OBJECTS", dwg_json))
    if not entities:
        raise CadDispatchError(
            "DWG 文件中未找到任何可识别实体（LINE/POLYLINE/CIRCLE/TEXT/INSERT）。"
            f" JSON 顶层键: {list(dwg_json.keys())[:10]}"
        )

    # ── 大文件内存防御 ──
    raw_size = ingestion.get("raw_size_bytes", 0)
    force_layer_filter = raw_size > LARGE_JSON_THRESHOLD_BYTES
    if force_layer_filter:
        import warnings
        warnings.warn(
            f"DWG JSON 输出 {raw_size / 1_048_576:.0f} MB，"
            f"超过 {LARGE_JSON_THRESHOLD_BYTES // 1_048_576} MB 阈值。"
            f"强制开启图层白名单过滤（关键词: {WALL_LAYER_KEYWORDS}），"
            f"家具/标注/管线等冗余图层将被丢弃以避免 OOM。"
        )

    unit_scale = 1.0
    try:
        header_vars = dwg_json.get("header", dwg_json.get("HEADER", {}))
        insunits = header_vars.get("INSUNITS", header_vars.get("$INSUNITS", 0))
        insunits = int(insunits) if insunits else 0
        unit_scale = _DWG_UNIT_TO_M.get(insunits, 1.0)
    except (TypeError, ValueError):
        pass

    walls, polygons, skipped = _extract_dwg_geometry(
        entities, unit_scale, wall_layer_filter=force_layer_filter,
    )
    if not walls:
        raise CadDispatchError("DWG 中未提取到任何墙体几何。")

    semantics = _extract_dwg_semantics(entities)

    xs = [v for (sx, sy), (ex, ey) in walls for v in (sx, ex)]
    ys = [v for (sx, sy), (ex, ey) in walls for v in (sy, ey)]
    bbox = (min(xs) if xs else 0.0, min(ys) if ys else 0.0,
            max(xs) if xs else 10.0, max(ys) if ys else 10.0)

    # DWG 没有显式 room_boundary → 用 wall bbox 外扩 padding_m 构建
    boundary_poly: List[Tuple[float, float]] = [
        (bbox[0] - padding_m, bbox[1] - padding_m),
        (bbox[2] + padding_m, bbox[1] - padding_m),
        (bbox[2] + padding_m, bbox[3] + padding_m),
        (bbox[0] - padding_m, bbox[3] + padding_m),
    ]

    cad_data: Dict[str, Any] = {
        "layout_id": path.stem,
        "room_type": "dwg_industrial",
        "units": "meters",
        "boundary": boundary_poly,
        "walls": walls,
        "doors": [],
        "windows": [],
        "objects": polygons,
        "bbox": bbox,
        "_dwg_version": dwg_version,
        "_dwg_unit_scale": unit_scale,
        "_dwg_entity_count": len(entities),
        "_layer_filter_enabled": force_layer_filter,
        "_layer_filter_skipped": skipped,
    }

    # 自动降采样：超 4000px 时增大 resolution 避免 OOM
    effective_res = resolution
    max_dim = max(
        (bbox[2] - bbox[0] + 2 * padding_m) / resolution,
        (bbox[3] - bbox[1] + 2 * padding_m) / resolution,
    )
    if max_dim > 4000:
        effective_res = resolution * (max_dim / 2000.0)

    grid, transform = rasterize_to_grid(
        cad_data, resolution=effective_res,
        padding_m=padding_m, wall_thickness_m=wall_thickness_m,
    )

    return {
        "mode": "dwg",
        "grid": grid,
        "transform": transform,
        "cad_data": cad_data,
        "semantics": semantics,
        "source_path": str(path),
        "has_semantics": True,
        "note": (
            f"DWG {dwg_version}，{len(walls)} 段墙线，"
            f"{len(polygons)} 个多边形，"
            f"{len(semantics['layers'])} 图层，"
            f"{len(semantics['texts'])} 文本，"
            f"{len(semantics['blocks'])} 块参照。"
            + (
                f" 图层过滤: {'强制' if force_layer_filter else '未'}开启"
                f"（跳过 {skipped} 个非墙体图层实体）。"
                if force_layer_filter else ""
            )
            + " AI 语义流已就绪，可调用 semantic_extractor 提取业务节点。"
        ),
    }


# ══════════════════════════════════════════════════════════════════════
# 主入口
# ══════════════════════════════════════════════════════════════════════

def dispatch_cad(
    path: str,
    resolution: float = 0.05,
    padding_m: float = 0.5,
    wall_thickness_m: float = 0.10,
    png_threshold: int = 128,
    png_invert: bool = False,
) -> Dict[str, Any]:
    """
    根据文件扩展名走对应 parser，统一产出 ParseResult 字典。

    Raises:
        CadDispatchError: 文件不存在 / 格式不支持 / 解析失败。
    """
    p = Path(path)
    if not p.exists():
        raise CadDispatchError(f"文件不存在：{path}")

    ext = p.suffix.lower()
    if ext in JSON_EXTS:
        result = _parse_json(p, resolution, padding_m, wall_thickness_m)
        result["grid"] = weld_double_line_walls(
            result["grid"], resolution=resolution, max_gap_m=0.30,
        )
        result["grid"] = remove_exterior_freespace(result["grid"], close_gaps_px=1)
        result["grid"] = bridge_thin_walls(result["grid"], kernel_px=2)
        return result
    if ext in DWG_EXTS:
        result = _parse_dwg(p, resolution, padding_m, wall_thickness_m)
        # 双线墙焊接必须在 remove_exterior_freespace 之前执行：
        # 先焊死双线墙缝隙，防止外部泛洪从墙缝漏入建筑内部
        result["grid"] = weld_double_line_walls(
            result["grid"], resolution=result["transform"]["resolution"],
            max_gap_m=0.30,
        )
        result["grid"] = remove_exterior_freespace(result["grid"], close_gaps_px=2)
        result["grid"] = bridge_thin_walls(result["grid"], kernel_px=2)
        return result
    if ext in SVG_EXTS:
        result = _parse_svg(p, resolution, padding_m, wall_thickness_m)
        result["grid"] = weld_double_line_walls(
            result["grid"], resolution=resolution, max_gap_m=0.30,
        )
        # SVG: grid 初始全为 1，外部泛洪最严重；门往往是 1~2px 空缺
        result["grid"] = remove_exterior_freespace(
            result["grid"], close_gaps_px=2,
        )
        result["grid"] = bridge_thin_walls(result["grid"], kernel_px=2)
        return result
    if ext in RASTER_EXTS:
        result = _parse_raster(
            p, resolution, padding_m,
            invert=png_invert, threshold=png_threshold,
        )
        result["grid"] = weld_double_line_walls(
            result["grid"], resolution=resolution, max_gap_m=0.30,
        )
        # PNG: 白色背景会泄漏；扫描图常有锯齿墙体 + 门虚线
        result["grid"] = remove_exterior_freespace(
            result["grid"], close_gaps_px=2,
        )
        result["grid"] = bridge_thin_walls(result["grid"], kernel_px=3)
        return result

    raise CadDispatchError(
        f"不支持的扩展名：{ext}。"
        f"支持的格式：JSON {sorted(JSON_EXTS)} / "
        f"DWG {sorted(DWG_EXTS)} / "
        f"SVG {sorted(SVG_EXTS)} / "
        f"栅格 {sorted(RASTER_EXTS)}。"
    )


def supported_formats_info() -> str:
    """返回 Markdown 表格说明四种格式的能力差异。"""
    return (
        "| 格式 | 能力 | Demo 2 语义 | Demo 3 物理求解 |\n"
        "|------|------|-------------|------------------|\n"
        "| **JSON** (FloorplanQA) | 完整：墙/门/家具 | ✅ 可作词汇表 | ✅ |\n"
        "| **DWG** (LibreDWG) | 工业级：双轨提取 | ✅ AI 语义流 | ✅ |\n"
        "| **SVG** (line/polyline) | 仅边界 | ❌ 无对象语义 | ✅ |\n"
        "| **PNG/JPG** (二值化) | 仅占据栅格 | ❌ 无对象语义 | ✅ |\n"
    )


def get_optional_seeds_world(
    parse_result: Dict[str, Any],
    distance_field: Optional[np.ndarray] = None,
    robot_radius_px: float = 0.0,
) -> List[Tuple[float, float]]:
    """
    给非 JSON 模式（PNG/SVG）派生默认种子点：
    取距离场最大的 1~3 个像素作为安全自由空间中心。

    JSON 模式直接走原 ``extract_door_seeds`` 路径，不调本函数。
    """
    if distance_field is None:
        return []
    df = distance_field.astype(np.float32)
    tx = parse_result["transform"]
    res = tx["resolution"]
    ox, oy = tx["origin"]
    H, W = df.shape

    flat = df.ravel()
    n = max(1, min(3, flat.size))
    top_idx = np.argpartition(flat, -n)[-n:]
    seeds_world: List[Tuple[float, float]] = []
    for idx in top_idx:
        if df.flat[idx] < max(1.0, robot_radius_px):
            continue
        r, c = divmod(int(idx), W)
        x = ox + c * res
        y = oy + r * res
        seeds_world.append((x, y))
    if not seeds_world:
        seeds_world.append((ox + W * res / 2, oy + H * res / 2))
    return seeds_world
