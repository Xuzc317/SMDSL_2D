"""
astar_topology.py — Demo 1: CAD 矢量平面图 → 拓扑地图

实现的四步流水线（按 PROJECT_CONTEXT 的强约束顺序）：

  1. 栅格化与二值化
     - 加载 FloorplanQA JSON（room_boundary / walls / objects / openings）
     - 渲染为 H×W 占据栅格：walls / objects / windows = 0；门 / 自由空间 = 1。

  2. 距离场生成
     - 使用 scipy.ndimage.distance_transform_edt 计算自由像素到最近墙的
       欧氏距离（像素单位），得到 Distance Map。

  3. 安全代价 A* 漫水
     - 节点代价 f(n) = g(n) + h(n)。
     - 若 distance_map[n] < R_robot_px，则该节点为 +∞（物理防穿透）。
     - 多源 Dijkstra（启发式 h≡0）漫水探测 + visit count 热力图。

  4. 拓扑图元提取
     - A* 频繁经过的连通区域 → CLASS_PATH
     - distance_map < R_robot 或不可达 → CLASS_OBSTACLE
     - 留有 CLASS_LOADING 接口（上料 / 下料位置）。
"""

from __future__ import annotations

import heapq
import json
import math
from pathlib import Path
from typing import Any, Dict, List, Optional, Tuple

import numpy as np
from scipy import ndimage
from skimage.draw import line as bresenham_line
from skimage.draw import polygon as draw_polygon


# ── 语义类别常量 ───────────────────────────────────────
CLASS_UNKNOWN = 0       # 未标注 / 不可达
CLASS_LOADING = 1       # 上料 / 下料位置（预留）
CLASS_PATH = 2          # 道路 / 可通行区域
CLASS_OBSTACLE = 3      # 障碍物 / 墙体（含物理膨胀失败区）
CLASS_INFLATED = 4      # 距离场 < R_robot 的“可视但不可走”膨胀缓冲区

CLASS_NAMES: Dict[int, str] = {
    CLASS_UNKNOWN: "unknown",
    CLASS_LOADING: "loading_zone",
    CLASS_PATH: "path",
    CLASS_OBSTACLE: "obstacle",
    CLASS_INFLATED: "inflated_buffer",
}


# ══════════════════════════════════════════════════════
# 1. 矢量数据加载
# ══════════════════════════════════════════════════════

def load_cad_vector(cad_path: str) -> Dict[str, Any]:
    """
    加载 FloorplanQA 矢量平面图 JSON。

    支持的 schema（参见 floorplanqa README）::

        {
            "layout_id": int,
            "room_type": str,
            "room_boundary": [{x,y}, ...],
            "walls": [{"start": {x,y}, "end": {x,y}}, ...],
            "openings": {
                "windows": [{"label": str, "points": [{x,y}, ...]}, ...],
                "doors":   [{"label": str, "points": [{x,y}, ...]}, ...]
            },
            "objects": [{"label": str, "points": [{x,y}, ...]}, ...],
            "units": "meters"
        }

    Args:
        cad_path: JSON 文件路径。

    Returns:
        归一化后的字典，含 ``boundary``、``walls``、``doors``、
        ``windows``、``objects``、``bbox``、``units`` 字段。
        ``bbox`` 为 (min_x, min_y, max_x, max_y)，单位米。
    """
    p = Path(cad_path)
    if not p.is_file():
        raise FileNotFoundError(f"CAD path not a file: {cad_path}")

    raw = json.loads(p.read_text(encoding="utf-8"))
    units = raw.get("units", "meters")

    def _pt(d: Dict[str, float]) -> Tuple[float, float]:
        return float(d["x"]), float(d["y"])

    boundary: List[Tuple[float, float]] = [
        _pt(v) for v in raw.get("room_boundary", [])
    ]
    walls: List[Tuple[Tuple[float, float], Tuple[float, float]]] = [
        (_pt(w["start"]), _pt(w["end"])) for w in raw.get("walls", [])
    ]
    openings = raw.get("openings", {})
    windows: List[Dict[str, Any]] = [
        {"label": w.get("label", "window"),
         "points": [_pt(p) for p in w["points"]]}
        for w in openings.get("windows", [])
    ]
    doors: List[Dict[str, Any]] = [
        {"label": d.get("label", "door"),
         "points": [_pt(p) for p in d["points"]]}
        for d in openings.get("doors", [])
    ]
    objects: List[Dict[str, Any]] = [
        {"label": o.get("label", "object"),
         "points": [_pt(p) for p in o["points"]]}
        for o in raw.get("objects", [])
    ]

    # bbox 取所有顶点的并集
    all_pts: List[Tuple[float, float]] = list(boundary)
    for s, e in walls:
        all_pts.extend([s, e])
    for grp in (windows, doors, objects):
        for item in grp:
            all_pts.extend(item["points"])
    if not all_pts:
        raise ValueError(f"layout 不含任何顶点: {cad_path}")
    xs, ys = zip(*all_pts)
    bbox = (min(xs), min(ys), max(xs), max(ys))

    return {
        "layout_id": raw.get("layout_id"),
        "room_type": raw.get("room_type"),
        "units": units,
        "boundary": boundary,
        "walls": walls,
        "doors": doors,
        "windows": windows,
        "objects": objects,
        "bbox": bbox,
    }


# ══════════════════════════════════════════════════════
# 2. 栅格化（矢量 → 二值占据栅格）
# ══════════════════════════════════════════════════════

def _world_to_grid(
    pt_xy: Tuple[float, float],
    origin: Tuple[float, float],
    resolution: float,
) -> Tuple[int, int]:
    """米 → 像素 (row, col)。

    注意图像坐标系约定: ``row = y_idx``、``col = x_idx``，
    Y 轴朝下；rasterize 时整体翻转一次，使输出图与 CAD 视图同向。
    """
    ox, oy = origin
    col = int(round((pt_xy[0] - ox) / resolution))
    row = int(round((pt_xy[1] - oy) / resolution))
    return row, col


def _polygon_to_pixels(
    poly_xy: List[Tuple[float, float]],
    origin: Tuple[float, float],
    resolution: float,
    grid_shape: Tuple[int, int],
) -> Tuple[np.ndarray, np.ndarray]:
    """多边形 → 内部填充像素 (rr, cc)，已裁剪到 grid 边界。"""
    rows, cols = zip(*[
        _world_to_grid(pt, origin, resolution) for pt in poly_xy
    ])
    rr, cc = draw_polygon(
        np.array(rows), np.array(cols), shape=grid_shape
    )
    return rr, cc


def rasterize_to_grid(
    cad_data: Dict[str, Any],
    resolution: float = 0.05,
    padding_m: float = 0.5,
    wall_thickness_m: float = 0.10,
) -> Tuple[np.ndarray, Dict[str, Any]]:
    """
    将矢量 CAD 数据栅格化为二值占据图。

    像素语义（per spec）：
      - 0 = 占据（墙、家具、窗）
      - 1 = 自由空间（包括门洞）

    渲染顺序（后写入覆盖前写入）：
      1) 全图填充为 0（默认占据），用于"边界外即障碍"语义；
      2) boundary 多边形内部置 1（房间内自由空间）；
      3) walls 线段以 wall_thickness 加粗后置 0；
      4) objects 多边形置 0（家具/设备视为障碍）；
      5) windows 多边形置 0（不可走）；
      6) doors 多边形置 1（门洞可通行，覆盖被墙截断的部分）。

    Args:
        cad_data: ``load_cad_vector`` 的返回。
        resolution: 米/像素。
        padding_m: 在 bbox 四周扩展的物理空间（米），便于外缘机器人转向。
        wall_thickness_m: 墙体物理厚度（米）。

    Returns:
        (grid, transform) 二元组：
          - grid: ``np.ndarray[H, W] uint8``，0=占据 / 1=自由。
          - transform: dict，含 ``origin`` (min_x, min_y)、``resolution``、
            ``shape`` (H, W)、``bbox_world``。
    """
    min_x, min_y, max_x, max_y = cad_data["bbox"]
    min_x -= padding_m
    min_y -= padding_m
    max_x += padding_m
    max_y += padding_m

    width_m = max_x - min_x
    height_m = max_y - min_y
    W = max(1, int(math.ceil(width_m / resolution)))
    H = max(1, int(math.ceil(height_m / resolution)))
    origin = (min_x, min_y)

    grid = np.zeros((H, W), dtype=np.uint8)  # 0 = obstacle by default

    # 2) boundary → 自由空间 (1)
    if cad_data["boundary"]:
        rr, cc = _polygon_to_pixels(
            cad_data["boundary"], origin, resolution, (H, W)
        )
        grid[rr, cc] = 1

    # 3) 墙体线段 (0)，按 wall_thickness 加粗
    wall_radius_px = max(1, int(round(
        (wall_thickness_m / 2.0) / resolution
    )))
    for (sx, sy), (ex, ey) in cad_data["walls"]:
        r0, c0 = _world_to_grid((sx, sy), origin, resolution)
        r1, c1 = _world_to_grid((ex, ey), origin, resolution)
        r0 = int(np.clip(r0, 0, H - 1))
        r1 = int(np.clip(r1, 0, H - 1))
        c0 = int(np.clip(c0, 0, W - 1))
        c1 = int(np.clip(c1, 0, W - 1))
        rr, cc = bresenham_line(r0, c0, r1, c1)
        grid[rr, cc] = 0
        # 用 ndimage.binary_dilation 反过来代价太高，这里直接局部小圆刷一遍
        for dr in range(-wall_radius_px, wall_radius_px + 1):
            for dc in range(-wall_radius_px, wall_radius_px + 1):
                if dr * dr + dc * dc > wall_radius_px * wall_radius_px:
                    continue
                rr2 = np.clip(rr + dr, 0, H - 1)
                cc2 = np.clip(cc + dc, 0, W - 1)
                grid[rr2, cc2] = 0

    # 4) objects → 0
    for obj in cad_data["objects"]:
        rr, cc = _polygon_to_pixels(
            obj["points"], origin, resolution, (H, W)
        )
        grid[rr, cc] = 0

    # 5) windows → 0
    for win in cad_data["windows"]:
        rr, cc = _polygon_to_pixels(
            win["points"], origin, resolution, (H, W)
        )
        grid[rr, cc] = 0

    # 6) doors → 1 (覆盖被墙截断的部分)
    for door in cad_data["doors"]:
        rr, cc = _polygon_to_pixels(
            door["points"], origin, resolution, (H, W)
        )
        grid[rr, cc] = 1

    transform = {
        "origin": origin,
        "resolution": resolution,
        "shape": (H, W),
        "bbox_world": (min_x, min_y, max_x, max_y),
        "wall_thickness_m": wall_thickness_m,
        "padding_m": padding_m,
    }
    return grid, transform


# ══════════════════════════════════════════════════════
# 2b. 外部自由空间剔除（解决"外部泛洪" Bug）
# ══════════════════════════════════════════════════════

def remove_exterior_freespace(
    grid: np.ndarray,
    close_gaps_px: int = 2,
) -> np.ndarray:
    """
    消除栅格四边缘以外的自由空间，彻底解决"外部泛洪（Exterior Flooding）"Bug。

    **问题根因**：
    - SVG：grid 初始化为全 1（自由），墙线画为 0；外部自由空间与室内完全同值。
    - PNG：图片白色背景直接二值化为 free space；外部白边与走廊同值。
    - JSON：grid 初始化为全 0，boundary 内部填 1；理论上外部为 0，
      但 padding 边缘可能出现漏缝（boundary 多边形细锯齿穿越 padding 区）。

    **算法（连通域边界剔除法）**：

    ① 形态学闭运算（可选）
       对障碍层（0 像素）做闭运算（先膨胀再腐蚀），
       封闭 ``close_gaps_px`` 像素以内的墙体缺口，
       防止外部自由空间从细缝"漏入"建筑内部。

    ② 连通域分析
       用 ``scipy.ndimage.label`` 对 free space（==1）做 4-连通分析，
       标记所有独立连通域。

    ③ 边界接触识别
       遍历栅格四边（top / bottom / left / right）的所有像素，
       收集其连通域 ID，即"外部连通域"集合。

    ④ 覆写外部
       将所有外部连通域的像素强制置为 0（障碍），
       让 EDT 和 A* 只能在真实建筑内部运行。

    Args:
        grid:          H×W uint8，0=占据 / 1=自由。不修改原数组，返回副本。
        close_gaps_px: 闭运算核半径（像素）。0 = 跳过闭运算。
                       对线宽 1px 的 SVG 墙体，推荐 2；
                       对 JSON 矢量图，通常 1 即可。

    Returns:
        处理后的栅格副本，外部自由空间已变为实体墙（0）。
    """
    g = grid.copy().astype(np.uint8)
    H, W = g.shape

    # ── ① 形态学闭运算 ──────────────────────────────────────────────
    # 对"障碍层"（0 像素 → 取反后为 1）做闭运算：
    #   膨胀障碍（= 腐蚀自由空间）→ 腐蚀障碍（= 膨胀自由空间）
    # 净效果：填充 ≤ close_gaps_px 宽的墙体裂缝，防止外部泄入。
    if close_gaps_px > 0:
        obstacle_layer = (g == 0).astype(np.uint8)
        try:
            import cv2  # noqa: PLC0415
            k = 2 * close_gaps_px + 1
            kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (k, k))
            # MORPH_CLOSE on obstacle layer = fill small holes in walls
            closed_obstacle = cv2.morphologyEx(
                obstacle_layer, cv2.MORPH_CLOSE, kernel,
            )
            g = (1 - closed_obstacle).astype(np.uint8)
        except ImportError:
            # cv2 不可用 → scipy 替代：对障碍层做 binary_closing
            struct = ndimage.generate_binary_structure(2, 1)  # 4-连通十字
            closed_obstacle = ndimage.binary_closing(
                obstacle_layer.astype(bool),
                structure=struct,
                iterations=close_gaps_px,
            ).astype(np.uint8)
            g = (1 - closed_obstacle).astype(np.uint8)

    # ── ② 连通域分析 ──────────────────────────────────────────────
    free_mask = g == 1
    if not free_mask.any():
        return g  # 全是障碍，直接返回

    # 4-连通（上下左右），不对角；防止两个独立房间通过 1px 对角被合并
    struct_4 = ndimage.generate_binary_structure(2, 1)
    labeled, _n = ndimage.label(free_mask, structure=struct_4)

    # ── ③ 边界接触识别 ──────────────────────────────────────────────
    # 四条边缘的所有像素的连通域 ID 合集 = 外部连通域
    border_ids: set = set()
    border_ids.update(labeled[0, :].tolist())       # top
    border_ids.update(labeled[-1, :].tolist())      # bottom
    border_ids.update(labeled[:, 0].tolist())       # left
    border_ids.update(labeled[:, -1].tolist())      # right
    border_ids.discard(0)   # 0 = 障碍像素，不是连通域编号，排除

    if not border_ids:
        return g  # 无外部连通域，返回（闭合建筑，纯内部自由空间）

    # ── ④ 覆写外部像素 ───────────────────────────────────────────────
    # np.isin 一次性标记所有属于外部连通域的像素
    exterior_mask = np.isin(labeled, list(border_ids))

    # 安全护栏：若覆写后内部自由空间彻底消失（< 0.5% 原 free pixels），
    # 说明输入图本身没有封闭建筑边界（典型如线稿 PNG / 开放 SVG），
    # 强行剔除外部反而会把整张图清零。这种情况保留原 grid，让下游
    # 看到"全是自由空间"——比"全是墙"更安全。
    n_free_before = int((g == 1).sum())
    n_free_after = int(((g == 1) & ~exterior_mask).sum())
    if n_free_before > 0 and n_free_after / n_free_before < 0.005:
        return g

    g[exterior_mask] = 0
    return g


# ══════════════════════════════════════════════════════
# 2c. 形态学破壁：合并被门 / 细线分割的房间
# ══════════════════════════════════════════════════════

def bridge_thin_walls(
    grid: np.ndarray,
    kernel_px: int = 2,
) -> np.ndarray:
    """
    用形态学闭运算消灭 ≤ ``kernel_px`` 像素宽的"门框线 / 隔断细线"，
    让本应连通的相邻房间在拓扑上合并。

    **数学动作**（在 free 像素 ==1 的 mask 上）::

        bridged_free = morph_close(free_mask, kernel) = erode(dilate(free_mask))

    - 膨胀：让自由空间向墙体扩张 ``kernel_px`` 像素 →
      若两房间间隔 ≤ 2·kernel_px 像素，它们的自由区在此步触碰相连。
    - 腐蚀：再将自由空间收缩 ``kernel_px`` 像素 → 主体房间形状基本复原，
      但被"打通"的薄墙已经永久消失（因为它在膨胀时被自由像素吞噬，
      腐蚀时附近仍有自由像素填充）。

    这等价于"在自由空间的连通图上做小核闭运算"，是 OpenCV 处理
    破损轮廓的标准技巧 (cv2.MORPH_CLOSE)。

    Args:
        grid:      H×W uint8，0=占据 / 1=自由（应已先经过 ``remove_exterior_freespace``）。
        kernel_px: 闭运算椭圆核半径（像素）。
                   - ``0``: 跳过破壁；
                   - ``1``: 仅打通 1px 标注线；
                   - ``2`` (默认): 打通典型门框 (2~4px)；
                   - ``≥3``: 也会吞噬较粗的隔断，慎用。

    Returns:
        破壁后的栅格副本。墙体不再独立——保留的是『安全凸壳后的自由空间』。
    """
    if kernel_px <= 0:
        return grid.copy()

    free_mask = (grid == 1).astype(np.uint8)
    k = 2 * kernel_px + 1
    try:
        import cv2  # noqa: PLC0415
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (k, k))
        bridged_free = cv2.morphologyEx(
            free_mask, cv2.MORPH_CLOSE, kernel,
        )
    except ImportError:
        struct = ndimage.generate_binary_structure(2, 2)  # 8-连通
        bridged_free = ndimage.binary_closing(
            free_mask.astype(bool),
            structure=struct,
            iterations=kernel_px,
        ).astype(np.uint8)

    return bridged_free.astype(np.uint8)


# ══════════════════════════════════════════════════════
# 2d. 双线墙焊接：合并工业 DWG 里的「双线墙缝隙」
# ══════════════════════════════════════════════════════

def weld_double_line_walls(
    grid: np.ndarray,
    resolution: float,
    max_gap_m: float = 0.30,
) -> np.ndarray:
    """
    把工业 DWG 里典型的「双线墙」之间的物理缝隙焊接成实心墙。

    **问题根因**：
    工业建筑 / 工厂 CAD 中，墙体通常用 *两条平行线段* 绘制（外皮 + 内皮），
    两线之间是真实物理墙厚度（典型 10~30 cm），栅格化时落在中间的像素
    会被识别为「自由空间」而不是「占据」。结果：
      - EDT 把墙体内部当作可达；
      - A* 漫水可能从墙缝穿过；
      - STL Distance 求解器拿到错误的 d_real_m。

    **算法（障碍层闭运算）**：
    对 *障碍层*（grid == 0 取反）做形态学闭运算：

        closed_obstacle = morph_close(obstacle_layer, ellipse_kernel)
                        = erode(dilate(obstacle_layer))

    - 膨胀：障碍向外扩张 ``radius_px`` 像素 → 两条平行墙线在中间相连；
    - 腐蚀：障碍再收缩 ``radius_px`` 像素 → 主体墙厚基本复原，但
      被焊接的「双线之间的内腔」永久变为障碍（因膨胀阶段已被填实）。

    与 ``bridge_thin_walls`` 的对偶关系::

        bridge_thin_walls       : MORPH_CLOSE on **free**     mask（破壁）
        weld_double_line_walls  : MORPH_CLOSE on **obstacle** mask（焊缝）

    核大小动态计算::

        radius_px = ceil(max_gap_m / resolution / 2)
        kernel    = ELLIPSE(2*radius_px + 1)

    例如 ``max_gap_m=0.30 m, resolution=0.05 m/px`` →
    ``radius_px = ceil(0.30 / 0.05 / 2) = 3 px``，核大小 7×7。

    Args:
        grid:        H×W uint8，0=占据 / 1=自由（通常来自 ``rasterize_to_grid``，
                     **建议先于 ``remove_exterior_freespace`` 调用**，否则
                     外部自由空间会让闭运算把整片建筑也焊死）。
        resolution:  米/像素，决定 ``max_gap_m`` 换算到栅格的半径。
        max_gap_m:   要焊接的双线墙最大物理缝隙宽度（米）。默认 0.30 m
                     覆盖大多数工业墙厚；CNC 厂房可设 0.50 m。
                     ``≤ 0`` 直接返回输入副本。

    Returns:
        焊接后的栅格副本（uint8，语义不变：0=占据 / 1=自由）。

    Note:
        本函数不修改原数组。运行时优先用 ``cv2``（毫秒级），
        缺失时用 ``scipy.ndimage.binary_closing`` 兜底（仍是 O(H·W·k²)
        但常数大，500×500 栅格 + k=7 大约 30~80 ms）。
    """
    if max_gap_m <= 0 or resolution <= 0:
        return grid.copy()

    radius_px = max(1, int(math.ceil(max_gap_m / resolution / 2.0)))
    k = 2 * radius_px + 1

    obstacle_layer = (grid == 0).astype(np.uint8)
    try:
        import cv2  # noqa: PLC0415
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (k, k))
        welded_obstacle = cv2.morphologyEx(
            obstacle_layer, cv2.MORPH_CLOSE, kernel,
        )
    except ImportError:
        struct = ndimage.generate_binary_structure(2, 2)  # 8-连通
        welded_obstacle = ndimage.binary_closing(
            obstacle_layer.astype(bool),
            structure=struct,
            iterations=radius_px,
        ).astype(np.uint8)

    welded = (1 - welded_obstacle).astype(np.uint8)
    return welded


# ══════════════════════════════════════════════════════
# 3. 距离场
# ══════════════════════════════════════════════════════

def compute_distance_field(grid: np.ndarray) -> np.ndarray:
    """
    计算每个自由像素到最近占据像素的欧氏距离（像素单位）。

    Args:
        grid: ``rasterize_to_grid`` 的二值输出，0=占据 / 1=自由。

    Returns:
        与 grid 同形的 float32 数组。
        在占据像素上值为 0；在自由像素上为到最近墙的欧氏距离（px）。
    """
    # distance_transform_edt: 对 True/非零像素，返回到最近 0 的距离
    dist = ndimage.distance_transform_edt(grid.astype(bool))
    return dist.astype(np.float32)


# ══════════════════════════════════════════════════════
# 3b. 全局拓扑分类（矩阵化 — 替代 BFS 漫水）
# ══════════════════════════════════════════════════════

def classify_topology_global(
    grid: np.ndarray,
    distance_field: np.ndarray,
    robot_radius_px: float,
) -> Tuple[np.ndarray, Dict[str, Any]]:
    """
    **纯矩阵**拓扑分类（无 BFS 漫水，O(N) 一次扫描，典型 < 5ms）。

    分类规则::

        topology[grid == 0]                                 = CLASS_OBSTACLE
        topology[(grid == 1) & (df <  robot_radius_px)]    = CLASS_INFLATED
        topology[(grid == 1) & (df >= robot_radius_px)]    = CLASS_PATH

    然后对 CLASS_PATH 像素做 8-连通分量分析，作为可走区域的
    "拓扑诚实度"指标返回，便于诊断房间是否仍被薄墙分割。

    Args:
        grid:            ``rasterize_to_grid`` + ``remove_exterior_freespace``
                         + （可选）``bridge_thin_walls`` 之后的二值栅格。
        distance_field:  对同一 ``grid`` 计算的 EDT 距离场。
        robot_radius_px: 机器人物理半径（像素）。

    Returns:
        ``(topology, stats)``：
          - ``topology``: H×W uint8，含 ``CLASS_*`` 常量；
          - ``stats``: 含::
              {
                "n_components": int,            # CLASS_PATH 的连通分量数
                "largest_component_frac": float,# 最大连通区占可走总像素的比例
                "component_sizes": List[int],   # 各连通区像素数 (降序)
                "n_path_px": int,
                "n_inflated_px": int,
                "n_obstacle_px": int,
              }
    """
    H, W = grid.shape
    topology = np.full((H, W), CLASS_UNKNOWN, dtype=np.uint8)

    obstacle_mask = (grid == 0)
    inflated_mask = (grid == 1) & (distance_field < robot_radius_px)
    path_mask = (grid == 1) & (distance_field >= robot_radius_px)

    topology[obstacle_mask] = CLASS_OBSTACLE
    topology[inflated_mask] = CLASS_INFLATED
    topology[path_mask] = CLASS_PATH

    # 连通分量分析（仅在 CLASS_PATH 上做 8-连通）
    struct_8 = ndimage.generate_binary_structure(2, 2)
    labeled, n_components = ndimage.label(path_mask, structure=struct_8)

    if n_components > 0:
        sizes = np.bincount(labeled.ravel())
        sizes = sizes[1:]  # 去掉背景 (label==0)
        sizes_sorted = np.sort(sizes)[::-1].tolist()
        total_path_px = int(sizes.sum())
        largest_frac = (int(sizes.max()) / total_path_px) if total_path_px else 0.0
    else:
        sizes_sorted = []
        largest_frac = 0.0

    stats: Dict[str, Any] = {
        "n_components": int(n_components),
        "largest_component_frac": float(largest_frac),
        "component_sizes": [int(s) for s in sizes_sorted[:10]],  # 只列前 10
        "n_path_px": int(path_mask.sum()),
        "n_inflated_px": int(inflated_mask.sum()),
        "n_obstacle_px": int(obstacle_mask.sum()),
    }
    return topology, stats


# ══════════════════════════════════════════════════════
# 4. 安全代价 A* 漫水（带 R_robot 防穿透）
# ══════════════════════════════════════════════════════

# 8 邻域偏移与代价
_NEIGHBORS: Tuple[Tuple[int, int, float], ...] = (
    (-1, 0, 1.0), (1, 0, 1.0), (0, -1, 1.0), (0, 1, 1.0),
    (-1, -1, math.sqrt(2)), (-1, 1, math.sqrt(2)),
    (1, -1, math.sqrt(2)), (1, 1, math.sqrt(2)),
)


def safety_aware_astar_flood(
    grid: np.ndarray,
    distance_field: np.ndarray,
    seed_points: List[Tuple[int, int]],
    robot_radius_px: float,
    max_steps: int = 2_000_000,
    safety_weight: float = 0.5,
) -> Tuple[np.ndarray, np.ndarray]:
    """
    带物理半径防穿透的多源 A* / Dijkstra 漫水探测。

    代价函数::

        g(n) = g(parent) + step_cost(neighbor)
             + safety_weight * (1 / max(1, distance_field[n]))

        f(n) = g(n) + h(n)         # h ≡ 0 → 退化为 Dijkstra；
                                   # 漫水阶段不需要单一目标的启发式。

        如果 distance_field[n] < robot_radius_px:
            cost(n) = +∞    # 物理防穿透：膨胀缓冲区禁止机器人进入

    Args:
        grid: 二值占据栅格（0=墙, 1=自由）。
        distance_field: 每个像素到最近墙的距离（像素）。
        seed_points: 种子点列表，每个为 ``(row, col)``。
        robot_radius_px: 机器人物理半径（像素）。
        max_steps: 最大扩展节点数（安全保险）。
        safety_weight: 1/distance 安全奖励的权重；置 0 即纯几何 Dijkstra。

    Returns:
        (topology, visit_count) 二元组：
          - topology: ``uint8 (H, W)`` 含 CLASS_* 常量；
          - visit_count: ``uint32 (H, W)`` 每像素被加入开放集的次数。
    """
    H, W = grid.shape
    topology = np.full((H, W), CLASS_UNKNOWN, dtype=np.uint8)
    visit_count = np.zeros((H, W), dtype=np.uint32)
    g_score = np.full((H, W), np.inf, dtype=np.float32)

    # 墙体先验标注
    topology[grid == 0] = CLASS_OBSTACLE
    # 安全膨胀区先验标注（距离 < 半径 但本身是自由空间）
    inflated_mask = (grid == 1) & (distance_field < robot_radius_px)
    topology[inflated_mask] = CLASS_INFLATED

    # 优先队列
    pq: List[Tuple[float, int, int]] = []
    for sy, sx in seed_points:
        if not (0 <= sy < H and 0 <= sx < W):
            continue
        if grid[sy, sx] == 0:
            continue
        if distance_field[sy, sx] < robot_radius_px:
            # 种子落在膨胀缓冲区 — 跳过；调用方应改投有效种子
            continue
        g_score[sy, sx] = 0.0
        heapq.heappush(pq, (0.0, sy, sx))
        topology[sy, sx] = CLASS_PATH

    steps = 0
    while pq and steps < max_steps:
        cost, y, x = heapq.heappop(pq)
        if cost > g_score[y, x]:
            continue  # 过期堆元素
        visit_count[y, x] += 1
        steps += 1

        for dy, dx, step_cost in _NEIGHBORS:
            ny, nx = y + dy, x + dx
            if not (0 <= ny < H and 0 <= nx < W):
                continue
            if grid[ny, nx] == 0:
                topology[ny, nx] = CLASS_OBSTACLE
                continue
            d_val = distance_field[ny, nx]
            if d_val < robot_radius_px:
                topology[ny, nx] = CLASS_INFLATED
                continue
            # 对角线 corner-cutting 防止穿墙
            if dx != 0 and dy != 0:
                if grid[y, nx] == 0 or grid[ny, x] == 0:
                    continue
                if distance_field[y, nx] < robot_radius_px:
                    continue
                if distance_field[ny, x] < robot_radius_px:
                    continue

            safety_penalty = safety_weight / max(1.0, float(d_val))
            tentative = cost + step_cost + safety_penalty
            if tentative < g_score[ny, nx]:
                g_score[ny, nx] = tentative
                topology[ny, nx] = CLASS_PATH
                heapq.heappush(pq, (tentative, ny, nx))

    return topology, visit_count


# ══════════════════════════════════════════════════════
# 4b. 两点 A* 最短路径（Demo 1 起终点联动 Demo 3 用）
# ══════════════════════════════════════════════════════

def astar_shortest_path(
    grid: np.ndarray,
    distance_field: np.ndarray,
    start_rc: Tuple[int, int],
    goal_rc: Tuple[int, int],
    robot_radius_px: float,
    safety_weight: float = 0.5,
    max_steps: int = 2_000_000,
) -> Optional[List[Tuple[int, int]]]:
    """
    在带物理半径防穿透的栅格上求 start → goal 的 A* 最短路径。

    与 ``safety_aware_astar_flood`` 共用代价函数，但启发式 h(n) = 欧氏距离
    （admissible），因此这是真正的 A*（不是 Dijkstra）。

    Args:
        grid: 二值占据栅格（0=墙, 1=自由）。
        distance_field: EDT 距离场。
        start_rc: 起点 (row, col)。
        goal_rc: 终点 (row, col)。
        robot_radius_px: 机器人物理半径（像素）。
        safety_weight: 1/distance 安全奖励权重。
        max_steps: 最大节点扩展次数。
    Returns:
        像素坐标列表 ``[(r, c), ...]``（含起点与终点），或 ``None`` 表示不可达。
    """
    H, W = grid.shape
    sy, sx = start_rc
    gy, gx = goal_rc

    if not (0 <= sy < H and 0 <= sx < W):
        return None
    if not (0 <= gy < H and 0 <= gx < W):
        return None
    if grid[sy, sx] == 0 or grid[gy, gx] == 0:
        return None
    if distance_field[sy, sx] < robot_radius_px:
        return None
    if distance_field[gy, gx] < robot_radius_px:
        return None

    g_score = np.full((H, W), np.inf, dtype=np.float32)
    g_score[sy, sx] = 0.0
    came_from: Dict[Tuple[int, int], Tuple[int, int]] = {}
    pq: List[Tuple[float, int, int]] = []

    def heuristic(y: int, x: int) -> float:
        return math.hypot(y - gy, x - gx)

    heapq.heappush(pq, (heuristic(sy, sx), sy, sx))

    steps = 0
    while pq and steps < max_steps:
        _, y, x = heapq.heappop(pq)
        if (y, x) == (gy, gx):
            path = [(y, x)]
            cur: Tuple[int, int] = (y, x)
            while cur in came_from:
                cur = came_from[cur]
                path.append(cur)
            path.reverse()
            return path
        steps += 1
        for dy, dx, step_cost in _NEIGHBORS:
            ny, nx = y + dy, x + dx
            if not (0 <= ny < H and 0 <= nx < W):
                continue
            # 硬性拦截：墙
            if grid[ny, nx] == 0:
                continue
            # 硬性拦截：距离不足 → 物理碰撞
            d_val = distance_field[ny, nx]
            if d_val < robot_radius_px:
                continue
            # 硬性拦截：对角线 corner-cutting 防止穿墙
            if dx != 0 and dy != 0:
                if grid[y, nx] == 0 or grid[ny, x] == 0:
                    continue
                if distance_field[y, nx] < robot_radius_px:
                    continue
                if distance_field[ny, x] < robot_radius_px:
                    continue
            safety_penalty = safety_weight / max(1.0, float(d_val))
            tentative_g = g_score[y, x] + step_cost + safety_penalty
            if tentative_g < g_score[ny, nx]:
                g_score[ny, nx] = tentative_g
                came_from[(ny, nx)] = (y, x)
                f = tentative_g + heuristic(ny, nx)
                heapq.heappush(pq, (f, ny, nx))

    return None


def path_pixels_to_trajectory(
    path_rc: List[Tuple[int, int]],
    transform: Dict[str, Any],
    total_time_s: float = 5.0,
    z: float = 0.0,
    yaw_rad: float = 0.0,
    sample_step: int = 4,
) -> List[Dict[str, float]]:
    """
    把像素路径下采样并换算为世界坐标轨迹（米 + 时间）。

    返回 [{t, x, y, z, roll, pitch, yaw}, ...] 与 Demo 3 的接口对齐。
    """
    if not path_rc:
        return []
    origin = transform["origin"]
    res = transform["resolution"]

    sampled = path_rc[::max(1, int(sample_step))]
    if sampled[-1] != path_rc[-1]:
        sampled.append(path_rc[-1])

    n = len(sampled)
    if n == 1:
        ts = [0.0]
    else:
        ts = [total_time_s * i / (n - 1) for i in range(n)]

    traj: List[Dict[str, float]] = []
    for (r, c), t in zip(sampled, ts):
        traj.append({
            "t": round(t, 3),
            "x": round(origin[0] + c * res, 4),
            "y": round(origin[1] + r * res, 4),
            "z": z,
            "roll": 0.0,
            "pitch": 0.0,
            "yaw": yaw_rad,
        })
    return traj


# ══════════════════════════════════════════════════════
# 5. 距离场梯度路径微调 (Phase 2.2)
# ══════════════════════════════════════════════════════

def compute_field_gradient(
    distance_field: np.ndarray,
) -> Tuple[np.ndarray, np.ndarray]:
    """计算 EDT 梯度场 ∇d = (∂d/∂x, ∂d/∂y)。使用 np.gradient 中心差分。"""
    dy, dx = np.gradient(distance_field.astype(np.float64))
    return dx, dy


def refine_path_via_gradient(
    path_rc: List[Tuple[int, int]],
    distance_field: np.ndarray,
    gradient_field: Tuple[np.ndarray, np.ndarray],
    robot_radius_px: float,
    iterations: int = 5,
    step_size: float = 0.5,
) -> List[Tuple[int, int]]:
    """
    梯度微调：沿 ∇d（远离障碍物方向）微调路径内部点。
    保持起点终点不变；保证新位置不穿墙（d >= robot_radius_px）。
    """
    if len(path_rc) < 3:
        return path_rc[:]

    gx, gy = gradient_field
    H, W = distance_field.shape
    refined = path_rc[:]
    req_clearance = max(1.0, robot_radius_px)

    for _ in range(iterations):
        new_path = [refined[0]]
        for i in range(1, len(refined) - 1):
            r, c = refined[i]
            # 离散索引 clamp
            ri = max(0, min(H - 1, int(round(r))))
            ci = max(0, min(W - 1, int(round(c))))
            dr = gy[ri, ci] * step_size
            dc = gx[ri, ci] * step_size
            nr = r + dr
            nc = c + dc
            ri_new = max(0, min(H - 1, int(round(nr))))
            ci_new = max(0, min(W - 1, int(round(nc))))
            if distance_field[ri_new, ci_new] >= req_clearance:
                new_path.append((nr, nc))
            else:
                new_path.append((r, c))
        new_path.append(refined[-1])
        refined = new_path

    return [(int(round(r)), int(round(c))) for r, c in refined]


# ══════════════════════════════════════════════════════
# 6. 种子点 / 上料位接口
# ══════════════════════════════════════════════════════

def extract_door_seeds(
    cad_data: Dict[str, Any],
    transform: Dict[str, Any],
    distance_field: Optional[np.ndarray] = None,
    robot_radius_px: float = 0.0,
) -> List[Tuple[int, int]]:
    """
    将门洞多边形质心转为像素种子点。若没有门，退化为 boundary 质心。

    若提供 ``distance_field`` + ``robot_radius_px``，会自动把种子推到
    最近的安全像素（避免落在门框膨胀区）。
    """
    origin = transform["origin"]
    resolution = transform["resolution"]
    H, W = transform["shape"]

    candidates: List[Tuple[int, int]] = []

    doors = cad_data.get("doors", [])
    if doors:
        for door in doors:
            pts = door["points"]
            cx = sum(p[0] for p in pts) / len(pts)
            cy = sum(p[1] for p in pts) / len(pts)
            r, c = _world_to_grid((cx, cy), origin, resolution)
            if 0 <= r < H and 0 <= c < W:
                candidates.append((r, c))
    else:
        boundary = cad_data.get("boundary", [])
        if boundary:
            cx = sum(p[0] for p in boundary) / len(boundary)
            cy = sum(p[1] for p in boundary) / len(boundary)
            r, c = _world_to_grid((cx, cy), origin, resolution)
            if 0 <= r < H and 0 <= c < W:
                candidates.append((r, c))

    if distance_field is None or robot_radius_px <= 0:
        return candidates

    # 把每个种子推到 R_robot 之外的最近自由像素
    safe_mask = distance_field >= robot_radius_px
    if not safe_mask.any():
        return candidates  # 整张图都没安全像素，原样返回让调用方报错

    rr_safe, cc_safe = np.where(safe_mask)
    safe_xy = np.stack([rr_safe, cc_safe], axis=1)
    refined: List[Tuple[int, int]] = []
    for r, c in candidates:
        d2 = (safe_xy[:, 0] - r) ** 2 + (safe_xy[:, 1] - c) ** 2
        idx = int(np.argmin(d2))
        refined.append((int(safe_xy[idx, 0]), int(safe_xy[idx, 1])))
    return refined


def mark_loading_zone(
    topology: np.ndarray,
    x: int, y: int,
    radius: int = 3,
) -> None:
    """标记 (x, y) 周围 radius 像素为上料/下料区域（CLASS_LOADING）。"""
    H, W = topology.shape
    y0, y1 = max(0, y - radius), min(H, y + radius + 1)
    x0, x1 = max(0, x - radius), min(W, x + radius + 1)
    sub = topology[y0:y1, x0:x1]
    # 只覆盖可通行区域，不抹掉墙
    sub[(sub == CLASS_PATH) | (sub == CLASS_UNKNOWN)] = CLASS_LOADING


# ══════════════════════════════════════════════════════
# 6. JSON 拓扑地图输出
# ══════════════════════════════════════════════════════

def extract_topology_json(
    topology: np.ndarray,
    visit_count: np.ndarray,
    distance_field: Optional[np.ndarray] = None,
    transform: Optional[Dict[str, Any]] = None,
    seeds: Optional[List[Tuple[int, int]]] = None,
    robot_radius_m: Optional[float] = None,
    include_grid: bool = True,
) -> Dict[str, Any]:
    """
    把标签图 + 元信息序列化为可被下游 SMDSL / Rust 消费的 JSON。

    Args:
        topology: ``safety_aware_astar_flood`` 输出。
        visit_count: A* 访问频次热力图。
        distance_field: 可选；用于报告"最小 clearance"等指标。
        transform: ``rasterize_to_grid`` 返回的 transform 字典。
        seeds: A* 起始点（像素坐标）。
        robot_radius_m: 机器人物理半径（米），写入 metadata。
        include_grid: 是否在 JSON 中嵌入 H×W 的标签矩阵。
                      H、W > 数百时会很大，可设 False 仅留统计量。

    Returns:
        JSON 兼容字典。
    """
    H, W = topology.shape
    unique, counts = np.unique(topology, return_counts=True)
    class_pixels = {
        CLASS_NAMES.get(int(k), f"class_{int(k)}"): int(v)
        for k, v in zip(unique, counts)
    }
    total_visits = int(visit_count.sum())

    transform = transform or {}
    metadata: Dict[str, Any] = {
        "width_pixels": int(W),
        "height_pixels": int(H),
        "resolution_m_per_px": float(transform.get("resolution", 1.0)),
        "origin_world": {
            "x": float(transform.get("origin", (0.0, 0.0))[0]),
            "y": float(transform.get("origin", (0.0, 0.0))[1]),
        },
        "bbox_world": transform.get("bbox_world"),
        "robot_radius_m": robot_radius_m,
        "wall_thickness_m": transform.get("wall_thickness_m"),
        "padding_m": transform.get("padding_m"),
    }

    summary: Dict[str, Any] = {
        "class_pixels": class_pixels,
        "total_a_star_visits": total_visits,
        "n_seeds": len(seeds) if seeds else 0,
    }
    if distance_field is not None:
        free_mask = topology != CLASS_OBSTACLE
        summary["min_clearance_px"] = float(
            distance_field[free_mask].min() if free_mask.any() else 0.0
        )
        summary["max_clearance_px"] = float(distance_field.max())
        if metadata["resolution_m_per_px"]:
            summary["max_clearance_m"] = (
                summary["max_clearance_px"]
                * metadata["resolution_m_per_px"]
            )

    out: Dict[str, Any] = {
        "metadata": metadata,
        "classes": {
            str(k): {"name": CLASS_NAMES[k], "id": k}
            for k in sorted(CLASS_NAMES)
        },
        "summary": summary,
        "seeds_pixel": [
            {"row": int(r), "col": int(c)} for r, c in (seeds or [])
        ],
    }

    if include_grid:
        out["grid"] = topology.astype(int).tolist()
        # heatmap 用对数压缩，否则 visit_count 可能爆数量级
        heat = np.log1p(visit_count.astype(np.float64))
        if heat.max() > 0:
            heat = (heat / heat.max() * 1000.0).astype(int)
        out["heatmap_log_x1000"] = heat.tolist()

    return out


# ══════════════════════════════════════════════════════
# 7. 端到端便捷封装
# ══════════════════════════════════════════════════════

def run_pipeline(
    cad_path: str,
    resolution: float = 0.05,
    robot_radius_m: float = 0.30,
    padding_m: float = 0.5,
    wall_thickness_m: float = 0.10,
    safety_weight: float = 0.5,
    max_steps: int = 2_000_000,
    seeds_world: Optional[List[Tuple[float, float]]] = None,
    include_grid: bool = True,
) -> Dict[str, Any]:
    """
    Demo 1 端到端：CAD 路径 → 拓扑 JSON。

    用例::

        result = run_pipeline(
            "data/cad_samples/floorplanqa/layouts/kitchen/room_0.json",
            resolution=0.02, robot_radius_m=0.25,
        )

    Returns:
        含 ``cad_data``、``grid``、``distance_field``、``topology``、
        ``visit_count``、``seeds``、``topology_json`` 全部中间产物的字典。
    """
    cad_data = load_cad_vector(cad_path)
    grid, transform = rasterize_to_grid(
        cad_data,
        resolution=resolution,
        padding_m=padding_m,
        wall_thickness_m=wall_thickness_m,
    )
    distance_field = compute_distance_field(grid)

    robot_radius_px = robot_radius_m / resolution

    if seeds_world is not None:
        seeds = []
        for x, y in seeds_world:
            r, c = _world_to_grid(
                (x, y), transform["origin"], resolution,
            )
            if 0 <= r < transform["shape"][0] and 0 <= c < transform["shape"][1]:
                seeds.append((r, c))
    else:
        seeds = extract_door_seeds(
            cad_data, transform,
            distance_field=distance_field,
            robot_radius_px=robot_radius_px,
        )

    topology, visit_count = safety_aware_astar_flood(
        grid, distance_field, seeds,
        robot_radius_px=robot_radius_px,
        max_steps=max_steps,
        safety_weight=safety_weight,
    )

    topology_json = extract_topology_json(
        topology, visit_count,
        distance_field=distance_field,
        transform=transform,
        seeds=seeds,
        robot_radius_m=robot_radius_m,
        include_grid=include_grid,
    )

    return {
        "cad_data": cad_data,
        "grid": grid,
        "distance_field": distance_field,
        "topology": topology,
        "visit_count": visit_count,
        "transform": transform,
        "seeds": seeds,
        "robot_radius_px": robot_radius_px,
        "topology_json": topology_json,
    }


# ══════════════════════════════════════════════════════
# 8. Demo 3 桥接：TopologyBundle 适配器
# ══════════════════════════════════════════════════════

def to_topology_bundle(pipeline_result: Dict[str, Any]) -> Dict[str, Any]:
    """
    将 ``run_pipeline()`` 的完整产物精简为 Demo 3 STL 求解所需的紧凑包：

        {
            "distance_field": np.ndarray (H, W) float32,  # 像素单位
            "grid_transform": {
                "origin": (min_x, min_y),                 # 米
                "resolution": float,                       # 米/像素
                "shape": (H, W),
            },
            "robot_radius_m": float,
            "robot_radius_px": float,
            "layout_id": str,
            "room_type": str,
        }

    这个 bundle 直接传给
    ``spatial_api_stub.check_stl_constraint_violation(topology_bundle=...)``
    即可让 Distance > D_safe 走"距离场逐点查询"的官方求解路径。
    """
    df = pipeline_result["distance_field"]
    tx = pipeline_result["transform"]
    cad = pipeline_result.get("cad_data", {})
    rpx = float(pipeline_result.get("robot_radius_px", 0.0))
    res = float(tx["resolution"])

    return {
        "distance_field": df.astype(np.float32, copy=False),
        "grid_transform": {
            "origin": tuple(tx["origin"]),
            "resolution": res,
            "shape": tuple(tx["shape"]),
        },
        "robot_radius_m": rpx * res,
        "robot_radius_px": rpx,
        "layout_id": cad.get("layout_id", ""),
        "room_type": cad.get("room_type", ""),
    }


def save_topology_bundle(bundle: Dict[str, Any], path: str | Path) -> Path:
    """将 TopologyBundle 持久化为 ``.npz``（field 为压缩 float16）+ 同名 ``.json``。"""
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    npz_path = path.with_suffix(".npz")
    json_path = path.with_suffix(".json")

    np.savez_compressed(
        npz_path,
        distance_field=bundle["distance_field"],
    )
    meta = {
        "grid_transform": {
            "origin": list(bundle["grid_transform"]["origin"]),
            "resolution": bundle["grid_transform"]["resolution"],
            "shape": list(bundle["grid_transform"]["shape"]),
        },
        "robot_radius_m": bundle["robot_radius_m"],
        "robot_radius_px": bundle["robot_radius_px"],
        "layout_id": bundle.get("layout_id", ""),
        "room_type": bundle.get("room_type", ""),
        "field_npz": npz_path.name,
    }
    json_path.write_text(json.dumps(meta, indent=2), encoding="utf-8")
    return npz_path


def load_topology_bundle(path: str | Path) -> Dict[str, Any]:
    """加载 ``save_topology_bundle`` 生成的 bundle（npz + json sidecar）。"""
    path = Path(path)
    json_path = path.with_suffix(".json")
    npz_path = path.with_suffix(".npz")
    meta = json.loads(json_path.read_text(encoding="utf-8"))
    with np.load(npz_path) as data:
        df = data["distance_field"].astype(np.float32)
    return {
        "distance_field": df,
        "grid_transform": {
            "origin": tuple(meta["grid_transform"]["origin"]),
            "resolution": float(meta["grid_transform"]["resolution"]),
            "shape": tuple(meta["grid_transform"]["shape"]),
        },
        "robot_radius_m": meta["robot_radius_m"],
        "robot_radius_px": meta["robot_radius_px"],
        "layout_id": meta.get("layout_id", ""),
        "room_type": meta.get("room_type", ""),
    }
