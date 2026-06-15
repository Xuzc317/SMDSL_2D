"""
experiment_runner.py — 核心对比实验引擎

纯函数设计 | float64 EDT | 严格 ROS Costmap 模拟 | 全异常隔离
单次实验入口 run_experiment(grid, start, goal, ...) → flat dict
"""

from __future__ import annotations

import math
import time
from typing import Any

import numpy as np
from scipy.ndimage import distance_transform_edt, binary_dilation
from skimage.morphology import disk

_FREE = 1
_WALL = 0


# ─── 工具 A* ──────────────────────────────────────────────

def _astar_search(
    cost_grid: np.ndarray,
    start: tuple[int, int],
    goal: tuple[int, int],
    max_steps: int = 500_000,
    time_limit: float = 30.0,
) -> tuple[list[tuple[int, int]], int]:
    """
    通用 A* 搜索。cost_grid: (H,W) float, 值越高代价越大, inf=不可通行。
    返回 (path_rc, n_expanded)。找不到路径时 path_rc=[]。
    """
    import heapq

    h, w = cost_grid.shape
    sr, sc = start
    gr, gc = goal

    if cost_grid[sr, sc] == float("inf") or cost_grid[gr, gc] == float("inf"):
        return [], 0

    g_score = np.full((h, w), float("inf"), dtype=np.float64)
    g_score[sr, sc] = 0.0
    parent: dict = {}

    heap = [(0.0, sr, sc)]
    closed = set()
    t0 = time.monotonic()
    n_expanded = 0

    while heap and n_expanded < max_steps:
        if n_expanded & 1023 == 0 and time.monotonic() - t0 > time_limit:
            return [], n_expanded
        f, r, c = heapq.heappop(heap)
        if (r, c) in closed:
            continue
        closed.add((r, c))
        n_expanded += 1

        if (r, c) == (gr, gc):
            # rebuild path
            path = [(gr, gc)]
            cur = (gr, gc)
            while cur != (sr, sc):
                cur = parent[cur]
                path.append(cur)
            path.reverse()
            return path, n_expanded

        for dr, dc in ((1, 0), (-1, 0), (0, 1), (0, -1)):
            nr, nc = r + dr, c + dc
            if not (0 <= nr < h and 0 <= nc < w):
                continue
            cost = cost_grid[nr, nc]
            if cost == float("inf"):
                continue
            if (nr, nc) in closed:
                continue
            ng = g_score[r, c] + cost
            if ng < g_score[nr, nc]:
                g_score[nr, nc] = ng
                parent[(nr, nc)] = (r, c)
                h_val = math.hypot(nr - gr, nc - gc)
                heapq.heappush(heap, (ng + h_val, nr, nc))

    return [], n_expanded


# ─── Costmap 基线（严格模拟 ROS Navigation Stack）────────────

def costmap_baseline(
    grid: np.ndarray,
    robot_radius_px: float,
    start: tuple[int, int],
    goal: tuple[int, int],
    resolution: float = 0.05,
) -> dict[str, Any]:
    """
    严格模拟 ROS Navigation Stack 的 costmap 膨胀层 + 二值 A*。

    步骤:
      1. 占据网格提取 (grid == _WALL)
      2. disk(ceil(robot_radius_px)) 膨胀
      3. 膨胀区域标记为不可通行 (cost=inf), 其余区域 cost=1.0
      4. A* 搜索（仅二值代价，无安全梯度信息）

    返回:
        method, path_found, collision, clearance_min_mm,
        clearance_mean_mm, rho_min_mm, path_length_m,
        planning_time_ms, n_expanded, error
    """
    t0 = time.perf_counter()
    result: dict[str, Any] = {
        "method": "costmap",
        "path_found": False,
        "collision": True,
        "clearance_min_mm": float("nan"),
        "clearance_mean_mm": float("nan"),
        "rho_min_mm": float("nan"),
        "path_length_m": float("nan"),
        "planning_time_ms": float("nan"),
        "n_expanded": 0,
        "error": "",
    }

    try:
        r_px = int(math.ceil(robot_radius_px))
        disk_se = disk(r_px) if r_px > 0 else np.ones((1, 1), dtype=bool)
        occupied = (grid == _WALL).astype(np.uint8)
        inflated = binary_dilation(occupied, structure=disk_se)
        cost_grid = np.where(inflated, float("inf"), 1.0)

        path, n_exp = _astar_search(cost_grid, start, goal)
        dt = (time.perf_counter() - t0) * 1000

        if not path:
            result["path_found"] = False
            result["collision"] = True
            result["planning_time_ms"] = round(dt, 3)
            result["n_expanded"] = n_exp
            return result

        path_len_px = sum(
            math.hypot(path[i][0]-path[i-1][0], path[i][1]-path[i-1][1])
            for i in range(1, len(path))
        )
        path_len_m = path_len_px * resolution

        # 沿路径计算 clearance
        dt_field = distance_transform_edt(grid.astype(bool)).astype(np.float64)
        clearances_px = []
        for r, c in path:
            d = float(dt_field[r, c])
            clearances_px.append(d)
        clr_min_px = float(min(clearances_px))
        clr_mean_px = float(np.mean(clearances_px))

        # 碰撞判定：如果路径上任何点 clearance < robot_radius_px → 碰撞
        collision_flag = clr_min_px < robot_radius_px

        # STL 鲁棒度 ρ = clearance_m - D_safe（D_safe = robot_radius）
        d_safe_m = robot_radius_px * resolution
        rho_vals_m = [(d * resolution - d_safe_m) for d in clearances_px]
        rho_min_m = min(rho_vals_m) if rho_vals_m else 0.0

        result.update({
            "path_found": True,
            "collision": collision_flag,
            "clearance_min_mm": round(clr_min_px * resolution * 1000, 3),
            "clearance_mean_mm": round(clr_mean_px * resolution * 1000, 3),
            "rho_min_mm": round(rho_min_m * 1000, 3),
            "path_length_m": round(path_len_m, 4),
            "planning_time_ms": round(dt, 3),
            "n_expanded": n_exp,
        })

    except Exception as e:
        dt = (time.perf_counter() - t0) * 1000
        result["planning_time_ms"] = round(dt, 3)
        result["error"] = f"costmap_baseline: {type(e).__name__}: {e}"

    return result


# ─── EDT 基线（连续距离场 + 安全代价 A*）──────────────────

def edt_baseline(
    grid: np.ndarray,
    robot_radius_px: float,
    start: tuple[int, int],
    goal: tuple[int, int],
    safety_weight: float = 0.5,
    resolution: float = 0.05,
) -> dict[str, Any]:
    """
    基于 EDT 连续距离场的路径规划。

    步骤:
      1. distance_transform_edt(float64) 计算精确欧氏距离场
      2. A* 的代价函数: cost = 1.0 + safety_weight / (1 + d/r)
         - d: 该点到最近障碍物的 EDT 值 (float64)
         - r: robot_radius_px
         - 路径自然偏向通道中心（d 最大处）
      3. STL 鲁棒度 ρ = d_real - D_safe（连续值）

    返回:
        method, path_found, collision, clearance_min_mm,
        clearance_mean_mm, rho_min_mm, path_length_m,
        planning_time_ms, n_expanded, error
    """
    t0 = time.perf_counter()
    result: dict[str, Any] = {
        "method": "edt",
        "path_found": False,
        "collision": True,
        "clearance_min_mm": float("nan"),
        "clearance_mean_mm": float("nan"),
        "rho_min_mm": float("nan"),
        "path_length_m": float("nan"),
        "planning_time_ms": float("nan"),
        "n_expanded": 0,
        "error": "",
    }

    try:
        # EDT — float64 精度，严格禁止降精度
        dt_field = distance_transform_edt(grid.astype(bool)).astype(np.float64)

        r_px = max(robot_radius_px, 0.1)
        h, w = grid.shape
        cost_grid = np.ones((h, w), dtype=np.float64)

        # 墙壁区域标记为不可通行
        cost_grid[grid == _WALL] = float("inf")

        # 空地: cost = 1.0 + safety_weight / (1 + d/r)
        free_mask = (grid == _FREE)
        d_vals = dt_field[free_mask]
        cost_grid[free_mask] = 1.0 + safety_weight / (1.0 + d_vals / r_px)

        path, n_exp = _astar_search(cost_grid, start, goal)
        dt = (time.perf_counter() - t0) * 1000

        if not path:
            result["path_found"] = False
            result["collision"] = True
            result["planning_time_ms"] = round(dt, 3)
            result["n_expanded"] = n_exp
            return result

        path_len_px = sum(
            math.hypot(path[i][0]-path[i-1][0], path[i][1]-path[i-1][1])
            for i in range(1, len(path))
        )
        path_len_m = path_len_px * resolution

        # 沿路径读取 EDT 值（float64）
        clearances_px = []
        for r, c in path:
            d = float(dt_field[r, c])
            clearances_px.append(d)
        clr_min_px = float(min(clearances_px))
        clr_mean_px = float(np.mean(clearances_px))

        collision_flag = clr_min_px < robot_radius_px

        # STL 鲁棒度（连续值）
        d_safe_m = robot_radius_px * resolution
        rho_vals_m = [(d * resolution - d_safe_m) for d in clearances_px]
        rho_min_m = min(rho_vals_m) if rho_vals_m else 0.0

        result.update({
            "path_found": True,
            "collision": collision_flag,
            "clearance_min_mm": round(clr_min_px * resolution * 1000, 3),
            "clearance_mean_mm": round(clr_mean_px * resolution * 1000, 3),
            "rho_min_mm": round(rho_min_m * 1000, 3),
            "path_length_m": round(path_len_m, 4),
            "planning_time_ms": round(dt, 3),
            "n_expanded": n_exp,
        })

    except Exception as e:
        dt = (time.perf_counter() - t0) * 1000
        result["planning_time_ms"] = round(dt, 3)
        result["error"] = f"edt_baseline: {type(e).__name__}: {e}"

    return result


# ─── 单次实验入口（CSV 友好输出）─────────────────────────

CSV_COLUMNS: list[str] = [
    "map_index", "method", "difficulty",
    "narrow_width_px", "wide_width_px", "narrow_clearance_edt_px",
    "n_obstacles", "path_found", "collision",
    "clearance_min_mm", "clearance_mean_mm", "rho_min_mm",
    "path_length_m", "planning_time_ms", "n_expanded",
    "error",
]


def run_experiment(
    grid: np.ndarray,
    start_rc: tuple[int, int],
    goal_rc: tuple[int, int],
    robot_radius_px: float = 6.0,
    safety_weight: float = 0.5,
    resolution: float = 0.05,
    metadata: dict[str, Any] | None = None,
) -> dict[str, Any]:
    """
    对单张地图同时运行 EDT 和 Costmap 两种方法。

    返回两个 flat dict（每条 CSV 行），已填充 metadata 信息。

    参数:
        grid: (H,W) uint8, 1=free, 0=wall
        start_rc, goal_rc: 起终点像素坐标 (row, col)
        robot_radius_px: 机器人半径（像素）
        safety_weight: EDT A* 的安全代价权重
        resolution: 米/像素（用于将像素值换算为物理米）
        metadata: 来自 procedural_maps 的元数据 dict

    返回:
        [row_edt, row_costmap], 每行是可直接写入 CSV 的 flat dict
        所有 A* 失败通过返回 path_found=False 表示，不抛异常
    """
    meta = metadata or {}

    def _merge(base: dict) -> dict[str, Any]:
        row: dict[str, Any] = {
            "map_index": meta.get("map_index", -1),
            "method": base["method"],
            "difficulty": meta.get("difficulty", "unknown"),
            "narrow_width_px": meta.get("narrow_width_px", -1),
            "wide_width_px": meta.get("wide_width_px", -1),
            "narrow_clearance_edt_px": meta.get("narrow_clearance_edt_px", -1.0),
            "n_obstacles": meta.get("n_obstacles", 0),
            "path_found": base["path_found"],
            "collision": base["collision"],
            "clearance_min_mm": base["clearance_min_mm"],
            "clearance_mean_mm": base["clearance_mean_mm"],
            "rho_min_mm": base["rho_min_mm"],
            "path_length_m": base["path_length_m"],
            "planning_time_ms": base["planning_time_ms"],
            "n_expanded": base["n_expanded"],
            "error": base["error"],
        }
        return row

    edt_res = edt_baseline(
        grid, robot_radius_px, start_rc, goal_rc,
        safety_weight=safety_weight, resolution=resolution,
    )
    cost_res = costmap_baseline(
        grid, robot_radius_px, start_rc, goal_rc,
        resolution=resolution,
    )

    return [_merge(edt_res), _merge(cost_res)]
