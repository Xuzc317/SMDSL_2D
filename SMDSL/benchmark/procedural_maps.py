"""
procedural_maps.py — 程序化窄通道地图生成器

纯函数设计 | 固定 Seed | 严格可复现 | 所有地图保证有解
输出: (grid, metadata), metadata 含窄通道宽度/障碍物数/难度标签
"""

from __future__ import annotations

import math
from typing import Any

import numpy as np
from scipy.ndimage import distance_transform_edt

_FREE = 1
_WALL = 0


def _add_border(grid: np.ndarray, t: int = 2) -> None:
    grid[:t, :] = _WALL; grid[-t:, :] = _WALL
    grid[:, :t] = _WALL; grid[:, -t:] = _WALL


def _bfs_connect(grid: np.ndarray, start: tuple, goal: tuple) -> bool:
    if grid[start] == _WALL or grid[goal] == _WALL:
        return False
    h, w = grid.shape
    seen = set()
    q = [start]
    seen.add(start)
    while q:
        r, c = q.pop()
        if (r, c) == goal:
            return True
        for dr, dc in ((1,0),(-1,0),(0,1),(0,-1)):
            nr, nc = r+dr, c+dc
            if 0<=nr<h and 0<=nc<w and (nr,nc) not in seen and grid[nr,nc]==_FREE:
                seen.add((nr,nc))
                q.append((nr,nc))
    return False


def _min_clearance_on_path(grid: np.ndarray, start: tuple, goal: tuple) -> float:
    h, w = grid.shape
    parent: dict = {start: None}
    q = [start]
    seen = set([start])
    found = False
    while q and not found:
        r, c = q.pop(0)
        for dr, dc in ((1,0),(-1,0),(0,1),(0,-1)):
            nr, nc = r+dr, c+dc
            if 0<=nr<h and 0<=nc<w and (nr,nc) not in seen and grid[nr,nc]==_FREE:
                seen.add((nr,nc))
                parent[(nr,nc)] = (r,c)
                if (nr,nc) == goal:
                    found = True; break
                q.append((nr,nc))
    if not found:
        return 0.0
    path = []
    cur = goal
    while cur is not None:
        path.append(cur)
        cur = parent.get(cur)
    dt = distance_transform_edt(grid)
    return float(min(dt[r,c] for r,c in path))


def _difficulty_label(clearance_px: float, r_px: float) -> str:
    ratio = clearance_px / max(r_px, 1.0)
    if ratio < 1.2: return "extreme"
    if ratio < 1.8: return "hard"
    if ratio < 2.5: return "medium"
    return "easy"


def generate_single_map(
    height: int, width: int,
    narrow_width: int, wide_width: int,
    seed: int, robot_radius_px: int = 6,
) -> tuple[np.ndarray, dict[str, Any]]:
    """生成一张含窄通道的地图。左右两厅，中间走廊，走廊中央狭窄段。"""
    rng = np.random.RandomState(seed)
    for attempt in range(10):
        ls = seed + attempt * 9973
        lr = np.random.RandomState(ls)
        g = np.ones((height, width), dtype=np.uint8)
        _add_border(g, 2)
        pad = 4
        hw = int(math.ceil(wide_width / 2.0))
        cy = height // 2
        g[2:cy-hw, 2:-2] = _WALL
        g[cy+hw:-2, 2:-2] = _WALL
        hn = int(math.ceil(narrow_width / 2.0))
        cx = width // 2
        nl, nr = cx - hn, cx + hn
        fh = max(0, hw - hn)
        if fh > 0:
            g[cy-hw:cy-hw+fh, nl:nr] = _WALL
            g[cy+hw-fh:cy+hw, nl:nr] = _WALL
        obs = 0
        for _ in range(16):
            if obs >= lr.randint(3, 8):
                break
            px = lr.randint(pad+2, width-pad-2)
            py = lr.randint(pad+2, height-pad-2)
            if abs(py-cy) < hw + 10:
                continue
            r = lr.randint(2, 5)
            rs, re = max(0,py-r), min(height,py+r)
            cs, ce = max(0,px-r), min(width,px+r)
            block = g[rs:re, cs:ce]
            mask = (block == _FREE); block[mask] = _WALL
            obs += 1
        if lr.rand() < 0.3 and wide_width > narrow_width*2:
            ws = lr.choice([0, 1])
            off = lr.randint(hn+2, hw-2)
            wl = lr.randint(wide_width//4, wide_width//2)
            wy = cy-hw+off if ws==0 else cy+hw-off
            wsx = cx+nr+2
            wex = min(width-pad, wsx+wl)
            g[wy-1:wy+2, wsx:wex] = _WALL
        free = np.argwhere(g == _FREE)
        if len(free) < 20:
            continue
        left = free[free[:, 1] < width//3]
        right = free[free[:, 1] > width*2//3]
        if len(left) < 3 or len(right) < 3:
            continue
        start = tuple(left[lr.randint(len(left))])
        goal = tuple(right[lr.randint(len(right))])
        if not _bfs_connect(g, start, goal):
            continue
        clr = _min_clearance_on_path(g, start, goal)
        meta: dict[str, Any] = {
            "seed": seed, "local_seed": ls,
            "height": height, "width": width,
            "narrow_width_px": narrow_width, "wide_width_px": wide_width,
            "narrow_clearance_edt_px": round(clr, 2),
            "n_obstacles": obs, "difficulty": _difficulty_label(clr, robot_radius_px),
            "start_rc": list(start), "goal_rc": list(goal),
        }
        return g, meta
    raise RuntimeError(f"generate_single_map failed after 10 attempts (seed={seed})")


DIFFICULTY_PRESETS: dict[str, list[int]] = {
    "easy": [12, 55], "medium": [8, 48],
    "hard": [5, 42], "extreme": [3, 36],
}


def generate_batch(
    n_maps: int,
    map_height: int = 200, map_width: int = 300,
    difficulty: str = "mixed",
    seed: int = 42, robot_radius_px: int = 6,
) -> list[tuple[np.ndarray, dict[str, Any]]]:
    """批量生成 n 张窄通道地图。difficulty: easy/medium/hard/extreme/mixed"""
    rng = np.random.RandomState(seed)
    results: list[tuple[np.ndarray, dict[str, Any]]] = []
    fails = 0
    for i in range(n_maps):
        d = rng.choice(["easy","medium","hard","extreme"]) if difficulty=="mixed" else difficulty
        nw, ww = DIFFICULTY_PRESETS[d]
        nw = max(2, nw + rng.randint(-1,2))
        ww = max(nw+8, ww + rng.randint(-3,4))
        ms = seed + i*7919 + 1
        try:
            g,m = generate_single_map(height=map_height,width=map_width,narrow_width=nw,wide_width=ww,seed=ms,robot_radius_px=robot_radius_px)
            m["difficulty"] = d; m["map_index"] = i
            results.append((g,m))
        except RuntimeError:
            fails += 1
            if fails > n_maps * 0.2:
                raise
            continue
    return results
