"""
bench_edt_vs_costmap.py — Phase 2.3: EDT vs Costmap 对比基准

在 FloorplanQA / 合成布局上对比两种安全空间表示：
  - EDT 方案: safety_aware_astar_flood（连续距离场）
  - Costmap 方案: binary_dilation + 二值 A*（膨胀层）

指标:
  - clearance_min: 路径上最小 clearance (m)
  - clearance_mean: 路径上平均 clearance (m)
  - narrow_passage_rate: 窄通道通过率 (%)
  - planning_time_ms: 规划耗时 (ms)

用法::

    python benchmark/bench_edt_vs_costmap.py
    python benchmark/bench_edt_vs_costmap.py --n_layouts 50 --n_pairs 3
"""

from __future__ import annotations

import argparse
import json
import math
import os
import sys
import time
from pathlib import Path
from typing import Any, Dict, List, Optional, Tuple

import numpy as np

_PROJECT_ROOT = Path(__file__).resolve().parent.parent
if str(_PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(_PROJECT_ROOT))


def _costmap_astar(
    grid: np.ndarray,
    start_rc: Tuple[int, int],
    goal_rc: Tuple[int, int],
    robot_radius_px: float,
) -> Optional[List[Tuple[int, int]]]:
    """二值膨胀 → 二值 A*（costmap 方案）。"""
    from scipy import ndimage
    from cad_parser.astar_topology import astar_shortest_path

    r_px = max(1, int(math.ceil(robot_radius_px)))
    se = ndimage.generate_binary_structure(2, 1)
    inflated = ndimage.binary_dilation(grid == 0, structure=se, iterations=r_px)
    costmap = (~inflated).astype(np.uint8)

    H, W = costmap.shape
    fake_df = np.where(costmap == 1, r_px * 0.5 + 0.1, 0.0).astype(np.float32)
    if not (0 <= start_rc[0] < H and 0 <= start_rc[1] < W and costmap[start_rc] == 1):
        return None
    if not (0 <= goal_rc[0] < H and 0 <= goal_rc[1] < W and costmap[goal_rc] == 1):
        return None

    return astar_shortest_path(
        costmap, fake_df, start_rc, goal_rc,
        robot_radius_px=r_px * 0.5,
        safety_weight=0.1, max_steps=2_000_000,
    )


def _edt_astar(
    grid: np.ndarray,
    distance_field: np.ndarray,
    start_rc: Tuple[int, int],
    goal_rc: Tuple[int, int],
    robot_radius_px: float,
) -> Optional[List[Tuple[int, int]]]:
    """EDT 距离场 → 安全代价 A*（EDT 方案）。"""
    from cad_parser.astar_topology import astar_shortest_path
    return astar_shortest_path(
        grid, distance_field, start_rc, goal_rc,
        robot_radius_px=robot_radius_px,
        safety_weight=0.5, max_steps=2_000_000,
    )


def _path_clearance(
    path_rc: List[Tuple[int, int]],
    distance_field: np.ndarray,
    resolution: float,
) -> Tuple[float, float]:
    """计算路径上各点的 clearance (米)。"""
    if not path_rc:
        return 0.0, 0.0
    vals = [distance_field[r, c] * resolution for r, c in path_rc]
    return min(vals), sum(vals) / len(vals)


def _find_seed_pair(
    grid: np.ndarray,
    distance_field: np.ndarray,
    robot_radius_px: float,
) -> Tuple[Tuple[int, int], Tuple[int, int]]:
    """在自由空间中找两个距离较远的种子点。"""
    H, W = grid.shape
    free = (grid == 1) & (distance_field >= robot_radius_px)
    rows, cols = np.where(free)
    if len(rows) < 2:
        rows, cols = np.where(grid == 1)
    if len(rows) < 2:
        return (0, 0), (H - 1, W - 1)

    pts = np.column_stack([rows, cols])
    idx_max = np.argmax(distance_field[rows, cols])
    first = pts[idx_max]

    dists = np.sqrt(np.sum((pts - first) ** 2, axis=1))
    second = pts[np.argmax(dists)]

    return (int(first[0]), int(first[1])), (int(second[0]), int(second[1]))


def run_single_benchmark(
    grid: np.ndarray,
    distance_field: np.ndarray,
    resolution: float,
    robot_radius_px: float,
) -> Dict[str, Any]:
    """在单个布局上运行 EDT vs Costmap 对比。"""
    from cad_parser.astar_topology import bridge_thin_walls, remove_exterior_freespace
    from scipy.ndimage import distance_transform_edt

    grid = remove_exterior_freespace(grid, close_gaps_px=1)
    grid = bridge_thin_walls(grid, kernel_px=2)
    df = distance_transform_edt(grid.astype(np.uint8)).astype(np.float32)
    if distance_field is not None and distance_field.shape == df.shape:
        df = distance_field

    start_rc, goal_rc = _find_seed_pair(grid, df, robot_radius_px)

    result: Dict[str, Any] = {}

    # ── Costmap 方案 ──
    t0 = time.perf_counter()
    path_cm = _costmap_astar(grid, start_rc, goal_rc, robot_radius_px)
    t_cm = (time.perf_counter() - t0) * 1000.0
    result["costmap_path_found"] = path_cm is not None and len(path_cm) >= 2
    result["costmap_time_ms"] = t_cm
    if path_cm and len(path_cm) >= 2:
        cm_min, cm_mean = _path_clearance(path_cm, df, resolution)
        result["costmap_clearance_min"] = cm_min
        result["costmap_clearance_mean"] = cm_mean
        result["costmap_path_len"] = len(path_cm)
    else:
        result["costmap_clearance_min"] = 0.0
        result["costmap_clearance_mean"] = 0.0
        result["costmap_path_len"] = 0

    # ── EDT 方案 ──
    t0 = time.perf_counter()
    path_edt = _edt_astar(grid, df, start_rc, goal_rc, robot_radius_px)
    t_edt = (time.perf_counter() - t0) * 1000.0
    result["edt_path_found"] = path_edt is not None and len(path_edt) >= 2
    result["edt_time_ms"] = t_edt
    if path_edt and len(path_edt) >= 2:
        edt_min, edt_mean = _path_clearance(path_edt, df, resolution)
        result["edt_clearance_min"] = edt_min
        result["edt_clearance_mean"] = edt_mean
        result["edt_path_len"] = len(path_edt)
    else:
        result["edt_clearance_min"] = 0.0
        result["edt_clearance_mean"] = 0.0
        result["edt_path_len"] = 0

    return result


def generate_synthetic_grid(
    rng: np.random.Generator,
    size: int = 200,
) -> np.ndarray:
    """生成随机合成 floor plan 栅格。"""
    grid = np.ones((size, size), dtype=np.uint8)
    n_walls = rng.integers(4, 12)
    for _ in range(n_walls):
        orientation = rng.choice(["h", "v"])
        if orientation == "h":
            y = rng.integers(size // 8, 7 * size // 8)
            x1 = rng.integers(0, size // 4)
            x2 = rng.integers(3 * size // 4, size)
            grid[y - 1:y + 2, x1:x2] = 0
            gap = rng.integers(size // 6, size // 3)
            gap_x = rng.integers(size // 4, 3 * size // 4)
            grid[y - 1:y + 2, gap_x:gap_x + gap] = 1
        else:
            x = rng.integers(size // 8, 7 * size // 8)
            y1 = rng.integers(0, size // 4)
            y2 = rng.integers(3 * size // 4, size)
            grid[y1:y2, x - 1:x + 2] = 0
            gap = rng.integers(size // 6, size // 3)
            gap_y = rng.integers(size // 4, 3 * size // 4)
            grid[gap_y:gap_y + gap, x - 1:x + 2] = 1
    return grid


def main(argv: Optional[List[str]] = None) -> int:
    parser = argparse.ArgumentParser(description="EDT vs Costmap 基准对比")
    parser.add_argument("--n_layouts", type=int, default=10,
                        help="合成布局数量 (默认 10)")
    parser.add_argument("--resolution", type=float, default=0.05,
                        help="栅格分辨率 m/px (默认 0.05)")
    parser.add_argument("--robot_radius_m", type=float, default=0.25,
                        help="机器人半径 m (默认 0.25)")
    parser.add_argument("--output", type=str, default=None,
                        help="结果输出 JSON 路径")
    args = parser.parse_args(argv)

    res = args.resolution
    robot_r_px = args.robot_radius_m / res

    print(f"EDT vs Costmap Benchmark")
    print(f"  Layouts: {args.n_layouts}")
    print(f"  Resolution: {res} m/px")
    print(f"  Robot radius: {args.robot_radius_m} m ({robot_r_px:.0f} px)")
    print()

    rng = np.random.default_rng(42)
    results: List[Dict[str, Any]] = []

    for i in range(args.n_layouts):
        grid = generate_synthetic_grid(rng, size=200)
        r = run_single_benchmark(grid, None, res, robot_r_px)
        r["layout_id"] = i
        results.append(r)

        edt_ok = "OK" if r.get("edt_path_found") else "FAIL"
        cm_ok = "OK" if r.get("costmap_path_found") else "FAIL"
        edt_min = r.get("edt_clearance_min", 0.0) * 100
        cm_min = r.get("costmap_clearance_min", 0.0) * 100
        delta = edt_min - cm_min
        print(f"  [{i:3d}] EDT={edt_ok} ({edt_min:5.1f}cm)  "
              f"Costmap={cm_ok} ({cm_min:5.1f}cm)  "
              f"Δ={delta:+.1f}cm  "
              f"t_edt={r.get('edt_time_ms', 0):.0f}ms  "
              f"t_cm={r.get('costmap_time_ms', 0):.0f}ms")

    # Aggregate statistics
    edt_clearances = [r.get("edt_clearance_min", 0.0)
                      for r in results if r.get("edt_path_found")]
    cm_clearances = [r.get("costmap_clearance_min", 0.0)
                     for r in results if r.get("costmap_path_found")]
    edt_paths = sum(1 for r in results if r.get("edt_path_found"))
    cm_paths = sum(1 for r in results if r.get("costmap_path_found"))
    edt_time = np.mean([r.get("edt_time_ms", 0) for r in results])
    cm_time = np.mean([r.get("costmap_time_ms", 0) for r in results])

    print(f"\n{'='*60}")
    print(f"  Summary (n={len(results)})")
    print(f"  EDT  paths found: {edt_paths}/{len(results)}")
    print(f"  Costmap paths found: {cm_paths}/{len(results)}")
    if edt_clearances:
        print(f"  EDT  clearance_min (mean): {np.mean(edt_clearances)*100:.1f} cm")
    if cm_clearances:
        print(f"  Costmap clearance_min (mean): {np.mean(cm_clearances)*100:.1f} cm")
    if edt_clearances and cm_clearances:
        improvement = (np.mean(edt_clearances) - np.mean(cm_clearances)) / max(
            np.mean(cm_clearances), 1e-9) * 100
        print(f"  EDT improvement: {improvement:+.1f}%")
    print(f"  EDT  avg time: {edt_time:.0f} ms")
    print(f"  Costmap avg time: {cm_time:.0f} ms")
    print(f"{'='*60}")

    if args.output:
        out_path = Path(args.output)
        out_path.parent.mkdir(parents=True, exist_ok=True)
        out_path.write_text(json.dumps(results, indent=2, default=str),
                            encoding="utf-8")
        print(f"\nResults saved to {out_path}")

    return 0


if __name__ == "__main__":
    sys.exit(main())
