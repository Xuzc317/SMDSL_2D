"""
bench_edt_vs_costmap.py — EDT vs Costmap 2D 路径规划对比基准 (Phase 2.3)

数据集：FloorplanQA（1981 layouts），默认采样 200 layouts x 5 对起终点
方法：Ours（EDT + safety_aware_astar）vs Baseline（膨胀层 + 二值 A*）
指标：clearance_min / mean、低于阈值占比、窄通道通过率、路径长度、规划时间
输出：汇总表格（按 room_type 分组）+ 箱线图
"""

from __future__ import annotations

import argparse
import json
import math
import os
import sys
import time
from collections import defaultdict
from pathlib import Path
from typing import Any, Dict, List, Optional, Tuple

import numpy as np

# ── 路径设置 ──
_PROJ = Path(__file__).resolve().parent.parent
_SMDSL = _PROJ / "SMDSL"
if str(_SMDSL) not in sys.path:
    sys.path.insert(0, str(_SMDSL))
if str(_PROJ) not in sys.path:
    sys.path.insert(0, str(_PROJ))

from cad_parser.astar_topology import (
    CLASS_INFLATED,
    compute_distance_field,
    safety_aware_astar_flood,
    astar_shortest_path,
)
from cad_parser.dispatcher import dispatch_cad
from scipy.ndimage import binary_dilation
from skimage.morphology import disk


def costmap_baseline(
    grid: np.ndarray,
    robot_radius_px: float,
) -> Tuple[np.ndarray, np.ndarray]:
    """复现传统二值膨胀 + 二值 A* 的 costmap。"""
    struct = disk(math.ceil(robot_radius_px))
    occupied = (grid == 0)
    inflated = binary_dilation(occupied, structure=struct)
    costmap = np.where(inflated, np.inf, 1.0).astype(np.float32)
    binary_grid = np.where(inflated, 0, 1).astype(np.uint8)
    return costmap, binary_grid


def _astar_binary(
    grid: np.ndarray,
    start_rc: Tuple[int, int],
    goal_rc: Tuple[int, int],
) -> Optional[List[Tuple[int, int]]]:
    """在二值 grid（0=不可通行, 1=自由）上运行 A*（costmap baseline）。"""
    dummy_df = np.ones_like(grid, dtype=np.float64) * 10.0
    dummy_df[grid == 0] = 0.0
    return astar_shortest_path(
        grid=grid,
        distance_field=dummy_df,
        start_rc=start_rc,
        goal_rc=goal_rc,
        robot_radius_px=1.0,
        safety_weight=0.0,
    )


def path_clearance_stats(
    path_rc: List[Tuple[int, int]],
    distance_field: np.ndarray,
    resolution: float,
) -> Dict[str, float]:
    """计算一条路径上的 clearance 统计（向量化版本）。"""
    if not path_rc:
        return {"clearance_min": 0.0, "clearance_mean": 0.0, "clearance_below_1px": 1.0}
    H, W = distance_field.shape
    arr = np.array(path_rc)
    rows = np.clip(arr[:, 0].astype(int), 0, H - 1)
    cols = np.clip(arr[:, 1].astype(int), 0, W - 1)
    vals = distance_field[rows, cols]
    vals_m = vals * resolution
    below = float(np.sum(vals < 1.0)) / max(1, len(vals))
    return {
        "clearance_min": float(np.min(vals_m)),
        "clearance_mean": float(np.mean(vals_m)),
        "clearance_below_1px": below,
    }


def _pick_random_start_goal(
    grid: np.ndarray,
    distance_field: np.ndarray,
    robot_radius_px: float,
    n_pairs: int = 5,
) -> List[Tuple[Tuple[int, int], Tuple[int, int]]]:
    """在自由空间中随机选取起终点对。"""
    H, W = grid.shape
    safe_mask = (grid == 1) & (distance_field >= robot_radius_px)
    idx = np.argwhere(safe_mask)
    if len(idx) < 2:
        return []
    pairs = []
    min_dist = max(H, W) * 0.15
    for _ in range(n_pairs * 10):
        if len(pairs) >= n_pairs:
            break
        a = tuple(idx[np.random.randint(len(idx))])
        b = tuple(idx[np.random.randint(len(idx))])
        d = math.hypot(a[0] - b[0], a[1] - b[1])
        if d >= min_dist:
            pairs.append(((int(a[0]), int(a[1])), (int(b[0]), int(b[1]))))
    return pairs


def _group_by_room_type(filepath: str) -> str:
    fname = Path(filepath).stem.lower()
    for rt in ["bedroom", "living_room", "kitchen", "bathroom", "corridor"]:
        if rt in fname:
            return rt
    return "other"


def run_single_comparison(
    layout_path: str,
    robot_radius_m: float = 0.25,
    resolution: float = 0.05,
    n_pairs: int = 5,
    d_safe_m: float = 0.3,
) -> List[Dict[str, Any]]:
    """在单个 layout 上运行 EDT vs Costmap 对比。"""
    try:
        parsed = dispatch_cad(layout_path, resolution=resolution)
    except Exception:
        return []

    grid = parsed["grid"]
    df = compute_distance_field(grid)
    robot_radius_px = robot_radius_m / resolution

    # 拓扑分类获取可用起终点候选
    topology, _ = safety_aware_astar_flood(
        grid, df, seed_points=[], robot_radius_px=robot_radius_px, safety_weight=0.5,
    )
    pairs = _pick_random_start_goal(grid, df, robot_radius_px, n_pairs)
    if not pairs:
        return []

    room_type = _group_by_room_type(layout_path)
    results = []

    # Hoist costmap computation once per layout (not per-pair)
    _, bin_grid = costmap_baseline(grid, robot_radius_px)

    for start_rc, goal_rc in pairs:
        # ── EDT 方法 ──
        t0 = time.perf_counter()
        path_edt = astar_shortest_path(
            grid=grid, distance_field=df,
            start_rc=start_rc, goal_rc=goal_rc,
            robot_radius_px=robot_radius_px,
            safety_weight=1.0,
        )
        t_edt = (time.perf_counter() - t0) * 1000.0
        cl_edt = path_clearance_stats(path_edt, df, resolution) if path_edt else {"clearance_min": 0, "clearance_mean": 0, "clearance_below_1px": 1.0}

        # ── Costmap 方法 (costmap pre-computed once above) ──
        t0 = time.perf_counter()
        path_cm = _astar_binary(bin_grid, start_rc, goal_rc)
        t_cm = (time.perf_counter() - t0) * 1000.0
        cl_cm = path_clearance_stats(path_cm, df, resolution) if path_cm else {"clearance_min": 0, "clearance_mean": 0, "clearance_below_1px": 1.0}

        results.append({
            "room_type": room_type,
            "layout": Path(layout_path).name,
            "start": start_rc,
            "goal": goal_rc,
            "edt": {
                "path_len": len(path_edt) if path_edt else 0,
                "time_ms": round(t_edt, 2),
                **cl_edt,
            },
            "costmap": {
                "path_len": len(path_cm) if path_cm else 0,
                "time_ms": round(t_cm, 2),
                **cl_cm,
            },
        })
    return results


def print_summary_table(all_results: List[Dict[str, Any]]):
    """打印按 room_type 分组的汇总表格。"""
    groups = defaultdict(lambda: {"edt": defaultdict(list), "cm": defaultdict(list)})
    for r in all_results:
        rt = r["room_type"]
        for key in ("clearance_min", "clearance_mean", "time_ms", "path_len"):
            groups[rt]["edt"][key].append(r["edt"][key])
            groups[rt]["cm"][key].append(r["costmap"][key])

    sep = "=" * 80
    print(f"\n{sep}")
    print("  EDT vs Costmap — Comparison Results")
    print(f"{sep}")
    header = (
        f"{'room_type':<15} | {'method':<10} | "
        f"{'clearance_min(mm)':<18} | {'clearance_mean(mm)':<19} | "
        f"{'time(ms)':<10} | {'path_len':<10}"
    )
    print(header)
    print("-" * 15 + "-+-" + "-" * 10 + "-+-" + "-" * 18 + "-+-" + "-" * 19 + "-+-" + "-" * 10 + "-+-" + "-" * 10)

    overall_edt_cmin = []
    overall_cm_cmin = []
    for rt in sorted(groups):
        g = groups[rt]
        edt_cmin = np.mean(g["edt"]["clearance_min"]) * 1000
        edt_cmean = np.mean(g["edt"]["clearance_mean"]) * 1000
        edt_t = np.mean(g["edt"]["time_ms"])
        edt_l = np.mean(g["edt"]["path_len"])
        cm_cmin = np.mean(g["cm"]["clearance_min"]) * 1000
        cm_cmean = np.mean(g["cm"]["clearance_mean"]) * 1000
        cm_t = np.mean(g["cm"]["time_ms"])
        cm_l = np.mean(g["cm"]["path_len"])

        overall_edt_cmin.extend([v * 1000 for v in g["edt"]["clearance_min"]])
        overall_cm_cmin.extend([v * 1000 for v in g["cm"]["clearance_min"]])

        print(f"{rt:<15} | {'EDT':<10} | {edt_cmin:>16.1f} | {edt_cmean:>17.1f} | {edt_t:>8.1f} | {edt_l:>8.1f}")
        print(f"{'':<15} | {'costmap':<10} | {cm_cmin:>16.1f} | {cm_cmean:>17.1f} | {cm_t:>8.1f} | {cm_l:>8.1f}")
        print("-" * 15 + "-+-" + "-" * 10 + "-+-" + "-" * 18 + "-+-" + "-" * 19 + "-+-" + "-" * 10 + "-+-" + "-" * 10)

    if overall_edt_cmin and overall_cm_cmin:
        imp = (np.mean(overall_edt_cmin) - np.mean(overall_cm_cmin)) / max(1e-6, np.mean(overall_cm_cmin)) * 100
        print(f"\n  Summary: EDT clearance_min improvement over costmap: {imp:.1f}%")
        print(f"  (EDT mean clearance_min = {np.mean(overall_edt_cmin):.1f} mm, "
              f"Costmap mean clearance_min = {np.mean(overall_cm_cmin):.1f} mm)")

    print(f"{sep}\n")


def run_synthetic_benchmark():
    """无 FloorplanQA 数据时使用合成数据运行快速对比。"""
    print("INFO: FloorplanQA data not found, using synthetic layouts for quick benchmark.")
    print("      After downloading FloorplanQA, run: python benchmark/bench_edt_vs_costmap.py --data_dir <path>\n")

    np.random.seed(42)
    results = []
    for i in range(10):
        H, W = 60 + np.random.randint(20), 60 + np.random.randint(20)
        grid = np.ones((H, W), dtype=np.uint8)
        # 添加一些障碍物
        for _ in range(np.random.randint(2, 6)):
            rh, rw = np.random.randint(5, 15), np.random.randint(5, 15)
            rr, rc = np.random.randint(5, H - 10), np.random.randint(5, W - 10)
            grid[rr:rr + rh, rc:rc + rw] = 0
        df = compute_distance_field(grid)
        rpx = 0.25 / 0.05  # robot_radius_m / resolution
        pairs = _pick_random_start_goal(grid, df, rpx, n_pairs=3)
        room_types = ["bedroom", "kitchen", "living_room"]

        # Hoist costmap once per layout
        _, bg = costmap_baseline(grid, rpx)

        for start_rc, goal_rc in pairs:
            t0 = time.perf_counter()
            p_edt = astar_shortest_path(grid, df, start_rc, goal_rc, robot_radius_px=rpx, safety_weight=1.0)
            t_edt = (time.perf_counter() - t0) * 1000
            c_edt = path_clearance_stats(p_edt, df, 0.05) if p_edt else {"clearance_min": 0, "clearance_mean": 0, "clearance_below_1px": 1.0}

            t0 = time.perf_counter()
            p_cm = _astar_binary(bg, start_rc, goal_rc)
            t_cm = (time.perf_counter() - t0) * 1000
            c_cm = path_clearance_stats(p_cm, df, 0.05) if p_cm else {"clearance_min": 0, "clearance_mean": 0, "clearance_below_1px": 1.0}

            results.append({
                "room_type": room_types[i % len(room_types)],
                "layout": f"synthetic_{i}",
                "start": start_rc, "goal": goal_rc,
                "edt": {"path_len": len(p_edt) if p_edt else 0, "time_ms": round(t_edt, 2), **c_edt},
                "costmap": {"path_len": len(p_cm) if p_cm else 0, "time_ms": round(t_cm, 2), **c_cm},
            })

    return results


def main():
    parser = argparse.ArgumentParser(description="EDT vs Costmap 2D 路径规划对比基准")
    parser.add_argument("--data_dir", type=str, default=None,
                        help="FloorplanQA layouts 目录")
    parser.add_argument("--n_layouts", type=int, default=10,
                        help="采样布局数（默认 10）")
    parser.add_argument("--n_pairs", type=int, default=5,
                        help="每布局的起终点对数")
    parser.add_argument("--output", type=str, default="benchmark/results/edt_vs_costmap_results.json")
    parser.add_argument("--seed", type=int, default=42,
                        help="随机种子（保证可复现）")
    parser.add_argument("--workers", type=int, default=1,
                        help="并行进程数（默认 1=串行，0=自动检测 CPU 核数）")
    args = parser.parse_args()

    np.random.seed(args.seed)

    all_results: List[Dict[str, Any]] = []

    if args.data_dir and Path(args.data_dir).exists():
        layout_files = sorted(Path(args.data_dir).rglob("*.json"))[:args.n_layouts]
        print(f"Found {len(layout_files)} layout files (seed={args.seed})")

        n_workers = args.workers if args.workers > 0 else os.cpu_count() or 4
        if n_workers > 1:
            from concurrent.futures import ProcessPoolExecutor, as_completed
            import itertools
            chunk_size = max(1, len(layout_files) // n_workers)
            with ProcessPoolExecutor(max_workers=n_workers) as executor:
                futures = {
                    executor.submit(run_single_comparison, str(lf), 0.25, 0.05, args.n_pairs): lf
                    for lf in layout_files
                }
                for future in as_completed(futures):
                    try:
                        all_results.extend(future.result())
                    except Exception as e:
                        print(f"  Worker failed for {futures[future].name}: {e}")
            print(f"  (Processed {len(layout_files)} layouts with {n_workers} workers)")
        else:
            for lf in layout_files:
                r = run_single_comparison(str(lf), n_pairs=args.n_pairs)
                all_results.extend(r)
    else:
        all_results = run_synthetic_benchmark()

    if not all_results:
        print("ERROR: No benchmark results. Check data path.")
        return

    print_summary_table(all_results)

    out_path = Path(args.output)
    out_path.parent.mkdir(parents=True, exist_ok=True)
    with open(out_path, "w", encoding="utf-8") as f:
        json.dump(all_results, f, ensure_ascii=False, indent=2, default=str)
    print(f"Results saved to: {out_path}")


if __name__ == "__main__":
    main()
