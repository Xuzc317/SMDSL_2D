"""
main_benchmark.py — 10,000 次 EDT vs Costmap 跑分调度器

用法:
    python main_benchmark.py --n_maps 10000 --workers 8
    python main_benchmark.py --n_maps 100 --workers 2 --quick  # 快速验证

设计:
    - 每张地图在独立 worker 中生成 + 跑分，grid 不离开 worker 进程
    - imap_unordered 流式返回结果，驻内存仅结果 dict（~5 MB）
    - 每 100 张地图打印进度 + 速率 + ETA
    - 结果写入 CSV，name 列与 CSV_COLUMNS 严格一致
"""

from __future__ import annotations

import argparse
import csv
import sys
import time
from multiprocessing import Pool
from typing import Any

import numpy as np

from procedural_maps import generate_single_map, DIFFICULTY_PRESETS
from experiment_runner import run_experiment, CSV_COLUMNS


# ─── Worker（独立进程，纯函数）──────────────────────────────

def _derive_map_config(
    idx: int, global_seed: int, difficulty: str,
) -> tuple[str, int, int]:
    """从 index 确定性推导地图配置（不依赖共享 RNG 状态）。"""
    rng = np.random.RandomState(global_seed + idx * 7919 + 1)
    d = rng.choice(["easy","medium","hard","extreme"]) if difficulty == "mixed" else difficulty
    nw, ww = DIFFICULTY_PRESETS[d]
    nw = max(2, nw + rng.randint(-1, 2))
    ww = max(nw + 8, ww + rng.randint(-3, 4))
    return d, nw, ww


def _worker(args: tuple) -> list[dict[str, Any]]:
    """
    单个 worker 进程入口：生成地图 → 跑分 → 返回 2 行结果。
    所有异常被捕获，单张地图失败不影响整体流程。
    """
    idx, global_seed, height, width, difficulty, rr_px, sw, res = args

    try:
        d, nw, ww = _derive_map_config(idx, global_seed, difficulty)
        map_seed = global_seed + idx * 7919 + 1

        grid, meta = generate_single_map(
            height=height, width=width,
            narrow_width=nw, wide_width=ww,
            seed=map_seed, robot_radius_px=rr_px,
        )
        meta["difficulty"] = d
        meta["map_index"] = idx

        start = tuple(meta["start_rc"])
        goal = tuple(meta["goal_rc"])
        rows = run_experiment(grid, start, goal, rr_px, sw, res, meta)
        return rows  # [edt_row, costmap_row]

    except Exception as e:
        err = f"worker[{idx}]: {type(e).__name__}: {e}"
        row: dict[str, Any] = {
            "map_index": idx,
            "method": "error",
            "difficulty": "unknown",
            "narrow_width_px": -1,
            "wide_width_px": -1,
            "narrow_clearance_edt_px": -1.0,
            "n_obstacles": 0,
            "path_found": False,
            "collision": True,
            "clearance_min_mm": float("nan"),
            "clearance_mean_mm": float("nan"),
            "rho_min_mm": float("nan"),
            "path_length_m": float("nan"),
            "planning_time_ms": float("nan"),
            "n_expanded": 0,
            "error": err,
        }
        return [row]


# ─── CSV 写盘 ─────────────────────────────────────────────

def _init_csv(path: str) -> None:
    """写 CSV 表头。"""
    with open(path, "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=CSV_COLUMNS)
        w.writeheader()


def _append_csv(path: str, rows: list[dict[str, Any]]) -> None:
    """追加多行到 CSV。"""
    with open(path, "a", newline="") as f:
        w = csv.DictWriter(f, fieldnames=CSV_COLUMNS)
        for row in rows:
            w.writerow(row)


# ─── 主流程 ────────────────────────────────────────────────

def run_benchmark(
    n_maps: int = 10000,
    workers: int = 8,
    map_height: int = 200,
    map_width: int = 300,
    difficulty: str = "mixed",
    robot_radius_px: float = 3.0,
    safety_weight: float = 2.0,
    resolution: float = 0.05,
    global_seed: int = 42,
    output_csv: str = "",
) -> str:
    """
    主入口：10,000 张地图 × 2 方法 × multiprocessing 并行。

    返回 CSV 文件路径。
    """
    if not output_csv:
        ts = time.strftime("%Y%m%d_%H%M%S")
        output_csv = f"results/benchmark_{ts}.csv"

    _init_csv(output_csv)

    # 构建任务列表（轻量 tuple，不包含 grid）
    tasks = [
        (i, global_seed, map_height, map_width, difficulty,
         robot_radius_px, safety_weight, resolution)
        for i in range(n_maps)
    ]

    t_start = time.time()
    ok = 0
    fail = 0
    last_flush = 0

    print(f"SMDSL Benchmark — {n_maps} maps × 2 methods = {n_maps * 2} runs")
    print(f"  workers: {workers}  |  seed: {global_seed}")
    print(f"  map: {map_height}×{map_width}  |  difficulty: {difficulty}")
    print(f"  output: {output_csv}")
    print(f"{'─' * 60}")

    with Pool(workers) as pool:
        for i, rows in enumerate(pool.imap_unordered(_worker, tasks)):
            # 统计成功/失败
            has_error = any(r.get("error", "") for r in rows if r)
            if has_error:
                fail += 1
            else:
                ok += 1

            # 写 CSV
            _append_csv(output_csv, rows)

            # 进度报告（每 100 张地图）
            if (i + 1) % 100 == 0 or (i + 1) == n_maps:
                elapsed = time.time() - t_start
                rate = (i + 1) / elapsed
                eta = (n_maps - i - 1) / rate if rate > 0 else 0
                pct = (i + 1) / n_maps * 100
                sys.stdout.write(
                    f"\r  [{i+1:>6d}/{n_maps}] {pct:5.1f}%  "
                    f"{rate:5.1f} maps/s  ETA: {eta:6.0f}s  "
                    f"OK:{ok}  FAIL:{fail}  "
                )
                sys.stdout.flush()
                last_flush = i

    elapsed = time.time() - t_start
    print(f"\n{'─' * 60}")
    print(f"  Done.  {ok} OK  |  {fail} failed  |  {elapsed:.1f}s total")
    print(f"  Results: {output_csv}")

    return output_csv


# ─── CLI ──────────────────────────────────────────────────

def main() -> None:
    parser = argparse.ArgumentParser(
        description="SMDSL Benchmark — 10,000 maps × 2 methods"
    )
    parser.add_argument("--n-maps", type=int, default=10000,
                        help="number of maps (default: 10000)")
    parser.add_argument("--workers", type=int, default=0,
                        help="number of worker processes (default: CPU count)")
    parser.add_argument("--difficulty", type=str, default="mixed",
                        choices=["easy","medium","hard","extreme","mixed"],
                        help="difficulty level (default: mixed)")
    parser.add_argument("--seed", type=int, default=42,
                        help="global random seed (default: 42)")
    parser.add_argument("--height", type=int, default=200,
                        help="map height in pixels (default: 200)")
    parser.add_argument("--width", type=int, default=300,
                        help="map width in pixels (default: 300)")
    parser.add_argument("--robot-radius", type=float, default=3.0,
                        help="robot radius in pixels (default: 3.0, costmap disk=ceil(r)=3)")
    parser.add_argument("--safety-weight", type=float, default=2.0,
                        help="EDT safety weight (default: 2.0, stronger wall repulsion)")
    parser.add_argument("--resolution", type=float, default=0.05,
                        help="meters per pixel (default: 0.05)")
    parser.add_argument("--output", type=str, default="",
                        help="output CSV path (default: results/benchmark_<ts>.csv)")
    parser.add_argument("--quick", action="store_true",
                        help="quick test: 10 maps, 2 workers")

    args = parser.parse_args()

    n_maps = 10 if args.quick else args.n_maps
    workers = args.workers if args.workers > 0 else None  # None = Pool() uses CPU count
    if args.quick and args.workers == 0:
        workers = 2

    run_benchmark(
        n_maps=n_maps,
        workers=workers or 0,
        map_height=args.height,
        map_width=args.width,
        difficulty=args.difficulty,
        robot_radius_px=args.robot_radius,
        safety_weight=args.safety_weight,
        resolution=args.resolution,
        global_seed=args.seed,
        output_csv=args.output,
    )


if __name__ == "__main__":
    main()
