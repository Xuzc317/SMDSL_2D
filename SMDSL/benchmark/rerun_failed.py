"""
rerun_failed.py — 补跑 MemoryError 失败的地图索引

用法:
    python rerun_failed.py --csv results/benchmark_20260616_022317.csv --workers 4
    python rerun_failed.py --csv results/benchmark_20260616_022317.csv --workers 2 --dry-run  # 仅列出失败索引
"""

from __future__ import annotations

import argparse
import csv
import sys
import time
from multiprocessing import Pool
from pathlib import Path
from typing import Any

import numpy as np

from procedural_maps import generate_single_map, DIFFICULTY_PRESETS
from experiment_runner import run_experiment, CSV_COLUMNS


def _derive_map_config(
    idx: int, global_seed: int, difficulty: str,
) -> tuple[str, int, int]:
    """从 index 确定性推导地图配置（与原版完全一致）。"""
    rng = np.random.RandomState(global_seed + idx * 7919 + 1)
    d = rng.choice(["easy","medium","hard","extreme"]) if difficulty == "mixed" else difficulty
    nw, ww = DIFFICULTY_PRESETS[d]
    nw = max(2, nw + rng.randint(-1, 2))
    ww = max(nw + 8, ww + rng.randint(-3, 4))
    return d, nw, ww


def _worker(args: tuple) -> list[dict[str, Any]]:
    """与原版 _worker 完全一致，仅用于补跑。"""
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


def extract_failed_indices(csv_path: str) -> list[int]:
    """从已有 CSV 中提取所有 error 行的 map_index。"""
    failed = []
    with open(csv_path, "r") as f:
        reader = csv.DictReader(f)
        for row in reader:
            if row.get("error", ""):
                failed.append(int(row["map_index"]))
    # 去重排序
    return sorted(set(failed))


def main():
    parser = argparse.ArgumentParser(description="补跑失败的 benchmark 地图")
    parser.add_argument("--csv", required=True, help="已有结果 CSV 路径")
    parser.add_argument("--workers", type=int, default=4, help="并行 worker 数 (default: 4)")
    parser.add_argument("--height", type=int, default=200)
    parser.add_argument("--width", type=int, default=300)
    parser.add_argument("--difficulty", type=str, default="mixed")
    parser.add_argument("--robot-radius", type=float, default=3.0)
    parser.add_argument("--safety-weight", type=float, default=2.0)
    parser.add_argument("--resolution", type=float, default=0.05)
    parser.add_argument("--seed", type=int, default=42)
    parser.add_argument("--dry-run", action="store_true", help="仅列出失败索引，不执行")
    args = parser.parse_args()

    failed_indices = extract_failed_indices(args.csv)
    print(f"发现 {len(failed_indices)} 个失败地图索引 (MemoryError)")

    if args.dry_run:
        print("失败索引列表:")
        for idx in failed_indices:
            print(f"  {idx}")
        return

    if not failed_indices:
        print("没有需要补跑的索引，退出。")
        return

    # 输出路径：在原 CSV 同目录下生成补跑结果
    csv_path = Path(args.csv)
    ts = time.strftime("%Y%m%d_%H%M%S")
    output_csv = csv_path.parent / f"rerun_{csv_path.stem}_{ts}.csv"

    # 写表头
    with open(output_csv, "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=CSV_COLUMNS)
        w.writeheader()

    # 构建任务列表（仅失败索引）
    tasks = [
        (i, args.seed, args.height, args.width, args.difficulty,
         args.robot_radius, args.safety_weight, args.resolution)
        for i in failed_indices
    ]

    t_start = time.time()
    ok = 0
    fail = 0
    n_total = len(failed_indices)

    print(f"补跑 {n_total} 张地图 × 2 方法, workers={args.workers}")
    print(f"  output: {output_csv}")
    print(f"{'─' * 60}")

    with Pool(args.workers) as pool:
        for i, rows in enumerate(pool.imap_unordered(_worker, tasks)):
            has_error = any(r.get("error", "") for r in rows if r)
            if has_error:
                fail += 1
            else:
                ok += 1

            # 写 CSV
            with open(output_csv, "a", newline="") as f:
                w = csv.DictWriter(f, fieldnames=CSV_COLUMNS)
                for row in rows:
                    w.writerow(row)

            if (i + 1) % 20 == 0 or (i + 1) == n_total:
                elapsed = time.time() - t_start
                rate = (i + 1) / elapsed
                eta = (n_total - i - 1) / rate if rate > 0 else 0
                pct = (i + 1) / n_total * 100
                sys.stdout.write(
                    f"\r  [{i+1:>4d}/{n_total}] {pct:5.1f}%  "
                    f"{rate:5.1f} maps/s  ETA: {eta:5.0f}s  "
                    f"OK:{ok}  FAIL:{fail}  "
                )
                sys.stdout.flush()

    elapsed = time.time() - t_start
    print(f"\n{'─' * 60}")
    print(f"  补跑完成: {ok} OK, {fail} 仍失败, {elapsed:.1f}s")
    print(f"  补跑结果: {output_csv}")

    # 提示合并命令
    print(f"\n  → 将补跑结果合并到原 CSV:")
    print(f"     合并后: {csv_path} (替换 error 行)")
    print(f"     备份原文件后再合并!")


if __name__ == "__main__":
    main()
