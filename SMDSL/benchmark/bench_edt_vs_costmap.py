"""
bench_edt_vs_costmap.py — Phase 1.2: EDT vs Costmap 对比基准

数据集：
  - FloorplanQA 200 layouts（bedroom×50, kitchen×50, living_room×50, hssd×50）
    → 当数据不可用时回退到合成数据
  - 每 layout n_pairs 对随机起终点（排除不可达对）

对比：
  - Ours (EDT):    distance_transform_edt + safety_aware_astar_flood
  - Baseline:      binary_dilation + 二值 A*

指标：
  - clearance_min (mm)              ← 主要指标
  - clearance_mean (mm)
  - clearance < robot_radius 占比 (%)
  - 窄通道（宽度 ≤ 1.5×机器人直径）通过率 (%)
  - path_length (m)
  - planning_time (ms)

输出：
  - 按 room_type 分组的汇总表格 (console)
  - 箱线图 → benchmark/results/boxplot_clearance.png
  - 原始数据 JSON → benchmark/results/bench_results.json

用法::

    python benchmark/bench_edt_vs_costmap.py
    python benchmark/bench_edt_vs_costmap.py --n_layouts 50 --n_pairs 5
    python benchmark/bench_edt_vs_costmap.py --use_floorplanqa  # 需数据已下载
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

_RESULTS_DIR = Path(__file__).resolve().parent / "results"


# ═══════════════════════════════════════════════════════════════
# A* wrappers
# ═══════════════════════════════════════════════════════════════

def _costmap_astar(
    grid: np.ndarray,
    start_rc: Tuple[int, int],
    goal_rc: Tuple[int, int],
    robot_radius_px: float,
) -> Optional[List[Tuple[int, int]]]:
    """binary_dilation → 二值可通行图 → A*（Baseline 方案）。"""
    from scipy import ndimage
    from cad_parser.astar_topology import astar_shortest_path

    r_px = max(1, int(math.ceil(robot_radius_px)))
    from skimage.morphology import disk
    struct = disk(r_px)
    inflated = ndimage.binary_dilation(grid == 0, structure=struct)
    costmap = (~inflated).astype(np.uint8)

    H, W = costmap.shape
    half_px = max(0.5, r_px * 0.5)
    fake_df = np.where(costmap == 1, half_px, 0.0).astype(np.float32)
    if not (0 <= start_rc[0] < H and 0 <= start_rc[1] < W and costmap[start_rc] == 1):
        return None
    if not (0 <= goal_rc[0] < H and 0 <= goal_rc[1] < W and costmap[goal_rc] == 1):
        return None

    return astar_shortest_path(
        costmap, fake_df, start_rc, goal_rc,
        robot_radius_px=half_px,
        safety_weight=0.1, max_steps=2_000_000,
    )


def _edt_astar(
    grid: np.ndarray,
    distance_field: np.ndarray,
    start_rc: Tuple[int, int],
    goal_rc: Tuple[int, int],
    robot_radius_px: float,
) -> Optional[List[Tuple[int, int]]]:
    """EDT 距离场 → safety_aware_astar_flood（Ours 方案）。"""
    from cad_parser.astar_topology import astar_shortest_path
    return astar_shortest_path(
        grid, distance_field, start_rc, goal_rc,
        robot_radius_px=robot_radius_px,
        safety_weight=0.5, max_steps=2_000_000,
    )


# ═══════════════════════════════════════════════════════════════
# Metrics
# ═══════════════════════════════════════════════════════════════

def _path_clearance(
    path_rc: List[Tuple[int, int]],
    distance_field: np.ndarray,
    resolution: float,
) -> Tuple[float, float, float]:
    """(clearance_min_m, clearance_mean_m, fraction_below_robot_radius)."""
    if not path_rc:
        return 0.0, 0.0, 0.0
    vals_mm = np.array([distance_field[r, c] * resolution * 1000 for r, c in path_rc])
    below = (vals_mm < (resolution * 1000 * 5)).mean()  # < 5 px in mm
    return float(vals_mm.min()), float(vals_mm.mean()), float(below)


def _narrow_passage_exists(
    grid: np.ndarray,
    distance_field: np.ndarray,
    robot_radius_px: float,
) -> bool:
    """检测是否存在窄通道（宽度 ≤ 1.5×机器人直径）。"""
    narrow_thresh = robot_radius_px * 1.5
    free = (grid == 1) & (distance_field > 0)
    if not free.any():
        return False
    narrow = free & (distance_field < narrow_thresh)
    # 窄像素占自由空间 > 5% 即认为存在窄通道
    return narrow.sum() / max(free.sum(), 1) > 0.05


def _narrow_passage_detection(
    grid: np.ndarray,
    robot_radius_px: float,
) -> Tuple[bool, int]:
    """
    检测布局中是否存在狭窄通道（通道宽度 ≤ 1.5×机器人直径）。

    方法：对自由空间做 distance_transform，找到局部最小值区域（距离场 < 阈值），
    再通过连通分量分析统计独立的窄区域数量。

    Args:
        grid: H×W uint8 占据栅格（0=墙, 1=自由）。
        robot_radius_px: 机器人半径（像素）。

    Returns:
        (exists, narrow_regions): exists 表示是否存在窄通道；
        narrow_regions 表示独立窄区域的数量。
    """
    from scipy.ndimage import distance_transform_edt
    df = distance_transform_edt(grid.astype(np.uint8)).astype(np.float32)

    narrow_thresh = robot_radius_px * 1.5
    free = (grid == 1) & (df > 0)
    if not free.any():
        return False, 0

    narrow_mask = free & (df < narrow_thresh)
    if not narrow_mask.any():
        return False, 0

    # 连通分量分析 → 独立窄区域数量
    from scipy.ndimage import label
    labeled, n_regions = label(narrow_mask)
    narrow_frac = narrow_mask.sum() / free.sum()
    exists = narrow_frac > 0.05

    return exists, n_regions


# ═══════════════════════════════════════════════════════════════
# Seed point generation
# ═══════════════════════════════════════════════════════════════

def _find_seed_pairs(
    grid: np.ndarray,
    distance_field: np.ndarray,
    robot_radius_px: float,
    n_pairs: int = 5,
    rng: Optional[np.random.Generator] = None,
) -> List[Tuple[Tuple[int, int], Tuple[int, int]]]:
    """在自由空间中找 n 对远离的种子点。排除不可达对。"""
    if rng is None:
        rng = np.random.default_rng(42)
    H, W = grid.shape
    free = (grid == 1) & (distance_field >= robot_radius_px)
    rows, cols = np.where(free)
    if len(rows) < 2:
        rows, cols = np.where(grid == 1)
    if len(rows) < 2:
        return [((0, 0), (H - 1, W - 1))]

    pts = np.column_stack([rows, cols]).astype(np.float32)
    pairs: List[Tuple[Tuple[int, int], Tuple[int, int]]] = []

    for _ in range(n_pairs):
        # Pick two random distant points
        idx = rng.choice(len(pts), size=min(10, len(pts)), replace=False)
        subset = pts[idx]
        d_mat = np.sqrt(np.sum((subset[:, None] - subset[None, :]) ** 2, axis=-1))
        best = np.unravel_index(np.argmax(d_mat), d_mat.shape)
        s = (int(subset[best[0], 0]), int(subset[best[0], 1]))
        g = (int(subset[best[1], 0]), int(subset[best[1], 1]))
        if s != g:
            pairs.append((s, g))

    return pairs or [((0, 0), (H - 1, W - 1))]


# ═══════════════════════════════════════════════════════════════
# Single layout benchmark
# ═══════════════════════════════════════════════════════════════

def run_layout_benchmark(
    grid: np.ndarray,
    room_type: str,
    resolution: float,
    robot_radius_px: float,
    n_pairs: int = 5,
    rng: Optional[np.random.Generator] = None,
) -> List[Dict[str, Any]]:
    """在单个布局上运行 EDT vs Costmap（每对起终点一条记录）。"""
    from cad_parser.astar_topology import bridge_thin_walls, remove_exterior_freespace
    from scipy.ndimage import distance_transform_edt

    if rng is None:
        rng = np.random.default_rng(42)

    grid = remove_exterior_freespace(grid, close_gaps_px=1)
    grid = bridge_thin_walls(grid, kernel_px=2)
    df = distance_transform_edt(grid.astype(np.uint8)).astype(np.float32)

    has_narrow = _narrow_passage_exists(grid, df, robot_radius_px)
    pairs = _find_seed_pairs(grid, df, robot_radius_px, n_pairs, rng)

    results: List[Dict[str, Any]] = []
    for pi, (start_rc, goal_rc) in enumerate(pairs):
        rec: Dict[str, Any] = {
            "room_type": room_type,
            "pair_id": pi,
            "has_narrow_passage": has_narrow,
        }

        # ── Costmap baseline ──
        t0 = time.perf_counter()
        path_cm = _costmap_astar(grid, start_rc, goal_rc, robot_radius_px)
        rec["costmap_time_ms"] = (time.perf_counter() - t0) * 1000.0
        rec["costmap_path_found"] = path_cm is not None and len(path_cm) >= 2
        if rec["costmap_path_found"]:
            cm_min, cm_mean, cm_below = _path_clearance(path_cm, df, resolution)
            rec["costmap_clearance_min_mm"] = cm_min
            rec["costmap_clearance_mean_mm"] = cm_mean
            rec["costmap_below_radius_pct"] = cm_below * 100
            rec["costmap_path_length_m"] = len(path_cm) * resolution
        else:
            rec["costmap_clearance_min_mm"] = 0.0
            rec["costmap_clearance_mean_mm"] = 0.0
            rec["costmap_below_radius_pct"] = 100.0
            rec["costmap_path_length_m"] = 0.0

        # ── EDT (Ours) ──
        t0 = time.perf_counter()
        path_edt = _edt_astar(grid, df, start_rc, goal_rc, robot_radius_px)
        rec["edt_time_ms"] = (time.perf_counter() - t0) * 1000.0
        rec["edt_path_found"] = path_edt is not None and len(path_edt) >= 2
        if rec["edt_path_found"]:
            edt_min, edt_mean, edt_below = _path_clearance(path_edt, df, resolution)
            rec["edt_clearance_min_mm"] = edt_min
            rec["edt_clearance_mean_mm"] = edt_mean
            rec["edt_below_radius_pct"] = edt_below * 100
            rec["edt_path_length_m"] = len(path_edt) * resolution
        else:
            rec["edt_clearance_min_mm"] = 0.0
            rec["edt_clearance_mean_mm"] = 0.0
            rec["edt_below_radius_pct"] = 100.0
            rec["edt_path_length_m"] = 0.0

        # Narrow passage pass rate
        rec["costmap_narrow_pass"] = (
            rec["costmap_path_found"] if has_narrow else None
        )
        rec["edt_narrow_pass"] = (
            rec["edt_path_found"] if has_narrow else None
        )

        results.append(rec)

    return results


# ═══════════════════════════════════════════════════════════════
# Synthetic data generator
# ═══════════════════════════════════════════════════════════════

def generate_synthetic_grid(
    rng: np.random.Generator,
    size: int = 200,
    room_type: str = "synthetic",
) -> Tuple[np.ndarray, str]:
    """生成随机合成 floor plan 栅格。含带缺口的墙体形成窄通道。"""
    grid = np.ones((size, size), dtype=np.uint8)
    n_walls = rng.integers(4, 12)
    for _ in range(n_walls):
        orientation = rng.choice(["h", "v"])
        if orientation == "h":
            y = rng.integers(size // 8, 7 * size // 8)
            x1 = rng.integers(0, size // 4)
            x2 = rng.integers(3 * size // 4, size)
            grid[y - 1:y + 2, x1:x2] = 0
            # Narrow gap
            gap = rng.integers(size // 10, size // 4)
            gap_x = rng.integers(size // 4, 3 * size // 4)
            grid[y - 1:y + 2, gap_x:gap_x + gap] = 1
        else:
            x = rng.integers(size // 8, 7 * size // 8)
            y1 = rng.integers(0, size // 4)
            y2 = rng.integers(3 * size // 4, size)
            grid[y1:y2, x - 1:x + 2] = 0
            gap = rng.integers(size // 10, size // 4)
            gap_y = rng.integers(size // 4, 3 * size // 4)
            grid[gap_y:gap_y + gap, x - 1:x + 2] = 1
    return grid, room_type


# ═══════════════════════════════════════════════════════════════
# Visualization
# ═══════════════════════════════════════════════════════════════

def _generate_boxplots(
    all_results: List[Dict[str, Any]],
    out_dir: Path,
) -> str:
    """生成 clearance_min 箱线图：按 room_type 分组，EDT vs Costmap 并排。"""
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    # Group by room_type
    from collections import defaultdict
    by_room: Dict[str, Dict[str, List[float]]] = defaultdict(
        lambda: {"edt": [], "costmap": []}
    )
    for r in all_results:
        rt = r.get("room_type", "unknown")
        if r.get("edt_path_found"):
            by_room[rt]["edt"].append(r.get("edt_clearance_min_mm", 0.0))
        if r.get("costmap_path_found"):
            by_room[rt]["costmap"].append(r.get("costmap_clearance_min_mm", 0.0))

    room_types = sorted(by_room.keys())
    n_rooms = len(room_types)
    if n_rooms == 0:
        return ""

    fig, ax = plt.subplots(figsize=(max(8, n_rooms * 2.5), 6))
    positions_edt = []
    positions_cm = []
    labels = []

    for i, rt in enumerate(room_types):
        x_edt = i * 2.5 + 0.5
        x_cm = i * 2.5 + 1.5
        edt_data = by_room[rt]["edt"]
        cm_data = by_room[rt]["costmap"]
        if edt_data:
            bp1 = ax.boxplot(edt_data, positions=[x_edt], widths=0.6,
                             patch_artist=True, showfliers=True,
                             boxprops=dict(facecolor="#4CAF50", alpha=0.6),
                             medianprops=dict(color="darkgreen", linewidth=2))
            positions_edt.append(x_edt)
        if cm_data:
            bp2 = ax.boxplot(cm_data, positions=[x_cm], widths=0.6,
                             patch_artist=True, showfliers=True,
                             boxprops=dict(facecolor="#FF9800", alpha=0.6),
                             medianprops=dict(color="darkorange", linewidth=2))
            positions_cm.append(x_cm)
        labels.append(rt)

    ax.set_xticks([i * 2.5 + 1.0 for i in range(n_rooms)])
    ax.set_xticklabels(labels, rotation=30, ha="right", fontsize=9)
    ax.set_ylabel("clearance_min (mm)")
    ax.set_title("EDT vs Costmap: Minimum Clearance by Room Type")
    ax.axhline(y=0, color="black", linewidth=0.5)

    # Legend
    from matplotlib.patches import Patch
    legend_elements = [
        Patch(facecolor="#4CAF50", alpha=0.6, label="EDT (Ours)"),
        Patch(facecolor="#FF9800", alpha=0.6, label="Costmap (Baseline)"),
    ]
    ax.legend(handles=legend_elements, loc="upper right")

    plt.tight_layout()
    out_path = out_dir / "boxplot_clearance.png"
    fig.savefig(out_path, dpi=150, bbox_inches="tight")
    plt.close(fig)
    return str(out_path)


def _generate_comparison_boxplots(
    results_df: "pd.DataFrame",
    output_path: str,
) -> None:
    """
    生成 EDT vs Costmap 的并排箱线图。

    指标：clearance_min、clearance_mean
    分组：按 room_type
    输出：PNG 保存到 output_path
    """
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    room_types = sorted(results_df["room_type"].unique())
    n_rooms = len(room_types)
    if n_rooms == 0:
        return

    # Two panels: clearance_min (top) + clearance_mean (bottom)
    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(max(8, n_rooms * 2.5), 10))

    for ax, metric, title in [
        (ax1, "edt_clearance_min_mm", "clearance_min (mm)"),
        (ax2, "edt_clearance_mean_mm", "clearance_mean (mm)"),
    ]:
        for i, rt in enumerate(room_types):
            subset = results_df[results_df["room_type"] == rt]
            edt_vals = subset.loc[subset["edt_path_found"], metric.replace("edt_", "edt_")].dropna()
            cm_vals = subset.loc[subset["costmap_path_found"], metric.replace("edt_", "costmap_")].dropna()

            x_edt = i * 2.5 + 0.5
            x_cm = i * 2.5 + 1.5
            if len(edt_vals) > 0:
                ax.boxplot(edt_vals, positions=[x_edt], widths=0.6,
                           patch_artist=True, showfliers=True,
                           boxprops=dict(facecolor="#4CAF50", alpha=0.6),
                           medianprops=dict(color="darkgreen", linewidth=2))
            if len(cm_vals) > 0:
                ax.boxplot(cm_vals, positions=[x_cm], widths=0.6,
                           patch_artist=True, showfliers=True,
                           boxprops=dict(facecolor="#FF9800", alpha=0.6),
                           medianprops=dict(color="darkorange", linewidth=2))

        ax.set_xticks([i * 2.5 + 1.0 for i in range(n_rooms)])
        ax.set_xticklabels(room_types, rotation=30, ha="right", fontsize=9)
        ax.set_ylabel(title)
        ax.set_title(f"EDT vs Costmap: {title} by Room Type")
        ax.axhline(y=0, color="black", linewidth=0.5)

    from matplotlib.patches import Patch
    legend_elements = [
        Patch(facecolor="#4CAF50", alpha=0.6, label="EDT (Ours)"),
        Patch(facecolor="#FF9800", alpha=0.6, label="Costmap (Baseline)"),
    ]
    ax1.legend(handles=legend_elements, loc="upper right")

    plt.tight_layout()
    Path(output_path).parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(output_path, dpi=150, bbox_inches="tight")
    plt.close(fig)


def _print_summary_table(all_results: List[Dict[str, Any]]) -> None:
    """打印按 room_type 分组的汇总表格。"""
    from collections import defaultdict
    by_room: Dict[str, List[Dict[str, Any]]] = defaultdict(list)
    for r in all_results:
        by_room[r.get("room_type", "unknown")].append(r)

    header = f"{'Room Type':<20} {'N':>5} {'EDT OK%':>8} {'CM OK%':>8} "
    header += f"{'EDT min':>9} {'CM min':>9} {'Δ mm':>8} {'EDT ms':>8} {'CM ms':>8}"
    sep = "=" * len(header)
    print(f"\n{sep}")
    print(header)
    print(sep)

    for rt in sorted(by_room.keys()):
        recs = by_room[rt]
        n = len(recs)
        edt_ok = sum(1 for r in recs if r.get("edt_path_found")) / max(n, 1) * 100
        cm_ok = sum(1 for r in recs if r.get("costmap_path_found")) / max(n, 1) * 100
        edt_min = np.mean([r.get("edt_clearance_min_mm", 0)
                          for r in recs if r.get("edt_path_found")])
        cm_min = np.mean([r.get("costmap_clearance_min_mm", 0)
                         for r in recs if r.get("costmap_path_found")])
        delta = edt_min - cm_min
        edt_t = np.mean([r.get("edt_time_ms", 0) for r in recs])
        cm_t = np.mean([r.get("costmap_time_ms", 0) for r in recs])
        print(f"{rt:<20} {n:>5} {edt_ok:>7.1f}% {cm_ok:>7.1f}% "
              f"{edt_min:>8.1f} {cm_min:>8.1f} {delta:>+7.1f} "
              f"{edt_t:>7.0f} {cm_t:>7.0f}")

    # ── Narrow passage analysis ──
    narrow_recs = [r for r in all_results if r.get("has_narrow_passage")]
    if narrow_recs:
        edt_np = sum(1 for r in narrow_recs if r.get("edt_narrow_pass")) / len(narrow_recs) * 100
        cm_np = sum(1 for r in narrow_recs if r.get("costmap_narrow_pass")) / len(narrow_recs) * 100
        print(f"\n  Narrow passage ({len(narrow_recs)}/{len(all_results)} layouts):")
        print(f"    EDT pass rate:  {edt_np:.1f}%")
        print(f"    Costmap pass rate: {cm_np:.1f}%")

    print(sep)


# ═══════════════════════════════════════════════════════════════
# Main
# ═══════════════════════════════════════════════════════════════

def main(argv: Optional[List[str]] = None) -> int:
    parser = argparse.ArgumentParser(
        description="EDT vs Costmap — 安全空间表示对比基准"
    )
    parser.add_argument("--n_layouts", type=int, default=50,
                        help="合成布局数量 (默认 50)")
    parser.add_argument("--n_pairs", type=int, default=5,
                        help="每布局起终点对数 (默认 5)")
    parser.add_argument("--resolution", type=float, default=0.05,
                        help="栅格分辨率 m/px (默认 0.05)")
    parser.add_argument("--robot_radius_m", type=float, default=0.25,
                        help="机器人半径 m (默认 0.25)")
    parser.add_argument("--use_floorplanqa", action="store_true",
                        help="使用 FloorplanQA 数据（需已下载）")
    parser.add_argument("--floorplanqa_root", type=str,
                        default="SMDSL/data/cad_samples/floorplanqa/layouts",
                        help="FloorplanQA 根目录")
    parser.add_argument("--output", type=str, default=None,
                        help="结果 JSON 输出路径 (默认 benchmark/results/)")
    parser.add_argument("--seed", type=int, default=42,
                        help="随机种子")
    args = parser.parse_args(argv)

    res = args.resolution
    robot_r_px = args.robot_radius_m / res

    print(f"{'='*60}")
    print(f"  EDT vs Costmap Benchmark")
    print(f"  Layouts: {args.n_layouts} × {args.n_pairs} pairs")
    print(f"  Resolution: {res} m/px  |  Robot radius: {args.robot_radius_m} m ({robot_r_px:.0f} px)")
    print(f"  Data source: {'FloorplanQA' if args.use_floorplanqa else 'Synthetic'}")
    print(f"{'='*60}")

    rng = np.random.default_rng(args.seed)
    all_results: List[Dict[str, Any]] = []

    if args.use_floorplanqa:
        fp_root = _PROJECT_ROOT / args.floorplanqa_root
        room_types = ["bedroom", "kitchen", "living_room", "hssd"]
        for rt in room_types:
            rt_dir = fp_root / rt
            if not rt_dir.is_dir():
                print(f"  [SKIP] {rt_dir} not found — skipping {rt}")
                continue
            json_files = sorted(rt_dir.glob("*.json"))[:50]
            print(f"\n  Room type: {rt} ({len(json_files)} layouts)")
            for jf in json_files:
                try:
                    from cad_parser.astar_topology import load_cad_vector, rasterize_to_grid
                    cad = load_cad_vector(str(jf))
                    grid, _ = rasterize_to_grid(cad, resolution=res, padding_m=0.5, wall_thickness_m=0.10)
                    results = run_layout_benchmark(grid, rt, res, robot_r_px, args.n_pairs, rng)
                    all_results.extend(results)
                except Exception as e:
                    print(f"    [ERR] {jf.name}: {e}")
    else:
        # Synthetic data with per-room-type groups
        synth_types = {
            "synthetic_bedroom": (args.n_layouts // 4) or 1,
            "synthetic_kitchen": (args.n_layouts // 4) or 1,
            "synthetic_living": (args.n_layouts // 4) or 1,
            "synthetic_narrow": max(1, args.n_layouts - 3 * (args.n_layouts // 4)),
        }
        idx = 0
        for rt, count in synth_types.items():
            for _ in range(count):
                grid, _ = generate_synthetic_grid(rng, size=200, room_type=rt)
                results = run_layout_benchmark(grid, rt, res, robot_r_px, args.n_pairs, rng)
                for rec in results:
                    rec["layout_id"] = idx
                all_results.extend(results)
                idx += 1
                if idx % 10 == 0:
                    print(f"  [{idx}/{args.n_layouts}] layouts processed...")

    # ── Summary ──
    if not all_results:
        print("No results collected. Exiting.")
        return 1

    _print_summary_table(all_results)

    # ── Box plots ──
    _RESULTS_DIR.mkdir(parents=True, exist_ok=True)
    boxplot_path = _generate_boxplots(all_results, _RESULTS_DIR)
    if boxplot_path:
        print(f"\n  Box plot saved to: {boxplot_path}")

    # ── JSON output ──
    out_path = Path(args.output) if args.output else (_RESULTS_DIR / "bench_results.json")
    out_path.parent.mkdir(parents=True, exist_ok=True)
    # Convert numpy types for JSON serialization
    out_path.write_text(
        json.dumps(all_results, indent=2, default=lambda o: float(o) if hasattr(o, "item") else str(o)),
        encoding="utf-8",
    )
    print(f"  Results JSON saved to: {out_path}")

    return 0


if __name__ == "__main__":
    sys.exit(main())
