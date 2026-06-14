"""
sweep.py — Demo 1 批量化量化基准

对 FloorplanQA 全部 1,981 份 layout 运行四步流水线，
聚合统计 path/obstacle/inflated/unknown 像素分布，输出：

  - results.csv         每个 layout 一行（room_type, layout_id, 像素分布,
                        path_fraction, deadzone_fraction, elapsed_ms, status）
  - summary.json        room_type 维度 + 全局聚合
  - histograms.png      四联图（path_fraction / deadzone_fraction /
                        max_clearance / elapsed 分布）
  - examples/           每个 room_type 的 best-3 + worst-3 可视化样本

用法::

    # 全量并行扫
    python -m cad_parser.sweep \
        --root data/cad_samples/floorplanqa/layouts \
        --output_dir out/demo1_sweep \
        --workers 8

    # 冷启动验证（仅前 20 个）
    python -m cad_parser.sweep --limit 20 --workers 4
"""

from __future__ import annotations

import argparse
import json
import os
import sys
import time
import traceback
from concurrent.futures import ProcessPoolExecutor, as_completed
from dataclasses import asdict, dataclass, field
from pathlib import Path
from typing import Any, Dict, List, Optional, Tuple

# 包内 / 脚本两种调用方式兼容
try:
    from cad_parser.astar_topology import (
        CLASS_INFLATED,
        CLASS_LOADING,
        CLASS_OBSTACLE,
        CLASS_PATH,
        CLASS_UNKNOWN,
        run_pipeline,
    )
except ImportError:
    HERE = Path(__file__).resolve().parent
    sys.path.insert(0, str(HERE.parent))
    from cad_parser.astar_topology import (
        CLASS_INFLATED,
        CLASS_LOADING,
        CLASS_OBSTACLE,
        CLASS_PATH,
        CLASS_UNKNOWN,
        run_pipeline,
    )


# ──────────────────────────────────────────────
# 单个 layout 的处理（用于多进程 worker）
# ──────────────────────────────────────────────

@dataclass
class LayoutResult:
    """单条 sweep 结果（直接被 CSV / JSON 序列化）。"""
    room_type: str
    layout_id: Any
    cad_path: str
    status: str = "ok"           # ok / error
    error_msg: str = ""

    grid_h: int = 0
    grid_w: int = 0
    n_walls: int = 0
    n_objects: int = 0
    n_windows: int = 0
    n_doors: int = 0
    n_seeds: int = 0

    pixels_path: int = 0
    pixels_obstacle: int = 0
    pixels_inflated: int = 0
    pixels_unknown: int = 0
    pixels_loading: int = 0

    # 可达性指标（基于房间内部像素，剔除墙）
    path_fraction: float = 0.0       # path / (path + inflated + unknown)
    deadzone_fraction: float = 0.0   # unknown / (path + inflated + unknown)
    safety_margin_fraction: float = 0.0  # inflated / (path + inflated + unknown)

    # 距离场
    max_clearance_px: float = 0.0
    max_clearance_m: float = 0.0
    min_clearance_px: float = 0.0
    median_clearance_m: float = 0.0

    elapsed_ms: float = 0.0


def _process_one(args: Tuple[str, Dict[str, Any]]) -> LayoutResult:
    """worker: 处理单个 layout JSON。捕获所有异常。"""
    cad_path, config = args
    p = Path(cad_path)
    # 从父目录推断 room_type（kitchen / bedroom / ...）
    room_type = p.parent.name
    try:
        layout_id = json.loads(p.read_text(encoding="utf-8"))\
            .get("layout_id")
    except Exception:
        layout_id = None

    result = LayoutResult(
        room_type=room_type,
        layout_id=layout_id,
        cad_path=str(p),
    )

    t0 = time.perf_counter()
    try:
        # 关闭 grid 嵌入，sweep 不需要存 H×W 矩阵
        pipe = run_pipeline(
            cad_path=str(p),
            resolution=config["resolution"],
            robot_radius_m=config["robot_radius_m"],
            padding_m=config["padding_m"],
            wall_thickness_m=config["wall_thickness_m"],
            safety_weight=config["safety_weight"],
            max_steps=config["max_steps"],
            include_grid=False,
        )
        elapsed = (time.perf_counter() - t0) * 1000

        cad = pipe["cad_data"]
        topology = pipe["topology"]
        dist = pipe["distance_field"]
        meta = pipe["topology_json"]["metadata"]
        summary = pipe["topology_json"]["summary"]

        H, W = topology.shape
        cp = summary["class_pixels"]
        n_path = int(cp.get("path", 0))
        n_obstacle = int(cp.get("obstacle", 0))
        n_inflated = int(cp.get("inflated_buffer", 0))
        n_unknown = int(cp.get("unknown", 0))
        n_loading = int(cp.get("loading_zone", 0))

        interior = n_path + n_inflated + n_unknown
        result.grid_h = H
        result.grid_w = W
        result.n_walls = len(cad.get("walls", []))
        result.n_objects = len(cad.get("objects", []))
        result.n_windows = len(cad.get("windows", []))
        result.n_doors = len(cad.get("doors", []))
        result.n_seeds = summary.get("n_seeds", 0)

        result.pixels_path = n_path
        result.pixels_obstacle = n_obstacle
        result.pixels_inflated = n_inflated
        result.pixels_unknown = n_unknown
        result.pixels_loading = n_loading

        if interior > 0:
            result.path_fraction = n_path / interior
            result.deadzone_fraction = n_unknown / interior
            result.safety_margin_fraction = n_inflated / interior

        # 距离场指标：只在自由像素（grid == 1）上做统计
        free_mask = pipe["grid"] == 1
        if free_mask.any():
            free_dist = dist[free_mask]
            result.max_clearance_px = float(free_dist.max())
            result.min_clearance_px = float(free_dist.min())
            result.median_clearance_m = float(
                float(__import__("numpy").median(free_dist))
                * meta["resolution_m_per_px"]
            )
        result.max_clearance_m = (
            result.max_clearance_px * meta["resolution_m_per_px"]
        )

        result.elapsed_ms = elapsed
    except Exception as e:  # noqa: BLE001
        result.status = "error"
        result.error_msg = f"{type(e).__name__}: {e}"
        # 不打印 traceback 到 worker stdout 避免污染 tqdm 进度条
        result.elapsed_ms = (time.perf_counter() - t0) * 1000

    return result


# ──────────────────────────────────────────────
# 发现 layouts
# ──────────────────────────────────────────────

def discover_layouts(root: Path) -> List[Path]:
    """返回 root 下所有 layout JSON 路径（按 room_type 子目录组织）。"""
    paths: List[Path] = []
    for sub in sorted(root.iterdir()):
        if not sub.is_dir():
            continue
        for p in sorted(sub.glob("*.json")):
            paths.append(p)
    return paths


# ──────────────────────────────────────────────
# 聚合
# ──────────────────────────────────────────────

def aggregate(results: List[LayoutResult]) -> Dict[str, Any]:
    """按 room_type + 全局两个维度聚合统计。"""
    import numpy as np

    ok = [r for r in results if r.status == "ok"]
    err = [r for r in results if r.status != "ok"]

    def stats(values: List[float]) -> Dict[str, float]:
        if not values:
            return {"n": 0, "mean": 0.0, "median": 0.0,
                    "p10": 0.0, "p90": 0.0, "min": 0.0, "max": 0.0}
        v = np.asarray(values, dtype=np.float64)
        return {
            "n": int(v.size),
            "mean": float(v.mean()),
            "median": float(np.median(v)),
            "p10": float(np.percentile(v, 10)),
            "p90": float(np.percentile(v, 90)),
            "min": float(v.min()),
            "max": float(v.max()),
        }

    def by_metric(rows: List[LayoutResult]) -> Dict[str, Any]:
        return {
            "n_layouts": len(rows),
            "path_fraction": stats([r.path_fraction for r in rows]),
            "deadzone_fraction": stats([r.deadzone_fraction for r in rows]),
            "safety_margin_fraction": stats(
                [r.safety_margin_fraction for r in rows]
            ),
            "max_clearance_m": stats([r.max_clearance_m for r in rows]),
            "median_clearance_m": stats(
                [r.median_clearance_m for r in rows]
            ),
            "elapsed_ms": stats([r.elapsed_ms for r in rows]),
            "n_seeds": stats([float(r.n_seeds) for r in rows]),
        }

    rooms: Dict[str, List[LayoutResult]] = {}
    for r in ok:
        rooms.setdefault(r.room_type, []).append(r)

    return {
        "total_layouts": len(results),
        "ok": len(ok),
        "errors": len(err),
        "error_samples": [
            {"cad_path": e.cad_path, "error_msg": e.error_msg}
            for e in err[:10]
        ],
        "by_room_type": {rt: by_metric(rs) for rt, rs in rooms.items()},
        "overall": by_metric(ok),
    }


# ──────────────────────────────────────────────
# 直方图四联图
# ──────────────────────────────────────────────

def render_histograms(
    results: List[LayoutResult],
    output_path: Path,
    dpi: int = 130,
) -> None:
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    import numpy as np

    ok = [r for r in results if r.status == "ok"]
    rooms = sorted({r.room_type for r in ok})
    palette = plt.get_cmap("tab10").colors

    fig, axes = plt.subplots(2, 2, figsize=(13, 9), dpi=dpi)
    (ax1, ax2), (ax3, ax4) = axes

    def hist_per_room(ax, getter, *, bins=20, x_range=None,
                      title="", xlabel=""):
        for i, rt in enumerate(rooms):
            vals = [getter(r) for r in ok if r.room_type == rt]
            if not vals:
                continue
            ax.hist(vals, bins=bins, range=x_range,
                    alpha=0.55, label=f"{rt} (n={len(vals)})",
                    color=palette[i % len(palette)])
        ax.set_title(title)
        ax.set_xlabel(xlabel)
        ax.set_ylabel("# layouts")
        ax.legend(fontsize=8, framealpha=0.85)
        ax.grid(alpha=0.3)

    hist_per_room(
        ax1, lambda r: r.path_fraction, bins=25, x_range=(0, 1),
        title="(a) path_fraction = path / (path+inflated+unknown)",
        xlabel="fraction of interior reachable by safety-A*",
    )
    hist_per_room(
        ax2, lambda r: r.deadzone_fraction, bins=25, x_range=(0, 1),
        title="(b) deadzone_fraction = unknown / interior",
        xlabel="fraction unreachable from any seed",
    )
    hist_per_room(
        ax3, lambda r: r.max_clearance_m, bins=30,
        title="(c) max clearance (m) — widest free space",
        xlabel="meters",
    )
    hist_per_room(
        ax4, lambda r: r.elapsed_ms, bins=30,
        title="(d) per-layout elapsed (ms)",
        xlabel="ms",
    )

    fig.suptitle(
        f"Demo 1 sweep — {len(ok)} layouts ok, "
        f"{len(results) - len(ok)} errors",
        fontsize=13,
    )
    fig.tight_layout(rect=(0, 0, 1, 0.96))
    output_path.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(output_path, dpi=dpi, bbox_inches="tight")
    plt.close(fig)


# ──────────────────────────────────────────────
# Best / Worst 案例渲染
# ──────────────────────────────────────────────

def render_extremes(
    results: List[LayoutResult],
    output_dir: Path,
    config: Dict[str, Any],
    top_k: int = 3,
) -> Dict[str, List[Dict[str, Any]]]:
    """对每个 room_type 选 path_fraction 最高/最低各 top_k 个，渲染 PNG。"""
    try:
        from cad_parser.visualize import render_pipeline_quad
    except ImportError:
        from visualize import render_pipeline_quad  # type: ignore

    ok = [r for r in results if r.status == "ok"]
    rooms = sorted({r.room_type for r in ok})
    output_dir.mkdir(parents=True, exist_ok=True)
    catalog: Dict[str, List[Dict[str, Any]]] = {}

    for rt in rooms:
        rs = sorted(
            (r for r in ok if r.room_type == rt),
            key=lambda r: r.path_fraction,
        )
        worst = rs[:top_k]
        best = rs[-top_k:][::-1]
        catalog[rt] = []
        for tag, group in (("worst", worst), ("best", best)):
            for rank, r in enumerate(group, 1):
                pipe = run_pipeline(
                    cad_path=r.cad_path,
                    resolution=config["resolution"],
                    robot_radius_m=config["robot_radius_m"],
                    padding_m=config["padding_m"],
                    wall_thickness_m=config["wall_thickness_m"],
                    safety_weight=config["safety_weight"],
                    max_steps=config["max_steps"],
                    include_grid=False,
                )
                out_png = (
                    output_dir / f"{rt}_{tag}{rank}_id{r.layout_id}.png"
                )
                render_pipeline_quad(
                    pipe, output_path=str(out_png),
                    title_prefix=(
                        f"Demo 1 sweep [{rt} {tag} #{rank}] "
                        f"path_frac={r.path_fraction:.2f}"
                    ),
                )
                catalog[rt].append({
                    "tag": tag,
                    "rank": rank,
                    "layout_id": r.layout_id,
                    "cad_path": r.cad_path,
                    "path_fraction": r.path_fraction,
                    "deadzone_fraction": r.deadzone_fraction,
                    "png": str(out_png),
                })
    return catalog


# ──────────────────────────────────────────────
# 主入口
# ──────────────────────────────────────────────

def main(argv: Optional[List[str]] = None) -> int:
    parser = argparse.ArgumentParser(
        description="Demo 1 sweep — 对全部 FloorplanQA layouts 跑 A* 拓扑流水线"
    )
    parser.add_argument(
        "--root", type=str,
        default="data/cad_samples/floorplanqa/layouts",
        help="layouts/ 根目录",
    )
    parser.add_argument(
        "--output_dir", type=str, default="out/demo1_sweep",
        help="输出目录（CSV + JSON + PNG）",
    )
    parser.add_argument(
        "--resolution", type=float, default=0.05,
        help="栅格分辨率（米/像素）。sweep 用粗一点更快。",
    )
    parser.add_argument(
        "--robot_radius", type=float, default=0.25,
        help="机器人物理半径（米）",
    )
    parser.add_argument("--padding", type=float, default=0.5)
    parser.add_argument("--wall_thickness", type=float, default=0.10)
    parser.add_argument("--safety_weight", type=float, default=0.5)
    parser.add_argument("--max_steps", type=int, default=1_000_000)
    parser.add_argument(
        "--workers", type=int, default=max(1, (os.cpu_count() or 4) - 1),
        help="并行 worker 数（默认 CPU 核数 - 1）",
    )
    parser.add_argument(
        "--limit", type=int, default=0,
        help="只跑前 N 个（冒烟测试用，默认全跑）",
    )
    parser.add_argument(
        "--top_k", type=int, default=3,
        help="每 room_type 渲染 best/worst-K 个样例",
    )
    parser.add_argument(
        "--no_examples", action="store_true",
        help="跳过 best/worst 样例渲染（仅做统计）",
    )
    args = parser.parse_args(argv)

    root = Path(args.root)
    output_dir = Path(args.output_dir)
    examples_dir = output_dir / "examples"
    output_dir.mkdir(parents=True, exist_ok=True)

    layouts = discover_layouts(root)
    if args.limit and args.limit > 0:
        layouts = layouts[: args.limit]
    print(f"[discover] {len(layouts)} layouts under {root}")
    if not layouts:
        print(f"[ERROR] 没有发现 layouts: {root}", file=sys.stderr)
        return 2

    config = {
        "resolution": args.resolution,
        "robot_radius_m": args.robot_radius,
        "padding_m": args.padding,
        "wall_thickness_m": args.wall_thickness,
        "safety_weight": args.safety_weight,
        "max_steps": args.max_steps,
    }

    # ── 多进程并行 ──
    from tqdm import tqdm

    results: List[LayoutResult] = []
    started_at = time.perf_counter()

    if args.workers <= 1:
        for p in tqdm(layouts, desc="sweep", unit="layout"):
            results.append(_process_one((str(p), config)))
    else:
        with ProcessPoolExecutor(max_workers=args.workers) as pool:
            futures = [
                pool.submit(_process_one, (str(p), config))
                for p in layouts
            ]
            with tqdm(total=len(futures), desc="sweep",
                      unit="layout") as bar:
                for f in as_completed(futures):
                    results.append(f.result())
                    bar.update()

    total_elapsed = time.perf_counter() - started_at

    # 结果按原始路径排序（多进程顺序不稳定）
    results.sort(key=lambda r: r.cad_path)

    n_ok = sum(1 for r in results if r.status == "ok")
    n_err = len(results) - n_ok
    print(
        f"[done] ok={n_ok} / err={n_err} / "
        f"total={total_elapsed:.1f}s "
        f"({total_elapsed * 1000 / max(1, len(results)):.0f} ms/layout)"
    )

    # ── 写 CSV ──
    csv_path = output_dir / "results.csv"
    import csv

    fieldnames = list(asdict(LayoutResult(
        room_type="", layout_id=None, cad_path=""
    )).keys())
    with csv_path.open("w", encoding="utf-8", newline="") as fh:
        writer = csv.DictWriter(fh, fieldnames=fieldnames)
        writer.writeheader()
        for r in results:
            writer.writerow(asdict(r))
    print(f"[csv] {csv_path}")

    # ── 写 summary.json ──
    summary = aggregate(results)
    summary["config"] = config
    summary["wall_clock_s"] = total_elapsed
    summary_path = output_dir / "summary.json"
    summary_path.write_text(
        json.dumps(summary, ensure_ascii=False, indent=2),
        encoding="utf-8",
    )
    print(f"[summary] {summary_path}")

    # ── 写 histograms.png ──
    hist_path = output_dir / "histograms.png"
    render_histograms(results, hist_path)
    print(f"[histograms] {hist_path}")

    # ── 写 best/worst 样例 ──
    if not args.no_examples and n_ok > 0:
        catalog = render_extremes(
            results, examples_dir, config, top_k=args.top_k,
        )
        catalog_path = output_dir / "examples_catalog.json"
        catalog_path.write_text(
            json.dumps(catalog, ensure_ascii=False, indent=2),
            encoding="utf-8",
        )
        n_examples = sum(len(v) for v in catalog.values())
        print(f"[examples] {n_examples} PNG → {examples_dir}")

    print("OK")
    return 0


if __name__ == "__main__":
    sys.exit(main())
