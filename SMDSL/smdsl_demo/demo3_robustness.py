"""
demo3_robustness.py — Demo 3 端到端：STL Robustness Degree on Distance Field

把 Demo 1 (CAD → distance_field) 与 Demo 3 (spatial_api_stub.check_stl_constraint
_violation) 串起来，演示 4 种典型轨迹的鲁棒性度量与结构化反馈：

    A. SAFE         — 沿房间中线行走，ρ > 0 全程安全
    B. GRAZE_WALL   — 贴墙行走，ρ ≈ 0 / 轻微违反
    C. THROUGH_OBJ  — 穿过基本柜（base_cabinet_1）→ COLLISION，ρ ≪ 0
    D. OUT_OF_BOUND — 走出房间外（栅格外）→ ρ = -D_safe（场返回 0）

输出物（默认写到 ``out/demo3/``）：
  - ``robustness_report.json``：4 条轨迹 × 1 STL 约束 的逐点 ρ + 违规节点 + 结构化反馈
  - ``trajectories.png``：距离场底图 + 4 条轨迹叠加 + 违规点 ✕ 红标记

CLI 用法::

    python -m smdsl_demo.demo3_robustness \
        --layout data/cad_samples/floorplanqa/layouts/kitchen/room_24.json \
        --d_safe 0.30 --robot_radius 0.25 --resolution 0.05
"""

from __future__ import annotations

import argparse
import json
import math
import sys
from pathlib import Path
from typing import Any, Dict, List, Tuple

import numpy as np

# 让本脚本既能 `python -m smdsl_demo.demo3_robustness` 也能直接 `python ...`
_REPO_ROOT = Path(__file__).resolve().parent.parent
if str(_REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(_REPO_ROOT))

from cad_parser.astar_topology import run_pipeline, to_topology_bundle  # noqa: E402
from smdsl_demo.metrics import (  # noqa: E402
    FailureTaxonomy,
    generate_structured_feedback,
    run_diagnostic_pipeline,
)
from smdsl_demo.spatial_api_stub import (  # noqa: E402
    check_stl_constraint_violation,
    query_clearance_at,
)


# ──────────────────────────────────────────────────────
# 1. 轨迹合成器
# ──────────────────────────────────────────────────────

def _interp_polyline(
    waypoints: List[Tuple[float, float]],
    n_samples_per_seg: int = 20,
    dt: float = 0.05,
) -> List[Dict[str, float]]:
    """在 waypoints 之间均匀线性插值，返回带 t 的轨迹点列表。"""
    pts: List[Dict[str, float]] = []
    t = 0.0
    for (x0, y0), (x1, y1) in zip(waypoints, waypoints[1:]):
        for k in range(n_samples_per_seg):
            f = k / n_samples_per_seg
            x = x0 + (x1 - x0) * f
            y = y0 + (y1 - y0) * f
            pts.append({"t": round(t, 4), "x": x, "y": y, "z": 0.0,
                        "roll": 0.0, "pitch": 0.0, "yaw": 0.0})
            t += dt
    # 终点
    x1, y1 = waypoints[-1]
    pts.append({"t": round(t, 4), "x": x1, "y": y1, "z": 0.0,
                "roll": 0.0, "pitch": 0.0, "yaw": 0.0})
    return pts


def synth_trajectories(
    cad_data: Dict[str, Any],
) -> Dict[str, List[Dict[str, float]]]:
    """
    针对 kitchen room_24（4.5×4.2 m）合成 4 条对照轨迹。
    若你想换房间，只需调整 waypoints。
    """
    door_in = (4.10, 2.10)        # 门内侧
    center = (2.50, 2.40)          # 房间中心
    sink_pre = (2.40, 1.10)        # 水槽前安全停靠
    fridge_pre = (1.20, 3.60)      # 冰箱前操作位
    near_corner_safe = (3.80, 3.60)  # 远离右墙的安全角

    # A. 安全：door → center → sink_pre
    safe = [door_in, center, sink_pre]

    # B. 贴墙：door → 紧贴右墙 → 紧贴顶墙
    graze = [door_in, (4.30, 3.00), (4.30, 3.95), (3.50, 3.95)]

    # C. 穿过 base_cabinet_1 (0.0-0.6, 1.2-3.6)
    through_obj = [door_in, center, (0.30, 3.00), (0.30, 1.50)]

    # D. 越界：从 door 走出右墙
    out_of_bound = [door_in, (5.20, 2.10), (5.50, 2.10)]

    return {
        "A_SAFE": _interp_polyline(safe),
        "B_GRAZE_WALL": _interp_polyline(graze),
        "C_THROUGH_OBJ": _interp_polyline(through_obj),
        "D_OUT_OF_BOUND": _interp_polyline(out_of_bound),
    }


# ──────────────────────────────────────────────────────
# 2. 计算 + 反馈封装
# ──────────────────────────────────────────────────────

def evaluate_trajectory(
    trajectory: List[Dict[str, float]],
    d_safe: float,
    bundle: Dict[str, Any],
) -> Dict[str, Any]:
    """
    对单条轨迹计算 STL 鲁棒性 + 结构化反馈。

    构造一条 STL 约束 ``Distance > d_safe``，强制走"距离场"分支
    （bundle 已注入），返回完整诊断字典。
    """
    rule = {
        "type": "stl_constraint",
        "expr": f"Distance > {d_safe:.3f}",
        "unit": "m",
        "reference": "obstacle",
    }
    reports = check_stl_constraint_violation(
        trajectory, [rule], topology_bundle=bundle,
    )
    if not reports:
        return {"empty": True}
    rep = reports[0]

    # 把 stl_robustness 报告升级成 generate_structured_feedback 标准产物
    nodes = rep.get("violation_nodes", [])
    worst = min(nodes, key=lambda n: n.get("rho", 0.0)) if nodes else None
    if rep.get("violated"):
        feedback = generate_structured_feedback(
            FailureTaxonomy.COLLISION,
            rep.get("robustness", 0.0),
            {
                "rule": rep.get("rule_expr"),
                "source": rep.get("source"),
                "max_violation": rep.get("max_violation"),
                "violation_duration": rep.get("violation_duration"),
                "n_violation_nodes": len(nodes),
                "worst_node": worst,
                "violation_nodes_preview": nodes[:5],
            },
        )
    else:
        feedback = {
            "error": {
                "type": "none",
                "severity": "info",
                "robustness_score": round(rep.get("robustness", 0.0), 4),
            },
            "diagnosis": {
                "summary": "全程满足 Distance > D_safe",
                "suggestion": "可作为基线轨迹保留。",
                "hint": f"min ρ = {rep.get('robustness', 0.0):.3f} m。",
            },
        }

    return {
        "stl_report": {
            "rule_expr": rep.get("rule_expr"),
            "source": rep.get("source"),
            "robustness": rep.get("robustness"),
            "max_violation": rep.get("max_violation"),
            "violation_duration": rep.get("violation_duration"),
            "n_violation_nodes": len(nodes),
            "details_n": len(rep.get("details", [])),
        },
        "feedback": feedback,
        "violation_nodes": nodes,
        "details": rep.get("details", []),
    }


# ──────────────────────────────────────────────────────
# 3. 可视化
# ──────────────────────────────────────────────────────

def render(
    bundle: Dict[str, Any],
    trajectories: Dict[str, List[Dict[str, float]]],
    evaluations: Dict[str, Dict[str, Any]],
    d_safe: float,
    out_path: Path,
) -> None:
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    df_m = bundle["distance_field"] * bundle["grid_transform"]["resolution"]
    ox, oy = bundle["grid_transform"]["origin"]
    res = bundle["grid_transform"]["resolution"]
    H, W = bundle["grid_transform"]["shape"]
    extent = (ox, ox + W * res, oy, oy + H * res)
    R_robot_m = bundle["robot_radius_m"]

    fig, axes = plt.subplots(2, 2, figsize=(12, 11), constrained_layout=True)
    fig.suptitle(
        f"Demo 3 — STL Robustness on Distance Field\n"
        f"layout={bundle.get('layout_id', '?')} ({bundle.get('room_type', '?')})  "
        f"D_safe={d_safe:.2f} m   R_robot={R_robot_m:.2f} m",
        fontsize=13, fontweight="bold",
    )

    keys = list(trajectories.keys())
    for ax, key in zip(axes.flat, keys):
        traj = trajectories[key]
        ev = evaluations[key]

        im = ax.imshow(
            df_m, origin="lower", extent=extent,
            cmap="viridis", vmin=0.0, vmax=max(1.0, df_m.max()),
        )
        cs = ax.contour(
            df_m, levels=[d_safe], origin="lower", extent=extent,
            colors="orange", linewidths=1.2, linestyles="--",
        )
        ax.clabel(cs, fmt={d_safe: f"D_safe={d_safe:.2f}m"}, fontsize=8)

        xs = [p["x"] for p in traj]
        ys = [p["y"] for p in traj]
        ax.plot(xs, ys, "-", color="white", linewidth=2.4, alpha=0.85)
        ax.plot(xs, ys, "-", color="black", linewidth=0.9, alpha=0.95)

        nodes = ev.get("violation_nodes", [])
        if nodes:
            vx = [n["x"] for n in nodes]
            vy = [n["y"] for n in nodes]
            ax.scatter(
                vx, vy, marker="x", color="red", s=70,
                linewidths=2.2, zorder=5,
                label=f"violations (n={len(nodes)})",
            )
            worst = min(nodes, key=lambda n: n.get("rho", 0.0))
            ax.scatter(
                [worst["x"]], [worst["y"]],
                marker="o", facecolors="none", edgecolors="red",
                s=240, linewidths=2.2, zorder=6,
                label=f"worst ρ={worst.get('rho', 0):.3f}m",
            )

        # 起点 / 终点
        ax.plot(xs[0], ys[0], "o", color="lime",
                markersize=10, markeredgecolor="black",
                label="start", zorder=4)
        ax.plot(xs[-1], ys[-1], "s", color="cyan",
                markersize=10, markeredgecolor="black",
                label="goal", zorder=4)

        rho_min = ev["stl_report"].get("robustness", float("inf"))
        violated = ev["feedback"]["error"]["type"]
        ax.set_title(
            f"{key}\nmin ρ = {rho_min:.3f} m   →   {violated}",
            fontsize=10,
        )
        ax.set_xlabel("x (m)")
        ax.set_ylabel("y (m)")
        ax.set_aspect("equal")
        ax.legend(loc="upper right", fontsize=7, framealpha=0.85)
        plt.colorbar(im, ax=ax, fraction=0.04, label="d_real (m)")

    out_path.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(out_path, dpi=140, bbox_inches="tight")
    plt.close(fig)


# ──────────────────────────────────────────────────────
# 4. CLI
# ──────────────────────────────────────────────────────

def main(argv: List[str] | None = None) -> int:
    ap = argparse.ArgumentParser(
        description="Demo 3: STL Robustness on distance field",
    )
    ap.add_argument(
        "--layout", default="data/cad_samples/floorplanqa/layouts/kitchen/room_24.json",
        help="FloorplanQA JSON 路径",
    )
    ap.add_argument("--d_safe", type=float, default=0.30,
                    help="STL Distance > D_safe 阈值（米）")
    ap.add_argument("--robot_radius", type=float, default=0.25,
                    help="机器人物理半径（米，影响 Demo 1 膨胀）")
    ap.add_argument("--resolution", type=float, default=0.05,
                    help="栅格分辨率（米/像素）")
    ap.add_argument("--out_dir", default="out/demo3",
                    help="输出目录")
    args = ap.parse_args(argv)

    layout_path = Path(args.layout)
    out_dir = Path(args.out_dir)
    out_dir.mkdir(parents=True, exist_ok=True)

    print(f"[1/4] running Demo 1 pipeline on {layout_path} ...")
    pipe = run_pipeline(
        str(layout_path),
        resolution=args.resolution,
        robot_radius_m=args.robot_radius,
        include_grid=False,
    )
    bundle = to_topology_bundle(pipe)
    print(f"      grid {bundle['grid_transform']['shape']} "
          f"@ {bundle['grid_transform']['resolution']} m/px"
          f"  R_robot = {bundle['robot_radius_m']:.3f} m")

    print(f"[2/4] synthesizing 4 trajectories ...")
    trajectories = synth_trajectories(pipe["cad_data"])
    for k, t in trajectories.items():
        # 抽样打印起终点 + clearance
        s = t[0]
        e = t[-1]
        cs = query_clearance_at(s["x"], s["y"], bundle)
        ce = query_clearance_at(e["x"], e["y"], bundle)
        print(f"      {k}: n={len(t)}  start=({s['x']:.2f},{s['y']:.2f}) "
              f"clearance={cs:.3f}m → end=({e['x']:.2f},{e['y']:.2f}) "
              f"clearance={ce:.3f}m")

    print(f"[3/4] evaluating STL robustness (D_safe={args.d_safe} m) ...")
    evaluations: Dict[str, Dict[str, Any]] = {}
    for k, traj in trajectories.items():
        ev = evaluate_trajectory(traj, args.d_safe, bundle)
        evaluations[k] = ev
        rep = ev["stl_report"]
        fb = ev["feedback"]["error"]
        print(f"      {k:18s}  min ρ = {rep['robustness']:+.3f} m   "
              f"violated_nodes = {rep['n_violation_nodes']:3d}   "
              f"taxonomy = {fb['type']:10s}  severity = {fb['severity']}")

    print(f"[4/4] rendering visualization + JSON report ...")
    png_path = out_dir / "trajectories.png"
    render(bundle, trajectories, evaluations, args.d_safe, png_path)
    print(f"      png  → {png_path}")

    # 序列化 JSON 报告（剔除 numpy / 大数组）
    serializable: Dict[str, Any] = {
        "config": {
            "layout": str(layout_path),
            "d_safe": args.d_safe,
            "robot_radius_m": args.robot_radius,
            "resolution": args.resolution,
        },
        "bundle_meta": {
            "layout_id": bundle.get("layout_id"),
            "room_type": bundle.get("room_type"),
            "shape": list(bundle["grid_transform"]["shape"]),
            "origin": list(bundle["grid_transform"]["origin"]),
            "resolution": bundle["grid_transform"]["resolution"],
            "robot_radius_m": bundle["robot_radius_m"],
            "robot_radius_px": bundle["robot_radius_px"],
        },
        "trajectories": {
            k: {
                "n_points": len(t),
                "start": {"x": t[0]["x"], "y": t[0]["y"]},
                "goal": {"x": t[-1]["x"], "y": t[-1]["y"]},
            } for k, t in trajectories.items()
        },
        "evaluations": {
            k: {
                "stl_report": ev["stl_report"],
                "feedback": ev["feedback"],
                # 完整违规节点（结构化反馈核心数据）
                "violation_nodes": ev["violation_nodes"],
            }
            for k, ev in evaluations.items()
        },
    }
    json_path = out_dir / "robustness_report.json"
    json_path.write_text(
        json.dumps(serializable, indent=2, ensure_ascii=False),
        encoding="utf-8",
    )
    print(f"      json → {json_path}")

    return 0


if __name__ == "__main__":
    sys.exit(main())
