"""
main.py — Demo 1: CAD 2D Spatial Parsing 入口

四步流水线（PROJECT_CONTEXT 强约束）：
    CAD 矢量图 → 栅格化 → 距离场 → 安全代价 A* → 标签 JSON 拓扑地图

用法::

    # 最小示例
    python -m cad_parser.main \
        --cad_path data/cad_samples/floorplanqa/layouts/kitchen/room_0.json \
        --output  out/kitchen_room_0.topology.json

    # 带可视化 + 自定义机器人半径
    python -m cad_parser.main \
        --cad_path data/cad_samples/floorplanqa/layouts/living_room/room_0.json \
        --resolution 0.02 --robot_radius 0.25 \
        --visualize out/living_room_room_0.png \
        --output out/living_room_room_0.topology.json
"""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path
from typing import List, Optional, Tuple

# ── 终端编码修复：Windows GBK 无法打印 Unicode 符号（✓/✗/—） ──
if sys.stdout.encoding != "utf-8":
    try:
        sys.stdout.reconfigure(encoding="utf-8")
    except (AttributeError, OSError):
        pass
if sys.stderr.encoding != "utf-8":
    try:
        sys.stderr.reconfigure(encoding="utf-8")
    except (AttributeError, OSError):
        pass

# 同包导入，兼容 `python -m cad_parser.main` 与 `python cad_parser/main.py`
try:
    from cad_parser.astar_topology import run_pipeline
except ImportError:
    HERE = Path(__file__).resolve().parent
    sys.path.insert(0, str(HERE.parent))
    from cad_parser.astar_topology import run_pipeline


def _parse_seed_points_world(text: str) -> List[Tuple[float, float]]:
    """解析 'x1,y1;x2,y2' 形式的世界坐标种子点。"""
    pairs = [p.strip() for p in text.split(";") if p.strip()]
    out: List[Tuple[float, float]] = []
    for p in pairs:
        x, y = p.split(",")
        out.append((float(x), float(y)))
    return out


def main(argv: Optional[List[str]] = None) -> int:
    parser = argparse.ArgumentParser(
        description="Demo 1 — CAD 矢量平面图 → A* 拓扑地图",
    )
    parser.add_argument(
        "--cad_path", type=str, required=True,
        help="FloorplanQA JSON 文件路径",
    )
    parser.add_argument(
        "--output", type=str, default="topology.json",
        help="输出 JSON 路径（含 grid + 元数据）",
    )
    parser.add_argument(
        "--resolution", type=float, default=0.05,
        help="栅格分辨率 米/像素（默认 0.05 = 5cm/px）",
    )
    parser.add_argument(
        "--robot_radius", type=float, default=0.30,
        help="机器人物理半径（米），距离场 < 此值的像素视为不可通行 (默认 0.30)",
    )
    parser.add_argument(
        "--padding", type=float, default=0.5,
        help="bbox 外延填充（米）",
    )
    parser.add_argument(
        "--wall_thickness", type=float, default=0.10,
        help="墙体物理厚度（米）",
    )
    parser.add_argument(
        "--safety_weight", type=float, default=0.5,
        help="A* 中 1/distance 安全惩罚的权重（0 = 纯 Dijkstra）",
    )
    parser.add_argument(
        "--max_steps", type=int, default=2_000_000,
        help="A* 最大扩展节点数",
    )
    parser.add_argument(
        "--seeds_world", type=str, default=None,
        help="种子点（世界坐标，单位米）格式 'x1,y1;x2,y2'，"
             "默认自动从门洞中心提取",
    )
    parser.add_argument(
        "--no_grid", action="store_true",
        help="不在输出 JSON 中嵌入 H×W 标签矩阵（仅写元数据）",
    )
    parser.add_argument(
        "--visualize", type=str, default=None,
        help="生成 4 联图 PNG（栅格 / 距离场 / 访问热力 / 拓扑标签）",
    )
    args = parser.parse_args(argv)

    cad_path = Path(args.cad_path)
    if not cad_path.is_file():
        print(f"[ERROR] CAD file not found: {cad_path}", file=sys.stderr)
        return 2

    seeds_world = (
        _parse_seed_points_world(args.seeds_world)
        if args.seeds_world else None
    )

    print(f"[1/4] 加载 CAD: {cad_path}")
    print(f"[2/4] 栅格化 (resolution={args.resolution}m/px, "
          f"wall_thickness={args.wall_thickness}m)")
    print(f"[3/4] 距离场 → 安全 A* (R_robot={args.robot_radius}m)")

    result = run_pipeline(
        cad_path=str(cad_path),
        resolution=args.resolution,
        robot_radius_m=args.robot_radius,
        padding_m=args.padding,
        wall_thickness_m=args.wall_thickness,
        safety_weight=args.safety_weight,
        max_steps=args.max_steps,
        seeds_world=seeds_world,
        include_grid=not args.no_grid,
    )

    summary = result["topology_json"]["summary"]
    meta = result["topology_json"]["metadata"]
    print(
        f"      grid={meta['height_pixels']}×{meta['width_pixels']}, "
        f"seeds={summary['n_seeds']}, "
        f"visits={summary['total_a_star_visits']}, "
        f"min_clearance_px={summary.get('min_clearance_px', '-')}, "
        f"class_pixels={summary['class_pixels']}"
    )

    print(f"[4/4] 写出拓扑 JSON → {args.output}")
    out_path = Path(args.output)
    out_path.parent.mkdir(parents=True, exist_ok=True)
    out_path.write_text(
        json.dumps(result["topology_json"], ensure_ascii=False, indent=2),
        encoding="utf-8",
    )

    if args.visualize:
        try:
            from cad_parser.visualize import render_pipeline_quad
        except ImportError:
            from visualize import render_pipeline_quad  # type: ignore
        viz_path = Path(args.visualize)
        viz_path.parent.mkdir(parents=True, exist_ok=True)
        render_pipeline_quad(result, output_path=str(viz_path))
        print(f"      可视化 PNG → {viz_path}")

    print("OK")
    return 0


if __name__ == "__main__":
    sys.exit(main())
