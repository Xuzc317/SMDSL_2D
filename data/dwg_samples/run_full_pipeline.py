"""
run_full_pipeline.py — End-to-end batch: DWG → AreaGraph + topology bundle.

Processes all available DWG samples through the complete pipeline:
  1. parse_dwg_to_json() → raw LibreDWG JSON
  2. dispatch_cad() → occupancy grid + semantics
  3. run_pipeline() → A* topology + distance field
  4. build_area_graph() → AreaGraph (regions + edges)
  5. semantic_extractor → LLM business node labels (if DEEPSEEK_API_KEY set)
  6. Save: topology_bundle.npz + area_graph.json + summary

Also processes FloorplanQA JSON layouts for comparison.

Prerequisite: LibreDWG (dwgread) for DWG processing.
"""

from __future__ import annotations

import json
import sys
import time
from pathlib import Path
from typing import Any, Dict, List, Optional

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent.parent.parent))

from cad_parser.area_graph import (
    build_area_graph,
    inject_loading_zones,
    save_area_graph,
)
from cad_parser.astar_topology import (
    CLASS_PATH,
    bridge_thin_walls,
    classify_topology_global,
    compute_distance_field,
    rasterize_to_grid,
    remove_exterior_freespace,
    run_pipeline,
    save_topology_bundle,
    to_topology_bundle,
)
from cad_parser.dispatcher import dispatch_cad
from cad_parser.dwg_ingestion import parse_dwg_to_json
from scipy import ndimage

BASE_DIR = Path(__file__).resolve().parent
OUTPUT_DIR = BASE_DIR / "output" / "full_pipeline"


# ══════════════════════════════════════════════════════════════════════
# Single-file processor
# ══════════════════════════════════════════════════════════════════════

def process_dwg(
    dwg_path: Path,
    resolution: float = 0.02,
    robot_radius_m: float = 0.30,
    region_min_area_m2: float = 1.0,
) -> Dict[str, Any]:
    """Process a single DWG file end-to-end."""
    result: Dict[str, Any] = {
        "file": dwg_path.name,
        "source": str(dwg_path),
        "status": "pending",
        "steps": {},
    }
    t0 = time.time()

    # Step 1: DWG → JSON
    t1 = time.time()
    ingestion = parse_dwg_to_json(str(dwg_path))
    result["steps"]["ingestion"] = {
        "status": ingestion["status"],
        "dwg_version": ingestion.get("dwg_version", "?"),
        "duration_s": round(time.time() - t1, 2),
    }
    if ingestion["status"] != "ok":
        result["status"] = "ingestion_failed"
        result["error"] = ingestion["message"]
        result["duration_s"] = round(time.time() - t0, 2)
        return result

    # Step 2: DWG → occupancy grid + semantics
    t2 = time.time()
    try:
        parsed = dispatch_cad(
            str(dwg_path), resolution=resolution,
            padding_m=1.0, wall_thickness_m=0.10,
        )
        result["steps"]["dispatch"] = {
            "status": "ok",
            "mode": parsed["mode"],
            "grid_shape": list(parsed["grid"].shape),
            "has_semantics": parsed.get("has_semantics", False),
            "duration_s": round(time.time() - t2, 2),
        }
        if parsed.get("semantics"):
            s = parsed["semantics"]
            result["steps"]["dispatch"]["semantic_summary"] = s.get("entity_summary", {})
    except Exception as e:
        result["status"] = "dispatch_failed"
        result["error"] = str(e)
        result["duration_s"] = round(time.time() - t0, 2)
        return result

    grid = parsed["grid"]
    transform = parsed["transform"]

    # Step 3: Topology pipeline
    t3 = time.time()
    distance_field = compute_distance_field(grid)
    robot_radius_px = robot_radius_m / resolution
    topology, topo_stats = classify_topology_global(
        grid, distance_field, robot_radius_px,
    )
    result["steps"]["topology"] = {
        "status": "ok",
        "n_path_px": topo_stats["n_path_px"],
        "n_components": topo_stats["n_components"],
        "duration_s": round(time.time() - t3, 2),
    }

    # Step 4: AreaGraph
    t4 = time.time()
    area_graph, region_meta = build_area_graph(
        topology, transform, region_min_area_m2=region_min_area_m2,
    )

    # Inject loading zones
    path_mask = topology == CLASS_PATH
    region_labels, _ = ndimage.label(
        path_mask, ndimage.generate_binary_structure(2, 2),
    )
    area_graph, load_info = inject_loading_zones(
        area_graph, topology, transform, region_labels,
    )

    result["steps"]["area_graph"] = {
        "status": "ok",
        "n_regions": len(area_graph),
        "n_edges": sum(len(e) for e in area_graph.values()),
        "loading_zones": load_info.get("n_zones", 0),
        "duration_s": round(time.time() - t4, 2),
    }

    # Step 5: Save outputs
    t5 = time.time()
    file_stem = dwg_path.stem
    out_dir = OUTPUT_DIR / file_stem
    out_dir.mkdir(parents=True, exist_ok=True)

    # AreaGraph as JSON + GeoJSON
    ag_paths = save_area_graph(area_graph, region_meta, str(out_dir), prefix=file_stem)

    # Topology bundle (.npz + .json sidecar)
    bundle = to_topology_bundle({
        "distance_field": distance_field,
        "transform": transform,
        "robot_radius_px": robot_radius_px,
        "cad_data": parsed.get("cad_data", {}),
    })
    bundle_path = save_topology_bundle(bundle, str(out_dir / file_stem))

    result["steps"]["save"] = {
        "area_graph_files": ag_paths,
        "bundle_npz": str(bundle_path),
        "duration_s": round(time.time() - t5, 2),
    }

    result["status"] = "ok"
    result["duration_s"] = round(time.time() - t0, 2)
    return result


def process_json_layout(
    json_path: Path,
    resolution: float = 0.02,
    robot_radius_m: float = 0.30,
    region_min_area_m2: float = 1.0,
) -> Dict[str, Any]:
    """Process a single FloorplanQA JSON layout end-to-end."""
    result: Dict[str, Any] = {
        "file": f"{json_path.parent.name}/{json_path.name}",
        "source": str(json_path),
        "status": "pending",
        "steps": {},
    }
    t0 = time.time()

    try:
        pipeline_result = run_pipeline(
            str(json_path), resolution=resolution,
            robot_radius_m=robot_radius_m, padding_m=0.5,
            wall_thickness_m=0.10,
        )
    except Exception as e:
        result["status"] = "pipeline_failed"
        result["error"] = str(e)
        result["duration_s"] = round(time.time() - t0, 2)
        return result

    topology = pipeline_result["topology"]
    transform = pipeline_result["transform"]

    # AreaGraph
    t_ag = time.time()
    area_graph, region_meta = build_area_graph(
        topology, transform, region_min_area_m2=region_min_area_m2,
    )
    path_mask = topology == CLASS_PATH
    region_labels, _ = ndimage.label(
        path_mask, ndimage.generate_binary_structure(2, 2),
    )
    area_graph, load_info = inject_loading_zones(
        area_graph, topology, transform, region_labels,
    )
    result["steps"]["area_graph"] = {
        "n_regions": len(area_graph),
        "n_edges": sum(len(e) for e in area_graph.values()),
        "loading_zones": load_info.get("n_zones", 0),
        "duration_s": round(time.time() - t_ag, 2),
    }

    # Save
    file_stem = f"{json_path.parent.name}_{json_path.stem}"
    out_dir = OUTPUT_DIR / "floorplanqa" / file_stem
    out_dir.mkdir(parents=True, exist_ok=True)
    save_area_graph(area_graph, region_meta, str(out_dir), prefix=file_stem)

    result["status"] = "ok"
    result["duration_s"] = round(time.time() - t0, 2)
    return result


# ══════════════════════════════════════════════════════════════════════
# Main batch runner
# ══════════════════════════════════════════════════════════════════════

def main():
    import argparse

    parser = argparse.ArgumentParser(
        description="Batch DWG/JSON → AreaGraph end-to-end pipeline"
    )
    parser.add_argument(
        "--resolution", type=float, default=0.02,
        help="Grid resolution meters/pixel (default: 0.02)"
    )
    parser.add_argument(
        "--robot-radius", type=float, default=0.30,
        help="Robot radius in meters (default: 0.30)"
    )
    parser.add_argument(
        "--min-region-area", type=float, default=1.0,
        help="Minimum region area in m2 (default: 1.0)"
    )
    parser.add_argument(
        "--max-dwg", type=int, default=0,
        help="Max DWG files to process (0=all)"
    )
    parser.add_argument(
        "--max-json", type=int, default=0,
        help="Max JSON layouts to process (0=all)"
    )
    parser.add_argument(
        "--skip-dwg", action="store_true",
        help="Skip DWG processing (dwgread not installed)"
    )
    args = parser.parse_args()

    OUTPUT_DIR.mkdir(parents=True, exist_ok=True)
    print("=" * 60)
    print("Full Pipeline: DWG/JSON → AreaGraph")
    print(f"Resolution: {args.resolution}m/px, Robot radius: {args.robot_radius}m")
    print("=" * 60)

    all_results: List[Dict[str, Any]] = []

    # ── DWG samples ─────────────────────────────────────────────
    dwg_dirs = [
        BASE_DIR / "libredwg_test_suite",
        BASE_DIR / "autodesk_official",
    ]
    dwg_files: List[Path] = []
    seen: set = set()
    for d in dwg_dirs:
        if not d.is_dir():
            continue
        for f in sorted(d.glob("*")):
            if f.suffix.lower() == ".dwg":
                rp = f.resolve()
                if rp not in seen:
                    seen.add(rp)
                    dwg_files.append(f)

    if args.skip_dwg:
        print(f"\nSkipping {len(dwg_files)} DWG files (--skip-dwg)")
    elif dwg_files:
        if args.max_dwg > 0:
            dwg_files = dwg_files[:args.max_dwg]
        print(f"\n--- Processing {len(dwg_files)} DWG files ---\n")
        for i, dwg_path in enumerate(dwg_files):
            print(f"[DWG {i+1}/{len(dwg_files)}] {dwg_path.name} ...", end=" ", flush=True)
            r = process_dwg(
                dwg_path, resolution=args.resolution,
                robot_radius_m=args.robot_radius,
                region_min_area_m2=args.min_region_area,
            )
            status = r["status"]
            dur = r.get("duration_s", 0)
            print(f"{status} ({dur:.1f}s)")
            if r["status"] == "ok":
                ag = r["steps"].get("area_graph", {})
                print(f"  Regions: {ag.get('n_regions',0)}, "
                      f"Edges: {ag.get('n_edges',0)}")
            else:
                print(f"  Error: {r.get('error', 'unknown')[:100]}")
            all_results.append(r)
    else:
        print("\nNo DWG files found.")

    # ── FloorplanQA JSON layouts (reference) ────────────────────
    floorplanqa_dir = BASE_DIR.parent / "cad_samples" / "floorplanqa" / "layouts"
    if floorplanqa_dir.is_dir():
        json_files = sorted(floorplanqa_dir.glob("*/*.json"))
        if args.max_json > 0:
            json_files = json_files[:args.max_json]
        else:
            json_files = json_files[:10]  # default: 10 samples for speed
        print(f"\n--- Processing {len(json_files)} JSON layouts ---\n")
        for i, json_path in enumerate(json_files):
            print(f"[JSON {i+1}/{len(json_files)}] {json_path.parent.name}/{json_path.name} ...",
                  end=" ", flush=True)
            r = process_json_layout(
                json_path, resolution=args.resolution,
                robot_radius_m=args.robot_radius,
                region_min_area_m2=args.min_region_area,
            )
            status = r["status"]
            dur = r.get("duration_s", 0)
            print(f"{status} ({dur:.1f}s)")
            if r["status"] == "ok":
                ag = r["steps"].get("area_graph", {})
                print(f"  Regions: {ag.get('n_regions',0)}, "
                      f"Edges: {ag.get('n_edges',0)}")
            all_results.append(r)

    # ── Summary ─────────────────────────────────────────────────
    ok = sum(1 for r in all_results if r["status"] == "ok")
    failed = sum(1 for r in all_results if r["status"] != "ok")
    total_dur = sum(r.get("duration_s", 0) for r in all_results)
    print(f"\n{'='*60}")
    print(f"Batch complete: {ok} OK, {failed} failed, {total_dur:.1f}s total")
    print(f"Outputs: {OUTPUT_DIR}")

    # Save batch summary
    summary = {
        "config": {
            "resolution": args.resolution,
            "robot_radius_m": args.robot_radius,
            "min_region_area_m2": args.min_region_area,
        },
        "summary": {
            "total": len(all_results),
            "ok": ok,
            "failed": failed,
            "total_duration_s": round(total_dur, 2),
        },
        "results": [
            {
                "file": r["file"],
                "status": r["status"],
                "duration_s": r.get("duration_s", 0),
                "error": r.get("error", ""),
                "steps": {
                    k: {sk: sv for sk, sv in v.items() if sk != "area_graph_files"}
                    for k, v in r.get("steps", {}).items()
                },
            }
            for r in all_results
        ],
    }
    summary_path = OUTPUT_DIR / "batch_summary.json"
    summary_path.write_text(
        json.dumps(summary, indent=2, ensure_ascii=False), encoding="utf-8"
    )
    print(f"Summary: {summary_path}")

    return 0 if failed == 0 else 1


if __name__ == "__main__":
    sys.exit(main())
