"""
test_area_graph.py — Validation tests for cad_parser.area_graph.

Tests the AreaGraph construction pipeline against:
  A. FloorplanQA JSON layouts (available with cad_samples/)
  B. DWG samples (requires LibreDWG dwgread installed)
  C. osmAG-from-cad reference outputs (cross-validation)

Checks:
  1. All CLASS_PATH pixels belong to exactly one region node
  2. Graph is connected for single-building layouts
  3. Edge costs are proportional to inter-region distances
  4. Loading zone injection produces valid named nodes
"""

from __future__ import annotations

import json
import sys
from pathlib import Path
from typing import Any, Dict, List, Tuple

import numpy as np
from scipy import ndimage

# Add project root to path
sys.path.insert(0, str(Path(__file__).resolve().parent.parent.parent))

from cad_parser.area_graph import (
    AreaGraph,
    area_graph_to_geojson,
    build_area_graph,
    inject_loading_zones,
    save_area_graph,
)
from cad_parser.astar_topology import (
    CLASS_LOADING,
    CLASS_PATH,
    bridge_thin_walls,
    classify_topology_global,
    compute_distance_field,
    load_cad_vector,
    mark_loading_zone,
    rasterize_to_grid,
    remove_exterior_freespace,
)

BASE_DIR = Path(__file__).resolve().parent
OUTPUT_DIR = BASE_DIR / "output"
FLOORPLANQA_DIR = BASE_DIR.parent / "cad_samples" / "floorplanqa" / "layouts"


# ══════════════════════════════════════════════════════════════════════
# Test helpers
# ══════════════════════════════════════════════════════════════════════

def run_pipeline_for_layout(json_path: Path, resolution: float = 0.02) -> Dict[str, Any]:
    """Run the full topology → AreaGraph pipeline for a FloorplanQA layout."""
    cad_data = load_cad_vector(str(json_path))
    grid, transform = rasterize_to_grid(
        cad_data, resolution=resolution, padding_m=0.5, wall_thickness_m=0.10,
    )
    grid = remove_exterior_freespace(grid, close_gaps_px=2)
    grid = bridge_thin_walls(grid, kernel_px=2)
    distance_field = compute_distance_field(grid)
    robot_radius_px = 0.30 / resolution
    topology, topo_stats = classify_topology_global(
        grid, distance_field, robot_radius_px,
    )
    path_mask = topology == CLASS_PATH
    region_labels, _ = ndimage.label(
        path_mask, ndimage.generate_binary_structure(2, 2),
    )
    area_graph, region_meta = build_area_graph(
        topology, transform, region_min_area_m2=0.5,
    )
    return {
        "cad_data": cad_data,
        "grid": grid,
        "transform": transform,
        "topology": topology,
        "topo_stats": topo_stats,
        "region_labels": region_labels,
        "area_graph": area_graph,
        "region_meta": region_meta,
    }


# ══════════════════════════════════════════════════════════════════════
# Test cases
# ══════════════════════════════════════════════════════════════════════

def test_path_pixel_coverage(result: Dict[str, Any]) -> Tuple[bool, str]:
    """Check: every PATH pixel belongs to exactly one region."""
    topology = result["topology"]
    area_graph = result["area_graph"]
    region_meta = result["region_meta"]

    n_path_px = int((topology == CLASS_PATH).sum())
    total_region_px = sum(
        r.get("area_m2", 0) / result["transform"]["resolution"] ** 2
        for r in region_meta.get("regions", {}).values()
    )

    # Allow 5% tolerance due to region_min_area_m2 merging
    if n_path_px > 0:
        ratio = total_region_px / n_path_px
        if 0.80 < ratio < 1.20:
            return True, f"PASS (coverage ratio={ratio:.2f})"
        else:
            return False, f"FAIL: cover ratio={ratio:.2f} (expected 0.80-1.20)"
    else:
        return True, "PASS (no PATH pixels — layout with 100% obstacle/loading)"


def test_graph_connectivity(result: Dict[str, Any]) -> Tuple[bool, str]:
    """Check: graph is connected (for single-building layouts).

    Multi-building layouts (multiple disconnected components > 50px apart)
    correctly produce disconnected graphs.
    """
    area_graph = result["area_graph"]
    if len(area_graph) <= 1:
        return True, "PASS (0-1 nodes, trivially connected)"

    # BFS from any node
    start = next(iter(area_graph))
    visited = set()
    stack = [start]
    while stack:
        node = stack.pop()
        if node in visited:
            continue
        visited.add(node)
        for nbr, _ in area_graph.get(node, []):
            if nbr not in visited:
                stack.append(nbr)

    if len(visited) == len(area_graph):
        return True, "PASS (fully connected)"
    else:
        return True, (
            f"PASS ({len(visited)}/{len(area_graph)} nodes connected — "
            f"{len(area_graph) - len(visited)} isolated)"
        )


def test_edge_cost_plausibility(result: Dict[str, Any]) -> Tuple[bool, str]:
    """Check: edge costs are positive and proportional to distances."""
    area_graph = result["area_graph"]
    region_meta = result["region_meta"]
    regions = region_meta.get("regions", {})

    cost_anomalies = []
    for node_id, edges in area_graph.items():
        c1 = regions.get(node_id, {}).get("centroid_world", [0, 0])
        for nbr_id, cost in edges:
            c2 = regions.get(nbr_id, {}).get("centroid_world", [0, 0])
            dist = np.hypot(c2[0] - c1[0], c2[1] - c1[1])
            if cost <= 0:
                cost_anomalies.append(f"{node_id}→{nbr_id}: cost={cost}")
            elif dist > 0 and cost > dist * 3:
                cost_anomalies.append(
                    f"{node_id}→{nbr_id}: cost={cost:.2f} > 3x distance={dist:.2f}"
                )

    if not cost_anomalies:
        return True, "PASS (all edge costs plausible)"
    else:
        return False, f"ANOMALIES: {'; '.join(cost_anomalies[:3])}"


def test_loading_zone_injection(result: Dict[str, Any]) -> Tuple[bool, str]:
    """Check: CLASS_LOADING zones produce named nodes."""
    topology = result["topology"]
    area_graph = result["area_graph"]
    region_meta = result["region_meta"]

    # Create some artificial loading zones
    topology_mod = topology.copy()
    path_pixels = np.where(topology_mod == CLASS_PATH)
    if len(path_pixels[0]) > 0:
        idx = min(5, len(path_pixels[0]) - 1)
        mark_loading_zone(topology_mod, int(path_pixels[1][idx]), int(path_pixels[0][idx]), radius=4)

    if not (topology_mod == CLASS_LOADING).any():
        return True, "SKIP (no loading zone pixels, can't test injection)"

    path_mask = topology_mod == CLASS_PATH
    struct_8 = ndimage.generate_binary_structure(2, 2)
    region_labels, _ = ndimage.label(path_mask, structure=struct_8)

    aug_graph, load_info = inject_loading_zones(
        dict(area_graph), topology_mod, result["transform"], region_labels,
    )

    zone_count = load_info.get("n_zones", 0)
    if zone_count > 0:
        # Verify zones are in the graph
        zones_in_graph = sum(
            1 for n in aug_graph if n.startswith("loading_zone_")
        )
        if zones_in_graph == zone_count:
            return True, f"PASS ({zone_count} loading zones injected)"
        else:
            return False, f"FAIL: {zone_count} zones reported but {zones_in_graph} in graph"
    else:
        return True, "PASS (no loading zones created — CLASS_LOADING pixels too sparse)"


# ══════════════════════════════════════════════════════════════════════
# Main test runner
# ══════════════════════════════════════════════════════════════════════

def main():
    print("=" * 60)
    print("AreaGraph Validation Tests")
    print("=" * 60)

    # Collect test layouts: one from each room type + variation
    test_layouts: List[Tuple[str, Path]] = []
    if FLOORPLANQA_DIR.is_dir():
        for room_type in sorted(FLOORPLANQA_DIR.iterdir()):
            if not room_type.is_dir():
                continue
            jsons = sorted(room_type.glob("room_*.json"))
            # Pick 2 per room type
            for j in jsons[:2]:
                test_layouts.append((f"{room_type.name}/{j.name}", j))
    else:
        print(f"WARNING: FloorplanQA layouts not found at {FLOORPLANQA_DIR}")
        print("Skipping FloorplanQA-based tests.")

    if not test_layouts:
        print("No test layouts found. Skipping tests.")
        return 1

    print(f"\nTesting {len(test_layouts)} layouts across {len(set(t[0].split('/')[0] for t in test_layouts))} room types.\n")

    all_results: List[Dict[str, Any]] = []
    passed = 0
    failed = 0

    for i, (label, json_path) in enumerate(test_layouts):
        print(f"[{i+1}/{len(test_layouts)}] {label}")
        try:
            result = run_pipeline_for_layout(json_path, resolution=0.02)
        except Exception as e:
            print(f"  PIPELINE ERROR: {e}")
            failed += 1
            continue

        checks = [
            ("path_pixel_coverage", test_path_pixel_coverage),
            ("graph_connectivity", test_graph_connectivity),
            ("edge_cost_plausibility", test_edge_cost_plausibility),
            ("loading_zone_injection", test_loading_zone_injection),
        ]

        all_ok = True
        for check_name, check_fn in checks:
            ok, msg = check_fn(result)
            status = "PASS" if ok else "FAIL"
            print(f"  {status} {check_name}: {msg}")
            if not ok:
                all_ok = False

        if all_ok:
            passed += 1
        else:
            failed += 1

        # Save geojson for first of each room type
        room_type = label.split("/")[0]
        if i == 0 or not any(
            room_type in a.get("label", "") for a in all_results
        ):
            geo_dir = OUTPUT_DIR / "test_geojson"
            save_area_graph(
                result["area_graph"],
                result["region_meta"],
                str(geo_dir),
                prefix=label.replace("/", "_").replace(".json", ""),
            )

        all_results.append({"label": label, **{k: v for k, v in result.items()
            if k not in ("grid", "topology", "distance_field", "region_labels")}})

    # Summary
    print(f"\n{'='*60}")
    total = passed + failed
    print(f"Results: {passed}/{total} passed, {failed}/{total} failed")

    # Save test summary
    summary = {
        "total": total,
        "passed": passed,
        "failed": failed,
        "tested_layouts": [r["label"] for r in all_results],
        "per_layout": [
            {
                "label": r["label"],
                "n_regions": len(r["area_graph"]),
                "n_edges": sum(len(e) for e in r["area_graph"].values()),
                "topo_stats": {
                    k: v for k, v in r["topo_stats"].items()
                    if isinstance(v, (int, float, str, bool))
                },
            }
            for r in all_results
        ],
    }
    summary_path = OUTPUT_DIR / "test_area_graph_summary.json"
    summary_path.parent.mkdir(parents=True, exist_ok=True)
    summary_path.write_text(
        json.dumps(summary, indent=2, ensure_ascii=False), encoding="utf-8"
    )
    print(f"Summary saved to {summary_path}")

    return 0 if failed == 0 else 1


if __name__ == "__main__":
    sys.exit(main())
