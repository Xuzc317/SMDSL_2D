"""
download_osm_networks.py — SMDSL OSM Real-World Road Network Dataset

Downloads real-world road networks via OSMnx and converts them to
SMDSL-compatible JSON format for EDT path planning benchmarks.

Usage:
    python data/download_osm_networks.py --city "Beijing, China" --n_networks 20
    python data/download_osm_networks.py --preset campus  --n_networks 10
"""

import argparse
import json
import math
import sys
from pathlib import Path
from typing import Any, Dict, List, Tuple

_PROJ = Path(__file__).resolve().parent.parent
_SMDSL = _PROJ / "SMDSL"
sys.path.insert(0, str(_SMDSL))

import numpy as np

PRESET_LOCATIONS = {
    "campus": [
        "Tsinghua University, Beijing, China",
        "Peking University, Beijing, China",
        "Shanghai Jiao Tong University, Shanghai, China",
        "Zhejiang University, Hangzhou, China",
        "University of Tokyo, Tokyo, Japan",
        "Seoul National University, Seoul, South Korea",
        "National University of Singapore, Singapore",
        "University of Cambridge, Cambridge, UK",
        "Stanford University, California, USA",
        "MIT, Cambridge, Massachusetts, USA",
        "ETH Zurich, Zurich, Switzerland",
        "TU Munich, Munich, Germany",
    ],
    "residential": [
        "Chaoyang District, Beijing, China",
        "Pudong, Shanghai, China",
        "Setagaya, Tokyo, Japan",
        "Brooklyn, New York, USA",
        "Kreuzberg, Berlin, Germany",
        "Le Marais, Paris, France",
        "Kensington, London, UK",
        "Trastevere, Rome, Italy",
    ],
    "industrial": [
        "Suzhou Industrial Park, Suzhou, China",
        "Shenzhen High-Tech Park, Shenzhen, China",
        "Detroit Industrial District, Michigan, USA",
        "Ruhr Industrial Area, Essen, Germany",
    ],
    "mixed": [
        "Manhattan, New York, USA",
        "Central Tokyo, Tokyo, Japan",
        "Central London, London, UK",
        "Marina Bay, Singapore",
        "Gangnam, Seoul, South Korea",
        "Haidian District, Beijing, China",
    ],
}


def osm_graph_to_smdsl_layout(
    G,
    layout_id: int,
    location_name: str,
    resolution: float = 1.0,
) -> Dict[str, Any]:
    """
    Convert an OSMnx graph to SMDSL JSON format.

    Strategy: extract node coordinates as vertices,
    edges as wall segments (for building occupancy grid),
    and generate a polygon boundary from the convex hull.
    """
    nodes_data = []
    for node_id, data in G.nodes(data=True):
        x = float(data.get("x", 0))
        y = float(data.get("y", 0))
        nodes_data.append((x, y))

    if not nodes_data:
        return {}

    xs = [p[0] for p in nodes_data]
    ys = [p[1] for p in nodes_data]
    min_x, max_x = min(xs), max(xs)
    min_y, max_y = min(ys), max(ys)

    # Shift to origin for consistent resolution
    ox, oy = min_x, min_y
    width_m = (max_x - min_x) * 111320 * math.cos(math.radians((min_y + max_y) / 2))
    height_m = (max_y - min_y) * 110540

    # Normalize scale: target ~50m x 50m max to avoid extreme grid sizes
    max_extent = max(width_m, height_m)
    scale = 1.0
    if max_extent > 500:
        scale = 500.0 / max_extent

    boundary: List[Dict[str, float]] = [
        {"x": 0.0, "y": 0.0},
        {"x": width_m * scale, "y": 0.0},
        {"x": width_m * scale, "y": height_m * scale},
        {"x": 0.0, "y": height_m * scale},
    ]

    walls: List[Dict[str, Any]] = []
    for u, v, data in G.edges(data=True):
        ux = G.nodes[u].get("x", 0)
        uy = G.nodes[u].get("y", 0)
        vx = G.nodes[v].get("x", 0)
        vy = G.nodes[v].get("y", 0)
        # Convert lat/lon difference to meters and scale
        sx = (ux - ox) * 111320 * math.cos(math.radians((min_y + max_y) / 2)) * scale
        sy = (uy - oy) * 110540 * scale
        ex = (vx - ox) * 111320 * math.cos(math.radians((min_y + max_y) / 2)) * scale
        ey = (vy - oy) * 110540 * scale
        walls.append({
            "start": {"x": float(sx), "y": float(sy)},
            "end": {"x": float(ex), "y": float(ey)},
        })

    # Estimate room type from graph characteristics
    n_nodes = len(nodes_data)
    n_edges = len(walls)
    density = n_edges / max(1, n_nodes)
    if density < 1.2:
        room_type = "road_network_sparse"
    elif density < 1.8:
        room_type = "road_network_grid"
    else:
        room_type = "road_network_dense"

    return {
        "layout_id": layout_id,
        "room_type": room_type,
        "room_boundary": boundary,
        "walls": walls,
        "openings": {"doors": [], "windows": []},
        "objects": [],
        "units": "meters",
        "_source": f"OSM/{location_name}",
        "_osm_stats": {
            "n_nodes": n_nodes,
            "n_edges": n_edges,
            "edge_density": round(density, 4),
            "original_bbox_m": f"{width_m:.0f}x{height_m:.0f}",
        },
    }


def download_single(location: str, layout_id: int, output_dir: Path, resolution: float = 1.0) -> bool:
    try:
        import osmnx as ox
        G = ox.graph_from_place(location, network_type="drive", simplify=True)
        if G.number_of_nodes() < 5:
            # Try walking network
            G = ox.graph_from_place(location, network_type="walk", simplify=True)
        if G.number_of_nodes() < 3:
            print(f"  SKIP {location}: too few nodes ({G.number_of_nodes()})")
            return False

        layout = osm_graph_to_smdsl_layout(G, layout_id, location, resolution)
        safe_name = location.replace(", ", "_").replace(" ", "_").replace("/", "_")[:80]
        fname = f"osm_{safe_name}.json"
        fpath = output_dir / fname
        with open(fpath, "w", encoding="utf-8") as f:
            json.dump(layout, f, ensure_ascii=False, indent=2)
        print(f"  OK  {location}: {G.number_of_nodes()} nodes, {G.number_of_edges()} edges -> {fname}")
        return True
    except Exception as e:
        print(f"  FAIL {location}: {e}")
        return False


def main():
    parser = argparse.ArgumentParser(description="Download OSM road networks for SMDSL benchmark")
    parser.add_argument("--city", type=str, default=None, help="Single city/location to download")
    parser.add_argument("--preset", type=str, default="mixed",
                        choices=["campus", "residential", "industrial", "mixed", "all"],
                        help="Preset location list")
    parser.add_argument("--n_networks", type=int, default=20, help="Max networks to download")
    parser.add_argument("--output", type=str, default="data/datasets/osm_networks")
    parser.add_argument("--resolution", type=float, default=1.0,
                        help="Meters per pixel for grid rasterization")
    args = parser.parse_args()

    out_dir = Path(args.output)
    out_dir.mkdir(parents=True, exist_ok=True)

    if args.city:
        locations = [args.city]
    elif args.preset == "all":
        locations = []
        for locs in PRESET_LOCATIONS.values():
            locations.extend(locs)
    else:
        locations = PRESET_LOCATIONS.get(args.preset, PRESET_LOCATIONS["mixed"])

    print(f"Downloading up to {args.n_networks} OSM road networks (preset={args.preset})...")
    count = 0
    for loc in locations:
        if count >= args.n_networks:
            break
        if download_single(loc, count, out_dir, args.resolution):
            count += 1

    print(f"\nDownloaded {count} networks to {out_dir}")
    # Generate manifest
    manifest = {
        "total_networks": count,
        "preset": args.preset,
        "resolution_m_per_px": args.resolution,
        "description": "Real-world OSM road networks converted to SMDSL JSON format",
    }
    with open(out_dir / "manifest.json", "w", encoding="utf-8") as f:
        json.dump(manifest, f, ensure_ascii=False, indent=2)


if __name__ == "__main__":
    main()
