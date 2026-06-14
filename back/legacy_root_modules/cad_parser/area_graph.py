"""
area_graph.py — Region adjacency graph construction from topology raster.

Converts the per-pixel topology classification from ``classify_topology_global()``
into an ``AreaGraph``: a region-level connectivity graph where nodes represent
distinct spatial zones (rooms/corridors/loading-zones) and edges represent
shared boundaries with traversal costs.

Algorithm (connected-component adjacency):
  1. Label connected components of CLASS_PATH pixels → region IDs
  2. Dilate each region to detect shared boundaries (thin walls / doors)
  3. Build adjacency graph: nodes = regions, edges = adjacent pairs
  4. Estimate edge costs from centroid distances weighted by passage width
  5. Inject CLASS_LOADING zones as named semantic nodes

Output:
  AreaGraph = Dict[NodeId, List[Tuple[NodeId, float]]]
  NodeId = str  (e.g., "region_0", "loading_zone_A")

References:
  - osmAG-from-cad (jiajiezhang7/osmAG-from-cad): Voronoi-based area graph
    generation; our approach uses connected-component adjacency as a simpler,
    deterministic alternative for industrial/factory layouts.
  - Hou, Yuan, Schwertfeger. "Area Graph: Generation of Topological Maps
    using the Voronoi Diagram." ICAR 2019.
"""

from __future__ import annotations

from typing import Any, Dict, List, Optional, Set, Tuple

import numpy as np
from scipy import ndimage

# Import topology constants from sibling module
try:
    from cad_parser.astar_topology import (
        CLASS_INFLATED,
        CLASS_LOADING,
        CLASS_OBSTACLE,
        CLASS_PATH,
        CLASS_UNKNOWN,
    )
except ImportError:
    from astar_topology import (  # type: ignore
        CLASS_INFLATED,
        CLASS_LOADING,
        CLASS_OBSTACLE,
        CLASS_PATH,
        CLASS_UNKNOWN,
    )

# Type aliases aligned with spatial_api_stub.py
NodeId = str
AreaGraph = Dict[NodeId, List[Tuple[NodeId, float]]]


# ══════════════════════════════════════════════════════════════════════
# Core: topology grid → AreaGraph
# ══════════════════════════════════════════════════════════════════════

def build_area_graph(
    topology: np.ndarray,
    transform: Dict[str, Any],
    region_min_area_m2: float = 2.0,
    dilation_px: int = 2,
) -> Tuple[AreaGraph, Dict[str, Any]]:
    """Build region adjacency graph from classified topology grid.

    Args:
        topology: H×W uint8 from ``classify_topology_global()``.
        transform: Dict with ``origin`` (x_m, y_m), ``resolution`` (m/px),
                   ``shape`` (H, W).
        region_min_area_m2: Merge regions smaller than this into neighbours.
        dilation_px: Dilation radius for adjacency detection. Larger values
                     bridge thicker walls, smaller preserves fine topology.

    Returns:
        (area_graph, region_meta) where:
          - area_graph: {node_id: [(neighbor_id, cost_m), ...]}
          - region_meta: dict with region centroids, areas, polygons, labels
    """
    H, W = topology.shape
    resolution = float(transform["resolution"])
    origin_x, origin_y = transform["origin"]

    # ── 1. Label connected components of CLASS_PATH ──────────────────
    path_mask = topology == CLASS_PATH
    if not path_mask.any():
        return {}, {"error": "No CLASS_PATH pixels in topology grid"}

    struct_8 = ndimage.generate_binary_structure(2, 2)
    labeled, n_regions = ndimage.label(path_mask, structure=struct_8)

    if n_regions == 0:
        return {}, {"error": "No connected regions found"}

    # ── 2. Compute region properties ─────────────────────────────────
    region_areas_px: Dict[int, int] = {}
    region_centroids_px: Dict[int, Tuple[float, float]] = {}
    for rid in range(1, n_regions + 1):
        mask = labeled == rid
        region_areas_px[rid] = int(mask.sum())
        ys, xs = np.where(mask)
        region_centroids_px[rid] = (float(xs.mean()), float(ys.mean()))

    # Convert to world units
    region_areas_m2: Dict[int, float] = {
        rid: area_px * resolution * resolution
        for rid, area_px in region_areas_px.items()
    }
    region_centroids_world: Dict[int, Tuple[float, float]] = {
        rid: (
            origin_x + cx * resolution,
            origin_y + cy * resolution,
        )
        for rid, (cx, cy) in region_centroids_px.items()
    }

    # ── 3. Detect region adjacency via dilation ──────────────────────
    adjacency: Dict[int, Set[int]] = {rid: set() for rid in range(1, n_regions + 1)}
    shared_boundary_px: Dict[Tuple[int, int], int] = {}

    for rid in range(1, n_regions + 1):
        region_mask = labeled == rid
        # Dilate this region
        dilated = ndimage.binary_dilation(
            region_mask, structure=struct_8, iterations=dilation_px,
        )
        # Find overlapping regions (excluding self and background)
        overlap_ids = np.unique(labeled[dilated & ~region_mask])
        for other_id in overlap_ids:
            if other_id in (0, rid):
                continue
            adjacency[rid].add(int(other_id))
            adjacency[int(other_id)].add(rid)
            # Count shared boundary pixels (more = passage, fewer = thin wall)
            edge = (dilated & ~region_mask) & (labeled == other_id)
            key = tuple(sorted([rid, int(other_id)]))
            shared_boundary_px[key] = shared_boundary_px.get(key, 0) + int(edge.sum())

    # ── 4. Build AreaGraph with edge costs ───────────────────────────
    area_graph: AreaGraph = {}
    for rid in range(1, n_regions + 1):
        node_id = f"region_{rid - 1}"
        edges: List[Tuple[NodeId, float]] = []
        for nbr_id in sorted(adjacency[rid]):
            nbr_node = f"region_{nbr_id - 1}"
            # Cost = centroid distance / (1 + boundary connectivity)
            cx1, cy1 = region_centroids_world[rid]
            cx2, cy2 = region_centroids_world[nbr_id]
            centroid_dist = np.hypot(cx2 - cx1, cy2 - cy1)
            key = tuple(sorted([rid, nbr_id]))
            boundary_px = shared_boundary_px.get(key, 1)
            # More shared boundary → easier passage → lower cost
            cost_m = centroid_dist / (1.0 + boundary_px * resolution)
            cost_m = round(cost_m, 3)
            edges.append((nbr_node, cost_m))
        area_graph[node_id] = edges

    # ── 5. Merge small regions ───────────────────────────────────────
    if region_min_area_m2 > 0:
        area_graph, region_areas_m2, region_centroids_world = _merge_small_regions(
            area_graph, region_areas_m2, region_centroids_world,
            adjacency, region_min_area_m2,
        )

    # ── 6. Build region metadata ─────────────────────────────────────
    region_meta: Dict[str, Any] = {
        "n_regions": len(area_graph),
        "regions": {
            node_id: {
                "centroid_world": region_centroids_world.get(rid, (0.0, 0.0)),
                "area_m2": round(region_areas_m2.get(rid, 0.0), 2),
                "n_neighbors": len(edges),
            }
            for node_id, edges in area_graph.items()
            for rid in [int(node_id.split("_")[1]) + 1]
            # This comprehension is awkward; rebuild properly
        },
        "resolution_m_per_px": resolution,
    }

    # Rebuild region_meta properly
    region_meta["regions"] = {}
    for node_id, edges in area_graph.items():
        rid = _node_to_region_id(node_id)
        region_meta["regions"][node_id] = {
            "centroid_world": list(region_centroids_world.get(rid, (0.0, 0.0))),
            "area_m2": round(region_areas_m2.get(rid, 0.0), 2),
            "n_neighbors": len(edges),
        }

    return area_graph, region_meta


# ══════════════════════════════════════════════════════════════════════
# Loading zone injection
# ══════════════════════════════════════════════════════════════════════

def inject_loading_zones(
    area_graph: AreaGraph,
    topology: np.ndarray,
    transform: Dict[str, Any],
    region_labels: np.ndarray,
    region_meta: Optional[Dict[str, Any]] = None,
) -> Tuple[AreaGraph, Dict[str, Any]]:
    """Overlay CLASS_LOADING zones as named semantic nodes in the AreaGraph.

    For each CLASS_LOADING cluster, identifies the enclosing region and adds
    a named node (e.g., "loading_zone_A") with a zero-cost edge to that region.

    Args:
        area_graph: Output of ``build_area_graph()``.
        topology: H×W topology grid.
        transform: Transform dict with resolution and origin.
        region_labels: Connected-component labels from build_area_graph.
        region_meta: Optional region metadata to update.

    Returns:
        (augmented_area_graph, loading_zone_info)
    """
    loading_mask = topology == CLASS_LOADING
    if not loading_mask.any():
        return area_graph, {"zones": [], "note": "No CLASS_LOADING pixels found"}

    resolution = float(transform["resolution"])
    origin_x, origin_y = transform["origin"]

    # Cluster loading zones (8-connected)
    struct_8 = ndimage.generate_binary_structure(2, 2)
    load_labels, n_zones = ndimage.label(loading_mask, structure=struct_8)

    zone_names: List[str] = []
    for zid in range(1, n_zones + 1):
        zone_mask = load_labels == zid
        ys, xs = np.where(zone_mask)
        cx_px = float(xs.mean())
        cy_px = float(ys.mean())
        cx_w = origin_x + cx_px * resolution
        cy_w = origin_y + cy_px * resolution

        # Find enclosing region
        # Take the most frequent region ID under the zone mask (dilated slightly)
        dilated_zone = ndimage.binary_dilation(zone_mask, structure=struct_8, iterations=1)
        enclosing_rids = region_labels[dilated_zone & (region_labels > 0)]
        if len(enclosing_rids) == 0:
            continue
        enclosing_rid = int(np.bincount(enclosing_rids).argmax())
        enclosing_node = f"region_{enclosing_rid - 1}"

        zone_node = f"loading_zone_{chr(65 + zid - 1)}"  # A, B, C, ...
        zone_names.append(zone_node)

        # Add node with edge to enclosing region (cost = 0: immediate access)
        area_graph[zone_node] = [(enclosing_node, 0.0)]
        # Add reverse edge from enclosing region
        area_graph[enclosing_node].append((zone_node, 0.0))

    return area_graph, {
        "zones": zone_names,
        "n_zones": len(zone_names),
    }


# ══════════════════════════════════════════════════════════════════════
# Direct wall-segment → AreaGraph (bypasses rasterization)
# ══════════════════════════════════════════════════════════════════════

def build_area_graph_from_walls(
    walls: List[Tuple[Tuple[float, float], Tuple[float, float]]],
    boundary: Optional[List[Tuple[float, float]]] = None,
    resolution: float = 0.05,
    padding_m: float = 0.5,
    wall_thickness_m: float = 0.10,
    robot_radius_m: float = 0.30,
) -> Tuple[AreaGraph, Dict[str, Any]]:
    """Build AreaGraph directly from wall line segments.

    This internally rasterizes walls to an occupancy grid, runs the full
    topology pipeline, and then calls ``build_area_graph()``. It's a
    convenience wrapper for callers that have raw geometry but no grid yet.

    Args:
        walls: List of ((x1,y1), (x2,y2)) wall segments in meters.
        boundary: Optional list of (x,y) boundary polygon vertices.
        resolution: Grid resolution (m/px).
        padding_m: Padding around geometry (m).
        wall_thickness_m: Physical wall thickness (m).
        robot_radius_m: Robot radius for clearance computation.

    Returns:
        (area_graph, pipeline_info) — the pipeline_info dict contains the
        intermediate grid, distance_field, topology, and transform.
    """
    from cad_parser.astar_topology import (
        bridge_thin_walls,
        classify_topology_global,
        compute_distance_field,
        rasterize_to_grid,
        remove_exterior_freespace,
    )

    # Build synthetic cad_data from wall segments
    xs = [v for (sx, sy), (ex, ey) in walls for v in (sx, ex)]
    ys = [v for (sx, sy), (ex, ey) in walls for v in (sy, ey)]
    if not xs:
        return {}, {"error": "No wall geometry"}

    bbox = (min(xs), min(ys), max(xs), max(ys))
    if boundary is None:
        boundary = [
            (bbox[0] - padding_m, bbox[1] - padding_m),
            (bbox[2] + padding_m, bbox[1] - padding_m),
            (bbox[2] + padding_m, bbox[3] + padding_m),
            (bbox[0] - padding_m, bbox[3] + padding_m),
        ]

    cad_data = {
        "layout_id": "walls_direct",
        "room_type": "custom",
        "units": "meters",
        "boundary": boundary,
        "walls": walls,
        "doors": [],
        "windows": [],
        "objects": [],
        "bbox": bbox,
    }

    grid, transform = rasterize_to_grid(
        cad_data, resolution=resolution,
        padding_m=padding_m, wall_thickness_m=wall_thickness_m,
    )
    grid = remove_exterior_freespace(grid, close_gaps_px=2)
    grid = bridge_thin_walls(grid, kernel_px=2)

    distance_field = compute_distance_field(grid)
    robot_radius_px = robot_radius_m / resolution

    topology, topo_stats = classify_topology_global(
        grid, distance_field, robot_radius_px,
    )

    area_graph, region_meta = build_area_graph(topology, transform)

    # Inject loading zones if any
    path_mask = topology == CLASS_PATH
    struct_8 = ndimage.generate_binary_structure(2, 2)
    region_labels, _ = ndimage.label(path_mask, structure=struct_8)
    area_graph, load_info = inject_loading_zones(
        area_graph, topology, transform, region_labels,
    )

    pipeline_info = {
        "grid": grid,
        "distance_field": distance_field,
        "topology": topology,
        "topology_stats": topo_stats,
        "transform": transform,
        "region_meta": region_meta,
        "loading_zones": load_info,
    }
    return area_graph, pipeline_info


# ══════════════════════════════════════════════════════════════════════
# Serialization
# ══════════════════════════════════════════════════════════════════════

def area_graph_to_geojson(
    area_graph: AreaGraph,
    region_meta: Dict[str, Any],
) -> Dict[str, Any]:
    """Serialize AreaGraph as GeoJSON FeatureCollection.

    Nodes become Point features; edges become LineString features.
    Compatible with GIS tools and web map viewers.
    """
    features: List[Dict[str, Any]] = []

    # Nodes as Point features
    regions = region_meta.get("regions", {})
    for node_id, edges in area_graph.items():
        info = regions.get(node_id, {})
        centroid = info.get("centroid_world", [0.0, 0.0])
        features.append({
            "type": "Feature",
            "geometry": {
                "type": "Point",
                "coordinates": [centroid[0], centroid[1]],
            },
            "properties": {
                "node_id": node_id,
                "area_m2": info.get("area_m2", 0),
                "n_neighbors": len(edges),
                "is_loading_zone": node_id.startswith("loading_zone"),
            },
        })

    # Edges as LineString features
    for node_id, edges in area_graph.items():
        c1 = regions.get(node_id, {}).get("centroid_world", [0, 0])
        for nbr_id, cost in edges:
            c2 = regions.get(nbr_id, {}).get("centroid_world", [0, 0])
            features.append({
                "type": "Feature",
                "geometry": {
                    "type": "LineString",
                    "coordinates": [[c1[0], c1[1]], [c2[0], c2[1]]],
                },
                "properties": {
                    "source": node_id,
                    "target": nbr_id,
                    "cost_m": cost,
                },
            })

    return {
        "type": "FeatureCollection",
        "features": features,
        "metadata": {
            "n_nodes": len(area_graph),
            "n_edges": sum(len(e) for e in area_graph.values()),
        },
    }


def save_area_graph(
    area_graph: AreaGraph,
    region_meta: Dict[str, Any],
    output_dir: str,
    prefix: str = "area_graph",
) -> Dict[str, str]:
    """Persist AreaGraph as JSON + GeoJSON files.

    Returns dict mapping format to file path.
    """
    import json
    from pathlib import Path

    out = Path(output_dir)
    out.mkdir(parents=True, exist_ok=True)

    paths: Dict[str, str] = {}

    # Adjacency JSON
    adj_path = out / f"{prefix}_adjacency.json"
    adj_path.write_text(
        json.dumps(area_graph, indent=2, ensure_ascii=False), encoding="utf-8"
    )
    paths["adjacency"] = str(adj_path)

    # Region metadata JSON
    meta_path = out / f"{prefix}_regions.json"
    meta_path.write_text(
        json.dumps(region_meta, indent=2, ensure_ascii=False), encoding="utf-8"
    )
    paths["regions"] = str(meta_path)

    # GeoJSON
    geojson = area_graph_to_geojson(area_graph, region_meta)
    geo_path = out / f"{prefix}.geojson"
    geo_path.write_text(
        json.dumps(geojson, indent=2, ensure_ascii=False), encoding="utf-8"
    )
    paths["geojson"] = str(geo_path)

    return paths


# ══════════════════════════════════════════════════════════════════════
# Helpers
# ══════════════════════════════════════════════════════════════════════

def _node_to_region_id(node_id: str) -> int:
    """Convert "region_N" back to 1-based region ID."""
    if node_id.startswith("region_"):
        return int(node_id.split("_")[1]) + 1
    return -1


def _merge_small_regions(
    area_graph: AreaGraph,
    region_areas_m2: Dict[int, float],
    region_centroids: Dict[int, Tuple[float, float]],
    adjacency: Dict[int, Set[int]],
    min_area_m2: float,
) -> Tuple[AreaGraph, Dict[int, float], Dict[int, Tuple[float, float]]]:
    """Merge regions smaller than min_area_m2 into their largest neighbor.

    Operates on the 1-based region ID representation and returns updated
    AreaGraph with merged nodes removed.
    """
    # Build reverse mapping: 1-based rid → node_id
    rid_to_node: Dict[int, str] = {}
    for node_id in list(area_graph.keys()):
        if node_id.startswith("region_"):
            rid = _node_to_region_id(node_id)
            if rid > 0:
                rid_to_node[rid] = node_id

    # Identify small regions
    small_rids = {
        rid for rid, area in region_areas_m2.items()
        if area < min_area_m2
    }

    merged_into: Dict[int, int] = {}  # small_rid → absorb_rid
    for rid in sorted(small_rids, key=lambda r: region_areas_m2.get(r, 0)):
        if rid not in adjacency:
            continue
        neighbors = [n for n in adjacency[rid] if n not in small_rids]
        if not neighbors:
            continue
        # Merge into largest neighbor
        absorb_rid = max(neighbors, key=lambda n: region_areas_m2.get(n, 0))
        merged_into[rid] = absorb_rid
        # Update adjacency: absorb_rid inherits small_rid's neighbors
        adjacency[absorb_rid] |= adjacency[rid]
        adjacency[absorb_rid].discard(absorb_rid)
        # Remove small from neighbors' adjacency
        for nbr in list(adjacency[rid]):
            adjacency[nbr].discard(rid)
            if absorb_rid != nbr:
                adjacency[nbr].add(absorb_rid)
        del adjacency[rid]

    # Rebuild AreaGraph without merged nodes
    new_graph: AreaGraph = {}
    new_areas: Dict[int, float] = {}
    new_centroids: Dict[int, Tuple[float, float]] = {}

    merged_nodes_removed = set()
    for node_id, edges in area_graph.items():
        rid = _node_to_region_id(node_id)
        if rid in merged_into:
            merged_nodes_removed.add(node_id)
            continue
        # Rebuild edges, replacing merged nodes with their absorb node
        new_edges: List[Tuple[NodeId, float]] = []
        seen_nbrs: Set[str] = set()
        for nbr_id, cost in edges:
            nbr_rid = _node_to_region_id(nbr_id)
            if nbr_rid in merged_into:
                nbr_id = rid_to_node.get(merged_into[nbr_rid], nbr_id)
            if nbr_id not in seen_nbrs and nbr_id != node_id:
                new_edges.append((nbr_id, cost))
                seen_nbrs.add(nbr_id)
        new_graph[node_id] = new_edges
        new_areas[rid] = region_areas_m2.get(rid, 0)
        new_centroids[rid] = region_centroids.get(rid, (0.0, 0.0))

    return new_graph, new_areas, new_centroids
