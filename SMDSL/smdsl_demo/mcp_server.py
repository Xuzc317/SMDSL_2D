"""
mcp_server.py — SMDSL Model Context Protocol Server

Exposes SMDSL's core spatial-motion capabilities as MCP tools for LLM agents.

Usage:
    python -m smdsl_demo.mcp_server

Configure in claude_desktop_config.json:
    {
      "mcpServers": {
        "smdsl": {
          "command": "python",
          "args": ["-m", "smdsl_demo.mcp_server"],
          "cwd": "D:/Code/SMDSL_demo/SMDSL"
        }
      }
    }
"""

from __future__ import annotations

import json
import sys
from pathlib import Path
from typing import Any, Dict, List, Optional, Tuple

_PROJ = Path(__file__).resolve().parent.parent
if str(_PROJ) not in sys.path:
    sys.path.insert(0, str(_PROJ))

import numpy as np
from mcp.server import Server
from mcp.server.stdio import stdio_server
from mcp.types import Tool, TextContent

# ── SMDSL core imports ────────────────────────────────────────────
from cad_parser.dispatcher import dispatch_cad
from cad_parser.astar_topology import (
    compute_distance_field,
    astar_shortest_path,
    classify_topology_global,
    to_topology_bundle,
    CLASS_PATH,
    CLASS_INFLATED,
)
from smdsl_demo.spatial_api_stub import _eval_distance_via_field
from smdsl_demo.trajectory_smoother import smooth_path_to_trajectory

# ── Server ────────────────────────────────────────────────────────
server = Server("smdsl", version="0.2.0")


def _load_and_prepare(file_path: str, resolution: float = 0.05):
    parsed = dispatch_cad(file_path, resolution=resolution)
    grid = parsed["grid"]
    df = compute_distance_field(grid)
    transform = parsed["transform"]
    return parsed, grid, df, transform


def _world_to_pixel(x_m: float, y_m: float, transform: dict) -> Tuple[int, int]:
    ox, oy = transform["origin"]
    res = transform["resolution"]
    return int(round((y_m - oy) / res)), int(round((x_m - ox) / res))


# ── Tool listing ───────────────────────────────────────────────────
@server.list_tools()
async def list_tools() -> List[Tool]:
    return [
        Tool(
            name="cad_dispatch",
            description="Load and parse a CAD file. Returns grid metadata, room type, entity counts, and available semantics.",
            inputSchema={
                "type": "object",
                "properties": {
                    "file_path": {"type": "string", "description": "Absolute path to CAD file"},
                    "resolution": {"type": "number", "description": "Grid resolution m/px", "default": 0.05},
                },
                "required": ["file_path"],
            },
        ),
        Tool(
            name="compute_topology",
            description="Compute EDT distance field and topology. Returns free-space ratio, max clearance, and component stats.",
            inputSchema={
                "type": "object",
                "properties": {
                    "file_path": {"type": "string", "description": "Path to CAD file"},
                    "robot_radius_m": {"type": "number", "default": 0.25},
                    "resolution": {"type": "number", "default": 0.05},
                },
                "required": ["file_path"],
            },
        ),
        Tool(
            name="plan_path",
            description="Plan a collision-free EDT-aware A* path from start to goal. Returns path pixels and clearance stats.",
            inputSchema={
                "type": "object",
                "properties": {
                    "file_path": {"type": "string"},
                    "start_x": {"type": "number"},
                    "start_y": {"type": "number"},
                    "goal_x": {"type": "number"},
                    "goal_y": {"type": "number"},
                    "robot_radius_m": {"type": "number", "default": 0.25},
                    "resolution": {"type": "number", "default": 0.05},
                    "safety_weight": {"type": "number", "default": 1.0},
                },
                "required": ["file_path", "start_x", "start_y", "goal_x", "goal_y"],
            },
        ),
        Tool(
            name="validate_trajectory",
            description="Validate trajectory against STL constraints using distance field. Returns robustness and violations.",
            inputSchema={
                "type": "object",
                "properties": {
                    "file_path": {"type": "string"},
                    "trajectory_json": {"type": "string", "description": "JSON array [{t,x,y,z},...]"},
                    "min_safety_distance_m": {"type": "number", "default": 0.3},
                    "resolution": {"type": "number", "default": 0.05},
                },
                "required": ["file_path", "trajectory_json"],
            },
        ),
        Tool(
            name="smooth_trajectory",
            description="Convert discrete pixel path to smooth time-parameterized trajectory with cubic spline and trapezoidal velocity profile.",
            inputSchema={
                "type": "object",
                "properties": {
                    "path_json": {"type": "string", "description": "JSON array of [row, col] pixel pairs"},
                    "resolution": {"type": "number", "default": 0.05},
                    "origin_x": {"type": "number", "default": 0.0},
                    "origin_y": {"type": "number", "default": 0.0},
                    "total_time_s": {"type": "number", "default": 5.0},
                    "v_max": {"type": "number", "default": 1.0},
                },
                "required": ["path_json", "resolution", "origin_x", "origin_y"],
            },
        ),
        Tool(
            name="analyze_scene",
            description="Complete scene analysis: load CAD, compute topology, find safe regions. Returns comprehensive scene report with safe navigation candidates.",
            inputSchema={
                "type": "object",
                "properties": {
                    "file_path": {"type": "string"},
                    "robot_radius_m": {"type": "number", "default": 0.25},
                    "safety_distance_m": {"type": "number", "default": 0.3},
                    "resolution": {"type": "number", "default": 0.05},
                },
                "required": ["file_path"],
            },
        ),
    ]


# ── Tool handlers ──────────────────────────────────────────────────
@server.call_tool()
async def call_tool(name: str, arguments: dict) -> List[TextContent]:
    try:
        if name == "cad_dispatch":
            return await _cad_dispatch(arguments)
        elif name == "compute_topology":
            return await _compute_topology(arguments)
        elif name == "plan_path":
            return await _plan_path(arguments)
        elif name == "validate_trajectory":
            return await _validate_trajectory(arguments)
        elif name == "smooth_trajectory":
            return await _smooth_trajectory(arguments)
        elif name == "analyze_scene":
            return await _analyze_scene(arguments)
        return [TextContent(type="text", text=f"Unknown tool: {name}")]
    except Exception as e:
        return [TextContent(type="text", text=f"Error [{type(e).__name__}]: {str(e)}")]


async def _cad_dispatch(args: dict) -> List[TextContent]:
    parsed = dispatch_cad(args["file_path"], resolution=float(args.get("resolution", 0.05)))
    grid = parsed["grid"]
    result = {
        "mode": parsed["mode"],
        "grid_shape": list(grid.shape),
        "free_cells": int(np.sum(grid == 1)),
        "occupied_cells": int(np.sum(grid == 0)),
        "free_ratio": round(float(np.sum(grid == 1)) / grid.size, 4),
        "room_type": parsed.get("cad_data", {}).get("room_type", "unknown"),
        "has_semantics": parsed.get("has_semantics", False),
        "note": parsed.get("note", ""),
    }
    return [TextContent(type="text", text=json.dumps(result, ensure_ascii=False, indent=2))]


async def _compute_topology(args: dict) -> List[TextContent]:
    parsed, grid, df, transform = _load_and_prepare(args["file_path"], float(args.get("resolution", 0.05)))
    robot_radius_px = float(args.get("robot_radius_m", 0.25)) / transform["resolution"]
    topology, topo_stats = classify_topology_global(grid, df, robot_radius_px=robot_radius_px)
    res = float(transform["resolution"])
    result = {
        "grid_shape": list(grid.shape),
        "free_ratio": round(float(np.sum(grid == 1)) / grid.size, 4),
        "max_clearance_m": round(float(df.max()) * res, 3),
        "mean_clearance_m": round(float(df[grid == 1].mean()) * res, 3),
        "n_path_pixels": int(np.sum(topology == CLASS_PATH)),
        "n_inflated_pixels": int(np.sum(topology == CLASS_INFLATED)),
        "n_components": topo_stats["n_components"],
        "largest_component_ratio": round(topo_stats["largest_component_frac"], 4),
    }
    return [TextContent(type="text", text=json.dumps(result, ensure_ascii=False, indent=2))]


async def _plan_path(args: dict) -> List[TextContent]:
    parsed, grid, df, transform = _load_and_prepare(args["file_path"], float(args.get("resolution", 0.05)))
    robot_radius_px = float(args.get("robot_radius_m", 0.25)) / transform["resolution"]
    start_rc = _world_to_pixel(float(args["start_x"]), float(args["start_y"]), transform)
    goal_rc = _world_to_pixel(float(args["goal_x"]), float(args["goal_y"]), transform)
    H, W = grid.shape
    if not (0 <= start_rc[0] < H and 0 <= start_rc[1] < W):
        return [TextContent(type="text", text=json.dumps({"error": "start out of bounds"}))]
    if not (0 <= goal_rc[0] < H and 0 <= goal_rc[1] < W):
        return [TextContent(type="text", text=json.dumps({"error": "goal out of bounds"}))]
    path_rc = astar_shortest_path(
        grid=grid, distance_field=df,
        start_rc=start_rc, goal_rc=goal_rc,
        robot_radius_px=robot_radius_px * 1.3,
        safety_weight=float(args.get("safety_weight", 1.0)),
    )
    if path_rc is None:
        return [TextContent(type="text", text=json.dumps({"error": "no path found"}))]
    vals = df[tuple(np.array(path_rc).T)]
    result = {
        "path_length_px": len(path_rc),
        "path_length_m": round(len(path_rc) * transform["resolution"], 3),
        "clearance_min_m": round(float(vals.min()) * transform["resolution"], 4),
        "clearance_mean_m": round(float(vals.mean()) * transform["resolution"], 4),
        "start_pixel": list(start_rc),
        "goal_pixel": list(goal_rc),
        "path_sample": [[int(r), int(c)] for r, c in path_rc[::max(1, len(path_rc) // 20)]],
    }
    return [TextContent(type="text", text=json.dumps(result, ensure_ascii=False))]


async def _validate_trajectory(args: dict) -> List[TextContent]:
    parsed, grid, df, transform = _load_and_prepare(args["file_path"], float(args.get("resolution", 0.05)))
    trajectory = json.loads(args["trajectory_json"])
    bundle = to_topology_bundle({"grid": grid, "distance_field": df, "transform": transform, "robot_radius_m": 0.25})
    report = {"robustness": 0.0, "violated": False, "source": "distance_field", "details": [], "violation_nodes": []}
    report = _eval_distance_via_field(
        trajectory=trajectory,
        min_dist_m=float(args.get("min_safety_distance_m", 0.3)),
        bundle=bundle, report=report,
    )
    result = {
        "robustness": report["robustness"],
        "violated": report["violated"],
        "source": report["source"],
        "n_violations": len(report.get("violation_nodes", [])),
        "n_samples": len(report.get("details", [])),
    }
    return [TextContent(type="text", text=json.dumps(result, ensure_ascii=False))]


async def _smooth_trajectory(args: dict) -> List[TextContent]:
    path_rc = [tuple(p) for p in json.loads(args["path_json"])]
    traj = smooth_path_to_trajectory(
        path_rc=path_rc,
        resolution=float(args.get("resolution", 0.05)),
        origin_xy=(float(args.get("origin_x", 0.0)), float(args.get("origin_y", 0.0))),
        total_time_s=float(args.get("total_time_s", 5.0)),
        v_max=float(args.get("v_max", 1.0)),
    )
    result = {
        "n_points": len(traj),
        "duration_s": round(traj[-1]["t"], 2) if traj else 0,
        "start": traj[0] if traj else None,
        "end": traj[-1] if traj else None,
    }
    return [TextContent(type="text", text=json.dumps(result, ensure_ascii=False))]


async def _analyze_scene(args: dict) -> List[TextContent]:
    parsed, grid, df, transform = _load_and_prepare(args["file_path"], float(args.get("resolution", 0.05)))
    robot_radius_px = float(args.get("robot_radius_m", 0.25)) / transform["resolution"]
    topology, topo_stats = classify_topology_global(grid, df, robot_radius_px=robot_radius_px)
    safe_mask = (grid == 1) & (df >= robot_radius_px)
    safe_indices = np.argwhere(safe_mask)
    candidates = []
    if len(safe_indices) > 0:
        safe_vals = df[safe_mask]
        top_k = min(5, len(safe_vals))
        top_idx = np.argpartition(safe_vals, -top_k)[-top_k:]
        for i in top_idx:
            r, c = safe_indices[i]
            ox_m, oy_m = transform["origin"]
            res = transform["resolution"]
            candidates.append({
                "pixel": [int(r), int(c)],
                "world_x": round(ox_m + c * res, 3),
                "world_y": round(oy_m + r * res, 3),
                "clearance_m": round(float(df[r, c]) * res, 3),
            })
    res = float(transform["resolution"])
    result = {
        "mode": parsed["mode"],
        "room_type": parsed.get("cad_data", {}).get("room_type", "unknown"),
        "grid_shape": list(grid.shape),
        "free_ratio": round(float(np.sum(grid == 1)) / grid.size, 4),
        "max_clearance_m": round(float(df.max()) * res, 3),
        "mean_clearance_m": round(float(df[grid == 1].mean()) * res, 3),
        "n_components": topo_stats["n_components"],
        "largest_component_ratio": round(topo_stats["largest_component_frac"], 4),
        "safe_candidates": candidates,
    }
    return [TextContent(type="text", text=json.dumps(result, ensure_ascii=False, indent=2))]


def main():
    import asyncio
    async def _run():
        async with stdio_server() as (read_stream, write_stream):
            await server.run(read_stream, write_stream, server.create_initialization_options())
    asyncio.run(_run())


if __name__ == "__main__":
    main()
