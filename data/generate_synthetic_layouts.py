"""
generate_synthetic_layouts.py — SMDSL Synthetic Layout Generator

Generates FloorplanQA-compatible JSON layouts for EDT path planning benchmarks.
Supports multiple room types with randomized walls, doors, and furniture objects.

Usage:
    python data/generate_synthetic_layouts.py --n_layouts 200 --output data/datasets/synthetic_benchmark/
"""

import argparse
import json
import math
import os
import random
import sys
from pathlib import Path
from typing import Any, Dict, List, Tuple

_PROJ = Path(__file__).resolve().parent.parent
_SMDSL = _PROJ / "SMDSL"
sys.path.insert(0, str(_SMDSL))

RoomSpecs = {
    "bedroom":     {"w": (4.0, 6.0), "h": (3.5, 5.5), "wall_t": 0.15, "n_objects": (2, 5)},
    "living_room": {"w": (5.0, 8.0), "h": (4.0, 7.0), "wall_t": 0.15, "n_objects": (3, 8)},
    "kitchen":     {"w": (3.0, 5.0), "h": (2.5, 4.5), "wall_t": 0.12, "n_objects": (3, 6)},
    "bathroom":    {"w": (1.8, 3.5), "h": (1.8, 3.0), "wall_t": 0.10, "n_objects": (1, 3)},
    "corridor":    {"w": (1.5, 2.5), "h": (3.0, 8.0), "wall_t": 0.12, "n_objects": (0, 1)},
    "office":      {"w": (4.0, 7.0), "h": (3.5, 6.0), "wall_t": 0.15, "n_objects": (3, 7)},
    "warehouse":   {"w": (8.0, 20.0), "h": (6.0, 15.0), "wall_t": 0.20, "n_objects": (5, 15)},
}

FurniturePool = {
    "bedroom": [
        {"label": "bed", "w": (1.8, 2.2), "h": (1.4, 2.0)},
        {"label": "wardrobe", "w": (0.6, 1.2), "h": (1.5, 2.5)},
        {"label": "nightstand", "w": (0.4, 0.6), "h": (0.4, 0.6)},
        {"label": "desk", "w": (0.8, 1.5), "h": (0.5, 0.8)},
    ],
    "living_room": [
        {"label": "sofa", "w": (1.8, 2.8), "h": (0.8, 1.2)},
        {"label": "coffee_table", "w": (0.6, 1.2), "h": (0.5, 0.8)},
        {"label": "tv_stand", "w": (1.0, 2.0), "h": (0.4, 0.6)},
        {"label": "bookshelf", "w": (0.3, 0.5), "h": (1.5, 2.5)},
        {"label": "armchair", "w": (0.7, 1.0), "h": (0.7, 1.0)},
    ],
    "kitchen": [
        {"label": "counter", "w": (0.6, 0.8), "h": (2.0, 3.5)},
        {"label": "fridge", "w": (0.7, 0.9), "h": (0.7, 0.9)},
        {"label": "stove", "w": (0.6, 0.8), "h": (0.6, 0.7)},
        {"label": "table", "w": (0.8, 1.5), "h": (0.6, 1.2)},
        {"label": "sink", "w": (0.5, 0.7), "h": (0.5, 0.6)},
    ],
    "bathroom": [
        {"label": "toilet", "w": (0.4, 0.5), "h": (0.6, 0.7)},
        {"label": "sink", "w": (0.4, 0.6), "h": (0.3, 0.5)},
        {"label": "bathtub", "w": (0.7, 0.9), "h": (1.5, 1.8)},
    ],
    "corridor": [],
    "office": [
        {"label": "desk", "w": (1.0, 1.8), "h": (0.6, 0.9)},
        {"label": "chair", "w": (0.5, 0.7), "h": (0.5, 0.7)},
        {"label": "bookshelf", "w": (0.3, 0.5), "h": (1.5, 3.0)},
        {"label": "cabinet", "w": (0.5, 1.0), "h": (0.5, 1.5)},
    ],
    "warehouse": [
        {"label": "shelf", "w": (0.6, 1.0), "h": (2.0, 5.0)},
        {"label": "pallet", "w": (1.0, 1.3), "h": (1.0, 1.3)},
        {"label": "machine", "w": (1.0, 3.0), "h": (1.0, 3.0)},
    ],
}


def _random_point_in_rect(
    x_min: float, y_min: float, x_max: float, y_max: float, margin: float = 0.0
) -> Tuple[float, float]:
    return (
        random.uniform(x_min + margin, x_max - margin),
        random.uniform(y_min + margin, y_max - margin),
    )


def _rects_overlap(
    ax1: float, ay1: float, ax2: float, ay2: float,
    bx1: float, by1: float, bx2: float, by2: float,
    margin: float = 0.3,
) -> bool:
    return not (
        ax2 + margin < bx1 or bx2 + margin < ax1
        or ay2 + margin < by1 or by2 + margin < ay1
    )


def _segment_intersects_rect(
    sx: float, sy: float, ex: float, ey: float,
    rx1: float, ry1: float, rx2: float, ry2: float,
) -> bool:
    """Check if line segment (sx,sy)-(ex,ey) intersects rectangle."""
    # Simple: check if either endpoint is inside
    if rx1 <= sx <= rx2 and ry1 <= sy <= ry2:
        return True
    if rx1 <= ex <= rx2 and ry1 <= ey <= ry2:
        return True
    # Check segment-rectangle edge intersections (simplified)
    corners = [(rx1, ry1), (rx2, ry1), (rx2, ry2), (rx1, ry2)]
    for i in range(4):
        ax, ay = corners[i]
        bx, by = corners[(i + 1) % 4]
        denom = (ex - sx) * (by - ay) - (ey - sy) * (bx - ax)
        if abs(denom) < 1e-10:
            continue
        t = ((ax - sx) * (by - ay) - (ay - sy) * (bx - ax)) / denom
        u = -((sx - ax) * (ey - sy) - (sy - ay) * (ex - sx)) / denom
        if 0 <= t <= 1 and 0 <= u <= 1:
            return True
    return False


def generate_single_layout(layout_id: int, room_type: str) -> Dict[str, Any]:
    spec = RoomSpecs[room_type]
    rw = random.uniform(*spec["w"])
    rh = random.uniform(*spec["h"])
    wt = spec["wall_t"]

    # Room boundary (counter-clockwise)
    boundary: List[Dict[str, float]] = [
        {"x": 0.0, "y": 0.0},
        {"x": rw, "y": 0.0},
        {"x": rw, "y": rh},
        {"x": 0.0, "y": rh},
    ]

    # Walls (4 segments)
    walls: List[Dict[str, Any]] = [
        {"start": {"x": 0.0, "y": 0.0}, "end": {"x": rw, "y": 0.0}},
        {"start": {"x": rw, "y": 0.0}, "end": {"x": rw, "y": rh}},
        {"start": {"x": rw, "y": rh}, "end": {"x": 0.0, "y": rh}},
        {"start": {"x": 0.0, "y": rh}, "end": {"x": 0.0, "y": 0.0}},
    ]

    # Door: random gap on one wall (not on corners)
    door_wall = random.randint(0, 3)
    door_size = 0.9
    if door_wall in (0, 2):  # horizontal walls
        door_center = random.uniform(door_size / 2 + 0.5, rw - door_size / 2 - 0.5)
        if door_wall == 0:
            door_pts = [
                {"x": door_center - door_size / 2, "y": 0.0},
                {"x": door_center + door_size / 2, "y": 0.0},
            ]
        else:
            door_pts = [
                {"x": door_center - door_size / 2, "y": rh},
                {"x": door_center + door_size / 2, "y": rh},
            ]
    else:  # vertical walls
        door_center = random.uniform(door_size / 2 + 0.5, rh - door_size / 2 - 0.5)
        if door_wall == 1:
            door_pts = [
                {"x": rw, "y": door_center - door_size / 2},
                {"x": rw, "y": door_center + door_size / 2},
            ]
        else:
            door_pts = [
                {"x": 0.0, "y": door_center - door_size / 2},
                {"x": 0.0, "y": door_center + door_size / 2},
            ]

    doors = [{"label": "door_0", "points": door_pts}]

    # Window: random opening on a different wall
    win_wall = (door_wall + random.randint(1, 2)) % 4
    win_size = random.uniform(0.6, 1.5)
    if win_wall in (0, 2):
        wc = random.uniform(win_size / 2 + 0.3, rw - win_size / 2 - 0.3)
        if win_wall == 0:
            win_pts = [{"x": wc - win_size / 2, "y": 0.0}, {"x": wc + win_size / 2, "y": 0.0}]
        else:
            win_pts = [{"x": wc - win_size / 2, "y": rh}, {"x": wc + win_size / 2, "y": rh}]
    else:
        wc = random.uniform(win_size / 2 + 0.3, rh - win_size / 2 - 0.3)
        if win_wall == 1:
            win_pts = [{"x": rw, "y": wc - win_size / 2}, {"x": rw, "y": wc + win_size / 2}]
        else:
            win_pts = [{"x": 0.0, "y": wc - win_size / 2}, {"x": 0.0, "y": wc + win_size / 2}]

    windows = [{"label": "window_0", "points": win_pts}]

    # Furniture (avoiding walls, door, and window areas)
    objects: List[Dict[str, Any]] = []
    placed_rects: List[Tuple[float, float, float, float]] = []
    # Add wall margin
    placed_rects.append((0.0, 0.0, rw, 0.2))    # bottom wall
    placed_rects.append((0.0, rh - 0.2, rw, rh))  # top wall
    placed_rects.append((0.0, 0.0, 0.2, rh))      # left wall
    placed_rects.append((rw - 0.2, 0.0, rw, rh))  # right wall
    # Add door area
    placed_rects.append((door_center - 0.6, -0.1, door_center + 0.6, 0.5))
    # Add window area
    if win_wall == 0:
        placed_rects.append((wc - 0.3, -0.1, wc + 0.3, 0.3))
    elif win_wall == 2:
        placed_rects.append((wc - 0.3, rh - 0.3, wc + 0.3, rh + 0.1))

    n_objects = random.randint(*spec["n_objects"])
    furniture = FurniturePool.get(room_type, FurniturePool["office"])
    if not furniture:
        n_objects = 0  # corridor etc. has no furniture
    attempts = 0
    while len(objects) < n_objects and attempts < 200:
        attempts += 1
        furn = random.choice(furniture)
        fw = random.uniform(*furn["w"])
        fh = random.uniform(*furn["h"])
        fx, fy = _random_point_in_rect(0.3, 0.3, rw - fw - 0.3, rh - fh - 0.3)
        ok = True
        for (px1, py1, px2, py2) in placed_rects:
            if _rects_overlap(fx, fy, fx + fw, fy + fh, px1, py1, px2, py2):
                ok = False
                break
        if ok:
            pts = [
                {"x": fx, "y": fy},
                {"x": fx + fw, "y": fy},
                {"x": fx + fw, "y": fy + fh},
                {"x": fx, "y": fy + fh},
            ]
            objects.append({"label": f"{furn['label']}_{len(objects)}", "points": pts})
            placed_rects.append((fx, fy, fx + fw, fy + fh))

    return {
        "layout_id": layout_id,
        "room_type": room_type,
        "room_boundary": boundary,
        "walls": walls,
        "openings": {"doors": doors, "windows": windows},
        "objects": objects,
        "units": "meters",
    }


def main():
    parser = argparse.ArgumentParser(description="Generate synthetic FloorplanQA layouts")
    parser.add_argument("--n_layouts", type=int, default=200)
    parser.add_argument("--output", type=str, default="data/datasets/synthetic_benchmark")
    parser.add_argument("--seed", type=int, default=42)
    args = parser.parse_args()

    random.seed(args.seed)
    out_dir = Path(args.output)
    out_dir.mkdir(parents=True, exist_ok=True)

    room_types = list(RoomSpecs.keys())
    # Distribution: more common room types get more layouts
    weights = [25, 25, 20, 10, 10, 5, 5]  # bedroom, living, kitchen, bath, corridor, office, warehouse

    generated = 0
    for i in range(args.n_layouts):
        rt = random.choices(room_types, weights=weights, k=1)[0]
        layout = generate_single_layout(layout_id=i, room_type=rt)
        fname = f"{rt}_room_{i}.json"
        fpath = out_dir / rt / fname
        fpath.parent.mkdir(parents=True, exist_ok=True)
        with open(fpath, "w", encoding="utf-8") as f:
            json.dump(layout, f, ensure_ascii=False, indent=2)
        generated += 1

    print(f"Generated {generated} layouts in {out_dir}")
    # Generate manifest
    manifest = {
        "total_layouts": generated,
        "room_types": {rt: RoomSpecs[rt] for rt in room_types},
        "seed": args.seed,
        "description": "Synthetic FloorplanQA-compatible layouts for SMDSL EDT benchmark",
    }
    with open(out_dir / "manifest.json", "w", encoding="utf-8") as f:
        json.dump(manifest, f, ensure_ascii=False, indent=2)
    print(f"Manifest saved to {out_dir / 'manifest.json'}")


if __name__ == "__main__":
    main()
