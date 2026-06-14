"""
trajectory_smoother.py — 三次样条 + 梯形速度 Profile 轨迹合成 (Phase 2.1)

离散像素路径 → 三次样条平滑 → 梯形速度 Profile → 2D 轨迹点。
z 始终为 0（方向一的 2D 约束）。
"""

from __future__ import annotations

import math
from typing import Dict, List, Tuple

from scipy.interpolate import CubicSpline


def _cubic_spline_2d(
    world_pts: List[Tuple[float, float]],
    n_samples: int,
) -> Tuple[List[float], List[float], List[float]]:
    """纯 2D 三次样条插值 (x,y)。使用 scipy CubicSpline bc_type='natural'。"""
    n = len(world_pts)
    if n < 2:
        return (
            [world_pts[0][0]] * n_samples if n else [],
            [world_pts[0][1]] * n_samples if n else [],
            [0.0] * n_samples,
        )

    xs_raw = [p[0] for p in world_pts]
    ys_raw = [p[1] for p in world_pts]

    chords = [0.0]
    for i in range(1, n):
        chords.append(chords[-1] + math.hypot(
            xs_raw[i] - xs_raw[i - 1], ys_raw[i] - ys_raw[i - 1],
        ))

    total = chords[-1]
    if total <= 0:
        return xs_raw, ys_raw, [0.0] * n

    cs_x = CubicSpline(chords, xs_raw, bc_type="natural")
    cs_y = CubicSpline(chords, ys_raw, bc_type="natural")

    s_vals = [i / max(n_samples - 1, 1) * total for i in range(n_samples)]
    return (
        [float(cs_x(s)) for s in s_vals],
        [float(cs_y(s)) for s in s_vals],
        s_vals,
    )


def trapezoidal_velocity_profile(
    total_distance_m: float,
    total_time_s: float,
    v_max: float = 1.0,
    a_accel: float = 0.5,
    n_points: int = 50,
) -> List[float]:
    """
    三段式 Profile：加速 → 匀速（可选）→ 减速。
    距离太短时退化为三角形 Profile。
    返回: [s0, s1, ..., sN] 归一化弧长参数 [0, 1]
    """
    if total_distance_m <= 0 or total_time_s <= 0:
        return [0.0] * n_points

    t_accel = v_max / a_accel
    d_accel = 0.5 * a_accel * t_accel ** 2

    if 2 * d_accel >= total_distance_m:
        t_accel = math.sqrt(total_distance_m / a_accel)
        t_const = 0.0
        v_peak = a_accel * t_accel
    else:
        t_const = (total_distance_m - 2 * d_accel) / v_max
        v_peak = v_max

    t_total = 2 * t_accel + t_const
    if t_total <= 0:
        return [0.0] * n_points

    samples = [i / max(n_points - 1, 1) * t_total for i in range(n_points)]

    def s_at_t(t: float) -> float:
        if t <= t_accel:
            return 0.5 * a_accel * t ** 2
        elif t <= t_accel + t_const:
            return d_accel + v_peak * (t - t_accel)
        else:
            t_dec = t - t_accel - t_const
            return total_distance_m - 0.5 * a_accel * (t_accel - t_dec) ** 2

    return [s_at_t(t) / total_distance_m for t in samples]


def smooth_path_to_trajectory(
    path_rc: List[Tuple[int, int]],
    resolution: float,
    origin_xy: Tuple[float, float],
    total_time_s: float = 5.0,
    v_max: float = 1.0,
    a_accel: float = 0.5,
    sample_dt: float = 0.05,
) -> List[Dict[str, float]]:
    """
    离散像素路径 → 三次样条 → 梯形速度 Profile → 2D 轨迹点。
    z 始终为 0（方向一的 2D 约束）。
    返回: [{t, x, y, z=0, roll=0, pitch=0, yaw=0}, ...]
    """
    if not path_rc or len(path_rc) < 2:
        return []

    ox, oy = origin_xy

    world_pts: List[Tuple[float, float]] = [
        (ox + c * resolution, oy + r * resolution)
        for (r, c) in path_rc
    ]

    # Step 1: Cubic spline smoothing → dense interpolated path
    n_spline = max(500, len(path_rc) * 4)
    x_smooth, y_smooth, s_fine = _cubic_spline_2d(world_pts, n_samples=n_spline)
    if not s_fine:
        return []

    total_dist = s_fine[-1]

    # Step 2: Trapezoidal velocity profile → normalized arc-length parameters
    n_profile = max(50, int(total_time_s / sample_dt))
    s_params = trapezoidal_velocity_profile(
        total_dist, total_time_s,
        v_max=v_max, a_accel=a_accel,
        n_points=n_profile,
    )

    # Step 3: Map profile to spline → trajectory points
    trajectory: List[Dict[str, float]] = []
    for i, s_norm in enumerate(s_params):
        s_target = s_norm * total_dist
        idx = min(range(len(s_fine)), key=lambda j: abs(s_fine[j] - s_target))
        t = i * sample_dt
        trajectory.append({
            "t": round(t, 3),
            "x": round(x_smooth[idx], 4),
            "y": round(y_smooth[idx], 4),
            "z": 0.0,
            "roll": 0.0,
            "pitch": 0.0,
            "yaw": 0.0,
        })

    return trajectory
