"""
motion_profile.py — Trapezoidal Velocity Profile Generator

Optimize 1 from Direction 1:
Replaces linear interpolation in trajectory synthesis with a physically
plausible trapezoidal velocity profile (accel -> const -> decel).
"""

from __future__ import annotations

import math
from typing import List, Tuple


def trapezoidal_velocity_profile(
    total_distance_m: float,
    total_time_s: float,
    v_max: float = 1.0,
    a_accel: float = 0.5,
    n_points: int = 50,
) -> List[float]:
    if total_distance_m <= 0 or total_time_s <= 0:
        return [0.0] * n_points

    t_accel = v_max / a_accel
    d_accel = 0.5 * a_accel * t_accel ** 2

    if 2 * d_accel >= total_distance_m:
        t_accel = math.sqrt(total_distance_m / a_accel)
        t_const = 0.0
        v_peak = a_accel * t_accel
        d_const = 0.0
    else:
        t_const = (total_distance_m - 2 * d_accel) / v_max
        v_peak = v_max
        d_const = v_peak * t_const

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


def cubic_spline_path(
    waypoints_xy: List[Tuple[float, float]],
    n_samples: int = 100,
) -> Tuple[List[float], List[float], List[float]]:
    n = len(waypoints_xy)
    if n < 2:
        if n == 1:
            return [waypoints_xy[0][0]] * n_samples, [waypoints_xy[0][1]] * n_samples, [0.0] * n_samples
        return [], [], []

    xs_raw = [p[0] for p in waypoints_xy]
    ys_raw = [p[1] for p in waypoints_xy]

    chord_lengths = [0.0]
    for i in range(1, n):
        dx = xs_raw[i] - xs_raw[i - 1]
        dy = ys_raw[i] - ys_raw[i - 1]
        chord_lengths.append(chord_lengths[-1] + math.hypot(dx, dy))

    total = chord_lengths[-1]
    if total <= 0:
        return xs_raw, ys_raw, [0.0] * n

    from scipy.interpolate import CubicSpline

    cs_x = CubicSpline(chord_lengths, xs_raw, bc_type="natural")
    cs_y = CubicSpline(chord_lengths, ys_raw, bc_type="natural")

    s_samples = [i / max(n_samples - 1, 1) * total for i in range(n_samples)]
    x_out = [float(cs_x(s)) for s in s_samples]
    y_out = [float(cs_y(s)) for s in s_samples]

    return x_out, y_out, s_samples


def synthesize_trajectory_with_profile(
    waypoints_xy: List[Tuple[float, float]],
    total_time_s: float = 5.0,
    v_max: float = 1.0,
    a_accel: float = 0.5,
    sample_dt: float = 0.05,
    fixed_z: float = 0.0,
) -> List[dict]:
    x_smooth, y_smooth, arc_params = cubic_spline_path(waypoints_xy)
    total_dist = arc_params[-1] if arc_params else 0.0

    n_profile = max(50, int(total_time_s / sample_dt))
    s_params = trapezoidal_velocity_profile(
        total_dist, total_time_s,
        v_max=v_max, a_accel=a_accel,
        n_points=n_profile,
    )

    trajectory = []
    fine_n = max(500, n_profile)
    x_fine, y_fine, s_fine = cubic_spline_path(waypoints_xy, n_samples=fine_n)
    if not s_fine:
        return []

    for i, s_norm in enumerate(s_params):
        s_target = s_norm * total_dist
        idx = min(range(len(s_fine)), key=lambda j: abs(s_fine[j] - s_target))
        t = i * sample_dt
        trajectory.append({
            "t": round(t, 3),
            "x": round(x_fine[idx], 4),
            "y": round(y_fine[idx], 4),
            "z": fixed_z,
            "roll": 0.0,
            "pitch": 0.0,
            "yaw": 0.0,
        })

    return trajectory
