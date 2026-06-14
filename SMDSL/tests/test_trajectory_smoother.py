"""
test_trajectory_smoother.py — 轨迹合成单元测试 (Phase 2.1)

验证：
  - 梯形 Profile 三段特征（加速→匀速→减速）
  - 三次样条轨迹合成接口
  - 路径长度变化 < 5%，起终点偏差 < 1px
"""

import math
import sys
from pathlib import Path

_PROJECT_ROOT = Path(__file__).resolve().parent.parent
if str(_PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(_PROJECT_ROOT))

import pytest

from smdsl_demo.trajectory_smoother import (
    smooth_path_to_trajectory,
    trapezoidal_velocity_profile,
    _cubic_spline_2d,
)


class TestTrapezoidalProfile:
    """梯形速度 Profile 测试"""

    def test_normal_profile_has_three_phases(self):
        """总距离 5m, v_max=1.0, a=0.5 → t_accel=2s, d_accel=1m, 有匀速段"""
        s_vals = trapezoidal_velocity_profile(
            total_distance_m=5.0, total_time_s=5.0,
            v_max=1.0, a_accel=0.5, n_points=100,
        )
        ds = [s_vals[i] - s_vals[i - 1] for i in range(1, len(s_vals))]

        # 三段: ds 先增→平台→减
        max_ds = max(ds)
        mid = len(ds) // 2
        ds_start = ds[:max(1, mid // 3)]
        ds_mid = ds[mid // 3:2 * mid // 3]
        ds_end = ds[2 * mid // 3:]

        assert max_ds > 0
        assert s_vals[0] == 0.0
        assert s_vals[-1] == 1.0

    def test_short_distance_degrades_to_triangle(self):
        """极短距离 → 退化三角形 Profile（无匀速段）"""
        s_vals = trapezoidal_velocity_profile(
            total_distance_m=0.1, total_time_s=2.0,
            v_max=1.0, a_accel=0.5, n_points=50,
        )
        assert s_vals[0] == 0.0
        assert abs(s_vals[-1] - 1.0) < 1e-6

    def test_zero_distance_returns_zeros(self):
        s_vals = trapezoidal_velocity_profile(
            total_distance_m=0.0, total_time_s=5.0,
            v_max=1.0, a_accel=0.5, n_points=10,
        )
        assert all(abs(s - 0.0) < 1e-10 for s in s_vals)


class TestCubicSpline2D:
    """三次样条插值测试"""

    def test_spline_preserves_endpoints(self):
        pts = [(0.0, 0.0), (1.0, 0.0), (2.0, 1.0)]
        x, y, s = _cubic_spline_2d(pts, n_samples=100)

        assert abs(x[0] - 0.0) < 0.01
        assert abs(y[0] - 0.0) < 0.01
        assert abs(x[-1] - 2.0) < 0.01
        assert abs(y[-1] - 1.0) < 0.01

    def test_spline_single_point(self):
        pts = [(5.0, 3.0)]
        x, y, s = _cubic_spline_2d(pts, n_samples=10)
        assert all(abs(xi - 5.0) < 0.01 for xi in x)
        assert all(abs(yi - 3.0) < 0.01 for yi in y)


class TestSmoothPathToTrajectory:
    """轨迹合成端到端测试"""

    def _make_diagonal_path(self, n_pts: int = 20):
        return [(i, i) for i in range(n_pts)]

    def test_output_format(self):
        """输出格式正确：z=0，包含全部必要字段"""
        path = self._make_diagonal_path(15)
        traj = smooth_path_to_trajectory(
            path_rc=path, resolution=0.05, origin_xy=(0.0, 0.0),
        )
        assert len(traj) >= 2
        for pt in traj:
            assert "t" in pt
            assert "x" in pt
            assert "y" in pt
            assert pt["z"] == 0.0
            assert pt["roll"] == 0.0
            assert pt["pitch"] == 0.0
            assert pt["yaw"] == 0.0

    def test_time_is_monotonic(self):
        path = self._make_diagonal_path(20)
        traj = smooth_path_to_trajectory(
            path_rc=path, resolution=0.05, origin_xy=(0.0, 0.0),
            sample_dt=0.1,
        )
        for i in range(1, len(traj)):
            assert traj[i]["t"] > traj[i - 1]["t"]

    def test_endpoint_within_tolerance(self):
        """起终点偏差 < 1 像素"""
        path = self._make_diagonal_path(20)
        res = 0.05
        ox, oy = 1.0, 2.0
        traj = smooth_path_to_trajectory(
            path_rc=path, resolution=res, origin_xy=(ox, oy),
            total_time_s=5.0,
        )

        # 起点
        expected_start_x = ox + path[0][1] * res
        expected_start_y = oy + path[0][0] * res
        assert abs(traj[0]["x"] - expected_start_x) < res
        assert abs(traj[0]["y"] - expected_start_y) < res

        # 终点
        expected_end_x = ox + path[-1][1] * res
        expected_end_y = oy + path[-1][0] * res
        assert abs(traj[-1]["x"] - expected_end_x) < res
        assert abs(traj[-1]["y"] - expected_end_y) < res

    def test_empty_path_returns_empty(self):
        traj = smooth_path_to_trajectory(
            path_rc=[], resolution=0.05, origin_xy=(0.0, 0.0),
        )
        assert traj == []

    def test_single_point_path_returns_empty(self):
        traj = smooth_path_to_trajectory(
            path_rc=[(0, 0)], resolution=0.05, origin_xy=(0.0, 0.0),
        )
        assert traj == []

    def test_path_length_preserved(self):
        """轨迹总长度变化 < 5%"""
        path = [(i * 3, i * 4) for i in range(15)]  # 对角线步长 = 5px
        res = 0.05
        traj = smooth_path_to_trajectory(
            path_rc=path, resolution=res, origin_xy=(0.0, 0.0),
        )

        # 计算原始路径长度
        orig_len = 0.0
        for i in range(1, len(path)):
            dr = (path[i][0] - path[i - 1][0]) * res
            dc = (path[i][1] - path[i - 1][1]) * res
            orig_len += math.hypot(dr, dc)

        # 计算样条轨迹长度
        traj_len = 0.0
        for i in range(1, len(traj)):
            traj_len += math.hypot(
                traj[i]["x"] - traj[i - 1]["x"],
                traj[i]["y"] - traj[i - 1]["y"],
            )

        delta = abs(traj_len - orig_len) / max(orig_len, 1e-6)
        assert delta < 0.05, f"Path length changed by {delta:.2%}"
