"""
test_z_axis.py — Z-axis hard assertion unit tests (Phase 1.1)

验证 spatial_api_stub._eval_distance_via_field 的 Z 轴检测逻辑：
  - 含 Z 变化的 3D 轨迹 → robustness = -inf, source = z_axis_not_supported
  - 纯 2D 轨迹 → 正常执行距离场采样
"""

import sys
from pathlib import Path

_PROJECT_ROOT = Path(__file__).resolve().parent.parent
if str(_PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(_PROJECT_ROOT))

import numpy as np
import pytest

from smdsl_demo.spatial_api_stub import _eval_distance_via_field, _world_to_grid, _bilinear_sample


def _make_trajectory_2d(n_pts: int = 20):
    """纯 2D 轨迹：沿 x 轴移动，z 恒为 0。"""
    return [
        {"t": float(i), "x": float(i) * 0.5, "y": 0.0, "z": 0.0}
        for i in range(n_pts)
    ]


def _make_trajectory_3d(n_pts: int = 20, z_max: float = 0.5):
    """含 Z 变化的 3D 轨迹。"""
    return [
        {"t": float(i), "x": float(i) * 0.5, "y": 0.0, "z": float(i) / n_pts * z_max}
        for i in range(n_pts)
    ]


def _make_minimal_bundle():
    """构造最小可用 topology_bundle 供距离场采样使用。"""
    H, W = 100, 100
    df = np.ones((H, W), dtype=np.float32) * 2.0  # 全部自由空间，距离=2.0
    tx = {"origin": (0.0, 0.0), "resolution": 0.05, "shape": (H, W)}
    return {
        "distance_field": df,
        "grid_transform": tx,
        "robot_radius": 0.25,
    }


def _make_empty_report():
    return {
        "robustness": 0.0,
        "violated": False,
        "source": "distance_field",
        "details": [],
        "violation_nodes": [],
    }


class TestZAxisRejection:
    """Z 轴硬断言测试集"""

    def test_z_axis_rejected_with_minus_inf(self):
        """输入含 Z 变化的轨迹 → robustness = -inf"""
        trajectory = _make_trajectory_3d(n_pts=10, z_max=0.5)
        bundle = _make_minimal_bundle()
        report = _make_empty_report()

        result = _eval_distance_via_field(
            trajectory=trajectory,
            min_dist_m=0.3,
            bundle=bundle,
            report=report,
        )

        assert result["robustness"] == float("-inf"), (
            f"Expected robustness=-inf for 3D trajectory, got {result['robustness']}"
        )
        assert result["violated"] is True
        assert result["source"] == "z_axis_not_supported"

    def test_z_axis_uniform_but_nonzero(self):
        """所有 z=0.5（均匀非零）→ z_range=0 → 不触发断言"""
        trajectory = [
            {"t": float(i), "x": float(i) * 0.5, "y": 0.0, "z": 0.5}
            for i in range(10)
        ]
        bundle = _make_minimal_bundle()
        report = _make_empty_report()

        result = _eval_distance_via_field(
            trajectory=trajectory,
            min_dist_m=0.3,
            bundle=bundle,
            report=report,
        )

        assert result["robustness"] != float("-inf")
        assert result["source"] != "z_axis_not_supported"

    def test_z_axis_tiny_variation_not_triggered(self):
        """z_range = 0.005 < 0.01 → 不触发断言"""
        trajectory = [
            {"t": float(i), "x": float(i) * 0.5, "y": 0.0, "z": 0.005 * i / 10.0}
            for i in range(10)
        ]
        bundle = _make_minimal_bundle()
        report = _make_empty_report()

        result = _eval_distance_via_field(
            trajectory=trajectory,
            min_dist_m=0.3,
            bundle=bundle,
            report=report,
        )

        assert result["source"] != "z_axis_not_supported"

    def test_pure_2d_trajectory_executes_normally(self):
        """纯 2D 轨迹 → 正常执行采样，返回 details 节点"""
        trajectory = _make_trajectory_2d(n_pts=10)
        bundle = _make_minimal_bundle()
        report = _make_empty_report()

        result = _eval_distance_via_field(
            trajectory=trajectory,
            min_dist_m=0.3,
            bundle=bundle,
            report=report,
        )

        assert result["source"] == "distance_field"
        assert len(result["details"]) >= 10  # 每个轨迹点至少一个 detail

    def test_empty_trajectory(self):
        """空轨迹 → z_range=0 → 不触发断言，但采样结果为空"""
        bundle = _make_minimal_bundle()
        report = _make_empty_report()

        result = _eval_distance_via_field(
            trajectory=[],
            min_dist_m=0.3,
            bundle=bundle,
            report=report,
        )

        assert result["robustness"] != float("-inf")
