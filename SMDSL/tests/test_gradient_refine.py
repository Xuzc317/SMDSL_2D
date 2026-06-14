"""
test_gradient_refine.py — 梯度路径微调单元测试 (Phase 2.2)

验证：
  - compute_field_gradient 输出维度正确
  - refine_path_via_gradient 保持起终点不变
  - 微调后路径不穿墙 (d >= robot_radius_px)
  - 路径点数量不膨胀
"""

import sys
from pathlib import Path

_PROJECT_ROOT = Path(__file__).resolve().parent.parent
if str(_PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(_PROJECT_ROOT))

import numpy as np
import pytest

from cad_parser.astar_topology import (
    compute_field_gradient,
    refine_path_via_gradient,
    compute_distance_field,
)


class TestFieldGradient:
    """距离场梯度计算测试"""

    def test_gradient_shapes_match(self):
        df = np.ones((50, 60), dtype=np.float32) * 5.0
        df[10:20, 10:20] = 0.0  # 障碍物区域
        gx, gy = compute_field_gradient(df)
        assert gx.shape == df.shape
        assert gy.shape == df.shape

    def test_gradient_zero_in_uniform_region(self):
        df = np.ones((30, 30), dtype=np.float32) * 3.0
        gx, gy = compute_field_gradient(df)
        assert np.allclose(gx, 0, atol=1e-6)
        assert np.allclose(gy, 0, atol=1e-6)

    def test_gradient_points_away_from_obstacle(self):
        # 左边障碍物(0), 右边自由空间(10)
        df = np.zeros((20, 40), dtype=np.float32)
        for r in range(20):
            for c in range(40):
                df[r, c] = float(c) * 0.5  # d 从左到右递增
        gx, gy = compute_field_gradient(df)
        # 梯度 ∂d/∂x (gx) 应 > 0（远离障碍物 = 向右）
        mid_row = gx[10, :]
        assert np.mean(mid_row) > 0


class TestRefinePathViaGradient:
    """路径梯度微调测试"""

    def _make_corridor_grid(self, H=50, W=50, corridor_width=20):
        """创建简单走廊：中间宽自由空间，两边障碍物"""
        grid = np.zeros((H, W), dtype=np.uint8)
        lo = (W - corridor_width) // 2
        hi = lo + corridor_width
        grid[:, lo:hi] = 1
        return grid

    def test_start_end_preserved(self):
        """起终点坐标不变"""
        grid = self._make_corridor_grid(50, 50, 20)
        df = compute_distance_field(grid)
        gx, gy = compute_field_gradient(df)

        path = [(25, 15), (25, 20), (25, 25), (25, 30), (25, 35)]
        refined = refine_path_via_gradient(
            path_rc=path, distance_field=df,
            gradient_field=(gx, gy), robot_radius_px=2.0,
        )

        assert refined[0] == path[0]
        assert refined[-1] == path[-1]

    def test_refined_path_stays_in_free_space(self):
        """微调后路径点不穿墙 (d >= robot_radius_px)"""
        grid = self._make_corridor_grid(50, 50, 20)
        df = compute_distance_field(grid)
        gx, gy = compute_field_gradient(df)

        path = [(25, 20), (25, 25), (25, 30)]
        refined = refine_path_via_gradient(
            path_rc=path, distance_field=df,
            gradient_field=(gx, gy), robot_radius_px=1.0,
            iterations=3, step_size=0.5,
        )

        for r, c in refined:
            rc = int(round(r))
            cc = int(round(c))
            rc = max(0, min(df.shape[0] - 1, rc))
            cc = max(0, min(df.shape[1] - 1, cc))
            assert df[rc, cc] >= 1.0, (
                f"Point ({r},{c}) has d={df[rc,cc]:.2f} < 1.0"
            )

    def test_path_length_preserved(self):
        """路径点数不膨胀"""
        grid = self._make_corridor_grid(50, 50, 20)
        df = compute_distance_field(grid)
        gx, gy = compute_field_gradient(df)

        n_pts = 10
        path = [(25, 10 + i * 3) for i in range(n_pts)]
        refined = refine_path_via_gradient(
            path_rc=path, distance_field=df,
            gradient_field=(gx, gy), robot_radius_px=2.0,
            iterations=5,
        )

        assert len(refined) == len(path)

    def test_short_path_unchanged(self):
        """路径 < 3 点 → 直接返回（不微调）"""
        grid = self._make_corridor_grid(30, 30, 15)
        df = compute_distance_field(grid)
        gx, gy = compute_field_gradient(df)

        path = [(15, 10), (15, 20)]
        refined = refine_path_via_gradient(
            path_rc=path, distance_field=df,
            gradient_field=(gx, gy), robot_radius_px=2.0,
        )
        assert refined == path
