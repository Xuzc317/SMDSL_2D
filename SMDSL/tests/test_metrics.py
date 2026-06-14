"""
test_metrics.py — metrics 模块单元测试 (Phase Next)

测试 metrics.py 中可独立测试的计算函数。
"""

import math
import sys
from pathlib import Path

_PROJECT_ROOT = Path(__file__).resolve().parent.parent
if str(_PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(_PROJECT_ROOT))

import pytest

from smdsl_demo.metrics import (
    FailureTaxonomy,
    stl_robustness,
    path_efficiency,
    parsing_accuracy,
    precision_recall,
    end_to_end_latency,
    generate_structured_feedback,
)


def _make_stub_trajectory(n: int = 10) -> list:
    return [
        {"t": float(i), "x": float(i) * 0.5, "y": 0.0, "z": 0.0}
        for i in range(n)
    ]


def _make_safe_constraint(dist_m: float = 0.5) -> dict:
    return {"type": "distance", "op": ">", "value": dist_m}


class TestStlRobustness:
    """STL 鲁棒度计算测试"""

    def test_empty_trajectory(self):
        r = stl_robustness([], [_make_safe_constraint(0.3)])
        assert r == 0.0

    def test_single_constraint_satisfied(self):
        traj = _make_stub_trajectory(5)
        c = _make_safe_constraint(0.05)
        r = stl_robustness(traj, [c])
        assert isinstance(r, float)

    def test_no_constraints(self):
        traj = _make_stub_trajectory(3)
        r = stl_robustness(traj, [])
        # Empty constraints => no violation, returns neutral value
        assert isinstance(r, float)


class TestPathEfficiency:
    """路径效率测试"""

    def test_straight_line_perfect(self):
        traj = [
            {"t": 0.0, "x": 0.0, "y": 0.0},
            {"t": 1.0, "x": 1.0, "y": 0.0},
            {"t": 2.0, "x": 2.0, "y": 0.0},
        ]
        eff = path_efficiency(traj, optimal_cost=2.0)
        assert eff == pytest.approx(1.0)

    def test_detour_penalty(self):
        traj = [
            {"t": 0.0, "x": 0.0, "y": 0.0},
            {"t": 1.0, "x": 1.0, "y": 1.0},
            {"t": 2.0, "x": 2.0, "y": 0.0},
        ]
        actual = math.hypot(1, 1) + math.hypot(1, -1)
        eff = path_efficiency(traj, optimal_cost=2.0)
        assert eff == pytest.approx(2.0 / actual)

    def test_empty_trajectory(self):
        eff = path_efficiency([], optimal_cost=1.0)
        assert eff == 0.0

    def test_zero_optimal_cost(self):
        traj = _make_stub_trajectory(2)
        eff = path_efficiency(traj, optimal_cost=0.0)
        assert isinstance(eff, float)


class TestParsingAccuracy:
    """RoboIR 解析准确率测试"""

    def test_valid_json(self):
        roboir_str = '{"intent":"navigate","target_frame":"room_A"}'
        acc = parsing_accuracy(roboir_str)
        assert isinstance(acc, float)

    def test_invalid_json(self):
        acc = parsing_accuracy("not valid json")
        assert isinstance(acc, float)

    def test_empty_string(self):
        acc = parsing_accuracy("")
        assert isinstance(acc, float)


class TestPrecisionRecall:
    """精度/召回率测试"""

    def test_identical_dicts(self):
        gt = {"intent": "navigate", "target_frame": "room_A"}
        gen = {"intent": "navigate", "target_frame": "room_A"}
        score = precision_recall(gt, gen)
        assert score > 0.5

    def test_different_dicts(self):
        gt = {"intent": "navigate", "target_frame": "room_A"}
        gen = {"intent": "pick", "target_frame": "room_B"}
        score = precision_recall(gt, gen)
        assert isinstance(score, float)

    def test_empty_dicts(self):
        score = precision_recall({}, {})
        assert isinstance(score, float)


class TestEndToEndLatency:
    """端到端延迟测试"""

    def test_simple_latency(self):
        lat = end_to_end_latency(0.0, 5.0, 5.0, 5.1)
        assert lat == pytest.approx(5.1)

    def test_llm_dominant_latency(self):
        lat = end_to_end_latency(0.0, 4.5, 0.0, 0.0)
        assert lat == pytest.approx(4.5)


class TestGenerateStructuredFeedback:
    """结构化反馈测试"""

    def test_distance_violation_feedback(self):
        fb = generate_structured_feedback(
            error_type=FailureTaxonomy.STL_VIOLATION,
            robustness_score=-0.15,
            context={"n_violation_nodes": 3},
        )
        assert fb is not None
        assert "error" in fb

    def test_collision_feedback(self):
        fb = generate_structured_feedback(
            error_type=FailureTaxonomy.COLLISION_WALL,
            robustness_score=-0.5,
        )
        assert fb is not None
