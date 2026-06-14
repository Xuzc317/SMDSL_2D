"""
test_roboir_diff.py — RoboIR Diff 硬校验单元测试 (Phase 1.2 / Phase 4)

验证 intent / target_frame 不变性检查逻辑：
  - LLM 修改 intent → 被拦截（diff reject）
  - LLM 修改 target_frame → 被拦截
  - 仅修改 stl_constraints → 通过
"""

import sys
from pathlib import Path

_PROJECT_ROOT = Path(__file__).resolve().parent.parent
if str(_PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(_PROJECT_ROOT))

import pytest


def check_roboir_diff(original: dict, corrected: dict, errors: list) -> bool:
    """
    提取自 test_closed_loop_recovery.py run_correction_loop 中的 diff 检查逻辑。
    返回 True = 通过（未修改核心字段），False = 被拦截。
    """
    corrected_intent = corrected.get("intent")
    corrected_target = corrected.get("target_frame")
    original_intent = original.get("intent")
    original_target = original.get("target_frame")

    if corrected_intent != original_intent or corrected_target != original_target:
        errors.append(
            f"LLM modified intent ({original_intent}->{corrected_intent}) "
            f"or target_frame ({original_target}->{corrected_target})"
        )
        return False
    return True


class TestRoboIRDiff:
    """RoboIR diff 硬校验测试"""

    def test_intact_roboir_passes(self):
        """仅修改 stl_constraints → diff 检查通过"""
        original = {
            "intent": "navigate",
            "target_frame": "room_A",
            "stl_constraints": [{"type": "distance", "value": 0.3}],
        }
        corrected = {
            "intent": "navigate",
            "target_frame": "room_A",
            "stl_constraints": [{"type": "distance", "value": 0.5}],
        }
        errors = []
        assert check_roboir_diff(original, corrected, errors) is True
        assert len(errors) == 0

    def test_intent_modified_is_rejected(self):
        """LLM 修改 intent → 被拦截"""
        original = {"intent": "navigate", "target_frame": "room_A", "stl_constraints": []}
        corrected = {"intent": "pick", "target_frame": "room_A", "stl_constraints": []}
        errors = []
        assert check_roboir_diff(original, corrected, errors) is False
        assert len(errors) == 1
        assert "navigate->pick" in errors[0]

    def test_target_frame_modified_is_rejected(self):
        """LLM 修改 target_frame → 被拦截"""
        original = {"intent": "navigate", "target_frame": "room_A", "stl_constraints": []}
        corrected = {"intent": "navigate", "target_frame": "room_B", "stl_constraints": []}
        errors = []
        assert check_roboir_diff(original, corrected, errors) is False
        assert len(errors) == 1

    def test_both_modified_is_rejected(self):
        """同时修改 intent 和 target_frame → 被拦截"""
        original = {"intent": "navigate", "target_frame": "room_A", "stl_constraints": []}
        corrected = {"intent": "pick", "target_frame": "room_B", "stl_constraints": []}
        errors = []
        assert check_roboir_diff(original, corrected, errors) is False
        assert len(errors) == 1

    def test_none_fields_handled(self):
        """缺失字段 vs None → 正确处理"""
        original = {"intent": "navigate"}
        corrected = {"intent": "navigate", "target_frame": None}
        errors = []
        # original.target_frame=None, corrected.target_frame=None → passes
        assert check_roboir_diff(original, corrected, errors) is True

    def test_missing_fields_unequal(self):
        """original 缺少 target_frame，corrected 有 → 被拦截"""
        original = {"intent": "navigate"}
        corrected = {"intent": "navigate", "target_frame": "room_A"}
        errors = []
        assert check_roboir_diff(original, corrected, errors) is False
