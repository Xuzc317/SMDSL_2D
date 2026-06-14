"""
test_rust_compiler.py — Rust 编译器存根单元测试 (Phase Next)

测试验证逻辑 (validate)。compile 方法暂未实现。
"""

import json
import sys
from pathlib import Path

_PROJECT_ROOT = Path(__file__).resolve().parent.parent
if str(_PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(_PROJECT_ROOT))

import pytest

from smdsl_demo.rust_compiler_stub import RustCompilerStub, ValidationErrorType


def _valid_roboir_str() -> str:
    """符合 RustCompilerStub 预期 schema 的最小合法 RoboIR JSON。"""
    return json.dumps({
        "version": "1.0",
        "actions": [
            {
                "id": "move_to_goal",
                "target_pose": "goal",
                "constraints": [
                    {"type": "stl_constraint", "expr": "Distance > 0.3"},
                    {"type": "stl_constraint", "expr": "Time < 5.0"},
                ],
            }
        ],
        "frame_declarations": [
            {"id": "base_link"},
            {"id": "world"},
        ],
        "pose_declarations": [
            {"label": "goal", "x": 1.0, "y": 2.0, "z": 0.0},
        ],
    })


class TestRustCompilerValidate:
    """RustCompilerStub.validate 测试"""

    def test_valid_roboir_passes(self):
        compiler = RustCompilerStub()
        ok, errors = compiler.validate(_valid_roboir_str())
        assert ok is True, f"Errors: {[(e.error_type.value, e.message) for e in errors]}"
        assert errors == []

    def test_invalid_json(self):
        compiler = RustCompilerStub()
        ok, errors = compiler.validate("not valid json {{{")
        assert ok is False
        assert errors[0].error_type == ValidationErrorType.JSON_PARSE_ERROR

    def test_missing_version(self):
        compiler = RustCompilerStub()
        roboir = {
            "actions": [],
        }
        ok, errors = compiler.validate(json.dumps(roboir))
        assert ok is False
        assert any(e.error_type == ValidationErrorType.MISSING_FIELD for e in errors)

    def test_missing_actions(self):
        compiler = RustCompilerStub()
        roboir = {"version": "1.0"}
        ok, errors = compiler.validate(json.dumps(roboir))
        assert ok is False
        assert any(e.error_type == ValidationErrorType.MISSING_FIELD for e in errors)

    def test_unknown_frame(self):
        compiler = RustCompilerStub()
        roboir = {
            "version": "1.0",
            "actions": [],
            "frame_declarations": [
                {"id": "nonexistent_frame_xyz"},
            ],
        }
        ok, errors = compiler.validate(json.dumps(roboir))
        assert ok is False
        assert any(e.error_type == ValidationErrorType.UNKNOWN_FRAME for e in errors)

    def test_empty_stl_expression(self):
        """空 STL 表达式 → STL_SYNTAX_ERROR"""
        compiler = RustCompilerStub()
        roboir = {
            "version": "1.0",
            "actions": [
                {
                    "id": "act1",
                    "constraints": [
                        {"type": "stl_constraint", "expr": ""},
                    ],
                }
            ],
        }
        ok, errors = compiler.validate(json.dumps(roboir))
        assert ok is False
        assert any(e.error_type == ValidationErrorType.STL_SYNTAX_ERROR for e in errors)

    def test_invalid_stl_operator(self):
        """STL 表达式不含合法操作符 → STL_SYNTAX_ERROR"""
        compiler = RustCompilerStub()
        roboir = {
            "version": "1.0",
            "actions": [
                {
                    "id": "act1",
                    "constraints": [
                        {"type": "stl_constraint", "expr": "some garbage without valid op"},
                    ],
                }
            ],
        }
        ok, errors = compiler.validate(json.dumps(roboir))
        assert ok is False
        assert any(e.error_type == ValidationErrorType.STL_SYNTAX_ERROR for e in errors)

    def test_valid_stl_expression_passes(self):
        """合法 STL 表达式 'Distance > 0.3' → 通过"""
        compiler = RustCompilerStub()
        roboir = {
            "version": "1.0",
            "actions": [
                {
                    "id": "act1",
                    "constraints": [
                        {"type": "stl_constraint", "expr": "Distance > 0.3"},
                    ],
                }
            ],
        }
        ok, errors = compiler.validate(json.dumps(roboir))
        assert ok is True
