"""
rust_compiler_stub.py — Zone 2 语义编译: 静态类型检查 + Frame 坐标系校验

职责：
  - 验证 RoboIR 的语法、类型、坐标系一致性。
  - 拦截"异构坐标系直接相加"（Frame Mismatch）等语义错误。
  - 纯静态分析，不涉及任何数学求解（数学求解归 Zone 3）。
"""

from __future__ import annotations

import json
from typing import Tuple, List, Optional, Set
from enum import Enum

# Schema 常量必须从 vlm_parser 单一来源 import，禁止在此重复定义。
# 双重导入兜底（同时支持 ``from smdsl_demo.X`` 与 ``from X``）。
try:
    from .vlm_parser import VALID_FRAMES, VALID_GRASP_TYPES, VALID_STL_OPS
except ImportError:
    from vlm_parser import VALID_FRAMES, VALID_GRASP_TYPES, VALID_STL_OPS  # type: ignore


class ValidationErrorType(str, Enum):
    """静态检查错误类型枚举。"""
    JSON_PARSE_ERROR = "json_parse_error"
    MISSING_FIELD = "missing_field"
    UNKNOWN_FRAME = "unknown_frame"
    FRAME_MISMATCH = "frame_mismatch"          # 异构坐标系直接相加
    INVALID_POSE_REF = "invalid_pose_reference"  # 引用的 Pose 未声明
    INVALID_GRASP_TYPE = "invalid_grasp_type"
    STL_SYNTAX_ERROR = "stl_syntax_error"       # STL 约束表达式语法错误
    CONSTRAINT_WEIGHT_INVALID = "constraint_weight_invalid"


class ValidationError:
    """结构化的验证错误。"""

    def __init__(self, error_type: ValidationErrorType, message: str,
                 location: Optional[str] = None):
        self.error_type = error_type
        self.message = message
        self.location = location

    def __repr__(self) -> str:
        loc = f" @{self.location}" if self.location else ""
        return f"[{self.error_type.value}]{loc} {self.message}"

    def to_dict(self) -> dict:
        return {
            "type": self.error_type.value,
            "message": self.message,
            "location": self.location,
        }


class RustCompilerStub:
    """语义编译器 —— 静态类型检查 + Frame 坐标系一致性校验。"""

    # 预定义合法坐标系（从 vlm_parser.VALID_FRAMES 直接继承，避免重复定义）
    VALID_FRAMES: Set[str] = set(VALID_FRAMES)

    # Frame 层级关系（parent frame），用于检查不同层级 frame 是否可互操作
    FRAME_HIERARCHY: dict = {
        "world": None,
        "map": "world",
        "base_link": "map",
        "object_frame": "map",
        "camera_color_optical_frame": "base_link",
        "tool0": "base_link",
    }

    # Frame → 所属的"坐标系族" —— 用于检测 Frame Mismatch
    FRAME_FAMILY: dict = {
        "world": "world",
        "map": "world",            # map 和 world 同族
        "base_link": "robot_body",
        "tool0": "robot_end",      # 末端执行器特殊
        "camera_color_optical_frame": "sensor",
        "object_frame": "object",
    }

    def __init__(self, compiler_path: Optional[str] = None):
        self.compiler_path = compiler_path

    # ──────────────────────────────────────────────
    # 主入口
    # ──────────────────────────────────────────────

    def validate(self, robo_ir_str: str) -> Tuple[bool, List[ValidationError]]:
        """
        对 RoboIR 执行完整的静态类型检查。

        检查项：
        1. JSON 解析
        2. Schema 字段完整性
        3. Frame 坐标系名称合法性
        4. Frame Mismatch — 检测异构坐标系直接运算
        5. Pose 引用完整性
        6. Grasp 类型合法性
        7. STL 约束语法

        Args:
            robo_ir_str: RoboIR JSON 字符串

        Returns:
            (is_valid, error_list)
        """
        errors: List[ValidationError] = []

        # ── 1. JSON 解析 ──
        try:
            ir = json.loads(robo_ir_str)
        except json.JSONDecodeError as e:
            return False, [ValidationError(
                ValidationErrorType.JSON_PARSE_ERROR,
                f"JSON 解析失败: {e}",
            )]

        # ── 2. Schema 字段完整性 ──
        if "version" not in ir:
            errors.append(ValidationError(
                ValidationErrorType.MISSING_FIELD, "缺少 version 字段"))
        if "actions" not in ir:
            errors.append(ValidationError(
                ValidationErrorType.MISSING_FIELD, "缺少 actions 字段"))

        # ── 3. Frame 声明校验 ──
        declared_frames: Set[str] = set()
        for frame_decl in ir.get("frame_declarations", []):
            fid = frame_decl.get("id", "")
            declared_frames.add(fid)
            if fid not in self.VALID_FRAMES:
                errors.append(ValidationError(
                    ValidationErrorType.UNKNOWN_FRAME,
                    f"坐标系 '{fid}' 不在预定义集合中",
                    location=f"frame_declarations.{fid}",
                ))

        # ── 4. Frame Mismatch 检测 ──
        actions = ir.get("actions", [])
        for action in actions:
            self._check_frame_mismatch(action, errors)

        # ── 5. Pose 引用完整性 ──
        declared_poses: Set[str] = {p.get("label", "")
                                     for p in ir.get("pose_declarations", [])}
        for action in actions:
            target_pose = action.get("target_pose", "")
            if target_pose and target_pose not in declared_poses:
                errors.append(ValidationError(
                    ValidationErrorType.INVALID_POSE_REF,
                    f"action {action.get('id')} 引用了未声明的 Pose: '{target_pose}'",
                ))

        # ── 6. Grasp 类型校验（使用 schema 单一来源）──
        valid_grasp_types = VALID_GRASP_TYPES
        for decl in ir.get("grasp_declarations", []):
            gtype = decl.get("grasp_type", decl.get("type", ""))
            if gtype == "Grasp":
                continue  # 顶层 type 声明
        for action in actions:
            if action.get("type") == "grasp":
                gt = action.get("grasp_type", "")
                if gt and gt not in valid_grasp_types:
                    errors.append(ValidationError(
                        ValidationErrorType.INVALID_GRASP_TYPE,
                        f"未知抓取类型 '{gt}'",
                        location=f"action[{action.get('id')}].grasp_type",
                    ))

        # ── 7. STL 约束语法 ──
        for action in actions:
            for constraint in action.get("constraints", []):
                self._check_stl_syntax(constraint, errors,
                                       f"action[{action.get('id')}]")

        return (len(errors) == 0, errors)

    # ──────────────────────────────────────────────
    # Frame Mismatch 检测逻辑
    # ──────────────────────────────────────────────

    def _check_frame_mismatch(self, action: dict,
                               errors: List[ValidationError]) -> None:
        """
        检测 action 中是否存在异构坐标系直接相加的错误。

        规则：
        - 同一 Frame Family 内的坐标系可以互操作（如 map↔world）
        - 不同 Frame Family 的坐标系直接运算视为 Frame Mismatch
        - 例如 tool0 下的位移量直接加到 map 坐标上 → 错误
        """
        target = action.get("target", {})
        target_frame = target.get("frame", "")
        constraints = action.get("constraints", [])

        # 检查约束中是否有跨族系引用
        for constraint in constraints:
            constraint_frame = constraint.get("frame", "")
            if constraint_frame and target_frame:
                fam_target = self.FRAME_FAMILY.get(target_frame)
                fam_constraint = self.FRAME_FAMILY.get(constraint_frame)
                if fam_target and fam_constraint and fam_target != fam_constraint:
                    errors.append(ValidationError(
                        ValidationErrorType.FRAME_MISMATCH,
                        f"异构坐标系运算: action 目标位于 '{target_frame}'"
                        f"({fam_target}), 但约束参考了 '{constraint_frame}'"
                        f"({fam_constraint})",
                        location=f"action[{action.get('id')}]",
                    ))

        # 检查 pose 声明之间的坐标系一致性
        # 如果一个 action 中同时引用了多个不同 family 的 pose，标记为警告

    # ──────────────────────────────────────────────
    # STL 约束语法检查
    # ──────────────────────────────────────────────

    def _check_stl_syntax(self, constraint: dict,
                           errors: List[ValidationError],
                           location: str) -> None:
        """检查 STL 约束表达式的语法合法性。"""
        if constraint.get("type") != "stl_constraint":
            return
        expr = constraint.get("expr", "")
        if not expr:
            errors.append(ValidationError(
                ValidationErrorType.STL_SYNTAX_ERROR,
                "STL 约束表达式为空",
                location=location,
            ))
            return

        # 基本语法检查: 必须包含合法的 STL 操作符（schema 单一来源）
        valid_ops = VALID_STL_OPS
        has_operator = any(op in expr for op in valid_ops)
        if not has_operator:
            errors.append(ValidationError(
                ValidationErrorType.STL_SYNTAX_ERROR,
                f"STL 表达式 '{expr}' 未包含合法操作符 "
                f"({', '.join(sorted(valid_ops))})",
                location=location,
            ))

        # 检查数值有效性
        # 简单的 token 提取 -> 检查是否有数值
        import re
        numbers = re.findall(r"[-+]?\d*\.?\d+", expr)
        if not numbers:
            errors.append(ValidationError(
                ValidationErrorType.STL_SYNTAX_ERROR,
                f"STL 表达式 '{expr}' 缺少数值边界",
                location=location,
            ))

    # ──────────────────────────────────────────────
    # 编译（占位）
    # ──────────────────────────────────────────────

    def compile(self, robo_ir_str: str) -> bytes:
        """
        将 RoboIR 编译为底层动作序列（Phase 2 实现）。

        Args:
            robo_ir_str: RoboIR 字符串

        Returns:
            ProtoBuf 二进制动作序列
        """
        raise NotImplementedError("Rust 编译器将在 Phase 2 实现")
