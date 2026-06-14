"""
vlm_parser.py — Zone 1 → Zone 2: 自然语言 → RoboIR 编译器
                                  (DeepSeek-V4-Flash, OpenAI 兼容 SDK)

设计要点（与 PROJECT_CONTEXT 的"四大执行区"严格对齐）：

  - LLM 仅充当 *翻译官* —— 把自然语言翻译为声明式 STL 约束 + 业务逻辑。
  - System Prompt 中以【绝对禁止】护栏阻止 LLM 计算坐标、距离、角度。
  - Few-Shot 示例锁死输出结构，压制幻觉。
  - 网络/解析异常自动重试（默认 3 次，指数回退）。
  - API Key 必须从环境变量 DEEPSEEK_API_KEY 读取，禁止硬编码。

下游消费方：
  - app.py Tab 2：直接调用 ``parse_instruction_to_roboir``。
  - app.py Tab 3：通过 ``normalize_stl_constraints`` 把 LLM 紧凑输出
    (``[{"expr": "Distance > 0.10", "ref": "obstacle"}]``)
    转为 ``spatial_api_stub.check_stl_constraint_violation`` 期望的形式
    (``[{"type": "stl_constraint", "expr": "...", "unit": "m", "reference": "..."}]``)。
"""

from __future__ import annotations

import json
import os
import time
from typing import Any, Dict, List, Optional, Tuple

# OpenAI Python SDK >= 1.0（DeepSeek 完全兼容）
try:
    from openai import OpenAI
    from openai import (
        APIConnectionError,
        APIError,
        APIStatusError,
        APITimeoutError,
        RateLimitError,
    )
    _OPENAI_AVAILABLE = True
    _OPENAI_RETRIABLE: Tuple[type, ...] = (
        APIError, APIConnectionError, APITimeoutError,
        RateLimitError, APIStatusError,
    )
except ImportError:  # 软依赖：未装 openai 时本模块仍可被导入用于 schema 常量
    OpenAI = None  # type: ignore
    _OPENAI_AVAILABLE = False
    _OPENAI_RETRIABLE = (Exception,)


# ══════════════════════════════════════════════════════════════════════
# RoboIR Schema 常量 — Single Source of Truth
# rust_compiler_stub.py / metrics.py / app.py 全部从此处导入。
# ══════════════════════════════════════════════════════════════════════

VALID_FRAMES: frozenset[str] = frozenset({
    "base_link", "tool0", "world",
    "camera_color_optical_frame", "map", "object_frame",
})
VALID_GRASP_TYPES: frozenset[str] = frozenset({
    "pinch",             # 二指夹爪
    "suction",           # 吸盘
    "magnetic_gripper",  # 电磁夹爪
    "none",              # 仅移动 / 不抓取
})
VALID_STL_OPS: frozenset[str] = frozenset({
    "Time", "Distance",
    "Orientation", "OrientationDiff",
    "Velocity", "Force", "Torque",
})


# ══════════════════════════════════════════════════════════════════════
# DeepSeek 端点配置
# ══════════════════════════════════════════════════════════════════════

DEEPSEEK_BASE_URL = "https://api.deepseek.com"
DEEPSEEK_DEFAULT_MODEL = "deepseek-chat"   # DeepSeek 官方推荐生产别名
DEEPSEEK_API_KEY_ENV = "DEEPSEEK_API_KEY"
DEEPSEEK_API_KEY_ENV_FALLBACK = "DEEPSEEK_KEY"


# ══════════════════════════════════════════════════════════════════════
# System Prompt + Few-Shot
# ══════════════════════════════════════════════════════════════════════

ROBOIR_SYSTEM_PROMPT = """\
你是一个高级机器人任务规划编译器。你的任务是把自然语言指令翻译为 RoboIR JSON
——一种基于 Signal Temporal Logic (STL) / TAMP 的**声明式时空约束语言**。

══════════════════════════════════════════════════════════════════════
【核心原则：声明式 vs 命令式】

你不是在生成"动作脚本"。你描述的是"什么必须为真"，让下游的物理求解器
自己规划具体的轨迹。类比：你写 SQL `WHERE`，不写 `for` 循环。

──── 错误（命令式 / 拟人化） ──────────────────────
    "先移动到 A，再闭合夹爪"   ❌
    "等 5 秒后开始动作"         ❌
    "拿起杯子放到桌子上"        ❌

──── 正确（声明式 / STL） ─────────────────────────
    存在 t* ∈ [0, T] 使得 末端轨迹通过 A           ✅
    ∀t ∈ [0, T]: 距 obstacle ≥ 0.10 m              ✅
    ∀t ∈ [0, T]: orientation(mug) = vertical       ✅
    在 t = T 时 grasp_state = closed               ✅

══════════════════════════════════════════════════════════════════════
【绝对禁止】

1. 不得自己计算 XYZ 坐标、距离米数、角度值（除非用户原话给出）
2. 不得用"先 / 然后 / 接着 / 之后 / 再 / 最后"等时序词描述动作
3. 不得描述"如何做"，只描述"必须满足什么条件"
4. 不得引用 nearest_objects 之外的物体
5. 不得返回 Markdown ```json 代码块包裹

══════════════════════════════════════════════════════════════════════
【环境约束 — 来自 Spatial-RAG 语义画像】

若输入的 local_context 中包含 _global_scene_profile 字段，
说明上游已通过 CAD 拓扑分析 + LLM 推理为你提供了环境语义画像。
你必须严格参考其中的 recommended_global_constraints 来设置约束：

- safety_distance_m: 将所有 Distance > X 的 X 设为此值（除非用户指定了更大值）
- max_velocity_ms: 影响 Time < Z 的 Z 值设定
- requires_precise_grasp: 若为 true，避免使用 "none" 抓取类型
- high_risk_zones: 这些区域必须添加额外的 Distance 约束

══════════════════════════════════════════════════════════════════════
【输出 Schema】

合法 JSON，含且仅含 4 个顶层字段：

{
  "intent":         <snake_case 英文>,                    // 任务意图
  "target_frame":   <string ∈ nearest_objects>,           // 目标坐标系
  "grasp_type":     "pinch" | "suction" | "magnetic_gripper" | "none",
  "stl_constraints": [                                    // STL 约束列表
    {"expr": "Distance > 0.10", "ref": "<obj>"},           // 米
    {"expr": "Orientation == vertical", "ref": "<obj>"},   // 取值: vertical/horizontal/level
    {"expr": "Time < 5.0"}                                 // 秒，无 ref
  ]
}

══════════════════════════════════════════════════════════════════════
【Few-Shot Examples — 工业场景 STL 风格】

示例 1（基础空间导航与避障）：
输入:
  指令："从当前位置移动到下料区，注意避开中间的设备。"
  局部环境: {"nearest_objects": ["unloading_zone", "center_equipment"]}
输出 JSON:
{
  "intent": "navigate_to_unloading_zone",
  "target_frame": "unloading_zone",
  "grasp_type": "none",
  "stl_constraints": [
    {"expr": "Distance > 0.20", "ref": "center_equipment"}
  ]
}
（语义：∀t ∈ [0, T]: dist(robot, center_equipment) > 0.20 m → 全程避让中间设备）

示例 2（高危操作与时空双重约束）：
输入:
  指令："搬运化学物料穿过狭窄走廊，必须保持绝对垂直，且必须在 10 秒内通过。"
  局部环境: {"nearest_objects": ["chemical_material", "narrow_corridor_walls"]}
输出 JSON:
{
  "intent": "transport_hazardous_material",
  "target_frame": "narrow_corridor_exit",
  "grasp_type": "pinch",
  "stl_constraints": [
    {"expr": "Orientation == vertical", "ref": "chemical_material"},
    {"expr": "Distance > 0.05", "ref": "narrow_corridor_walls"},
    {"expr": "Time < 10.0"}
  ]
}
（语义：∀t ∈ [0, 10.0]: orient(chemical_material) = vertical ∧ dist(robot, narrow_corridor_walls) > 0.05 m）

示例 3（不确定性探测与重试反馈）：
输入:
  指令："靠近上料台，如果发现有障碍物挡住（距离小于0.3米），则等待 3 秒后再试。"
  局部环境: {"nearest_objects": ["loading_station", "unknown_obstacle"]}
输出 JSON:
{
  "intent": "approach_loading_station_with_retry",
  "target_frame": "loading_station",
  "grasp_type": "none",
  "stl_constraints": [
    {"expr": "Distance > 0.30", "ref": "unknown_obstacle"}
  ]
}
（语义：∃t* ∈ [0, T]: end_effector at loading_station ∧ ∀t: dist(robot, unknown_obstacle) > 0.30 m。重试逻辑由下游状态机处理。）

══════════════════════════════════════════════════════════════════════
【最后一条】只返回单一 JSON 对象，无 Markdown 包裹，无任何解释文字。
"""


# ══════════════════════════════════════════════════════════════════════
# 异常类
# ══════════════════════════════════════════════════════════════════════

class VlmParserError(RuntimeError):
    """VlmParser 顶层异常 — 网络/解析/校验失败统一抛出。"""


# ══════════════════════════════════════════════════════════════════════
# VlmParser 主类
# ══════════════════════════════════════════════════════════════════════

class VlmParser:
    """自然语言 → RoboIR 编译器（DeepSeek deepseek-chat 后端）。"""

    def __init__(
        self,
        model_name: str = DEEPSEEK_DEFAULT_MODEL,
        api_key: Optional[str] = None,
        base_url: str = DEEPSEEK_BASE_URL,
        max_retries: int = 3,
        retry_backoff_s: float = 1.5,
        request_timeout_s: float = 30.0,
        temperature: float = 0.1,
    ) -> None:
        if not _OPENAI_AVAILABLE:
            raise VlmParserError(
                "openai SDK 未安装。请运行 `pip install openai` 后重试。"
            )
        self.model_name: str = model_name
        self.base_url: str = base_url
        self.max_retries: int = max(1, int(max_retries))
        self.retry_backoff_s: float = float(retry_backoff_s)
        self.request_timeout_s: float = float(request_timeout_s)
        self.temperature: float = float(temperature)

        resolved_key = api_key or os.environ.get(DEEPSEEK_API_KEY_ENV, "") or os.environ.get(DEEPSEEK_API_KEY_ENV_FALLBACK, "")
        if not resolved_key:
            raise VlmParserError(
                f"未提供 DeepSeek API Key。请设置环境变量 "
                f"`{DEEPSEEK_API_KEY_ENV}` 或在构造器中显式传 api_key。"
            )

        self._client = OpenAI(
            api_key=resolved_key,
            base_url=base_url,
            timeout=self.request_timeout_s,
        )

    # ──────────────────────────────────────────────
    # Public API
    # ──────────────────────────────────────────────

    def parse_instruction_to_roboir(
        self,
        instruction: str,
        local_context: Optional[Dict[str, Any]] = None,
    ) -> Dict[str, Any]:
        """
        将自然语言指令编译为 RoboIR JSON dict。

        Args:
            instruction: 自然语言任务指令。
            local_context: 局部环境上下文（AreaGraph 切片），形如
                ``{"nearest_objects": ["green_mug", "center_table"]}``。
                可为 None 或空 dict。

        Returns:
            合法的 RoboIR dict，含字段:
                ``intent`` / ``target_frame`` / ``grasp_type`` / ``stl_constraints``。

        Raises:
            VlmParserError: 输入空、API 多次失败、JSON 解析失败、或字段校验失败。
        """
        if not instruction or not instruction.strip():
            raise VlmParserError("instruction 为空字符串。")

        ctx = local_context if isinstance(local_context, dict) else {}
        user_prompt = self._build_user_prompt(instruction, ctx)

        last_err: Optional[Exception] = None
        for attempt in range(1, self.max_retries + 1):
            try:
                content = self._call_api(user_prompt)
                parsed = self._safe_parse_json(content)
                self._validate_shape(parsed)
                return parsed
            except (json.JSONDecodeError, VlmParserError) as e:
                # JSON 不合法或字段校验失败 — 让模型重试一次（其行为有随机性）
                last_err = e
            except _OPENAI_RETRIABLE as e:
                last_err = e
            except Exception as e:
                # 非预期异常立即抛出，避免重试黑洞
                raise VlmParserError(f"非预期异常: {e}") from e

            if attempt < self.max_retries:
                time.sleep(self.retry_backoff_s * attempt)

        raise VlmParserError(
            f"DeepSeek 调用失败（已重试 {self.max_retries} 次）。"
            f"最后错误: {type(last_err).__name__}: {last_err}"
        )

    def nl_to_roboir(
        self,
        nl_instruction: str,
        cad_nodes: Optional[List[Dict[str, Any]]] = None,
    ) -> str:
        """
        旧版接口（向后兼容）：返回 JSON 字符串而非 dict。

        新代码请直接用 ``parse_instruction_to_roboir``。
        """
        ctx = {
            "nearest_objects": [n.get("label", "") for n in (cad_nodes or [])]
        }
        result = self.parse_instruction_to_roboir(nl_instruction, ctx)
        return json.dumps(result, ensure_ascii=False, indent=2)

    def set_model(self, model_name: str) -> None:
        """切换 DeepSeek 模型变体（如 deepseek-v4-flash → deepseek-v4-pro）。"""
        self.model_name = model_name

    # ──────────────────────────────────────────────
    # 辅助 API（演示/可视化用，不影响主翻译链路）
    # ──────────────────────────────────────────────

    def infer_local_context(self, instruction: str) -> Dict[str, Any]:
        """
        从自然语言指令里抽取实体名词，自动生成 nearest_objects JSON。

        例如："把红色的书放到书架第二层"
            → {"nearest_objects": ["red_book", "shelf", "shelf_layer_2"]}

        Returns:
            {"nearest_objects": [...], "_inferred_by": "deepseek"}
        Raises:
            VlmParserError: API 失败或解析失败。
        """
        if not instruction or not instruction.strip():
            raise VlmParserError("instruction 为空字符串。")

        sys_prompt = (
            "你是机器人场景实体抽取器。给定一句中文/英文自然语言指令，"
            "请抽取其中所有可能成为参照物的实体（物体、位置、家具、容器等），"
            "以 snake_case 英文返回。\n\n"
            "【绝对禁止】：不要描述动作、不要计算坐标、不要返回多余的解释。\n"
            "【输出格式】：严格的 JSON："
            '{"nearest_objects": ["entity_1", "entity_2", ...]}\n'
            "【示例】\n"
            '输入：把红色的书放到书架第二层\n'
            '输出：{"nearest_objects": ["red_book", "shelf", "shelf_layer_2"]}\n'
            '输入：避开桌子，把杯子搬到水槽边\n'
            '输出：{"nearest_objects": ["table", "mug", "sink_edge"]}'
        )
        user_prompt = f'输入：{instruction}\n输出：'

        last_err: Optional[Exception] = None
        for attempt in range(1, self.max_retries + 1):
            try:
                kwargs: Dict[str, Any] = dict(
                    model=self.model_name,
                    messages=[
                        {"role": "system", "content": sys_prompt},
                        {"role": "user", "content": user_prompt},
                    ],
                    temperature=0.0,
                    max_tokens=256,
                )
                try:
                    kwargs["response_format"] = {"type": "json_object"}
                    resp = self._client.chat.completions.create(**kwargs)
                except TypeError:
                    kwargs.pop("response_format", None)
                    resp = self._client.chat.completions.create(**kwargs)

                content = (resp.choices[0].message.content or "").strip()
                parsed = self._safe_parse_json(content)
                if not isinstance(parsed, dict):
                    raise VlmParserError("响应不是 dict")
                objs = parsed.get("nearest_objects", [])
                if not isinstance(objs, list):
                    raise VlmParserError("nearest_objects 必须是 list")
                return {
                    "nearest_objects": [str(o) for o in objs],
                    "_inferred_by": "deepseek",
                    "_source_instruction": instruction,
                }
            except (json.JSONDecodeError, VlmParserError) as e:
                last_err = e
            except _OPENAI_RETRIABLE as e:
                last_err = e
            except Exception as e:
                raise VlmParserError(f"非预期异常: {e}") from e
            if attempt < self.max_retries:
                time.sleep(self.retry_backoff_s * attempt)

        raise VlmParserError(
            f"infer_local_context 失败（重试 {self.max_retries} 次）。"
            f"最后错误: {type(last_err).__name__}: {last_err}"
        )

    def summarize_roboir(self, roboir: Dict[str, Any]) -> str:
        """
        把 RoboIR JSON 翻译回 **STL 风格**的中文声明式摘要（仅供阅读）。

        示例：
            intent=place_mug, target_frame=center_table, grasp_type=pinch,
            stl_constraints=[Distance>0.10 wrt obstacle_machine,
                             Orientation==vertical wrt green_mug]
          →
            "在 t ∈ [0, T] 内，末端轨迹存在某时刻 t* 使其位于 center_table；
             且 ∀t: 距 obstacle_machine ≥ 0.10 m；
             且 ∀t: green_mug 保持 vertical。
             抓取方式：pinch。"

        Args:
            roboir: parse_instruction_to_roboir 的返回 dict。
        Returns:
            STL 风格中文摘要字符串。
        Raises:
            VlmParserError: API 失败或返回空。
        """
        if not isinstance(roboir, dict):
            raise VlmParserError("roboir 必须是 dict")

        sys_prompt = (
            "你是 STL（Signal Temporal Logic）形式化语言翻译官。\n"
            "给定 RoboIR JSON，输出一段**声明式约束**的中文摘要。\n\n"
            "【硬性要求】\n"
            "1. 必须使用 STL 风格：『∀t ∈ [0, T]』、『∃t* ∈ [0, T]』、"
            "『且』、『使其』、『保持』、『使得』。\n"
            "2. 严禁出现命令式动词或时序词："
            "『先 / 然后 / 接着 / 之后 / 再 / 最后 / 移动到 / 拿起 / 放下』。\n"
            "3. 单段、无列表、无 Markdown、120 字以内。\n"
            "4. 必须包含：target_frame 通过约束、所有 stl_constraints、grasp_type。\n\n"
            "【参考输出风格】\n"
            "『在 t ∈ [0, T] 内，∃t* 使末端位于 <target_frame>；"
            "∀t: 距 <ref> ≥ 0.10 m；∀t: <obj> 保持 vertical。"
            "抓取方式：pinch。』"
        )
        user_prompt = (
            f'RoboIR JSON：\n{json.dumps(roboir, ensure_ascii=False, indent=2)}'
            f'\n\n请输出 STL 风格摘要：'
        )

        last_err: Optional[Exception] = None
        for attempt in range(1, self.max_retries + 1):
            try:
                resp = self._client.chat.completions.create(
                    model=self.model_name,
                    messages=[
                        {"role": "system", "content": sys_prompt},
                        {"role": "user", "content": user_prompt},
                    ],
                    temperature=0.2,
                    max_tokens=300,
                )
                content = (resp.choices[0].message.content or "").strip()
                if not content:
                    raise VlmParserError("空响应")
                return content
            except _OPENAI_RETRIABLE as e:
                last_err = e
            except Exception as e:
                raise VlmParserError(f"非预期异常: {e}") from e
            if attempt < self.max_retries:
                time.sleep(self.retry_backoff_s * attempt)

        raise VlmParserError(
            f"summarize_roboir 失败（重试 {self.max_retries} 次）。"
            f"最后错误: {type(last_err).__name__}: {last_err}"
        )

    def summarize_diagnostic(
        self,
        roboir: Dict[str, Any],
        trajectory: List[Dict[str, float]],
        demo3_output: Dict[str, Any],
    ) -> str:
        """
        让 LLM 把 Demo 3 的结构化反馈翻译成中文「诊断报告」。

        Args:
            roboir: Demo 2 输出的 RoboIR JSON。
            trajectory: Demo 3 输入的轨迹（list of pose dict）。
            demo3_output: ``demo3_run`` 返回 JSON 解析后的 dict，含
                ``feedbacks``, ``raw_reports_summary``, ``n_violations``。
        Returns:
            包含【任务理解】【执行结果】【失败原因】【改进建议】的中文报告。
        """
        if not isinstance(roboir, dict) or not isinstance(demo3_output, dict):
            raise VlmParserError("roboir 与 demo3_output 都必须是 dict。")

        # 简化轨迹避免 prompt 过长
        if len(trajectory) > 6:
            traj_brief = [trajectory[0], trajectory[len(trajectory) // 2],
                          trajectory[-1]]
            traj_note = f"（仅示头/中/尾，共 {len(trajectory)} 帧）"
        else:
            traj_brief = trajectory
            traj_note = ""

        sys_prompt = (
            "你是机器人执行回看分析师。给定一次任务的：\n"
            "(A) 目标声明 RoboIR JSON；\n"
            "(B) 实际执行轨迹（部分采样）；\n"
            "(C) 物理求解器给出的结构化反馈（含 STL 鲁棒度 ρ、违规节点、"
            "FailureTaxonomy 等）。\n\n"
            "请输出一段简洁的**中文诊断报告**，覆盖以下 4 节，"
            "每节 1~3 句，全报告控制在 250 字以内：\n\n"
            "**【1. 任务理解】**：用一句话复述 RoboIR 要求做什么。\n"
            "**【2. 执行结果】**：用 STL 风格判定全部满足 / 部分违反。"
            "若违反，指出哪条约束、最坏 ρ、何时发生。\n"
            "**【3. 失败原因（如有）】**：基于 FailureTaxonomy 与 worst_node"
            "推断物理层根因（例：『轨迹第 4 帧在 (4.3, 3.95) 处距墙仅 0.15m，"
            "低于安全阈值 0.30m』）。任务成功则写"
            "『无违规，全程满足安全冗余』。\n"
            "**【4. 改进建议】**：给规划者的具体建议（例：『放宽 Distance > 0.20』"
            "或『绕路经过 (3.5, 2.0) 区域』）。\n\n"
            "硬性要求：\n"
            "① 违规时必须引用具体数值：坐标 (x,y)、时刻 t、ρ 值、"
            "实际距离 d_real_m、阈值 threshold_m。\n"
            "② 任务完成状态（完成/部分完成/失败）必须明确写出。\n"
            "③ 用 STL 风格描述违规（『∀t / ∃t* / ρ = ...』）。\n"
            "④ 不要 Markdown 代码块，不要拟人化，禁命令式时序。"
        )

        # 提炼最关键信息，避免给 LLM 一堆冗余字段
        feedbacks_brief = []
        for fb in demo3_output.get("feedbacks", []):
            diag = fb.get("diagnosis") or {}
            err = fb.get("error") or {}
            details = (diag.get("details") or {})
            worst = details.get("worst_node") or {}
            # 提取轨迹时间点 + 坐标（如可用）
            entry: Dict[str, Any] = {
                "rule": diag.get("rule"),
                "taxonomy": err.get("type"),
                "rho": err.get("robustness_score"),
                "d_real_m": details.get("clearance_m"),
                "threshold_m": details.get("threshold_m"),
                "worst_pose": worst if worst else None,
                "n_checked": details.get("n_checked"),
            }
            feedbacks_brief.append(entry)

        compact = {
            "intent": roboir.get("intent"),
            "target_frame": roboir.get("target_frame"),
            "grasp_type": roboir.get("grasp_type"),
            "stl_constraints": [c.get("expr") + (
                f" wrt {c['ref']}" if c.get("ref") else ""
            ) for c in roboir.get("stl_constraints", [])],
            "n_violations": demo3_output.get("n_violations"),
            "n_total_constraints": demo3_output.get("n_total_constraints",
                                                    len(feedbacks_brief)),
            "traj_summary": {
                "n_waypoints": len(trajectory),
                "start": trajectory[0] if trajectory else None,
                "end": trajectory[-1] if trajectory else None,
                "total_time_s": (trajectory[-1].get("t", 0.0)
                                 if trajectory else 0.0),
            },
            "feedbacks": feedbacks_brief,
        }

        user_prompt = (
            f"(A) 任务目标 RoboIR：\n"
            f"{json.dumps(roboir, ensure_ascii=False, indent=2)}\n\n"
            f"(B) 执行轨迹（采样{traj_note}）：\n"
            f"{json.dumps(traj_brief, ensure_ascii=False, indent=2)}\n\n"
            f"(C) 物理求解器结构化反馈：\n"
            f"{json.dumps(compact, ensure_ascii=False, indent=2)}\n\n"
            f"严格要求：诊断文字中 **必须** 引用 (C) 里的具体数值，"
            f"例如坐标 (x, y)、时刻 t、ρ 值、d_real_m、threshold_m 等。"
            f"禁止空泛描述。请按四节格式输出："
        )

        last_err: Optional[Exception] = None
        for attempt in range(1, self.max_retries + 1):
            try:
                resp = self._client.chat.completions.create(
                    model=self.model_name,
                    messages=[
                        {"role": "system", "content": sys_prompt},
                        {"role": "user", "content": user_prompt},
                    ],
                    temperature=0.3,
                    max_tokens=600,
                )
                content = (resp.choices[0].message.content or "").strip()
                if not content:
                    raise VlmParserError("空响应")
                return content
            except _OPENAI_RETRIABLE as e:
                last_err = e
            except Exception as e:
                raise VlmParserError(f"非预期异常: {e}") from e
            if attempt < self.max_retries:
                time.sleep(self.retry_backoff_s * attempt)

        raise VlmParserError(
            f"summarize_diagnostic 失败（重试 {self.max_retries} 次）。"
            f"最后错误: {type(last_err).__name__}: {last_err}"
        )

    # ──────────────────────────────────────────────
    # Internals
    # ──────────────────────────────────────────────

    @staticmethod
    def _build_user_prompt(
        instruction: str,
        local_context: Dict[str, Any],
    ) -> str:
        ctx_text = json.dumps(local_context, ensure_ascii=False)
        return (
            f"输入:\n"
            f"  指令：\"{instruction}\"\n"
            f"  局部环境: {ctx_text}\n"
            f"输出 JSON:"
        )

    def _call_api(self, user_prompt: str) -> str:
        """单次 DeepSeek 调用（不含重试 — 重试由调用者处理）。"""
        kwargs: Dict[str, Any] = dict(
            model=self.model_name,
            messages=[
                {"role": "system", "content": ROBOIR_SYSTEM_PROMPT},
                {"role": "user", "content": user_prompt},
            ],
            temperature=self.temperature,
            max_tokens=1024,
        )
        # response_format 在 DeepSeek 上是软兼容；若服务端拒绝，
        # 第二次重试自动剥离该参数。
        try:
            kwargs["response_format"] = {"type": "json_object"}
            resp = self._client.chat.completions.create(**kwargs)
        except TypeError:
            kwargs.pop("response_format", None)
            resp = self._client.chat.completions.create(**kwargs)

        if not getattr(resp, "choices", None):
            raise VlmParserError("DeepSeek 返回空 choices。")
        msg = resp.choices[0].message
        content = (getattr(msg, "content", None) or "").strip()
        if not content:
            raise VlmParserError("DeepSeek 返回空 content。")
        return content

    @staticmethod
    def _safe_parse_json(content: str) -> Dict[str, Any]:
        """容忍偶发 Markdown code fence 包装的 JSON，使用正则剥离所有变体。

        LLM 常见输出形态（均被处理）：
          - 纯 JSON
          - ```json ... ```
          - ``` ... ```
          - 前后多余空白行 + JSON
          - JSON 前后有解释文字但 JSON 内嵌其中
        """
        import re as _re

        text = content.strip()

        # 优先尝试直接解析（最快路径：LLM 输出了干净 JSON）
        try:
            return json.loads(text)
        except json.JSONDecodeError:
            pass

        # 剥离 ```json ... ``` 或 ``` ... ``` 包装（.*? 非贪婪）
        fence_match = _re.search(
            r"```(?:json)?\s*\n?(.*?)```",
            text,
            flags=_re.DOTALL | _re.IGNORECASE,
        )
        if fence_match:
            text = fence_match.group(1).strip()
            try:
                return json.loads(text)
            except json.JSONDecodeError:
                pass

        # 最后兜底：从首个 { 到末尾 } 提取（抵御 LLM 前置说明文字）
        brace_match = _re.search(r"\{.*\}", text, flags=_re.DOTALL)
        if brace_match:
            text = brace_match.group(0).strip()
            return json.loads(text)

        # 彻底无法提取
        raise json.JSONDecodeError("No JSON object found", content, 0)

    @staticmethod
    def _validate_shape(parsed: Dict[str, Any]) -> None:
        """字段完整性 + 枚举值合法性校验。"""
        if not isinstance(parsed, dict):
            raise VlmParserError(
                f"RoboIR 顶层必须是 dict，实际是 {type(parsed).__name__}。"
            )

        for field in ("intent", "target_frame", "grasp_type", "stl_constraints"):
            if field not in parsed:
                raise VlmParserError(f"RoboIR 缺少必需字段: '{field}'。")

        gt = parsed["grasp_type"]
        if gt not in VALID_GRASP_TYPES:
            raise VlmParserError(
                f"非法 grasp_type='{gt}'，必须 ∈ {sorted(VALID_GRASP_TYPES)}。"
            )

        constraints = parsed["stl_constraints"]
        if not isinstance(constraints, list):
            raise VlmParserError(
                f"stl_constraints 必须是 list，实际是 {type(constraints).__name__}。"
            )
        for i, c in enumerate(constraints):
            if not isinstance(c, dict):
                raise VlmParserError(
                    f"stl_constraints[{i}] 必须是 dict。"
                )
            if "expr" not in c or not isinstance(c["expr"], str):
                raise VlmParserError(
                    f"stl_constraints[{i}].expr 缺失或非 str。"
                )
            # 软校验：expr 至少包含一个合法 STL 操作符关键词
            if not any(op in c["expr"] for op in VALID_STL_OPS):
                raise VlmParserError(
                    f"stl_constraints[{i}].expr='{c['expr']}' 未包含任何合法 STL 操作符。"
                )


# ══════════════════════════════════════════════════════════════════════
# 引用一致性校验（不抛异常 — 返回 warnings 列表）
# ══════════════════════════════════════════════════════════════════════

# 全局通配符：表示"任何障碍物 / 任何参照"，无需在 nearest_objects 里声明
_REF_WILDCARDS: frozenset[str] = frozenset({
    "obstacle", "any_obstacle", "wall", "walls", "floor", "ground",
    "self", "robot", "end_effector",
})


def validate_roboir_references(
    roboir: Dict[str, Any],
    local_context: Optional[Dict[str, Any]],
) -> List[Dict[str, Any]]:
    """
    检查 RoboIR 中所有 reference 是否都在 local_context.nearest_objects 里。

    返回 warning 字典列表（空列表 = 全部通过）：
      [{
        "level": "warning" | "info",
        "field": "target_frame" | "stl_constraints[0].ref",
        "value": "tissue_box",
        "message": "引用了未在 nearest_objects 中声明的物体: 'tissue_box'",
        "hint": "可能是 LLM 幻觉，或局部环境列表不完整。"
      }, ...]

    注意：本函数**不抛异常**，因为：
      1. 部分 ref（如 'obstacle'）是合法通配符，无需声明；
      2. 用户可能就是想测试"指令引用了不存在物体"这种边缘场景。

    Args:
        roboir: parse_instruction_to_roboir 的返回 dict。
        local_context: 编译时提供的局部环境 dict（含 nearest_objects 字段）。
    Returns:
        warning 字典列表。
    """
    if not isinstance(roboir, dict):
        return [{
            "level": "warning",
            "field": "_root",
            "value": str(type(roboir).__name__),
            "message": "RoboIR 不是 dict，无法校验引用",
            "hint": "",
        }]

    nearest = []
    if isinstance(local_context, dict):
        nearest = local_context.get("nearest_objects", []) or []
    nearest_set = {str(x).strip() for x in nearest if x}

    def is_known(ref: str) -> bool:
        if not ref:
            return True
        ref_norm = ref.strip()
        if ref_norm in _REF_WILDCARDS:
            return True
        return ref_norm in nearest_set

    warnings: List[Dict[str, Any]] = []

    target_frame = roboir.get("target_frame", "")
    if target_frame and not is_known(target_frame):
        warnings.append({
            "level": "warning",
            "field": "target_frame",
            "value": target_frame,
            "message": (
                f"target_frame='{target_frame}' 未在 nearest_objects 中声明。"
            ),
            "hint": (
                "可能原因：(a) LLM 幻觉出场景中没有的物体；"
                "(b) 你忘了在局部环境里列出该物体。"
                "Demo 3 物理求解时将无法找到该物体的世界坐标。"
            ),
        })

    constraints = roboir.get("stl_constraints", []) or []
    for i, c in enumerate(constraints):
        if not isinstance(c, dict):
            continue
        ref = c.get("ref", "")
        if not ref:
            continue
        if not is_known(ref):
            warnings.append({
                "level": "warning",
                "field": f"stl_constraints[{i}].ref",
                "value": ref,
                "message": (
                    f"约束 '{c.get('expr', '?')}' 引用了未声明的物体 '{ref}'。"
                ),
                "hint": (
                    "Demo 3 求解此约束时会回退到通用 obstacle "
                    "（如启用距离场），结果可能不符合你的本意。"
                ),
            })

    return warnings


# ══════════════════════════════════════════════════════════════════════
# LLM 输出 → spatial_api_stub 标准形式
# ══════════════════════════════════════════════════════════════════════

def normalize_stl_constraints(
    stl_constraints: List[Dict[str, Any]],
    default_unit_distance: str = "m",
    default_unit_time: str = "s",
    default_unit_angle: str = "deg",
) -> List[Dict[str, Any]]:
    """
    把 LLM 紧凑输出（``{"expr": "Distance > 0.10", "ref": "obstacle"}``）
    转为 ``spatial_api_stub.check_stl_constraint_violation`` 期望的形式
    （``{"type": "stl_constraint", "expr": "...", "unit": "m", "reference": "..."}``）。

    自动按 expr 内的关键词推断 unit；若 LLM 已显式给了 unit/reference 字段则保留。
    """
    out: List[Dict[str, Any]] = []
    for c in stl_constraints or []:
        if not isinstance(c, dict) or "expr" not in c:
            continue
        expr = str(c["expr"])
        unit = c.get("unit")
        if not unit:
            if "Distance" in expr or "Velocity" in expr:
                unit = default_unit_distance
            elif "Time" in expr:
                unit = default_unit_time
            elif "Orientation" in expr:
                unit = default_unit_angle
            else:
                unit = ""
        ref = c.get("reference") or c.get("ref")
        if not ref and ("Distance" in expr or "Orientation" in expr):
            ref = "obstacle"  # 默认参照物（与 Demo 3 默认一致）
        norm: Dict[str, Any] = {
            "type": "stl_constraint",
            "expr": expr,
            "unit": unit,
        }
        if ref:
            norm["reference"] = ref
        out.append(norm)
    return out


# ══════════════════════════════════════════════════════════════════════
# 模块级便捷函数
# ══════════════════════════════════════════════════════════════════════

def parse_instruction_to_roboir(
    instruction: str,
    local_context: Optional[Dict[str, Any]] = None,
    *,
    model_name: str = DEEPSEEK_DEFAULT_MODEL,
    api_key: Optional[str] = None,
    max_retries: int = 3,
) -> Dict[str, Any]:
    """
    便捷函数：一次性解析单条指令（每次调用会重建一个 OpenAI client，
    适合脚本式使用；高频调用请直接复用 ``VlmParser`` 实例）。
    """
    parser = VlmParser(
        model_name=model_name,
        api_key=api_key,
        max_retries=max_retries,
    )
    return parser.parse_instruction_to_roboir(instruction, local_context)


__all__ = [
    "VALID_FRAMES",
    "VALID_GRASP_TYPES",
    "VALID_STL_OPS",
    "DEEPSEEK_BASE_URL",
    "DEEPSEEK_DEFAULT_MODEL",
    "DEEPSEEK_API_KEY_ENV",
    "ROBOIR_SYSTEM_PROMPT",
    "VlmParser",
    "VlmParserError",
    "parse_instruction_to_roboir",
    "normalize_stl_constraints",
]
