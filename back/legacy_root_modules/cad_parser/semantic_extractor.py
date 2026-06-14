"""
semantic_extractor.py — DWG 图纸 AI 语义提取器

消费 Dispatcher 的「轨迹 B」语义数据（图层名 / 文本 / 块参照），
调用 DeepSeek API 从中识别关键业务节点：

  - 上料位置 (loading_zone)
  - 下料位置 (unloading_zone)
  - 设备名称 (equipment)
  - 区域/房间名称 (area)

识别结果作为 CLASS_LOADING 节点打入 AreaGraph 拓扑 JSON，
使机器人后续的 A* 路径规划能自动考虑"从哪上料、到哪下料"。

注意（四大执行区原则）：
  大模型在此仅做语义识别（"文本标签→语义类别"），
  不计算坐标、距离、路径。坐标来自 DWG 几何实体本身。
"""

from __future__ import annotations

import json
import os
import re
import time
from typing import Any, Dict, List, Optional


# ── 复用 vlm_parser 的 API 配置 ──────────────────────────

DEEPSEEK_API_KEY_ENV = "DEEPSEEK_API_KEY"
DEEPSEEK_API_KEY_ENV_FALLBACK = "DEEPSEEK_KEY"
DEEPSEEK_BASE_URL = "https://api.deepseek.com"
DEEPSEEK_DEFAULT_MODEL = "deepseek-v4-flash"


# ══════════════════════════════════════════════════════════════════════
# System Prompt — 工业图纸语义识别专用
# ══════════════════════════════════════════════════════════════════════

SEMANTIC_EXTRACTION_PROMPT = r"""
你是一个工业 CAD 图纸语义分析专家。你的输入是一张 DWG 工程图纸中提取出的结构化数据，包括：

1. **图层列表 (layers)**：图纸中使用的所有图层名称。
2. **文本标注 (texts)**：图纸中的文字（含坐标位置），如房间名、设备编号、尺寸标注。
3. **块参照 (blocks)**：图纸中的标准图块引用（含坐标位置），如设备符号、图框。

【你的任务】
从以上数据中识别出以下关键业务节点，并以 JSON 格式返回：

1. **loading_zones（上料位置）**：原材料或零件进入加工区域的位置。
   典型关键词：上料口、进料、原料区、投料、Loading、Feeding、入口、Infeed
2. **unloading_zones（下料位置）**：成品或半成品离开加工区域的位置。
   典型关键词：下料口、出料、成品区、卸料、Unloading、Outfeed、出口、Discharge
3. **equipment（设备）**：生产设备、机床、机器人工作站。
   典型关键词：CNC、铣床、车床、机器人、Robot、加工中心、压机、冲床
4. **areas（区域）**：功能区域名称。
   典型关键词：装配区、焊接区、喷涂区、仓库、通道、Assembly、Welding

【绝对禁止】
- 不要自行计算或生成任何 XYZ 坐标。坐标只能从输入数据的 position 字段中引用。
- 不要编造不存在于输入中的设备和区域名称。
- 如果文本内容无法确定类别，归类到 areas 而不是设备。

【输出格式】
{
  "loading_zones": [
    {"label": "上料口A", "position": [x, y], "confidence": 0.9, "source": "text"}
  ],
  "unloading_zones": [
    {"label": "下料口B", "position": [x, y], "confidence": 0.85, "source": "block"}
  ],
  "equipment": [
    {"label": "CNC铣床", "position": [x, y], "confidence": 0.95, "source": "text",
     "layer": "MACHINE"}
  ],
  "areas": [
    {"label": "装配区", "position": null, "confidence": 0.7, "source": "text",
     "note": "无精确坐标，来自文本描述"}
  ]
}

其中：
- label: 人类可读名称
- position: [x, y] 或 null（无坐标时）
- confidence: 0.0~1.0 置信度
- source: "text" | "block" | "layer"
- layer: 可选，来源图层名
- note: 可选，补充说明
"""


# ══════════════════════════════════════════════════════════════════════
# 异常类
# ══════════════════════════════════════════════════════════════════════

class SemanticExtractorError(RuntimeError):
    """语义提取异常。"""


# ══════════════════════════════════════════════════════════════════════
# SemanticExtractor
# ══════════════════════════════════════════════════════════════════════

class SemanticExtractor:
    """
    DWG 图纸语义提取器。

    用法::

        extractor = SemanticExtractor(api_key_env="DEEPSEEK_API_KEY")
        result = extractor.extract(semantics_data)
        print(result["loading_zones"])
    """

    def __init__(
        self,
        api_key_env: str = DEEPSEEK_API_KEY_ENV,
        model: str = DEEPSEEK_DEFAULT_MODEL,
        base_url: str = DEEPSEEK_BASE_URL,
        max_retries: int = 2,
        timeout_s: float = 30.0,
    ):
        api_key = os.getenv(api_key_env)
        if not api_key:
            raise SemanticExtractorError(
                f"环境变量 {api_key_env} 未设置。"
                f"在终端中执行：$env:{api_key_env} = '<your-key>'"
            )

        try:
            from openai import OpenAI
        except ImportError:
            raise SemanticExtractorError(
                "需要 openai SDK。请运行：pip install openai"
            )

        self._client = OpenAI(api_key=api_key, base_url=base_url)
        self._model = model
        self._max_retries = max_retries
        self._timeout_s = timeout_s

    # ── 主入口 ────────────────────────────────────────

    def extract(self, semantics: Dict[str, Any]) -> Dict[str, Any]:
        """
        从 DWG 语义数据中提取业务节点。

        Args:
            semantics: dispatcher._extract_dwg_semantics() 的返回，
                       含 layers / texts / blocks / entity_summary。

        Returns:
            {
                "loading_zones": [...],
                "unloading_zones": [...],
                "equipment": [...],
                "areas": [...],
                "raw_response": str,
            }
        """
        user_prompt = self._build_user_prompt(semantics)

        last_err: Optional[Exception] = None
        for attempt in range(1, self._max_retries + 1):
            try:
                raw = self._call_api(user_prompt)
                parsed = self._safe_parse_json(raw)
                validated = self._validate_result(parsed, semantics)
                validated["raw_response"] = raw
                return validated
            except (json.JSONDecodeError, SemanticExtractorError) as e:
                last_err = e
                if attempt < self._max_retries:
                    time.sleep(attempt * 1.5)

        raise SemanticExtractorError(
            f"语义提取失败（{self._max_retries} 次重试后）：{last_err}"
        )

    # ── Prompt 构建 ────────────────────────────────────

    def _build_user_prompt(self, semantics: Dict[str, Any]) -> str:
        """将语义数据格式化为 LLM 输入。"""
        layers_data = semantics.get("layers", [])
        texts_data = semantics.get("texts", [])
        blocks_data = semantics.get("blocks", [])

        # 采样避免 token 超限
        text_sample = texts_data[:200] if len(texts_data) > 200 else texts_data
        block_sample = blocks_data[:100] if len(blocks_data) > 100 else blocks_data

        parts: List[str] = [
            "请分析以下 DWG 工程图纸的语义数据，识别关键业务节点。",
            "",
            f"## 图纸概况",
            f"- 图层数：{len(layers_data)}",
            f"- 文本标注数：{len(texts_data)}（已采样 {len(text_sample)} 条）",
            f"- 块参照数：{len(blocks_data)}（已采样 {len(block_sample)} 条）",
            "",
            f"## 图层列表",
            json.dumps(layers_data, ensure_ascii=False, indent=2),
            "",
            f"## 文本标注（含坐标）",
            json.dumps(text_sample, ensure_ascii=False, indent=2),
            "",
            f"## 块参照（含坐标）",
            json.dumps(block_sample, ensure_ascii=False, indent=2),
            "",
            "请返回 JSON，只输出 JSON 不要任何解释。",
        ]

        return "\n".join(parts)

    # ── API 调用 ───────────────────────────────────────

    def _call_api(self, user_prompt: str) -> str:
        kwargs: Dict[str, Any] = dict(
            model=self._model,
            messages=[
                {"role": "system", "content": SEMANTIC_EXTRACTION_PROMPT},
                {"role": "user", "content": user_prompt},
            ],
            temperature=0.1,
            max_tokens=2048,
        )
        try:
            kwargs["response_format"] = {"type": "json_object"}
            resp = self._client.chat.completions.create(**kwargs)
        except TypeError:
            kwargs.pop("response_format", None)
            resp = self._client.chat.completions.create(**kwargs)

        content = resp.choices[0].message.content
        if not content:
            raise SemanticExtractorError("API 返回空内容")
        return content

    # ── JSON 解析 ──────────────────────────────────────

    @staticmethod
    def _safe_parse_json(raw: str) -> Dict[str, Any]:
        """容错解析：处理 markdown 包裹、尾部逗号等常见 LLM 输出瑕疵。"""
        cleaned = re.sub(r"^```(?:json)?\s*\n?", "", raw.strip())
        cleaned = re.sub(r"\n?```\s*$", "", cleaned)
        cleaned = re.sub(r",\s*([}\]])", r"\1", cleaned)
        return json.loads(cleaned)

    # ── 结果校验 ───────────────────────────────────────

    @staticmethod
    def _validate_result(
        parsed: Dict[str, Any],
        _semantics: Dict[str, Any],
    ) -> Dict[str, Any]:
        """确保返回包含必要字段，填充缺失键。"""
        required_keys = ["loading_zones", "unloading_zones", "equipment", "areas"]
        for key in required_keys:
            if key not in parsed:
                parsed[key] = []

        for zone_key in ("loading_zones", "unloading_zones"):
            for item in parsed[zone_key]:
                if "label" not in item:
                    item["label"] = zone_key.rstrip("s")
                if "position" not in item:
                    item["position"] = None
                if "confidence" not in item:
                    item["confidence"] = 0.5
                if "source" not in item:
                    item["source"] = "unknown"

        return parsed


# ══════════════════════════════════════════════════════════════════════
# 便捷函数：将语义结果注入拓扑 JSON
# ══════════════════════════════════════════════════════════════════════

def inject_semantic_nodes(
    topology_json: Dict[str, Any],
    extraction: Dict[str, Any],
    resolution: float = 0.05,
    origin: tuple = (0.0, 0.0),
) -> Dict[str, Any]:
    """
    将 LLM 识别出的上/下料位置作为 CLASS_LOADING 节点打入拓扑 JSON。

    Args:
        topology_json: astar_topology.extract_topology_json() 的返回。
        extraction: SemanticExtractor.extract() 的返回。
        resolution: 米/像素。
        origin: 栅格原点 (x_min, y_min)。

    Returns:
        注入 loading_nodes 后的拓扑 JSON（浅拷贝 + 新增字段）。
    """
    def world_to_rc(x: float, y: float) -> List[int]:
        col = int(round((x - origin[0]) / resolution))
        row = int(round((y - origin[1]) / resolution))
        return [row, col]

    loading_nodes: List[Dict[str, Any]] = []

    for zone in extraction.get("loading_zones", []):
        pos = zone.get("position")
        if pos and len(pos) == 2:
            loading_nodes.append({
                "type": "loading_zone",
                "label": zone.get("label", "上料口"),
                "world_xy": pos,
                "grid_rc": world_to_rc(pos[0], pos[1]),
                "confidence": zone.get("confidence", 0.5),
            })

    for zone in extraction.get("unloading_zones", []):
        pos = zone.get("position")
        if pos and len(pos) == 2:
            loading_nodes.append({
                "type": "unloading_zone",
                "label": zone.get("label", "下料口"),
                "world_xy": pos,
                "grid_rc": world_to_rc(pos[0], pos[1]),
                "confidence": zone.get("confidence", 0.5),
            })

    result = {k: v for k, v in topology_json.items()}
    result["loading_nodes"] = loading_nodes
    result["equipment"] = extraction.get("equipment", [])
    result["areas"] = extraction.get("areas", [])
    return result


# ══════════════════════════════════════════════════════════════════════
# 环境分析 Prompt — Demo 1 解析后生成环境描述与约束
# ══════════════════════════════════════════════════════════════════════

ENVIRONMENT_ANALYSIS_PROMPT = r"""
你是一个具身智能空间分析专家。你收到了一份建筑物/工厂平面图的结构化解析结果，
你需要从中提炼出对机器人导航和操作至关重要的环境信息。

【你的任务】
根据以下数据，输出一份**环境分析报告**（JSON 格式），涵盖：

1. **environment_summary**：1-2 句中文概括这个空间是什么、主要特征。
2. **spatial_layout**：空间的物理布局描述（尺寸、形状、连通性）。
3. **key_objects**：识别出的重要物体/设备/区域及其大致位置（不要编造坐标）。
4. **navigation_constraints**：机器人导航需要注意的约束：
   - 窄通道（宽度 < 机器人直径的区域）
   - 死胡同或不可达区域
   - 需要避开的障碍物
   - 建议的安全路线区域
5. **demo2_context**：给 Demo 2（语义编译器）的上下文提示：
   - 哪些物体可以交互（可抓取、可放置）
   - 哪些是固定障碍物
   - 建议的操作参考坐标系（如某个桌子、某个设备）
6. **demo3_constraints**：给 Demo 3（物理求解器）的约束建议：
   - 最小安全距离建议 (D_safe 推荐值，单位米)
   - 需要特别检查碰撞的区域
   - 轨迹时间约束建议

【绝对禁止】
- 不要自行计算或编造具体的 XYZ 坐标值
- 不要生成超出输入数据范围的假设
- 如果数据不足以做出判断，请明确标注 "insufficient_data"

【输出格式】只输出合法 JSON：
{
  "environment_summary": "string",
  "spatial_layout": {"dimensions": "...", "shape": "...", "connectivity": "..."},
  "key_objects": [{"name": "...", "type": "equipment|furniture|door|zone", "location_hint": "...", "interactable": true|false}],
  "navigation_constraints": [{"type": "narrow_passage|dead_end|obstacle|restricted_zone", "description": "...", "severity": "high|medium|low"}],
  "demo2_context": {"interactable_objects": ["..."], "fixed_obstacles": ["..."], "suggested_frames": ["..."]},
  "demo3_constraints": {"D_safe_m": 0.0, "collision_risk_zones": ["..."], "time_constraint_suggestion": "..."}
}
"""


def analyze_environment(
    parse_result: Dict[str, Any],
    topology_stats: Optional[Dict[str, Any]] = None,
    api_key: Optional[str] = None,
    model: str = DEEPSEEK_DEFAULT_MODEL,
) -> Dict[str, Any]:
    """
    用 LLM 分析 Demo 1 解析结果，生成环境描述和导航约束。

    Args:
        parse_result: dispatcher.dispatch_cad() 的返回。
        topology_stats: classify_topology_global() 的统计信息。
        api_key: DeepSeek API key，默认从环境变量读取。
        model: 模型名称。

    Returns:
        {"status": "ok"|"error", "analysis": {...}, "raw": "..."}
    """
    resolved_key = api_key or os.environ.get(DEEPSEEK_API_KEY_ENV, "") or os.environ.get(DEEPSEEK_API_KEY_ENV_FALLBACK, "")
    if not resolved_key:
        return {
            "status": "error",
            "analysis": None,
            "message": f"未设置 {DEEPSEEK_API_KEY_ENV}，跳过环境分析。",
        }

    try:
        from openai import OpenAI
    except ImportError:
        return {"status": "error", "analysis": None, "message": "需要 pip install openai"}

    # 构建用户 prompt
    mode = parse_result.get("mode", "unknown")
    grid = parse_result.get("grid")
    transform = parse_result.get("transform", {})
    cad_data = parse_result.get("cad_data", {}) or {}
    semantics = parse_result.get("semantics", {}) or {}
    note = parse_result.get("note", "")

    shape = transform.get("shape", [0, 0])
    resolution = transform.get("resolution", 0.05)
    origin = transform.get("origin", [0, 0])

    width_m = shape[1] * resolution if len(shape) > 1 else 0
    height_m = shape[0] * resolution if len(shape) > 0 else 0

    parts: List[str] = [
        "请分析以下平面图的结构化数据，生成环境分析报告。",
        "",
        f"## 基本信息",
        f"- 输入格式: {mode.upper()}",
        f"- 栅格尺寸: {shape[0]}×{shape[1]} px @ {resolution} m/px",
        f"- 物理尺寸: {width_m:.1f}m × {height_m:.1f}m",
        f"- 原点: ({origin[0]:.2f}, {origin[1]:.2f})",
        f"- 数据说明: {note}",
    ]

    # 拓扑统计
    if topology_stats:
        parts.append(f"## 拓扑统计")
        parts.append(f"- 连通域数: {topology_stats.get('n_components', '?')}")
        parts.append(f"- 主连通域占比: {topology_stats.get('largest_component_frac', 0):.1%}")
        sizes = topology_stats.get('component_sizes', [])
        if sizes:
            parts.append(f"- 连通域大小 (前5): {sizes[:5]}")

    if grid is not None:
        n_total = int(grid.size)
        n_free = int((grid == 1).sum())
        n_obstacle = int((grid == 0).sum())
        free_pct = n_free / max(1, n_total) * 100
        parts.append(f"## 占据统计")
        parts.append(f"- 总像素: {n_total}")
        parts.append(f"- 自由空间: {n_free} ({free_pct:.1f}%)")
        parts.append(f"- 障碍物: {n_obstacle} ({100-free_pct:.1f}%)")

    # JSON 模式：房间/门/物体
    if mode == "json" and cad_data:
        room_type = cad_data.get("room_type", "unknown")
        doors = cad_data.get("doors", [])
        objects = cad_data.get("objects", [])
        windows = cad_data.get("windows", [])
        parts.append(f"## 房间语义 (JSON)")
        parts.append(f"- 房间类型: {room_type}")
        parts.append(f"- 门: {len(doors)} 个")
        for d in doors[:5]:
            label = d.get("label", "door")
            pts = d.get("points", [])
            if pts:
                cx = sum(p[0] for p in pts) / len(pts)
                cy = sum(p[1] for p in pts) / len(pts)
            parts.append(f"  · {label} @ 大致 ({cx:.1f}, {cy:.1f})m")
        parts.append(f"- 家具/物体: {len(objects)} 个")
        for o in objects[:8]:
            parts.append(f"  · {o.get('label', 'object')}")
        parts.append(f"- 窗户: {len(windows)} 个")

    # DWG 模式：图层/文本/块
    if mode == "dwg" and semantics:
        layers = semantics.get("layers", [])
        texts = semantics.get("texts", [])
        blocks = semantics.get("blocks", [])
        e_summary = semantics.get("entity_summary", {})
        parts.append(f"## DWG 语义数据")
        parts.append(f"- 图层 ({len(layers)}): {layers[:10]}")
        parts.append(f"- 实体统计: {json.dumps(e_summary, ensure_ascii=False)}")
        if texts:
            parts.append(f"- 文本标注 ({len(texts)} 条):")
            for t in texts[:15]:
                pos = t.get("position", [0, 0])
                parts.append(f"  · \"{t.get('value', '')}\" @ ({pos[0]:.1f}, {pos[1]:.1f})")
        if blocks:
            parts.append(f"- 块参照 ({len(blocks)}):")
            for b in blocks[:10]:
                pos = b.get("position", [0, 0])
                parts.append(f"  · {b.get('name', '?')} @ ({pos[0]:.1f}, {pos[1]:.1f})")

    # SVG/PNG 模式：仅几何
    if mode in ("svg", "png"):
        parts.append(f"## 注意")
        parts.append(f"- {mode.upper()} 模式仅提供占据栅格，无对象语义。")
        parts.append(f"- 请仅基于自由空间/障碍物分布进行分析。")

    parts.append("")
    parts.append("请输出 JSON，只输出 JSON 不要任何解释。")

    user_prompt = "\n".join(parts)

    # 调用 API
    client = OpenAI(api_key=resolved_key, base_url=DEEPSEEK_BASE_URL)
    kwargs = dict(
        model=model,
        messages=[
            {"role": "system", "content": ENVIRONMENT_ANALYSIS_PROMPT},
            {"role": "user", "content": user_prompt},
        ],
        temperature=0.2,
        max_tokens=1536,
    )
    try:
        kwargs["response_format"] = {"type": "json_object"}
        resp = client.chat.completions.create(**kwargs)
    except TypeError:
        kwargs.pop("response_format", None)
        resp = client.chat.completions.create(**kwargs)

    content = resp.choices[0].message.content or ""

    # 解析 JSON
    cleaned = re.sub(r"^```(?:json)?\s*\n?", "", content.strip())
    cleaned = re.sub(r"\n?```\s*$", "", cleaned)
    cleaned = re.sub(r",\s*([}\]])", r"\1", cleaned)
    try:
        analysis = json.loads(cleaned)
    except json.JSONDecodeError:
        return {
            "status": "error",
            "analysis": None,
            "message": f"LLM 返回非 JSON: {content[:200]}",
        }

    return {
        "status": "ok",
        "analysis": analysis,
        "raw_response": content,
        "model_used": model,
    }
