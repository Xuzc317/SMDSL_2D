"""
test_entity_stats.py — DWG 实体统计单元测试 (Phase 1.3)

验证 _extract_dwg_geometry 的 entity_stats 计数：
  - 正常混合实体 → 正确的 processed / skipped 计数
  - 全已知实体 → drop_rate = 0
  - 全未知实体 → drop_rate = 1.0
  - _format_entity_drop_note 阈值行为
"""

import sys
from pathlib import Path

_PROJECT_ROOT = Path(__file__).resolve().parent.parent
if str(_PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(_PROJECT_ROOT))

import pytest

from cad_parser.dispatcher import _extract_dwg_geometry, _format_entity_drop_note


def _make_entity(entity_type: str, **kwargs):
    """创建 mock DWG 实体。"""
    ent = {"entity": entity_type}
    if entity_type in ("LINE",):
        ent["start"] = {"x": 0.0, "y": 0.0, "z": 0.0}
        ent["end"] = {"x": 1.0, "y": 1.0, "z": 0.0}
    elif entity_type in ("LWPOLYLINE",):
        ent["vertices"] = [
            {"x": 0.0, "y": 0.0, "z": 0.0},
            {"x": 1.0, "y": 1.0, "z": 0.0},
        ]
    elif entity_type in ("CIRCLE",):
        ent["center"] = {"x": 0.0, "y": 0.0, "z": 0.0}
        ent["radius"] = 1.0
    elif entity_type in ("ARC",):
        ent["center"] = {"x": 0.0, "y": 0.0, "z": 0.0}
        ent["radius"] = 1.0
        ent["start_angle"] = 0.0
        ent["end_angle"] = 3.14159
    ent.update(kwargs)
    return ent


class TestEntityStats:
    """DWG 实体统计测试集"""

    def test_mixed_entities_count(self):
        """混合已知/未知实体 → 正确计数"""
        entities = [
            _make_entity("LINE"),
            _make_entity("LWPOLYLINE"),
            _make_entity("CIRCLE"),
            _make_entity("TEXT"),       # 不在几何处理范围
            _make_entity("INSERT"),     # 不在几何处理范围
            _make_entity("MTEXT"),      # 不在几何处理范围
        ]
        walls, polygons, stats = _extract_dwg_geometry(entities)

        assert stats["total_entities"] == 6
        assert stats["processed"] == 3  # LINE + LWPOLYLINE + CIRCLE
        assert stats["skipped_by_type"].get("TEXT") == 1
        assert stats["skipped_by_type"].get("INSERT") == 1
        assert stats["skipped_by_type"].get("MTEXT") == 1

    def test_all_known_entities(self):
        """全已知实体 → processed = total, no skipped"""
        entities = [
            _make_entity("LINE"),
            _make_entity("LWPOLYLINE"),
            _make_entity("CIRCLE"),
            _make_entity("ARC"),
            _make_entity("LINE"),
        ]
        walls, polygons, stats = _extract_dwg_geometry(entities)

        assert stats["total_entities"] == 5
        assert stats["processed"] == 5
        assert len(stats["skipped_by_type"]) == 0

    def test_all_unknown_entities(self):
        """全未知实体 → processed = 0"""
        entities = [
            _make_entity("UNKNOWN_TYPE_A"),
            _make_entity("UNKNOWN_TYPE_B"),
        ]
        walls, polygons, stats = _extract_dwg_geometry(entities)

        assert stats["total_entities"] == 2
        assert stats["processed"] == 0
        assert stats["skipped_by_type"].get("UNKNOWN_TYPE_A") == 1
        assert stats["skipped_by_type"].get("UNKNOWN_TYPE_B") == 1

    def test_empty_entities(self):
        """空实体列表 → total = 0"""
        walls, polygons, stats = _extract_dwg_geometry([])
        assert stats["total_entities"] == 0
        assert stats["processed"] == 0

    def test_entity_with_acdb_prefix(self):
        """AcDb 前缀实体 → 正确去除前缀并处理"""
        entities = [
            _make_entity("AcDbLine"),
            _make_entity("AcDbPolyline"),
        ]
        walls, polygons, stats = _extract_dwg_geometry(entities)

        assert stats["total_entities"] == 2
        assert stats["processed"] == 2  # AcDbLine→LINE, AcDbPolyline→POLYLINE


class TestFormatEntityDropNote:
    """丢弃率格式化函数测试"""

    def test_no_drop_when_below_threshold(self):
        stats = {"total_entities": 100, "processed": 95, "skipped_by_type": {}}
        note = _format_entity_drop_note(stats)
        assert note == ""

    def test_no_drop_at_exactly_threshold(self):
        stats = {"total_entities": 100, "processed": 90, "skipped_by_type": {"TEXT": 10}}
        note = _format_entity_drop_note(stats)
        assert note == ""  # drop_rate = 0.1, not > 0.1

    def test_drop_above_threshold(self):
        stats = {
            "total_entities": 100,
            "processed": 70,
            "skipped_by_type": {"TEXT": 20, "INSERT": 10},
        }
        note = _format_entity_drop_note(stats)
        assert "30.0%" in note
        assert "TEXT=20" in note
        assert "INSERT=10" in note

    def test_empty_stats(self):
        stats = {"total_entities": 0, "processed": 0, "skipped_by_type": {}}
        note = _format_entity_drop_note(stats)
        assert note == ""
