"""
batch_dwgread.py — Batch dwgread -Ojson on all DWG samples + structure report.

Runs parse_dwg_to_json() on every .dwg in libredwg_test_suite/ and
autodesk_official/, saves JSON outputs, and generates STRUCTURE_REPORT.md
documenting schema variations, entity types, layer counts, and edge cases.

Prerequisite: LibreDWG (dwgread) must be installed.
"""

from __future__ import annotations

import json
import sys
import time
from collections import Counter
from pathlib import Path
from typing import Any, Dict, List

sys.path.insert(0, str(Path(__file__).resolve().parent.parent.parent))

BASE_DIR = Path(__file__).resolve().parent
OUTPUT_DIR = BASE_DIR / "output"
REPORT_PATH = BASE_DIR / "STRUCTURE_REPORT.md"

SOURCE_DIRS = [
    BASE_DIR / "libredwg_test_suite",
    BASE_DIR / "autodesk_official",
]


# ══════════════════════════════════════════════════════════════════════
# JSON structure analysis
# ══════════════════════════════════════════════════════════════════════

def analyze_dwg_json(dwg_json: Dict[str, Any]) -> Dict[str, Any]:
    """Extract structural stats from a LibreDWG JSON output."""
    stats: Dict[str, Any] = {}

    # Top-level keys
    stats["top_level_keys"] = sorted(dwg_json.keys())

    # Header variables
    header = dwg_json.get("header", dwg_json.get("HEADER", {}))
    if isinstance(header, dict):
        stats["header_vars"] = {
            "INSUNITS": header.get("INSUNITS", header.get("$INSUNITS")),
            "MEASUREMENT": header.get("MEASUREMENT", header.get("$MEASUREMENT")),
            "DWGCODEPAGE": header.get("DWGCODEPAGE", header.get("$DWGCODEPAGE")),
        }

    # Entities
    entities = _find_all_entities(dwg_json)
    stats["entity_count"] = len(entities)

    # Entity type distribution
    type_counter: Counter = Counter()
    for ent in entities:
        etype = str(ent.get("entity", ent.get("type", "?")))
        type_counter[etype] += 1
    stats["entity_types"] = dict(type_counter.most_common())

    # Layers
    layers: set = set()
    for ent in entities:
        layer = ent.get("layer")
        if isinstance(layer, str) and layer:
            layers.add(layer)
        elif isinstance(layer, list) and layer:
            layers.add("_".join(str(x) for x in layer))
    stats["layer_count"] = len(layers)
    stats["layer_names"] = sorted(layers)[:20]

    # Coordinate range
    xs, ys = [], []
    for ent in entities:
        for key in ("start", "end", "center", "ins_pt", "insertion_point", "position"):
            pt = ent.get(key)
            if isinstance(pt, dict):
                x, y = pt.get("x"), pt.get("y")
                if x is not None and y is not None:
                    xs.append(float(x))
                    ys.append(float(y))
        vertices = ent.get("vertices", ent.get("points", []))
        if isinstance(vertices, list):
            for v in vertices:
                if isinstance(v, dict):
                    xs.append(float(v.get("x", 0)))
                    ys.append(float(v.get("y", 0)))

    if xs and ys:
        stats["coord_range"] = {
            "x_min": round(min(xs), 2),
            "x_max": round(max(xs), 2),
            "y_min": round(min(ys), 2),
            "y_max": round(max(ys), 2),
            "extent_x": round(max(xs) - min(xs), 2),
            "extent_y": round(max(ys) - min(ys), 2),
        }

    # Object types (non-entity top-level keys)
    obj_keys = [k for k in dwg_json.keys() if k not in ("header", "HEADER", "OBJECTS")]
    stats["non_entity_sections"] = obj_keys

    return stats


def _find_all_entities(dwg_json: Dict[str, Any]) -> List[Dict[str, Any]]:
    """Find all entities in LibreDWG JSON recursively (max depth 10)."""
    entities: List[Dict[str, Any]] = []

    def _walk(obj: Any, depth: int) -> None:
        if depth > 10:
            return
        if isinstance(obj, dict):
            obj_list = obj.get("OBJECTS", obj.get("objects"))
            if isinstance(obj_list, list):
                for item in obj_list:
                    if isinstance(item, dict):
                        ent_name = item.get("entity", "")
                        if ent_name and ent_name not in (
                            "VX_CONTROL", "BLOCK_HEADER", "END_BLK",
                        ):
                            entities.append(item)
                        elif not ent_name:
                            tc = item.get("type")
                            if isinstance(tc, int) and tc in _DWG_TYPE_MAP:
                                item_copy = dict(item)
                                item_copy["entity"] = _DWG_TYPE_MAP[tc]
                                entities.append(item_copy)
                        _walk(item, depth + 1)
        elif isinstance(obj, list):
            for item in obj:
                _walk(item, depth)

    _DWG_TYPE_MAP = {
        1: "TEXT", 7: "INSERT", 18: "CIRCLE", 19: "LINE",
        21: "LWPOLYLINE", 22: "POLYLINE2D", 23: "POLYLINE3D",
        44: "MTEXT", 45: "ARC",
    }

    _walk(dwg_json, 0)
    return entities


# ══════════════════════════════════════════════════════════════════════
# Report generation
# ══════════════════════════════════════════════════════════════════════

def generate_report(
    results: List[Dict[str, Any]],
    dwgread_available: bool,
    elapsed_s: float,
) -> str:
    """Generate STRUCTURE_REPORT.md from batch results."""
    lines: List[str] = []
    lines.append("# DWG JSON Structure Report")
    lines.append("")
    lines.append(f"Generated: {time.strftime('%Y-%m-%d %H:%M:%S')}")
    lines.append(f"dwgread available: {dwgread_available}")
    lines.append(f"Processing time: {elapsed_s:.1f}s")
    lines.append("")

    ok = [r for r in results if r["status"] == "ok"]
    err = [r for r in results if r["status"] != "ok"]

    lines.append(f"## Summary")
    lines.append(f"- **{len(ok)}** files parsed successfully")
    lines.append(f"- **{len(err)}** files failed or pending")
    lines.append("")

    if not dwgread_available:
        lines.append(
            "> LibreDWG (dwgread) is not installed on this system. "
            "Install from https://www.gnu.org/software/libredwg/ "
            "and re-run this script to populate the report."
        )
        lines.append("")

    if err:
        lines.append("## Failed / Pending")
        lines.append("")
        for r in err:
            lines.append(f"- **{r['filename']}**: {r.get('message', 'unknown error')}")
        lines.append("")

    if ok:
        # ── Entity type summary across all files ──
        all_types: Counter = Counter()
        all_layers: Counter = Counter()
        versions: Counter = Counter()
        for r in ok:
            stats = r.get("stats", {})
            for etype, count in stats.get("entity_types", {}).items():
                all_types[etype] += count
            for layer in stats.get("layer_names", []):
                all_layers[layer] += 1
            versions[r.get("dwg_version", "?")] += 1

        lines.append("## DWG Version Distribution")
        lines.append("")
        lines.append("| Version | Count |")
        lines.append("|---------|-------|")
        for ver, count in versions.most_common():
            lines.append(f"| {ver} | {count} |")
        lines.append("")

        lines.append("## Entity Types (cross-file aggregate)")
        lines.append("")
        lines.append("| Entity Type | Total Count |")
        lines.append("|-------------|------------|")
        for etype, count in all_types.most_common():
            lines.append(f"| {etype} | {count} |")
        lines.append("")

        lines.append("## Common Layers")
        lines.append("")
        lines.append("| Layer Name | Files |")
        lines.append("|-----------|-------|")
        for layer, n in all_layers.most_common(20):
            lines.append(f"| {layer} | {n} |")
        lines.append("")

        lines.append("## Per-File Details")
        lines.append("")
        for r in ok:
            stats = r.get("stats", {})
            lines.append(f"### {r['filename']}")
            lines.append(f"- **Version**: {r.get('dwg_version', '?')}")
            lines.append(f"- **Entities**: {stats.get('entity_count', 0)}")
            lines.append(f"- **Layers**: {stats.get('layer_count', 0)}")
            lines.append(f"- **Entity types**: "
                + ", ".join(
                    f"{k}({v})" for k, v in
                    list(stats.get("entity_types", {}).items())[:10]
                ))
            coord = stats.get("coord_range")
            if coord:
                lines.append(
                    f"- **Extent**: {coord['extent_x']:.1f} x {coord['extent_y']:.1f} "
                    f"(X: {coord['x_min']:.1f}–{coord['x_max']:.1f}, "
                    f"Y: {coord['y_min']:.1f}–{coord['y_max']:.1f})"
                )
            header = stats.get("header_vars", {})
            if header:
                lines.append(f"- **Header**: INSUNITS={header.get('INSUNITS')}, "
                    f"MEASUREMENT={header.get('MEASUREMENT')}")
            lines.append("")

        lines.append("## Observed Schema Variations")
        lines.append("")
        lines.append(
            "1. **Top-level structure**: All DWG files produce JSON with at least "
            "`header` (or `HEADER`) and `OBJECTS` keys."
        )
        lines.append(
            "2. **Entity identification**: Entities are identified by `entity` string "
            "(e.g., LINE, CIRCLE, TEXT) or numeric `type` code. Both must be handled."
        )
        lines.append(
            "3. **Coordinate representation**: Points use `{x, y}` dicts within "
            "entity-specific keys (`start`/`end` for LINE, `center` for CIRCLE, "
            "`ins_pt` for TEXT/INSERT, `vertices` for POLYLINE)."
        )
        lines.append(
            "4. **Coordinate systems**: INSUNITS varies across files. The dispatcher "
            "applies `_DWG_UNIT_TO_M` scaling; files without INSUNITS default to 1.0 "
            "(assumed meters)."
        )
        lines.append(
            "5. **Layer references**: Layers are typically string names; in some cases "
            "may be numeric handle references (handled by dispatcher's "
            "`_extract_dwg_semantics`)."
        )

    return "\n".join(lines)


# ══════════════════════════════════════════════════════════════════════
# Main
# ══════════════════════════════════════════════════════════════════════

def main():
    OUTPUT_DIR.mkdir(parents=True, exist_ok=True)

    # Check dwgread availability
    dwgread_available = False
    dwgread_path = None
    try:
        from cad_parser.dwg_ingestion import _find_dwgread
        dwgread_path = _find_dwgread()
        dwgread_available = dwgread_path is not None
    except ImportError:
        print("WARNING: cad_parser not on PYTHONPATH, can't find dwgread")

    if dwgread_available:
        print(f"dwgread found at: {dwgread_path}")
    else:
        print("dwgread NOT FOUND. Report will show 'pending' status for all files.")
        print("Install LibreDWG: https://www.gnu.org/software/libredwg/")

    # Collect all DWG files (deduplicate by resolved path)
    dwg_files: List[Path] = []
    seen: set = set()
    for src_dir in SOURCE_DIRS:
        if src_dir.is_dir():
            for dwg in sorted(src_dir.glob("*")):
                if dwg.suffix.lower() not in (".dwg", ".dxf"):
                    continue
                rp = dwg.resolve()
                if rp not in seen:
                    seen.add(rp)
                    dwg_files.append(dwg)

    print(f"\nFound {len(dwg_files)} unique DWG files to process.\n")

    results: List[Dict[str, Any]] = []
    t0 = time.time()

    for dwg_path in dwg_files:
        name = dwg_path.name
        print(f"[{len(results)+1}/{len(dwg_files)}] {name} ...", end=" ", flush=True)

        if not dwgread_available:
            results.append({
                "filename": name,
                "status": "pending_dwgread",
                "dwg_version": "?",
                "message": "LibreDWG (dwgread) not installed",
            })
            print("SKIP (no dwgread)")
            continue

        # Run through the existing pipeline
        from cad_parser.dwg_ingestion import parse_dwg_to_json

        ingestion = parse_dwg_to_json(str(dwg_path))
        if ingestion["status"] != "ok":
            results.append({
                "filename": name,
                "status": "error",
                "dwg_version": ingestion.get("dwg_version", "?"),
                "message": ingestion["message"],
            })
            print(f"ERROR: {ingestion['message'][:80]}")
            continue

        dwg_json = ingestion["json"]
        dwg_version = ingestion["dwg_version"]

        # Save raw JSON
        json_out = OUTPUT_DIR / f"{dwg_path.stem}.json"
        json_out.write_text(
            json.dumps(dwg_json, indent=2, ensure_ascii=False), encoding="utf-8"
        )
        json_kb = json_out.stat().st_size // 1024

        # Structural analysis
        stats = analyze_dwg_json(dwg_json)

        results.append({
            "filename": name,
            "status": "ok",
            "dwg_version": dwg_version,
            "is_r2004": ingestion.get("is_r2004", False),
            "json_size_kb": json_kb,
            "stats": stats,
        })
        ent_count = stats.get("entity_count", 0)
        layer_count = stats.get("layer_count", 0)
        print(f"OK ({dwg_version}, {ent_count} entities, {layer_count} layers, {json_kb} KB)")

    elapsed = time.time() - t0

    # Generate report
    report_md = generate_report(results, dwgread_available, elapsed)
    REPORT_PATH.write_text(report_md, encoding="utf-8")
    print(f"\nReport written to {REPORT_PATH}")

    # Save structured results as JSON for downstream consumption
    summary_json = {
        "batch_metadata": {
            "dwgread_available": dwgread_available,
            "dwgread_path": dwgread_path,
            "elapsed_s": round(elapsed, 2),
            "total_files": len(dwg_files),
            "successful": sum(1 for r in results if r["status"] == "ok"),
            "pending": sum(1 for r in results if r["status"] == "pending_dwgread"),
            "failed": sum(1 for r in results if r["status"] == "error"),
        },
        "per_file": [
            {
                k: v for k, v in r.items()
                if k != "stats"  # stats already in per-file detail above
            }
            for r in results
        ],
    }
    summary_path = OUTPUT_DIR / "batch_summary.json"
    summary_path.write_text(
        json.dumps(summary_json, indent=2, ensure_ascii=False), encoding="utf-8"
    )
    print(f"Summary written to {summary_path}")

    return 0


if __name__ == "__main__":
    sys.exit(main())
