"""将 Cursor agent-transcripts 的 JSONL 转为可读 Markdown（脱敏用）。"""
from __future__ import annotations

import json
import re
import sys
from pathlib import Path


def _redact(text: str) -> str:
    # 粗略脱敏：sk- 开头的 key
    return re.sub(r"sk-[a-zA-Z0-9]{20,}", "[REDACTED_API_KEY]", text)


def jsonl_to_markdown(src: Path, dst: Path) -> int:
    lines_out: list[str] = [
        f"# 对话导出\n",
        f"- 源文件: `{src.name}`\n",
        f"- 说明: 由 Cursor agent-transcripts 自动转换；工具调用细节已省略。\n",
        "---\n",
    ]
    n = 0
    with src.open(encoding="utf-8") as f:
        for raw in f:
            raw = raw.strip()
            if not raw:
                continue
            try:
                obj = json.loads(raw)
            except json.JSONDecodeError:
                continue
            role = obj.get("role", "?")
            msg = obj.get("message") or {}
            parts = msg.get("content") or []
            texts = []
            for p in parts:
                if isinstance(p, dict) and p.get("type") == "text":
                    texts.append(p.get("text", ""))
            body = _redact("\n".join(texts).strip())
            if not body:
                continue
            n += 1
            title = "用户" if role == "user" else "助手"
            lines_out.append(f"\n## {title} ({n})\n\n{body}\n")
    dst.write_text("".join(lines_out), encoding="utf-8")
    return n


def main() -> None:
    repo = Path(__file__).resolve().parents[2]
    export_dir = repo / "smdsl_demo" / "conversation_exports"
    src = export_dir / "transcript_cabc44b2-f229-432b-b017-dbae5bb72cc9.jsonl"
    if not src.exists():
        print(f"Missing: {src}", file=sys.stderr)
        sys.exit(1)
    dst = export_dir / "CONVERSATION_EXPORT.md"
    count = jsonl_to_markdown(src, dst)
    print(f"Wrote {dst} ({count} messages)")


if __name__ == "__main__":
    main()
