# 对话导出（Cursor Agent Transcripts）

本目录保存与本仓库（`d:\code` / SMDSL 项目）相关的 Cursor 对话记录，便于归档、评审与测试交接。

## 文件说明

| 文件 | 说明 |
|------|------|
| **`CONVERSATION_EXPORT.md`** | **推荐阅读**：主会话的可读 Markdown（384 条消息），由 JSONL 自动转换；`sk-...` 类密钥已做粗略脱敏 |
| `transcript_cabc44b2-f229-432b-b017-dbae5bb72cc9.jsonl` | **主会话**（约 1.1 MB）：基线核查 → Demo 1/2/3 → Gradio UI → 外部泛洪/拓扑/测试方案等 |
| `transcript_f98046a0-0f39-4d33-9df6-889f299e9338.jsonl` | 较早会话（约 306 KB） |
| `transcript_b7dac382-4cba-4254-b496-3fab7f8b5d2d.jsonl` | 较早会话（约 20 KB） |

## 与项目文档的关系

- **开发问题与改法（人工整理）**：见 [`../DEVELOPMENT_JOURNAL.md`](../DEVELOPMENT_JOURNAL.md)
- **测试方案**：见 [`../QA_TEST_PLAN.md`](../QA_TEST_PLAN.md)
- **架构原则**：见 [`../PROJECT_CONTEXT.md`](../PROJECT_CONTEXT.md)

完整 JSONL 含工具调用元数据；日常查阅优先看 `CONVERSATION_EXPORT.md` 与 `DEVELOPMENT_JOURNAL.md`。

## 重新生成 Markdown

```powershell
cd D:\code
python smdsl_demo\scripts\export_transcript_to_md.py
```

## 安全提示

- 导出前已对 API Key 做简单替换，**仍请勿将本目录提交到公开仓库**（若含未脱敏片段）。
- 原始 transcript 位于 Cursor 工程目录：  
  `C:\Users\admin\.cursor\projects\d-code\agent-transcripts\`

## 来源

- 导出时间：以文件 `LastWriteTime` 为准  
- 主会话 ID：`cabc44b2-f229-432b-b017-dbae5bb72cc9`
