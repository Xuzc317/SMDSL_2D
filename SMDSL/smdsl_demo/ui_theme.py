"""
ui_theme.py — SMDSL Gradio UI 主题与 CSS (Phase 3.1)

从 app.py 拆分：CSS 常量 + 主题构建函数。
"""

import gradio as gr

SMDSL_THEME_CSS = """
/* ── 强制亮色主题：覆盖 Gradio dark-mode CSS 变量 ────────────── */
:root, .dark, body, html {
    --body-background-fill:           #fbfaf8 !important;
    --body-background-fill-secondary: #f5f4f1 !important;
    --panel-background-fill:          #ffffff !important;
    --block-background-fill:          #ffffff !important;
    --input-background-fill:          #ffffff !important;
    --input-background-fill-focus:    #ffffff !important;
    --chatbot-background-fill:        #ffffff !important;
    --border-color-primary:           #ece9e3 !important;
    --border-color-accent:            #cc785c !important;
    --color-accent:                   #cc785c !important;
    --color-accent-soft:              #fdf3ee !important;
    --body-text-color:                #1a1a1a !important;
    --body-text-color-subdued:        #6c6a65 !important;
    --block-label-text-color:         #2a2a2a !important;
    --block-title-text-color:         #181818 !important;
    --input-placeholder-color:        #a8a59f !important;
    --button-primary-background-fill: #1a1a1a !important;
    --button-primary-text-color:      #ffffff !important;
    --button-secondary-background-fill: #ffffff !important;
    --button-secondary-text-color:    #1a1a1a !important;
    --button-secondary-border-color:  #d9d6d0 !important;
    --table-even-background-fill:     #f7f5f1 !important;
    --table-odd-background-fill:      #ffffff !important;
    --code-background-fill:           #f7f5f1 !important;
    --shadow-drop:                    none !important;
    --shadow-drop-lg:                 none !important;
    --color-grey-100:                 #fbfaf8 !important;
    --color-grey-200:                 #f5f4f1 !important;
    --neutral-950:                    #1a1a1a !important;
    --neutral-900:                    #2a2a2a !important;
    --neutral-800:                    #404040 !important;
}

/* ── 全局字体 + 米白底 ─────────────────────────────────────────── */
.gradio-container, .gradio-container * {
    color-scheme: light !important;
}
.gradio-container {
    font-family: 'Inter', 'ui-sans-serif', -apple-system, BlinkMacSystemFont,
                 'Segoe UI', 'PingFang SC', 'Hiragino Sans GB',
                 'Microsoft YaHei', system-ui, sans-serif !important;
    background: #fbfaf8 !important;
    color: #1a1a1a !important;
    max-width: 1180px !important;
    margin: 0 auto !important;
    padding: 28px 36px !important;
    line-height: 1.6 !important;
}

/* ── 标题层级（致敬 docs.anthropic 文章页） ────────────────────── */
.gradio-container h1 {
    font-size: 30px !important;
    font-weight: 600 !important;
    letter-spacing: -0.02em !important;
    color: #181818 !important;
    border-bottom: none !important;
    margin: 6px 0 4px 0 !important;
}
.gradio-container h2 {
    font-size: 19px !important;
    font-weight: 600 !important;
    letter-spacing: -0.01em !important;
    color: #2a2a2a !important;
    margin: 22px 0 10px 0 !important;
}
.gradio-container h3 {
    font-size: 15px !important;
    font-weight: 600 !important;
    color: #404040 !important;
    margin-top: 18px !important;
}

/* ── 正文段落 ────────────────────────────────────────────────── */
.gradio-container p, .gradio-container li {
    color: #3a3a3a !important;
    font-size: 14.5px !important;
}

/* ── Tab 标签栏 ──────────────────────────────────────────────── */
.tab-nav button {
    background: transparent !important;
    border: none !important;
    border-bottom: 2px solid transparent !important;
    border-radius: 0 !important;
    padding: 12px 18px !important;
    font-weight: 500 !important;
    color: #6a6a6a !important;
    font-size: 14px !important;
    transition: all 0.15s ease !important;
}
.tab-nav button:hover {
    color: #cc785c !important;
}
.tab-nav button.selected {
    color: #cc785c !important;
    border-bottom: 2px solid #cc785c !important;
    background: transparent !important;
}

/* ── 按钮：克制 + 单一强调色 ─────────────────────────────────── */
button.primary, button[variant="primary"] {
    background: #1a1a1a !important;
    color: #ffffff !important;
    border: 1px solid #1a1a1a !important;
    border-radius: 8px !important;
    font-weight: 500 !important;
    box-shadow: none !important;
}
button.primary:hover {
    background: #2d2d2d !important;
    border-color: #2d2d2d !important;
}
button.secondary, button[variant="secondary"] {
    background: #ffffff !important;
    color: #1a1a1a !important;
    border: 1px solid #d9d6d0 !important;
    border-radius: 8px !important;
    font-weight: 500 !important;
    box-shadow: none !important;
}
button.secondary:hover {
    background: #f5f4f1 !important;
    border-color: #cc785c !important;
}

/* ── 输入框 / 文本域 ─────────────────────────────────────────── */
.gradio-container textarea,
.gradio-container input[type="text"],
.gradio-container input[type="number"] {
    background: #ffffff !important;
    border: 1px solid #e6e3dd !important;
    border-radius: 8px !important;
    font-family: 'JetBrains Mono', 'Menlo', 'Consolas', monospace !important;
    font-size: 13px !important;
}
.gradio-container textarea:focus,
.gradio-container input:focus {
    border-color: #cc785c !important;
    box-shadow: 0 0 0 3px rgba(204, 120, 92, 0.12) !important;
}

/* ── 容器卡片：薄边 + 圆角 ───────────────────────────────────── */
.block, .form, .panel {
    background: #ffffff !important;
    border: 1px solid #ece9e3 !important;
    border-radius: 12px !important;
    box-shadow: none !important;
}

/* ── 数据表 ─────────────────────────────────────────────────── */
.gradio-container table {
    border-collapse: collapse !important;
    font-size: 13.5px !important;
}
.gradio-container th {
    background: #f5f4f1 !important;
    font-weight: 600 !important;
    color: #2a2a2a !important;
    border-bottom: 2px solid #d9d6d0 !important;
}
.gradio-container td {
    border-bottom: 1px solid #ece9e3 !important;
    padding: 8px 12px !important;
}

/* ── 代码块 ─────────────────────────────────────────────────── */
.gradio-container pre, .gradio-container code {
    background: #f7f5f1 !important;
    border-radius: 6px !important;
    font-family: 'JetBrains Mono', 'Menlo', 'Consolas', monospace !important;
}

/* ── 分割线 ─────────────────────────────────────────────────── */
.gradio-container hr {
    border: none !important;
    border-top: 1px solid #ece9e3 !important;
    margin: 28px 0 !important;
}

/* ── 自定义流程导航卡片 ──────────────────────────────────────── */
.smdsl-flow-nav { font-family: inherit !important; }

/* ── Markdown 内 inline code ─────────────────────────────────── */
.gradio-container :not(pre) > code {
    background: #f5f0eb !important;
    color: #b25a3f !important;
    padding: 1px 6px !important;
    border-radius: 4px !important;
    font-size: 13px !important;
}

/* ── 强制 body / 页面背景为米白，消灭 dark 模式残留 ───────────── */
body, html { background: #f5f4f1 !important; }

/* ── Code 块亮色覆盖 ─────────────────────────────────────────── */
.codemirror-wrapper, .cm-editor, .cm-scroller,
.codemirror-wrapper *, .cm-editor * {
    background: #f7f5f1 !important;
    color: #1a1a1a !important;
}
"""


def build_theme() -> "gr.themes.Soft":
    """返回 SMDSL 默认主题对象。"""
    return gr.themes.Soft(
        primary_hue=gr.themes.Color(
            c50="#fcf5f1", c100="#f9ebe4", c200="#f0d4c5", c300="#e6b89f",
            c400="#dc9d7a", c500="#cc785c", c600="#b85e44", c700="#94472f",
            c800="#6f3322", c900="#4a2117", c950="#2a120c",
        ),
        neutral_hue=gr.themes.Color(
            c50="#fbfaf8", c100="#f5f4f1", c200="#ece9e3", c300="#d9d6d0",
            c400="#a8a59f", c500="#6c6a65", c600="#494844", c700="#2f2e2c",
            c800="#1f1e1d", c900="#141413", c950="#0a0a0a",
        ),
        font=[
            gr.themes.GoogleFont("Inter"),
            "ui-sans-serif", "system-ui", "sans-serif",
        ],
        font_mono=[
            gr.themes.GoogleFont("JetBrains Mono"),
            "ui-monospace", "Menlo", "Consolas", "monospace",
        ],
    )
