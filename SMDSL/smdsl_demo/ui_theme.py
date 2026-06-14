"""
ui_theme.py — SMDSL Gradio UI 主题与 CSS

Design: Claude (Anthropic) warm minimalism + Apple (macOS) glassmorphism.
支持亮色/暗色双模式，Apple 4px 间距系统，SF-style 微交互。
"""

import gradio as gr

# ═══════════════════════════════════════════════════════════════════
# Design Tokens — Apple 间距系统 (4px base)
# ═══════════════════════════════════════════════════════════════════

SMDSL_THEME_CSS = r"""
/* ───────────────────────────────────────────────────────────────── */
/*  SMDSL Design System — Claude × Apple                           */
/* ───────────────────────────────────────────────────────────────── */

/* ── CSS Custom Properties (Design Tokens) ──────────────────── */
:root {
    /* Spacing (Apple 4px grid) */
    --space-xs: 4px;
    --space-sm: 8px;
    --space-md: 16px;
    --space-lg: 24px;
    --space-xl: 32px;
    --space-2xl: 48px;
    --space-3xl: 64px;

    /* Typography scale (Apple HIG) */
    --text-caption: 11px;
    --text-callout: 12px;
    --text-subhead: 13px;
    --text-body: 14px;
    --text-title3: 16px;
    --text-title2: 19px;
    --text-title1: 24px;
    --text-large-title: 30px;

    /* Radius (Apple continuous curves) */
    --radius-sm: 6px;
    --radius-md: 10px;
    --radius-lg: 14px;
    --radius-xl: 20px;
    --radius-full: 9999px;

    /* Shadows (Apple layered system) */
    --shadow-sm: 0 1px 2px rgba(0,0,0,0.04), 0 1px 3px rgba(0,0,0,0.06);
    --shadow-md: 0 2px 8px rgba(0,0,0,0.06), 0 4px 16px rgba(0,0,0,0.04);
    --shadow-lg: 0 4px 24px rgba(0,0,0,0.08), 0 8px 32px rgba(0,0,0,0.04);
    --shadow-xl: 0 8px 40px rgba(0,0,0,0.10), 0 16px 48px rgba(0,0,0,0.04);

    /* Transitions (Apple spring-like) */
    --transition-fast: 0.15s cubic-bezier(0.4, 0, 0.2, 1);
    --transition-smooth: 0.25s cubic-bezier(0.4, 0, 0.2, 1);
    --transition-spring: 0.35s cubic-bezier(0.34, 1.56, 0.64, 1);

    /* ── Light Mode Palette ── */
    --bg-primary: #f5f5f7;
    --bg-secondary: #ffffff;
    --bg-tertiary: #fafafa;
    --bg-elevated: rgba(255,255,255,0.72);
    --bg-glass: rgba(255,255,255,0.65);

    --text-primary: #1d1d1f;
    --text-secondary: #6e6e73;
    --text-tertiary: #aeaeb2;
    --text-placeholder: #c7c7cc;

    --border-primary: rgba(0,0,0,0.06);
    --border-secondary: rgba(0,0,0,0.04);
    --border-focus: rgba(0,122,255,0.35);

    --accent: #0071e3;
    --accent-hover: #0077ed;
    --accent-soft: rgba(0,113,227,0.08);
    --accent-muted: rgba(0,113,227,0.04);

    --success: #34c759;
    --warning: #ff9500;
    --danger: #ff3b30;
    --info: #5ac8fa;

    /* Claude warm accent (terracotta/copper) */
    --claude-accent: #cc785c;
    --claude-accent-soft: rgba(204,120,92,0.08);
    --claude-accent-muted: rgba(204,120,92,0.04);

    --gradio-bg: var(--bg-primary);
    --gradio-panel: var(--bg-secondary);
    --gradio-input: var(--bg-secondary);
    --gradio-border: var(--border-primary);
    --gradio-text: var(--text-primary);
    --gradio-text-muted: var(--text-secondary);
}

/* ── Dark Mode Palette ── */
.dark, [data-theme="dark"] {
    --bg-primary: #1c1c1e;
    --bg-secondary: #2c2c2e;
    --bg-tertiary: #3a3a3c;
    --bg-elevated: rgba(44,44,46,0.72);
    --bg-glass: rgba(44,44,46,0.55);

    --text-primary: #f5f5f7;
    --text-secondary: #a1a1a6;
    --text-tertiary: #636366;
    --text-placeholder: #48484a;

    --border-primary: rgba(255,255,255,0.08);
    --border-secondary: rgba(255,255,255,0.04);
    --border-focus: rgba(10,132,255,0.45);

    --accent: #0a84ff;
    --accent-hover: #409cff;
    --accent-soft: rgba(10,132,255,0.12);
    --accent-muted: rgba(10,132,255,0.06);

    --shadow-sm: 0 1px 2px rgba(0,0,0,0.2);
    --shadow-md: 0 2px 8px rgba(0,0,0,0.25);
    --shadow-lg: 0 4px 24px rgba(0,0,0,0.3);
    --shadow-xl: 0 8px 40px rgba(0,0,0,0.35);

    --gradio-bg: var(--bg-primary);
    --gradio-panel: var(--bg-secondary);
    --gradio-input: var(--bg-tertiary);
    --gradio-border: var(--border-primary);
    --gradio-text: var(--text-primary);
    --gradio-text-muted: var(--text-secondary);
}

/* ── Gradio Variable Mapping ── */
:root, .dark, [data-theme="dark"] {
    --body-background-fill: var(--bg-primary) !important;
    --body-background-fill-secondary: var(--bg-tertiary) !important;
    --panel-background-fill: var(--bg-secondary) !important;
    --block-background-fill: var(--bg-secondary) !important;
    --input-background-fill: var(--gradio-input) !important;
    --input-background-fill-focus: var(--gradio-input) !important;
    --chatbot-background-fill: var(--bg-secondary) !important;
    --border-color-primary: var(--border-primary) !important;
    --border-color-accent: var(--accent) !important;
    --color-accent: var(--accent) !important;
    --color-accent-soft: var(--accent-soft) !important;
    --body-text-color: var(--text-primary) !important;
    --body-text-color-subdued: var(--text-secondary) !important;
    --block-label-text-color: var(--text-primary) !important;
    --block-title-text-color: var(--text-primary) !important;
    --input-placeholder-color: var(--text-placeholder) !important;
    --button-primary-background-fill: var(--accent) !important;
    --button-primary-text-color: #ffffff !important;
    --button-secondary-background-fill: var(--bg-secondary) !important;
    --button-secondary-text-color: var(--text-primary) !important;
    --button-secondary-border-color: var(--border-primary) !important;
    --table-even-background-fill: var(--bg-tertiary) !important;
    --table-odd-background-fill: var(--bg-secondary) !important;
    --code-background-fill: var(--bg-tertiary) !important;
    --shadow-drop: var(--shadow-sm) !important;
    --shadow-drop-lg: var(--shadow-md) !important;
}

/* ── Global Reset & Typography ── */
.gradio-container, .gradio-container * {
    color-scheme: light;
}
.dark .gradio-container, [data-theme="dark"] .gradio-container {
    color-scheme: dark;
}
.gradio-container {
    font-family: -apple-system, BlinkMacSystemFont, 'SF Pro Display',
                 'Inter', 'SF Pro Text', 'PingFang SC', 'Hiragino Sans GB',
                 'Microsoft YaHei', system-ui, sans-serif !important;
    background: var(--bg-primary) !important;
    color: var(--text-primary) !important;
    max-width: 1200px !important;
    margin: 0 auto !important;
    padding: var(--space-xl) var(--space-2xl) !important;
    line-height: 1.5 !important;
    -webkit-font-smoothing: antialiased;
    -moz-osx-font-smoothing: grayscale;
    transition: background var(--transition-smooth), color var(--transition-smooth);
}

/* ── Typography (Apple HIG Scale) ── */
.gradio-container h1 {
    font-size: var(--text-large-title) !important;
    font-weight: 700 !important;
    letter-spacing: -0.022em !important;
    color: var(--text-primary) !important;
    margin: var(--space-sm) 0 var(--space-sm) 0 !important;
}
.gradio-container h2 {
    font-size: var(--text-title2) !important;
    font-weight: 600 !important;
    letter-spacing: -0.018em !important;
    color: var(--text-primary) !important;
    margin: var(--space-lg) 0 var(--space-sm) 0 !important;
}
.gradio-container h3 {
    font-size: var(--text-title3) !important;
    font-weight: 600 !important;
    color: var(--text-secondary) !important;
    margin-top: var(--space-md) !important;
}
.gradio-container p, .gradio-container li {
    color: var(--text-secondary) !important;
    font-size: var(--text-body) !important;
}

/* ── Glass Cards ── */
.block, .form, .panel, .gr-box {
    background: var(--bg-glass) !important;
    backdrop-filter: blur(20px) saturate(180%);
    -webkit-backdrop-filter: blur(20px) saturate(180%);
    border: 1px solid var(--border-primary) !important;
    border-radius: var(--radius-lg) !important;
    box-shadow: var(--shadow-sm) !important;
    transition: box-shadow var(--transition-smooth),
                transform var(--transition-smooth),
                border-color var(--transition-smooth);
}
.block:hover {
    box-shadow: var(--shadow-md) !important;
}

/* ── Tab Bar (macOS Segmented Control Style) ── */
.tabs {
    border: none !important;
    gap: 0 !important;
}
.tab-nav {
    background: var(--bg-tertiary) !important;
    border-radius: var(--radius-md) !important;
    padding: 3px !important;
    display: flex !important;
    gap: 2px !important;
    margin-bottom: var(--space-lg) !important;
}
.tab-nav button {
    background: transparent !important;
    border: none !important;
    border-radius: var(--radius-sm) !important;
    padding: 8px 18px !important;
    font-size: var(--text-subhead) !important;
    font-weight: 500 !important;
    color: var(--text-secondary) !important;
    transition: all var(--transition-fast);
    cursor: pointer;
    flex: 1;
    text-align: center;
}
.tab-nav button:hover {
    color: var(--text-primary) !important;
    background: var(--bg-elevated) !important;
}
.tab-nav button.selected {
    background: var(--bg-secondary) !important;
    color: var(--text-primary) !important;
    font-weight: 600 !important;
    box-shadow: var(--shadow-sm) !important;
}

/* ── Buttons (Apple Style) ── */
button, .gr-button {
    border-radius: var(--radius-md) !important;
    font-weight: 500 !important;
    font-size: var(--text-subhead) !important;
    letter-spacing: -0.01em !important;
    transition: all var(--transition-fast) !important;
    cursor: pointer;
}
button.primary, button[variant="primary"], .gr-button-primary {
    background: var(--accent) !important;
    color: #ffffff !important;
    border: none !important;
    box-shadow: var(--shadow-sm) !important;
}
button.primary:hover {
    background: var(--accent-hover) !important;
    box-shadow: var(--shadow-md) !important;
    transform: translateY(-0.5px);
}
button.primary:active {
    transform: scale(0.98);
}
button.secondary, button[variant="secondary"], .gr-button-secondary {
    background: var(--bg-secondary) !important;
    color: var(--text-primary) !important;
    border: 1px solid var(--border-primary) !important;
    box-shadow: var(--shadow-sm) !important;
}
button.secondary:hover {
    background: var(--bg-tertiary) !important;
    border-color: var(--accent) !important;
}

/* ── Input Fields (macOS Style) ── */
.gradio-container textarea,
.gradio-container input[type="text"],
.gradio-container input[type="number"],
.gradio-container input[type="password"],
.gradio-container select {
    background: var(--gradio-input) !important;
    border: 1px solid var(--border-primary) !important;
    border-radius: var(--radius-md) !important;
    font-size: var(--text-body) !important;
    padding: 8px 12px !important;
    transition: all var(--transition-fast);
    color: var(--text-primary) !important;
}
.gradio-container textarea:focus,
.gradio-container input:focus,
.gradio-container select:focus {
    border-color: var(--accent) !important;
    box-shadow: 0 0 0 3px var(--border-focus) !important;
    outline: none !important;
}

/* ── Tables (Apple Numbers Style) ── */
.gradio-container table {
    border-collapse: collapse !important;
    font-size: var(--text-subhead) !important;
    width: 100%;
}
.gradio-container th {
    background: var(--bg-tertiary) !important;
    font-weight: 600 !important;
    color: var(--text-secondary) !important;
    text-transform: uppercase;
    font-size: var(--text-callout) !important;
    letter-spacing: 0.04em;
    border-bottom: 1px solid var(--border-primary) !important;
    padding: 10px 14px !important;
}
.gradio-container td {
    border-bottom: 1px solid var(--border-secondary) !important;
    padding: 10px 14px !important;
    color: var(--text-primary) !important;
}
.gradio-container tr:hover td {
    background: var(--accent-muted) !important;
}

/* ── Code Blocks (Xcode Style) ── */
.gradio-container pre, .gradio-container code {
    background: var(--bg-tertiary) !important;
    border-radius: var(--radius-md) !important;
    font-family: 'SF Mono', 'JetBrains Mono', 'Menlo', 'Consolas', monospace !important;
    font-size: var(--text-callout) !important;
}
.gradio-container :not(pre) > code {
    background: var(--accent-muted) !important;
    color: var(--accent) !important;
    padding: 2px 6px !important;
    border-radius: 4px !important;
    font-size: var(--text-callout) !important;
}

/* ── Dividers ── */
.gradio-container hr {
    border: none !important;
    border-top: 1px solid var(--border-primary) !important;
    margin: var(--space-lg) 0 !important;
}

/* ── Markdown Content ── */
.gradio-container .prose, .gradio-container .md {
    color: var(--text-primary) !important;
}

/* ── Status Indicators ── */
.status-dot {
    display: inline-block;
    width: 8px; height: 8px;
    border-radius: 50%;
    margin-right: 6px;
    animation: pulse 2s ease-in-out infinite;
}
.status-dot.ok { background: var(--success); }
.status-dot.warn { background: var(--warning); }
.status-dot.err { background: var(--danger); }
@keyframes pulse {
    0%, 100% { opacity: 1; }
    50% { opacity: 0.5; }
}

/* ── Skeleton Loading ── */
@keyframes shimmer {
    0% { background-position: -200px 0; }
    100% { background-position: 200px 0; }
}
.skeleton {
    background: linear-gradient(90deg,
        var(--bg-tertiary) 25%,
        var(--bg-secondary) 50%,
        var(--bg-tertiary) 75%
    ) !important;
    background-size: 400px 100% !important;
    animation: shimmer 1.5s ease-in-out infinite;
    border-radius: var(--radius-sm);
}

/* ── Toast / Alert ── */
.toast {
    border-radius: var(--radius-md) !important;
    padding: 12px 16px !important;
    font-size: var(--text-subhead) !important;
    font-weight: 500 !important;
    box-shadow: var(--shadow-lg) !important;
    backdrop-filter: blur(20px);
}
.toast-success { background: rgba(52,199,89,0.12) !important; border: 1px solid rgba(52,199,89,0.3) !important; color: var(--success) !important; }
.toast-warning { background: rgba(255,149,0,0.12) !important; border: 1px solid rgba(255,149,0,0.3) !important; color: var(--warning) !important; }
.toast-error { background: rgba(255,59,48,0.12) !important; border: 1px solid rgba(255,59,48,0.3) !important; color: var(--danger) !important; }

/* ── Theme Toggle Button ── */
#theme-toggle {
    position: fixed;
    top: 16px;
    right: 16px;
    z-index: 9999;
    width: 36px; height: 36px;
    border-radius: var(--radius-full);
    background: var(--bg-glass);
    backdrop-filter: blur(20px);
    border: 1px solid var(--border-primary);
    color: var(--text-primary);
    cursor: pointer;
    display: flex;
    align-items: center;
    justify-content: center;
    font-size: 16px;
    transition: all var(--transition-fast);
    box-shadow: var(--shadow-sm);
}
#theme-toggle:hover {
    box-shadow: var(--shadow-md);
    transform: scale(1.05);
}

/* ── Responsive ── */
@media (max-width: 768px) {
    .gradio-container {
        padding: var(--space-md) var(--space-md) !important;
        max-width: 100% !important;
    }
    .gradio-container h1 { font-size: var(--text-title1) !important; }
    .tab-nav { flex-direction: column; }
    .tab-nav button { padding: 6px 12px !important; font-size: var(--text-callout) !important; }
}
@media (max-width: 480px) {
    .gradio-container { padding: var(--space-sm) !important; }
}
"""

# ═══════════════════════════════════════════════════════════════════
# Theme Builders
# ═══════════════════════════════════════════════════════════════════

def build_theme() -> "gr.themes.Soft":
    """Apple-Claude 混合风格主题。"""
    return gr.themes.Soft(
        primary_hue=gr.themes.Color(
            c50="#fcf5f1", c100="#f9ebe4", c200="#f0d4c5", c300="#e6b89f",
            c400="#dc9d7a", c500="#cc785c", c600="#b85e44", c700="#94472f",
            c800="#6f3322", c900="#4a2117", c950="#2a120c",
        ),
        secondary_hue=gr.themes.Color(
            c50="#f0f7ff", c100="#d6e8ff", c200="#b3d4ff", c300="#80bdff",
            c400="#4da3ff", c500="#0071e3", c600="#0060c0", c700="#004d9e",
            c800="#003a7a", c900="#002856", c950="#001833",
        ),
        neutral_hue=gr.themes.Color(
            c50="#f5f5f7", c100="#e8e8ed", c200="#d2d2d7", c300="#aeaeb2",
            c400="#8e8e93", c500="#636366", c600="#48484a", c700="#3a3a3c",
            c800="#2c2c2e", c900="#1c1c1e", c950="#0a0a0a",
        ),
        font=[
            gr.themes.GoogleFont("Inter"),
            "-apple-system", "BlinkMacSystemFont", "SF Pro Display",
            "ui-sans-serif", "system-ui", "sans-serif",
        ],
        font_mono=[
            gr.themes.GoogleFont("JetBrains Mono"),
            "SF Mono", "ui-monospace", "Menlo", "Consolas", "monospace",
        ],
        radius_size=gr.themes.Size(sm="6px", md="10px", lg="14px"),
        spacing_size=gr.themes.Size(sm="8px", md="16px", lg="24px"),
    )


def build_theme_compact() -> "gr.themes.Soft":
    """紧凑版主题（适合移动端或嵌入式场景）。"""
    return gr.themes.Soft(
        primary_hue=gr.themes.Color(
            c50="#fcf5f1", c100="#f9ebe4", c200="#f0d4c5", c300="#e6b89f",
            c400="#dc9d7a", c500="#cc785c", c600="#b85e44", c700="#94472f",
            c800="#6f3322", c900="#4a2117", c950="#2a120c",
        ),
        neutral_hue=gr.themes.Color(
            c50="#f5f5f7", c100="#e8e8ed", c200="#d2d2d7", c300="#aeaeb2",
            c400="#8e8e93", c500="#636366", c600="#48484a", c700="#3a3a3c",
            c800="#2c2c2e", c900="#1c1c1e", c950="#0a0a0a",
        ),
        font=[
            gr.themes.GoogleFont("Inter"),
            "system-ui", "sans-serif",
        ],
        font_mono=[
            gr.themes.GoogleFont("JetBrains Mono"),
            "monospace",
        ],
        radius_size=gr.themes.Size(sm="4px", md="8px", lg="12px"),
        spacing_size=gr.themes.Size(sm="6px", md="12px", lg="20px"),
    )


# ── Theme toggle JavaScript ────────────────────────────────────────
THEME_TOGGLE_JS = """
<script>
(function() {
    const STORAGE_KEY = 'smdsl-theme';
    const saved = localStorage.getItem(STORAGE_KEY);
    if (saved === 'dark') {
        document.documentElement.setAttribute('data-theme', 'dark');
        document.documentElement.classList.add('dark');
    }

    function createToggle() {
        const btn = document.createElement('button');
        btn.id = 'theme-toggle';
        btn.innerHTML = saved === 'dark' ? '☀️' : '🌙';
        btn.title = 'Toggle dark mode (D)';
        btn.onclick = function() {
            const isDark = document.documentElement.classList.contains('dark');
            if (isDark) {
                document.documentElement.classList.remove('dark');
                document.documentElement.removeAttribute('data-theme');
                btn.innerHTML = '🌙';
                localStorage.setItem(STORAGE_KEY, 'light');
            } else {
                document.documentElement.classList.add('dark');
                document.documentElement.setAttribute('data-theme', 'dark');
                btn.innerHTML = '☀️';
                localStorage.setItem(STORAGE_KEY, 'dark');
            }
        };
        document.body.appendChild(btn);
    }

    // Keyboard shortcut: 'D' toggles theme
    document.addEventListener('keydown', function(e) {
        if (e.key === 'd' && e.metaKey === false && e.ctrlKey === false &&
            e.target.tagName !== 'INPUT' && e.target.tagName !== 'TEXTAREA') {
            const btn = document.getElementById('theme-toggle');
            if (btn) btn.click();
        }
    });

    if (document.readyState === 'loading') {
        document.addEventListener('DOMContentLoaded', createToggle);
    } else {
        createToggle();
    }
})();
</script>
"""
