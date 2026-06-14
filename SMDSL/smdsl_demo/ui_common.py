"""
ui_common.py — SMDSL Gradio UI 共享工具函数 (Phase 3.1)

从 app.py 拆分：流程导航、种子标签格式化、时间戳。
"""

import datetime as _dt


def flow_nav_md(active: int) -> str:
    """返回当前 Tab 的进度条 HTML（顶部紧凑版，节点可点击跳转）。

    Args:
        active: 当前 Demo 序号 (1/2/3)，0 表示全局头部（无激活节点）。
    """
    nodes = [
        (0, "自然语言 + CAD", "用户输入",      None),
        (1, "● 环境感知",    "Demo 1 · 距离场 / 拓扑", 0),
        (2, "语义编译",      "Demo 2 · STL RoboIR",   1),
        (3, "物理求解",      "Demo 3 · ρ + 反馈",     2),
    ]
    cells = []
    for idx, name, sub, tab_idx in nodes:
        if idx == active and idx != 0:
            style = (
                "background:#fdf3ee;color:#94472f;font-weight:600;"
                "border:1px solid #cc785c;"
            )
        elif idx == 0:
            style = (
                "background:#ffffff;color:#6c6a65;font-weight:500;"
                "border:1px dashed #d9d6d0;"
            )
        elif idx < active:
            style = (
                "background:#f7f5f1;color:#2a2a2a;"
                "border:1px solid #ece9e3;font-weight:500;"
            )
        else:
            style = (
                "background:#ffffff;color:#a8a59f;font-weight:400;"
                "border:1px solid #ece9e3;"
            )

        _demo_labels = ["Demo 1", "Demo 2", "Demo 3"]
        if tab_idx is not None:
            _kw = _demo_labels[tab_idx]
            onclick = (
                f"onclick=\"(function(){{"
                f"var tabs=Array.from(document.querySelectorAll('[role=tab]'));"
                f"var t=tabs.find(function(el){{return el.textContent.indexOf('{_kw}')>-1;}});"
                f"if(t)t.click();"
                f"}})()\" "
            )
            extra_style = "cursor:pointer;transition:box-shadow .15s;"
            hover_title = f"title='点击跳转到 {_kw}'"
        else:
            onclick = ""
            extra_style = ""
            hover_title = ""

        cells.append(
            f"<div {onclick}{hover_title} style='flex:1;text-align:center;"
            f"padding:10px 12px;border-radius:10px;{style}{extra_style}"
            f"font-size:13px;line-height:1.4'>"
            f"<div>{name}</div>"
            f"<div style='font-size:11px;opacity:.7;margin-top:3px;"
            f"font-weight:400'>{sub}</div>"
            "</div>"
        )
    arrow = (
        "<div style='align-self:center;color:#cfcbc2;font-size:14px;"
        "padding:0 6px'>→</div>"
    )
    inner = arrow.join(cells)
    return (
        "<div class='smdsl-flow-nav' style='display:flex;"
        "align-items:stretch;gap:0;margin:6px 0 16px 0;'>" + inner + "</div>"
    )


def format_seed_label(idx: int, kind: str, x: float, y: float,
                      extra: str = "") -> str:
    """统一格式化候选种子点的下拉标签。"""
    base = f"#{idx} [{kind}]  ({x:.2f}, {y:.2f}) m"
    return f"{base}  · {extra}" if extra else base


def current_timestamp() -> str:
    return _dt.datetime.now().strftime("%H:%M:%S")
