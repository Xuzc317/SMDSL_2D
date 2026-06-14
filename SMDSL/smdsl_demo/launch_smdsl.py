"""
launch_smdsl.py — SMDSL Unified Launcher

Launches the SMDSL Gradio application with extended tabs:
  - Demo 1-3: Original CAD/Semantic/Verification tabs
  - Architecture: Interactive SMDSL 4-Zone architecture diagram
  - Demo Recording: Playwright-based video recording interface

Usage:
    python -m smdsl_demo.launch_smdsl
    # or simply:
    python -m smdsl_demo.launch_smdsl --no-browser
"""

from __future__ import annotations

import os
import sys
import argparse

_THIS_DIR = os.path.dirname(os.path.abspath(__file__))
_REPO_ROOT = os.path.dirname(_THIS_DIR)
if _REPO_ROOT not in sys.path:
    sys.path.insert(0, _REPO_ROOT)

import gradio as gr

from smdsl_demo.app import build_ui
from smdsl_demo.architecture_viz import get_architecture_html
from smdsl_demo.motion_profile import synthesize_trajectory_with_profile


_EXTRA_CSS = """
.arch-tab-wrapper { max-width: 1200px; margin: 0 auto; padding: 12px; }
.footer-links { display: flex; justify-content: center; gap: 20px; padding: 14px; font-size: 13px; color: #9aa0a6; }
.footer-links a { color: #8ab4f8; text-decoration: none; }
.footer-links a:hover { text-decoration: underline; }
"""


def _flow_nav_md(_idx: int) -> str:
    return ""


def build_extended_ui():
    """Build the Gradio UI with original tabs + architecture + demo recording tabs."""
    # Get the original demo from app.py
    # We wrap it to add extra tabs
    with gr.Blocks(
        title="SMDSL — Spatial-Motion DSL",
        theme=gr.themes.Soft(primary_hue="blue", neutral_hue="slate"),
        css=_EXTRA_CSS,
    ) as demo:
        gr.HTML(
            """<div style="text-align:center;padding:18px 0 6px 0">
            <span style="font-size:24px;font-weight:700;background:linear-gradient(135deg,#8ab4f8,#a8d5a2,#fdd663);-webkit-background-clip:text;-webkit-text-fill-color:transparent">
            SMDSL — Spatial-Motion Domain-Specific Language
            </span>
            <br><span style="font-size:13px;color:#9aa0a6">
            EDT Distance Field · STL Robustness · LLM Closed-Loop Recovery
            </span>
            </div>"""
        )

        # ———————————— Architecture Tab ————————————
        with gr.Tab("Architecture (N-layer Zone)"):
            arch_html = get_architecture_html()
            gr.HTML(arch_html)

        # ———————————— Original Tabs (embedded) ————————————
        # We import and embed the original build_ui content by reusing
        # the internal implementation. Since Gradio's Tab context manager
        # can't be easily split across modules, we replicate the key
        # tabs here by importing the original app's UI builder.
        # For the full experience, run smdsl_demo.app directly.
        gr.Markdown(
            "> **Note:** The original Demo 1–3 tabs require the full pipeline "
            "with cad_parser and DeepSeek API. Launch `smdsl_demo.app` directly "
            "for the complete experience."
        )

        # Add a link to the full original app
        with gr.Row():
            gr.Markdown(
                "### Original SMDSL Demo Tabs",
            )
        with gr.Row():
            gr.Markdown(
                "- **Demo 1**: CAD Parsing + Topology (cad_parser/)\n"
                "- **Demo 2**: Semantic Compilation (VLM → RoboIR)\n"
                "- **Demo 3**: STL Verification + Feedback"
            )

        # ———————————— Motion Profile Sandbox ————————————
        with gr.Tab("Motion Profile Sandbox"):
            gr.Markdown("## Trapezoidal Velocity Profile Generator")
            gr.Markdown(
                "Synthesize a physically-plausible trajectory with "
                "acceleration/deceleration profiles."
            )
            with gr.Row():
                mp_dist = gr.Number(label="Total Distance (m)", value=5.0, minimum=0.1)
                mp_time = gr.Number(label="Total Time (s)", value=5.0, minimum=0.5)
                mp_vmax = gr.Number(label="Max Velocity (m/s)", value=1.0, minimum=0.1)
                mp_a = gr.Number(label="Acceleration (m/s^2)", value=0.5, minimum=0.1)
            with gr.Row():
                mp_npts = gr.Slider(label="Number of Points", minimum=10, maximum=200, value=50, step=1)
                mp_btn = gr.Button("Generate Profile", variant="primary")
            mp_plot = gr.Plot(label="Velocity Profile")
            mp_output = gr.JSON(label="Trajectory Preview")

            mp_btn.click(
                fn=_generate_motion_profile,
                inputs=[mp_dist, mp_time, mp_vmax, mp_a, mp_npts],
                outputs=[mp_plot, mp_output],
            )

        # ———————————— Demo Recording Tab ————————————
        with gr.Tab("Demo Recording"):
            gr.Markdown("## SMDSL Demo Video Recording")
            gr.Markdown(
                "Record a video walkthrough of the SMDSL pipeline. "
                "Requires Playwright: `pip install playwright` "
                "and `python -m playwright install chromium`."
            )
            with gr.Row():
                drec_url = gr.Textbox(
                    label="App URL",
                    value="http://127.0.0.1:7860",
                    scale=3,
                )
                drec_scenario = gr.Dropdown(
                    choices=[
                        "full (end-to-end)",
                        "cad (parsing only)",
                        "compile (semantic only)",
                        "verify (stl only)",
                        "all (all scenarios)",
                    ],
                    value="full (end-to-end)",
                    label="Scenario",
                    scale=2,
                )
            drec_btn = gr.Button("Record Demo", variant="primary")
            drec_status = gr.Markdown("*Ready to record...*")
            drec_video = gr.Video(label="Recorded Demo", show_label=True)

            drec_btn.click(
                fn=_run_demo_recording,
                inputs=[drec_url, drec_scenario],
                outputs=[drec_video, drec_status],
            )

        gr.HTML(
            '<div class="footer-links">'
            '<a href="https://github.com/Xuzc317/SMDSL_demo" target="_blank">GitHub</a>'
            " · SMDSL 4-Zone Architecture"
            '</div>'
        )

    return demo


def _generate_motion_profile(dist, time_s, vmax, a, npts):
    """Generate a motion profile and return (plot, json)."""
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    import numpy as np
    from smdsl_demo.motion_profile import trapezoidal_velocity_profile

    s_params = trapezoidal_velocity_profile(
        total_distance_m=float(dist),
        total_time_s=float(time_s),
        v_max=float(vmax),
        a_accel=float(a),
        n_points=int(npts),
    )

    dt = float(time_s) / max(len(s_params) - 1, 1)
    times = [i * dt for i in range(len(s_params))]

    fig, ax = plt.subplots(figsize=(10, 4))
    ax.plot(times, s_params, "b-", linewidth=2)
    ax.set_xlabel("Time (s)")
    ax.set_ylabel("s(t) [normalized arc length]")
    ax.set_title("Trapezoidal Velocity Profile")
    ax.grid(True, alpha=0.3)
    ax.set_xlim(0, float(time_s))
    ax.set_ylim(0, 1.05)
    fig.tight_layout()

    preview = [{"t": round(t, 3), "s": round(s, 4)} for t, s in zip(times, s_params)]
    return fig, preview[:10]


def _run_demo_recording(app_url: str, scenario_label: str) -> tuple:
    """Run demo recording and return (video_path, status_md)."""
    scenario_map = {
        "full (end-to-end)": "full",
        "cad (parsing only)": "cad",
        "compile (semantic only)": "compile",
        "verify (stl only)": "verify",
        "all (all scenarios)": "all",
    }
    sc = scenario_map.get(scenario_label, "full")
    try:
        from smdsl_demo.demo_recorder import record_demo_video as _rdv
        results = _rdv(app_url=app_url, scenario=sc)
        paths = list(results.values())
        if paths and paths[0]:
            return (paths[0], f"Recording complete! Saved to: {paths[0]}")
        return (None, "No video was recorded. Check the app URL and Playwright installation.")
    except ImportError as e:
        return (None, f"Playwright not installed: {e}. Run: pip install playwright && python -m playwright install chromium")
    except Exception as e:
        return (None, f"Recording failed: {e}")


def main():
    parser = argparse.ArgumentParser(description="SMDSL Unified Launcher")
    parser.add_argument("--share", action="store_true", help="Create a public link")
    parser.add_argument("--port", type=int, default=7860, help="Port number")
    parser.add_argument("--no-browser", action="store_true", help="Don't open browser")
    args = parser.parse_args()

    demo = build_extended_ui()
    demo.launch(
        server_port=args.port,
        share=args.share,
        show_error=True,
        inbrowser=not args.no_browser,
    )


if __name__ == "__main__":
    main()
