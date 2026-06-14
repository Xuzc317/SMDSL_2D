"""
app_tabs.py — SMDSL_2D Tab Extensions

Monkey-patches the original app.py to add Architecture and Demo Recording tabs.
Run this instead of app.py directly to get the extended interface.

Usage:
    python -m smdsl_demo.app_tabs
"""

from __future__ import annotations

import os
import sys
import tempfile
from pathlib import Path

_OUT_DIR = Path(tempfile.gettempdir()) / "smdsl_ui_cache"
_OUT_DIR.mkdir(parents=True, exist_ok=True)


def create_standalone_arch_page() -> str:
    """Create a standalone HTML file for the architecture diagram.
    Returns the file path.
    """
    from smdsl_demo.architecture_viz import get_architecture_html
    html = get_architecture_html()
    out_path = _OUT_DIR / "smdsl_architecture.html"
    out_path.write_text(html, encoding="utf-8")
    return str(out_path)


def create_standalone_arch_gradio() -> None:
    """Launch the architecture diagram as a standalone Gradio app."""
    import gradio as gr
    from smdsl_demo.architecture_viz import get_architecture_html

    with gr.Blocks(title="SMDSL_2D Architecture") as demo:
        gr.HTML(get_architecture_html())
    demo.launch(server_port=7861, share=False, inbrowser=True)


def create_standalone_demo_recorder_gradio() -> None:
    """Launch the demo recorder as a standalone Gradio app."""
    import gradio as gr
    from smdsl_demo.demo_recorder import record_demo_video

    with gr.Blocks(title="SMDSL_2D Demo Recorder") as demo:
        gr.Markdown("# SMDSL_2D Demo Recorder")
        gr.Markdown(
            "Record a video walkthrough of the SMDSL pipeline. "
            "Requires Playwright installed."
        )
        with gr.Row():
            url_box = gr.Textbox(
                label="App URL", value="http://127.0.0.1:7860", scale=3
            )
            scenario_dd = gr.Dropdown(
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
        rec_btn = gr.Button("Record Demo", variant="primary")
        status_md = gr.Markdown("*Ready...*")
        video_out = gr.Video(label="Recorded Demo")

        rec_btn.click(
            fn=_run_recording_wrapper,
            inputs=[url_box, scenario_dd],
            outputs=[video_out, status_md],
        )
    demo.launch(server_port=7862, share=False, inbrowser=True)


def _run_recording_wrapper(app_url: str, scenario_label: str) -> tuple:
    from smdsl_demo.demo_recorder import record_demo_video as _rdv
    scenario_map = {
        "full (end-to-end)": "full",
        "cad (parsing only)": "cad",
        "compile (semantic only)": "compile",
        "verify (stl only)": "verify",
        "all (all scenarios)": "all",
    }
    sc = scenario_map.get(scenario_label, "full")
    try:
        results = _rdv(app_url=app_url, scenario=sc)
        paths = list(results.values())
        if paths and paths[0]:
            return (paths[0], f"Recording complete! Saved to: {paths[0]}")
        return (None, "No video recorded.")
    except ImportError as e:
        return (None, f"Playwright not installed: {e}")
    except Exception as e:
        return (None, f"Recording failed: {e}")


def launch_extended() -> None:
    """Launch the original SMDSL app with an architecture page link and info."""
    import gradio as gr
    from smdsl_demo.app import build_ui as original_build_ui

    # We can't nest gr.Blocks, so we offer a companion tool approach
    # Generate the architecture HTML file
    arch_path = create_standalone_arch_page()

    print(f"\n{'='*60}")
    print("SMDSL Extended Launcher")
    print(f"{'='*60}")
    print(f"\nArchitecture diagram saved to: {arch_path}")
    print("\nAvailable companion services:")
    print("  - Architecture: python -c \"from smdsl_demo.app_tabs import create_standalone_arch_gradio; create_standalone_arch_gradio()\"")
    print("  - Demo Recorder: python -c \"from smdsl_demo.app_tabs import create_standalone_demo_recorder_gradio; create_standalone_demo_recorder_gradio()\"")
    print("\nLaunching original SMDSL app...\n")

    demo = original_build_ui()
    demo.launch(share=False, show_error=True)


def main():
    import argparse
    parser = argparse.ArgumentParser(description="SMDSL Extended Tools")
    parser.add_argument("mode", nargs="?", default="app",
                        choices=["app", "arch", "recorder", "help"],
                        help="Mode to run")
    parser.add_argument("--port", type=int, default=7860, help="Port")
    args = parser.parse_args()

    if args.mode == "help":
        print("SMDSL Extended Tools")
        print("  app      - Run original SMDSL app (with companion info)")
        print("  arch     - Launch architecture diagram as standalone page")
        print("  recorder - Launch demo recorder as standalone page")
    elif args.mode == "arch":
        create_standalone_arch_gradio()
    elif args.mode == "recorder":
        create_standalone_demo_recorder_gradio()
    else:
        launch_extended()


if __name__ == "__main__":
    main()
