"""
test_demo3_pipeline.py — 烟囱测试：metrics.run_diagnostic_pipeline + topology_bundle

验证 Demo 2 LLM 反馈循环（依赖 Demo 3 求解器）所走的完整路径：
  RoboIR JSON → RustCompilerStub.validate → STL 约束检测 (距离场)
  → FailureTaxonomy 分类 → generate_structured_feedback。

预期输出：1 条 COLLISION 反馈（穿过基本柜的轨迹）；
         无编译期错误；零 STL_VIOLATION（应被精细分类为 COLLISION）。
"""

from __future__ import annotations

import io
import json
import sys
from pathlib import Path

# 强制 UTF-8 stdout，避免 Windows GBK 编码错误
if sys.stdout.encoding and sys.stdout.encoding.lower() != "utf-8":
    sys.stdout = io.TextIOWrapper(sys.stdout.buffer, encoding="utf-8")

_REPO_ROOT = Path(__file__).resolve().parent.parent
if str(_REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(_REPO_ROOT))

from cad_parser.astar_topology import run_pipeline, to_topology_bundle  # noqa: E402
from smdsl_demo.demo3_robustness import synth_trajectories  # noqa: E402
from smdsl_demo.metrics import run_diagnostic_pipeline  # noqa: E402


# 一个最小可编译的 RoboIR
MINIMAL_ROBO_IR = json.dumps({
    "version": "0.1",
    "task_id": "demo3_test_kitchen_24",
    "frame_declarations": [
        {"id": "map", "type": "world"}
    ],
    "pose_declarations": [
        {"label": "P_door", "frame": "map",
         "position": {"x": 4.10, "y": 2.10, "z": 0.0},
         "orientation": {"roll": 0, "pitch": 0, "yaw": 0}},
        {"label": "P_goal", "frame": "map",
         "position": {"x": 0.30, "y": 1.50, "z": 0.0},
         "orientation": {"roll": 0, "pitch": 0, "yaw": 0}},
    ],
    "actions": [
        {
            "id": 0, "type": "navigate",
            "target_pose": "P_goal",
            "constraints": [
                {"type": "stl_constraint",
                 "expr": "Distance > 0.300",
                 "unit": "m",
                 "reference": "obstacle"}
            ]
        }
    ],
})


def main() -> int:
    print("[setup] running Demo 1 to obtain distance field ...")
    pipe = run_pipeline(
        "data/cad_samples/floorplanqa/layouts/kitchen/room_24.json",
        resolution=0.05, robot_radius_m=0.25, include_grid=False,
    )
    bundle = to_topology_bundle(pipe)

    constraints = [{
        "type": "stl_constraint",
        "expr": "Distance > 0.300",
        "unit": "m",
        "reference": "obstacle",
    }]

    trajs = synth_trajectories(pipe["cad_data"])

    print("\n[run] run_diagnostic_pipeline on each trajectory:")
    for name, traj in trajs.items():
        feedback_list = run_diagnostic_pipeline(
            MINIMAL_ROBO_IR,
            traj, constraints,
            topology_bundle=bundle,
        )
        types = [fb["error"]["type"] for fb in feedback_list]
        sevs = [fb["error"]["severity"] for fb in feedback_list]
        print(f"  {name:18s} → n_feedback={len(feedback_list):2d}  "
              f"types={types}  severities={sevs}")

        for fb in feedback_list:
            ctx = fb.get("context", {})
            if ctx.get("worst_node"):
                wn = ctx["worst_node"]
                print(f"      worst node: t={wn['t']:.2f}  "
                      f"({wn.get('x', 0):.2f},{wn.get('y', 0):.2f})  "
                      f"ρ={wn.get('rho', 0):+.3f} m")

    # 断言（成功的最低标准）
    fb_safe = run_diagnostic_pipeline(
        MINIMAL_ROBO_IR, trajs["A_SAFE"], constraints,
        topology_bundle=bundle,
    )
    fb_through = run_diagnostic_pipeline(
        MINIMAL_ROBO_IR, trajs["C_THROUGH_OBJ"], constraints,
        topology_bundle=bundle,
    )
    assert all(fb["error"]["type"] != "collision" for fb in fb_safe), \
        "A_SAFE should NOT raise collision"
    assert any(fb["error"]["type"] == "collision" for fb in fb_through), \
        "C_THROUGH_OBJ MUST raise collision (FailureTaxonomy.COLLISION)"
    print("\n[ok] all assertions passed ✓")
    return 0


if __name__ == "__main__":
    sys.exit(main())
