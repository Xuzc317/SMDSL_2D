# Changelog

## [0.2.0] — 2026-06-15

### Added (P0)
- **Z-axis hard assertion**: `_eval_distance_via_field` rejects 3D trajectories (delta_z > 0.01m) with robustness = -inf (`spatial_api_stub.py`)
- **RoboIR diff check**: `run_correction_loop` blocks LLM from modifying intent/target_frame; re-prompts with hard constraint (`test_closed_loop_recovery.py`)
- **DWG entity drop rate warning**: `_extract_dwg_geometry` tracks per-type skip stats; UI shows yellow banner when drop rate > 10% (`dispatcher.py`, `app.py`)

### Added (P1)
- **Trajectory smoother**: cubic spline + trapezoidal velocity profile replaces uniform-time path-to-trajectory conversion (`trajectory_smoother.py`)
- **Gradient path refinement**: EDT gradient nudges path interior points away from obstacles with 5-iteration gradient ascent (`astar_topology.py:refine_path_via_gradient`)
- **EDT vs Costmap benchmark**: synthetic + FloorplanQA comparison framework with clearance_min/mean/time metrics (`benchmark/bench_edt_vs_costmap.py`)

### Added (Tests)
- `tests/test_z_axis.py` — 5 tests (z-axis rejection)
- `tests/test_entity_stats.py` — 9 tests (DWG entity counting)
- `tests/test_trajectory_smoother.py` — 11 tests (trapezoidal profile + spline)
- `tests/test_gradient_refine.py` — 7 tests (gradient field + path refinement)
- `tests/test_roboir_diff.py` — 6 tests (intent/target_frame invariance)

### Changed
- `app.py`: `demo1_plan_path` pipeline now runs A* → gradient refine → smooth trajectory
- `app.py`: `demo1_run` shows DWG drop rate warning when applicable
- `test_closed_loop_recovery.py`: uses `smooth_path_to_trajectory` for replanning

### Added (Structure)
- `smdsl_demo/trajectory_smoother.py` — trajectory synthesis module
- `smdsl_demo/ui_theme.py` — CSS + theme extraction from app.py
- `smdsl_demo/ui_common.py` — shared UI utilities
- `benchmark/bench_edt_vs_costmap.py` — comparison benchmark
- `benchmark/results/` — benchmark output directory

## [0.1.0] — legacy

- Initial 4-Zone architecture (cad_parser, vlm_parser, spatial_api_stub, metrics)
- FloorplanQA integration (1981 layouts, 29.87ms avg, 0 errors)
- Closed-loop recovery test
- Gradio 3-tab debugging UI
