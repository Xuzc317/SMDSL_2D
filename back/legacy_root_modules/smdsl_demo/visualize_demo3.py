"""
visualize_demo3.py — Demo 3 交互式 3D 轨迹沙盘 (Plotly)

重构：从 matplotlib 静态 PNG → Plotly 3D 场景。

四层图元叠加：
  1. go.Surface    — 距离场地板热力图 @ Z=0
  2. go.Scatter3d  — 墙体线框（障碍物边缘多高度拉伸）
  3. go.Scatter3d  — 3D 悬浮轨迹 @ Z=1.0m（机器人执行器高度）
  4. go.Scatter3d  — 违规红色球体（ρ < 0 处高亮 + hover 详情）
"""

from __future__ import annotations

from typing import Any, Dict, List, Optional, Tuple

import numpy as np

try:
    import plotly.graph_objects as go
    HAS_PLOTLY = True
except ImportError:
    HAS_PLOTLY = False
    go = None  # type: ignore


# ── 内部工具 ──────────────────────────────────────────────

def _world_to_grid_xy(x_m: float, y_m: float, tx: Dict[str, Any]) -> Tuple[float, float]:
    origin = tx["origin"]
    res = tx["resolution"]
    return (x_m - origin[0]) / res, (y_m - origin[1]) / res


def _sample_distance_at(x_m: float, y_m: float, bundle: Dict[str, Any]) -> float:
    """双线性采样距离场，返回米。"""
    df = bundle["distance_field"]
    tx = bundle["grid_transform"]
    H, W = df.shape
    col_f, row_f = _world_to_grid_xy(x_m, y_m, tx)
    if not (0 <= row_f < H - 1 and 0 <= col_f < W - 1):
        return 0.0
    r0, c0 = int(row_f), int(col_f)
    dr, dc = row_f - r0, col_f - c0
    val_px = (
        df[r0, c0] * (1 - dr) * (1 - dc)
        + df[r0, c0 + 1] * (1 - dr) * dc
        + df[r0 + 1, c0] * dr * (1 - dc)
        + df[r0 + 1, c0 + 1] * dr * dc
    )
    return float(val_px) * float(tx["resolution"])


# ══════════════════════════════════════════════════════════════════════
# 3D 场景主函数
# ══════════════════════════════════════════════════════════════════════

def generate_3d_trajectory_scene(
    trajectory: List[Dict[str, float]],
    bundle: Optional[Dict[str, Any]] = None,
    topology: Optional[np.ndarray] = None,
    violation_nodes: Optional[List[Dict[str, Any]]] = None,
    min_distance_m: Optional[float] = None,
    wall_height: float = 2.5,
    title: str = "SMDSL 3D Trajectory Sandbox",
) -> "go.Figure":
    """
    生成交互式 3D 轨迹沙盘。

    Args:
        trajectory: [{t, x, y, ...}, ...] 轨迹点
        bundle: TopologyBundle 含 distance_field + grid_transform
        topology: 拓扑标签图 (CLASS_PATH=2 / OBSTACLE=3 / INFLATED=4)
        violation_nodes: [{t, x, y, d_real_m, rho}, ...] 违规节点
        min_distance_m: 安全距离阈值
        wall_height: 墙体拉伸高度（米）
        title: 图表标题

    Returns:
        plotly.graph_objects.Figure
    """
    if not HAS_PLOTLY:
        fig = go.Figure() if go else None
        raise ImportError("plotly 未安装。请运行: pip install plotly")

    fig = go.Figure()
    xs = [float(p.get("x", 0.0)) for p in trajectory]
    ys = [float(p.get("y", 0.0)) for p in trajectory]

    # ── Layer 1: 距离场地板 ────────────────────────────────
    if bundle is not None:
        df = bundle["distance_field"]
        tx = bundle["grid_transform"]
        H, W = df.shape
        res = float(tx["resolution"])
        ox, oy = float(tx["origin"][0]), float(tx["origin"][1])

        # 降采样以保证渲染性能（max ~200x200）
        ds = max(1, max(H, W) // 200)
        df_ds = df[::ds, ::ds]
        H_ds, W_ds = df_ds.shape
        df_meters = df_ds * res

        x_grid = np.linspace(ox, ox + W * res, W_ds)
        y_grid = np.linspace(oy, oy + H * res, H_ds)

        fig.add_trace(go.Surface(
            x=x_grid, y=y_grid, z=np.zeros_like(df_meters),
            surfacecolor=df_meters,
            colorscale="Viridis",
            colorbar=dict(title="Clearance (m)", x=1.02),
            name="Distance Field",
            showscale=True,
            hoverinfo="skip",
            contours_z=dict(
                show=True, usecolormap=True,
                highlightcolor="limegreen", project_z=False,
            ),
        ))

        # ── Layer 2: 墙体线框 ─────────────────────────────
        if topology is not None:
            _add_wall_wireframe(fig, topology, tx, wall_height=wall_height)

    # ── Layer 3: 3D 轨迹 ──────────────────────────────────
    traj_z = 1.0  # 执行器高度
    clearance_labels: List[str] = []
    for i, p in enumerate(trajectory):
        t = p.get("t", i)
        label = f"t={t:.2f}s<br>x={xs[i]:.3f}<br>y={ys[i]:.3f}"
        if bundle is not None:
            d = _sample_distance_at(xs[i], ys[i], bundle)
            label += f"<br>clearance={d:.4f}m"
        clearance_labels.append(label)

    fig.add_trace(go.Scatter3d(
        x=xs, y=ys, z=[traj_z] * len(xs),
        mode="lines+markers",
        line=dict(color="cyan", width=5),
        marker=dict(size=4, color="cyan", symbol="circle"),
        name="Trajectory (Z=1.0m)",
        text=clearance_labels,
        hoverinfo="text",
    ))

    # Start / End markers
    fig.add_trace(go.Scatter3d(
        x=[xs[0]], y=[ys[0]], z=[traj_z],
        mode="markers",
        marker=dict(size=8, color="lime", symbol="diamond"),
        name="Start",
        hovertext=f"Start: t={trajectory[0].get('t',0):.2f}s",
        hoverinfo="text",
    ))
    fig.add_trace(go.Scatter3d(
        x=[xs[-1]], y=[ys[-1]], z=[traj_z],
        mode="markers",
        marker=dict(size=10, color="yellow", symbol="cross"),
        name="End",
        hovertext=f"End: t={trajectory[-1].get('t', len(trajectory)-1):.2f}s",
        hoverinfo="text",
    ))

    # ── Layer 4: 违规高亮 ─────────────────────────────────
    if violation_nodes and len(violation_nodes) > 0:
        vx = [float(v.get("x", 0)) for v in violation_nodes]
        vy = [float(v.get("y", 0)) for v in violation_nodes]
        vz = [traj_z] * len(violation_nodes)
        vhovers = []
        for v in violation_nodes:
            t = v.get("t", "?")
            rho = v.get("rho", 0)
            d_real = v.get("d_real_m", 0)
            vhovers.append(
                f"<b>VIOLATION</b><br>"
                f"t={t}s<br>rho={rho:.4f}m<br>"
                f"d_real={d_real:.4f}m"
            )
        fig.add_trace(go.Scatter3d(
            x=vx, y=vy, z=vz,
            mode="markers",
            marker=dict(
                size=12, color="red", symbol="circle",
                line=dict(color="darkred", width=2),
            ),
            name=f"Violations ({len(violation_nodes)})",
            text=vhovers,
            hoverinfo="text",
        ))

        # 最深违规点加大标注
        worst = min(violation_nodes, key=lambda v: float(v.get("rho", 0)))
        fig.add_trace(go.Scatter3d(
            x=[float(worst.get("x", 0))],
            y=[float(worst.get("y", 0))],
            z=[traj_z],
            mode="markers+text",
            marker=dict(size=18, color="darkred", symbol="x"),
            name="Worst Violation",
            text=[f"WORST rho={float(worst.get('rho',0)):.4f}m"],
            textposition="top center",
            hoverinfo="text",
        ))

    # ── Layout ─────────────────────────────────────────────
    if bundle is not None:
        tx = bundle["grid_transform"]
        x_range = [float(tx["origin"][0]), float(tx["origin"][0]) + tx["shape"][1] * float(tx["resolution"])]
        y_range = [float(tx["origin"][1]), float(tx["origin"][1]) + tx["shape"][0] * float(tx["resolution"])]
    else:
        margin = 1.0
        x_range = [min(xs) - margin, max(xs) + margin]
        y_range = [min(ys) - margin, max(ys) + margin]

    fig.update_layout(
        title=dict(text=title, font=dict(size=16)),
        scene=dict(
            xaxis=dict(title="X (m)", range=x_range),
            yaxis=dict(title="Y (m)", range=y_range),
            zaxis=dict(title="Z (m)", range=[0, wall_height + 1.0]),
            aspectmode="data",
            camera=dict(
                eye=dict(x=1.5, y=1.5, z=1.2),
                up=dict(x=0, y=0, z=1),
            ),
        ),
        margin=dict(l=0, r=0, b=0, t=40),
        legend=dict(yanchor="top", y=0.99, xanchor="left", x=0.01),
        height=650,
    )

    return fig


def _add_wall_wireframe(
    fig: "go.Figure",
    topology: np.ndarray,
    transform: Dict[str, Any],
    wall_height: float = 2.5,
    n_levels: int = 5,
):
    """
    从拓扑图中提取障碍物边缘像素，拉伸为多层 3D 线框墙。

    算法：
      1. 取 OBSTACLE + INFLATED mask
      2. 用梯度/边缘检测取边界像素
      3. 在 n_levels 个 Z 高度绘制边界点 → 形成"笼式"墙体
    """
    from scipy import ndimage

    CLASS_OBSTACLE = 3
    CLASS_INFLATED = 4
    wall_mask = (topology == CLASS_OBSTACLE) | (topology == CLASS_INFLATED)

    # 降采样
    ds = max(1, max(wall_mask.shape) // 300)
    if ds > 1:
        H0, W0 = wall_mask.shape
        H, W = H0 // ds, W0 // ds
        wall_mask = wall_mask[: H * ds, : W * ds].reshape(H, ds, W, ds).max(axis=(1, 3))
    else:
        H, W = wall_mask.shape

    # 边缘检测
    if np.any(wall_mask) and not np.all(wall_mask):
        edges = ndimage.sobel(wall_mask.astype(float)) > 0.1
    else:
        return

    res = float(transform["resolution"]) * ds
    ox, oy = float(transform["origin"][0]), float(transform["origin"][1])

    edge_points = np.argwhere(edges)
    if len(edge_points) == 0:
        return

    # 采样控制点数量
    max_pts = 8000
    if len(edge_points) > max_pts:
        idx = np.random.choice(len(edge_points), max_pts, replace=False)
        edge_points = edge_points[idx]

    edge_x = ox + edge_points[:, 1] * res
    edge_y = oy + edge_points[:, 0] * res

    # Vertical line segments for wall wireframe (not dots)
    wall_x: List[float] = []
    wall_y: List[float] = []
    wall_z: List[float] = []
    for i in range(len(edge_x)):
        wall_x.extend([edge_x[i], edge_x[i], None])
        wall_y.extend([edge_y[i], edge_y[i], None])
        wall_z.extend([0.0, wall_height, None])

    if wall_x:
        fig.add_trace(go.Scatter3d(
            x=wall_x, y=wall_y, z=wall_z,
            mode="lines",
            line=dict(color="gray", width=1.5),
            name="Walls",
            hoverinfo="skip",
        ))


# ── 3D 轨迹 + 鲁棒度双面板（供 Gradio 使用） ────────────────

def generate_3d_dashboard(
    trajectory: List[Dict[str, float]],
    bundle: Optional[Dict[str, Any]] = None,
    topology: Optional[np.ndarray] = None,
    violation_nodes: Optional[List[Dict[str, Any]]] = None,
    min_distance_m: Optional[float] = None,
) -> Tuple["go.Figure", "go.Figure"]:
    """
    一次性生成两个 Plotly Figure：
      (a) 3D 轨迹沙盘
      (b) 鲁棒度 ρ 时间曲线
    """
    fig_3d = generate_3d_trajectory_scene(
        trajectory=trajectory,
        bundle=bundle,
        topology=topology,
        violation_nodes=violation_nodes,
        min_distance_m=min_distance_m,
        title="SMDSL 3D Trajectory Sandbox",
    )

    fig_rho = _generate_robustness_curve(
        trajectory=trajectory,
        bundle=bundle,
        min_distance_m=min_distance_m,
        title="STL Robustness ρ over Time",
    )

    return fig_3d, fig_rho


def _generate_robustness_curve(
    trajectory: List[Dict[str, float]],
    bundle: Optional[Dict[str, Any]] = None,
    min_distance_m: Optional[float] = None,
    title: str = "STL Robustness ρ(t)",
) -> "go.Figure":
    """Plotly 版鲁棒度曲线。"""
    fig = go.Figure()
    ts = [float(p.get("t", i)) for i, p in enumerate(trajectory)]

    if bundle is None or min_distance_m is None:
        fig.add_annotation(text="需要 TopologyBundle + Distance 约束", x=0.5, y=0.5, showarrow=False)
    else:
        xs = [float(p.get("x", 0.0)) for p in trajectory]
        ys = [float(p.get("y", 0.0)) for p in trajectory]
        clearances = [_sample_distance_at(x, y, bundle) for x, y in zip(xs, ys)]
        rhos = [d - min_distance_m for d in clearances]

        fig.add_trace(go.Scatter(
            x=ts, y=rhos, mode="lines+markers",
            line=dict(color="#1f77b4", width=2),
            marker=dict(size=6),
            name="rho(t)",
        ))
        # 违规区域填充
        fig.add_trace(go.Scatter(
            x=ts, y=[0] * len(ts), mode="lines",
            line=dict(color="red", dash="dash", width=1.2),
            name="Threshold (0)",
        ))
        viol_x, viol_y = [], []
        for i, r in enumerate(rhos):
            if r < 0:
                viol_x.append(ts[i])
                viol_y.append(r)
        if viol_x:
            fig.add_trace(go.Scatter(
                x=viol_x, y=viol_y, mode="markers",
                marker=dict(color="red", size=10, symbol="x"),
                name=f"Violations ({len(viol_x)})",
            ))

    fig.update_layout(
        title=title,
        xaxis_title="Time (s)",
        yaxis_title="rho = clearance - D_safe (m)",
        margin=dict(l=40, r=20, b=40, t=40),
        height=350,
        hovermode="x",
    )
    return fig


# ── 兼容旧接口的静态 PNG 渲染（保留向后兼容） ──────────────

def render_trajectory_overlay(*args, **kwargs) -> str:
    """[已弃用] 请使用 generate_3d_dashboard()。"""
    import warnings
    warnings.warn("render_trajectory_overlay 已弃用，请迁移到 generate_3d_dashboard()", DeprecationWarning)
    return ""


def render_robustness_curve(*args, **kwargs) -> str:
    """[已弃用] 请使用 generate_3d_dashboard()。"""
    import warnings
    warnings.warn("render_robustness_curve 已弃用，请迁移到 generate_3d_dashboard()", DeprecationWarning)
    return ""
