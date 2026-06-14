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


# ══════════════════════════════════════════════════════════════════════
# OSM 3D 场景渲染：房间板块 + 拓扑图层 + Hover 语义
# ══════════════════════════════════════════════════════════════════════

# 房间颜色板（循环使用）
_ROOM_COLORS = [
    "#4878CF", "#6ACC65", "#D65F5F", "#B47CC7",
    "#C4AD66", "#77BEDB", "#E8834A", "#5E9E6E",
    "#D47EBB", "#8FBFE0", "#F2C45A", "#6C8EBF",
    "#A8D5A2", "#F28B66", "#9B89C4", "#F5D06E",
]


def _triangulate_polygon_fan(
    pts: List[Tuple[float, float]],
) -> Tuple[List[float], List[float], List[int], List[int], List[int]]:
    """
    扇形三角剖分（fan triangulation from vertex 0）。
    对于凸多边形及大多数准凸室内空间足够精确。
    对于极端凹形区域（room_23 等超大走廊），Plotly 仍可正确渲染
    因为 Mesh3d 不会剔除重叠三角。

    Returns: (xs, ys, i_idx, j_idx, k_idx)
    """
    # 去掉闭合重复点
    pts_open = pts[:-1] if len(pts) > 1 and pts[0] == pts[-1] else list(pts)
    n = len(pts_open)
    if n < 3:
        return [], [], [], [], []

    xs = [float(p[0]) for p in pts_open]
    ys = [float(p[1]) for p in pts_open]
    ii, jj, kk = [], [], []
    for k in range(1, n - 1):
        ii.append(0)
        jj.append(k)
        kk.append(k + 1)
    return xs, ys, ii, jj, kk


def generate_osm_3d_scene(
    parse_result: Dict[str, Any],
    wall_height: float = 3.2,
    title: str = "OSM Indoor Scene Graph",
    show_topology_edges: bool = True,
    show_grid_floor: bool = True,
) -> "go.Figure":
    """
    生成 osmAG ParseResult 的交互式 3D 场景图。

    图层（从底到上）：
      0. Distance-field 地板热力图（若有 bundle，仅在 generate_3d_dashboard_osm 中启用）
      1. go.Mesh3d       — 各房间填色地板板块（Z=0，不同颜色区分）
      2. go.Scatter3d    — 墙体轮廓线（多边形边缘从 Z=0 拉伸至 Z=wall_height）
      3. go.Scatter3d    — 拓扑通路图层（门连线，悬浮在 Z=wall_height/2）
      4. go.Scatter3d    — 房间质心标注（Z=wall_height*0.6，显示房间名）

    Args:
        parse_result:  dispatcher.dispatch_cad() 返回的 mode="osm" 结构
        wall_height:   墙体拉伸高度（米，优先使用 osm_rooms 中的 height_m）
        title:         图表标题
        show_topology_edges: 是否绘制通路连线层
        show_grid_floor:     是否在 Mesh3d 下额外叠加 occupancy grid 地板

    Returns:
        plotly.graph_objects.Figure
    """
    if not HAS_PLOTLY:
        raise ImportError("plotly 未安装，请运行: pip install plotly")

    osm_rooms: List[Dict[str, Any]] = parse_result.get("osm_rooms", [])
    osm_edges: List[Dict[str, Any]] = parse_result.get("osm_edges", [])

    if not osm_rooms:
        fig = go.Figure()
        fig.add_annotation(text="无 OSM 房间数据", x=0.5, y=0.5, showarrow=False)
        return fig

    fig = go.Figure()

    # 构建房间名 → 质心的快查表（供通路连线使用）
    room_centroid: Dict[str, Tuple[float, float]] = {
        r["name"]: (r["centroid_m"][0], r["centroid_m"][1])
        for r in osm_rooms
    }

    # ── Layer 1+2: 房间地板 + 彩色侧墙（同房间颜色，半透明）─────
    for idx, room in enumerate(osm_rooms):
        color = _ROOM_COLORS[idx % len(_ROOM_COLORS)]
        pts = [(c[0], c[1]) for c in room.get("coords_m", [])]
        if len(pts) < 3:
            continue
        h = float(room.get("height_m", wall_height))

        hover_txt = (
            f"<b>{room['name']}</b><br>"
            f"类型: {room.get('area_type', 'room')}<br>"
            f"面积: {room.get('area_m2', 0):.1f} m²<br>"
            f"层高: {h:.1f} m<br>"
            f"楼层: {room.get('level', '1')}<br>"
            f"质心: ({room['centroid_m'][0]:.1f}, {room['centroid_m'][1]:.1f}) m"
        )

        # 1a) 地板：三角化多边形，实心高不透明
        xs_f, ys_f, ii_f, jj_f, kk_f = _triangulate_polygon_fan(pts)
        if xs_f:
            fig.add_trace(go.Mesh3d(
                x=xs_f, y=ys_f, z=[0.0] * len(xs_f),
                i=ii_f, j=jj_f, k=kk_f,
                color=color, opacity=0.92,
                name=room["name"],
                text=hover_txt, hoverinfo="text",
                showlegend=(idx < 12),
                flatshading=True,
                lighting=dict(ambient=0.85, diffuse=0.9, specular=0.1),
            ))

        # 1b) 侧墙：每段矩形 = 2 个三角形，同颜色半透明
        sw_x: List[float] = []
        sw_y: List[float] = []
        sw_z: List[float] = []
        sw_i: List[int] = []
        sw_j: List[int] = []
        sw_k: List[int] = []
        sv = 0
        for seg in range(len(pts)):
            nx = (seg + 1) % len(pts)
            x0, y0 = pts[seg]
            x1, y1 = pts[nx]
            sw_x.extend([x0, x1, x1, x0])
            sw_y.extend([y0, y1, y1, y0])
            sw_z.extend([0.0, 0.0, h, h])
            sw_i.extend([sv, sv])
            sw_j.extend([sv + 1, sv + 2])
            sw_k.extend([sv + 2, sv + 3])
            sv += 4
        if sw_x:
            fig.add_trace(go.Mesh3d(
                x=sw_x, y=sw_y, z=sw_z,
                i=sw_i, j=sw_j, k=sw_k,
                color=color, opacity=0.32,
                hoverinfo="skip", showlegend=False,
                flatshading=True,
                lighting=dict(ambient=0.6, diffuse=0.85, specular=0.05),
            ))

        # 1c) 边线框：底边 + 顶边 + 垂直角线（同颜色描边）
        el_x: List[Any] = []
        el_y: List[Any] = []
        el_z: List[Any] = []
        for seg in range(len(pts)):
            nx = (seg + 1) % len(pts)
            x0, y0 = pts[seg]
            x1, y1 = pts[nx]
            el_x += [x0, x1, None]        # 底边
            el_y += [y0, y1, None]
            el_z += [0.0, 0.0, None]
            el_x += [x0, x1, None]        # 顶边
            el_y += [y0, y1, None]
            el_z += [h, h, None]
            el_x += [x0, x0, None]        # 垂直线
            el_y += [y0, y0, None]
            el_z += [0.0, h, None]
        if el_x:
            fig.add_trace(go.Scatter3d(
                x=el_x, y=el_y, z=el_z,
                mode="lines",
                line=dict(color=color, width=2.0),
                hoverinfo="skip", showlegend=False,
            ))

    # ── Layer 3: 拓扑通路图层（门连线）────────────────────────
    if show_topology_edges and osm_edges:
        edge_x: List[Any] = []
        edge_y: List[Any] = []
        edge_z: List[Any] = []
        edge_hover: List[str] = []
        mid_x: List[float] = []
        mid_y: List[float] = []
        mid_z: List[float] = []
        mid_text: List[str] = []

        z_float = wall_height * 0.45  # 通路线悬浮高度

        for edge in osm_edges:
            src = edge.get("from_room", "")
            dst = edge.get("to_room", "")
            if src not in room_centroid or dst not in room_centroid:
                continue
            x0, y0 = room_centroid[src]
            x1, y1 = room_centroid[dst]
            mx, my = (x0 + x1) / 2, (y0 + y1) / 2

            edge_x.extend([x0, x1, None])
            edge_y.extend([y0, y1, None])
            edge_z.extend([z_float, z_float, None])

            # 门位置标注
            door_xy = edge.get("coords_m", [])
            door_str = ""
            if door_xy:
                dx, dy = door_xy[0][0], door_xy[0][1]
                door_str = f"<br>Door @ ({dx:.1f}, {dy:.1f})"
            mid_hover = (
                f"<b>{edge['name']}</b><br>"
                f"{src} → {dst}"
                f"{door_str}"
            )
            mid_x.append(mx)
            mid_y.append(my)
            mid_z.append(z_float)
            mid_text.append(mid_hover)

        if edge_x:
            fig.add_trace(go.Scatter3d(
                x=edge_x, y=edge_y, z=edge_z,
                mode="lines",
                line=dict(color="rgba(255,200,0,0.85)", width=3),
                name="Passages (Topology)",
                hoverinfo="skip",
                showlegend=True,
            ))
        if mid_x:
            fig.add_trace(go.Scatter3d(
                x=mid_x, y=mid_y, z=mid_z,
                mode="markers",
                marker=dict(
                    size=6, color="gold", symbol="diamond",
                    line=dict(color="darkorange", width=1),
                ),
                name="Door Midpoints",
                text=mid_text,
                hoverinfo="text",
                showlegend=True,
            ))

    # ── Layer 4: 房间质心标注 ──────────────────────────────────
    label_x, label_y, label_z, label_text = [], [], [], []
    for room in osm_rooms:
        cx, cy = room["centroid_m"]
        h = float(room.get("height_m", wall_height))
        label_x.append(cx)
        label_y.append(cy)
        label_z.append(h * 0.6)
        label_text.append(
            f"{room['name']}<br>{room.get('area_m2', 0):.0f}m²"
        )

    fig.add_trace(go.Scatter3d(
        x=label_x, y=label_y, z=label_z,
        mode="markers+text",
        marker=dict(size=3, color="white", opacity=0.9),
        text=label_text,
        textfont=dict(size=9, color="white"),
        textposition="top center",
        name="Room Labels",
        hoverinfo="text",
        showlegend=True,
    ))

    # ── Layout ─────────────────────────────────────────────
    all_coords = [
        (c[0], c[1])
        for room in osm_rooms
        for c in room.get("coords_m", [])
    ]
    if all_coords:
        x_vals = [c[0] for c in all_coords]
        y_vals = [c[1] for c in all_coords]
        x_range = [min(x_vals) - 2, max(x_vals) + 2]
        y_range = [min(y_vals) - 2, max(y_vals) + 2]
    else:
        x_range = [-50, 50]
        y_range = [-50, 50]

    avg_h = (
        sum(float(r.get("height_m", wall_height)) for r in osm_rooms) / len(osm_rooms)
        if osm_rooms else wall_height
    )

    # 计算 aspect ratio：给 Z 轴视觉放大，避免平面图过扁
    # 目标：Z 视觉高度 = XY 最大边长的 30%
    x_size = x_range[1] - x_range[0]
    y_size = y_range[1] - y_range[0]
    max_xy = max(x_size, y_size, 1.0)
    # z_ratio：使墙体在屏幕上占有 30% 的视觉高度
    target_z_visual = max_xy * 0.30
    z_boost = max(target_z_visual / max(avg_h, 0.5), 1.0)
    aspect_z = (avg_h * z_boost) / max_xy  # 归一化到 X=1

    fig.update_layout(
        title=dict(text=title, font=dict(size=16)),
        scene=dict(
            xaxis=dict(title="X (m)", range=x_range, showbackground=True,
                       backgroundcolor="rgba(30,30,50,1)"),
            yaxis=dict(title="Y (m)", range=y_range, showbackground=True,
                       backgroundcolor="rgba(30,30,50,1)"),
            zaxis=dict(title="Z (m)", range=[0, avg_h + 0.2],
                       showbackground=True,
                       backgroundcolor="rgba(20,20,40,1)"),
            # manual aspectratio：z 视觉权重被放大
            aspectmode="manual",
            aspectratio=dict(
                x=x_size / max_xy,
                y=y_size / max_xy,
                z=aspect_z,
            ),
            camera=dict(
                eye=dict(x=1.4, y=-1.8, z=1.2),
                up=dict(x=0, y=0, z=1),
                center=dict(x=0, y=0, z=-0.1),
            ),
            bgcolor="rgba(15,15,25,1)",
        ),
        paper_bgcolor="rgba(15,15,25,1)",
        plot_bgcolor="rgba(15,15,25,1)",
        font=dict(color="white"),
        margin=dict(l=0, r=0, b=0, t=50),
        legend=dict(
            yanchor="top", y=0.99, xanchor="left", x=0.01,
            bgcolor="rgba(30,30,40,0.85)",
            font=dict(size=10),
        ),
        height=700,
    )

    return fig


def generate_grid_3d_scene(
    pipeline_result: Dict[str, Any],
    wall_height: float = 3.0,
    title: str = "3D Space Graph",
) -> "go.Figure":
    """
    Grid-based 3D 场景图，用于非 OSM 模式（JSON / DWG / PNG）。

    图层：
      1. go.Surface   — 距离场地板热力图（Clearance 颜色映射）
      2. go.Mesh3d    — 墙体幕墙（skimage 轮廓 → 垂直三角网格，高度 wall_height）
      3. go.Scatter3d — 可导航路径点（绿色）
      4. go.Scatter3d — Seeds / 门洞（青色菱形）
      5. go.Scatter3d — DWG 文本标注

    修复 aspectmode 使高度方向视觉清晰。
    """
    if not HAS_PLOTLY:
        raise ImportError("plotly 未安装，请运行: pip install plotly")

    import numpy as _np  # noqa: PLC0415

    grid         = pipeline_result.get("grid")
    distance_field = pipeline_result.get("distance_field")
    topology     = pipeline_result.get("topology")
    transform    = pipeline_result.get("transform", {})
    seeds        = pipeline_result.get("seeds", [])
    semantics    = pipeline_result.get("semantics") or {}
    cad_data     = pipeline_result.get("cad_data") or {}

    if grid is None:
        fig = go.Figure()
        fig.add_annotation(text="无网格数据", x=0.5, y=0.5, showarrow=False)
        return fig

    res = float(transform.get("resolution", 0.05))
    origin = transform.get("origin", (0.0, 0.0))
    ox = float(origin[0])
    oy = float(origin[1])
    H0, W0 = grid.shape
    fig = go.Figure()

    # ── Layer 1: 距离场地板 ────────────────────────────────────
    if distance_field is not None:
        ds = max(1, max(H0, W0) // 220)
        df_ds = distance_field[::ds, ::ds]
        Hd, Wd = df_ds.shape
        df_m = df_ds * res
        fig.add_trace(go.Surface(
            x=_np.linspace(ox, ox + W0 * res, Wd),
            y=_np.linspace(oy, oy + H0 * res, Hd),
            z=_np.zeros((Hd, Wd)),
            surfacecolor=df_m,
            colorscale="Blues",
            colorbar=dict(title="Clearance(m)", x=1.02, len=0.6, thickness=14),
            name="Floor (Clearance)",
            showscale=True,
            hoverinfo="skip",
            opacity=0.9,
        ))
    elif grid is not None:
        # 没有距离场时，用可行区域（=0）作为灰色地板
        ds = max(1, max(H0, W0) // 220)
        g_ds = grid[::ds, ::ds]
        Hd, Wd = g_ds.shape
        free = (_np.array(g_ds) == 0).astype(float)
        fig.add_trace(go.Surface(
            x=_np.linspace(ox, ox + W0 * res, Wd),
            y=_np.linspace(oy, oy + H0 * res, Hd),
            z=_np.zeros((Hd, Wd)),
            surfacecolor=free,
            colorscale=[[0, "rgba(20,20,30,0)"], [1, "rgba(80,130,200,0.85)"]],
            showscale=False,
            name="Floor",
            hoverinfo="skip",
            opacity=0.85,
        ))

    # ── Layer 2: 墙体幕墙（skimage 等高线 → Mesh3d）──────────
    # 用 topology obstacle 或 grid obstacle 定义墙
    try:
        from skimage.measure import find_contours as _fc  # noqa: PLC0415
        CLASS_OBSTACLE = 3
        CLASS_INFLATED = 4
        if topology is not None:
            obs_mask = ((topology == CLASS_OBSTACLE) |
                        (topology == CLASS_INFLATED)).astype(_np.uint8)
        else:
            obs_mask = (_np.array(grid) > 0).astype(_np.uint8)

        # 降采样以控制轮廓点数
        ds_w = max(1, max(H0, W0) // 320)
        obs_ds = obs_mask[::ds_w, ::ds_w]
        res_w  = res * ds_w
        contours = _fc(obs_ds.astype(float), 0.5)

        wv_x: List[float] = []
        wv_y: List[float] = []
        wv_z: List[float] = []
        wv_i: List[int]   = []
        wv_j: List[int]   = []
        wv_k: List[int]   = []
        voff = 0
        for contour in contours:
            # 长轮廓降采样，避免顶点爆炸
            step = max(1, len(contour) // 500)
            pts_c = contour[::step]
            N = len(pts_c)
            if N < 2:
                continue
            cx = ox + pts_c[:, 1] * res_w   # col → x
            cy = oy + pts_c[:, 0] * res_w   # row → y
            # 底部 N 个顶点（z=0），顶部 N 个顶点（z=h）
            wv_x.extend(cx.tolist() + cx.tolist())
            wv_y.extend(cy.tolist() + cy.tolist())
            wv_z.extend([0.0] * N + [wall_height] * N)
            # 相邻段矩形 = 2 个三角形
            for seg in range(N - 1):
                b0, b1 = voff + seg,     voff + seg + 1
                t0, t1 = voff + N + seg, voff + N + seg + 1
                wv_i.extend([b0, b0])
                wv_j.extend([b1, t0])
                wv_k.extend([t0, t1])
            voff += 2 * N

        if wv_x:
            fig.add_trace(go.Mesh3d(
                x=wv_x, y=wv_y, z=wv_z,
                i=wv_i, j=wv_j, k=wv_k,
                color="rgba(170,175,200,1)",
                opacity=0.72,
                name="Walls",
                hoverinfo="skip",
                showlegend=True,
                flatshading=True,
                lighting=dict(ambient=0.65, diffuse=0.9, specular=0.08),
            ))
    except Exception:  # noqa: BLE001
        pass  # skimage 不可用时跳过墙体层

    # ── Layer 3: 可导航路径（绿色点云）──────────────────────────
    if topology is not None:
        CLASS_PATH = 2
        path = (_np.array(topology) == CLASS_PATH)
        dsp = max(1, max(H0, W0) // 150)
        if dsp > 1:
            Hp, Wp = H0 // dsp, W0 // dsp
            path = path[:Hp * dsp, :Wp * dsp].reshape(
                Hp, dsp, Wp, dsp).max(axis=(1, 3))
        ppts = _np.argwhere(path)
        if len(ppts) > 0:
            if len(ppts) > 3000:
                ppts = ppts[_np.random.choice(len(ppts), 3000, replace=False)]
            fig.add_trace(go.Scatter3d(
                x=ox + ppts[:, 1] * (res * dsp),
                y=oy + ppts[:, 0] * (res * dsp),
                z=[0.06] * len(ppts),
                mode="markers",
                marker=dict(size=2, color="limegreen", opacity=0.5),
                name="Navigable Path",
                hoverinfo="skip",
            ))

    # ── Layer 4: Seeds / 门洞 ─────────────────────────────────
    if seeds:
        sr = _np.array(seeds)
        fig.add_trace(go.Scatter3d(
            x=ox + sr[:, 1] * res,
            y=oy + sr[:, 0] * res,
            z=[wall_height * 0.5] * len(sr),
            mode="markers",
            marker=dict(
                size=8, color="cyan", symbol="diamond",
                line=dict(color="white", width=1),
            ),
            name="Seeds / Doors",
            hoverinfo="skip",
        ))

    # ── Layer 5: DWG 文字标注 / 语义文本 ──────────────────────
    texts = semantics.get("texts", [])
    if not texts:
        texts = cad_data.get("texts", [])
    if texts:
        tv, tx, ty = [], [], []
        for t in texts[:30]:
            v = t.get("value", "").strip()
            if v and len(v) < 30:
                p = t.get("position", [0, 0])
                tv.append(v[:20])
                tx.append(float(p[0]))
                ty.append(float(p[1]))
        if tv:
            fig.add_trace(go.Scatter3d(
                x=tx, y=ty,
                z=[wall_height * 0.85] * len(tx),
                mode="text",
                text=tv,
                textfont=dict(size=9, color="white"),
                name="Labels",
                hoverinfo="text",
            ))

    # ── Aspect ratio（给 Z 轴 30% 视觉权重）─────────────────────
    x_size = W0 * res
    y_size = H0 * res
    max_xy = max(x_size, y_size, 1.0)
    z_boost = max(max_xy * 0.30 / max(wall_height, 0.5), 1.0)
    aspect_z = (wall_height * z_boost) / max_xy

    fig.update_layout(
        title=dict(text=title, font=dict(size=15, color="white")),
        scene=dict(
            xaxis=dict(title="X (m)", showbackground=True,
                       backgroundcolor="rgba(25,25,45,1)"),
            yaxis=dict(title="Y (m)", showbackground=True,
                       backgroundcolor="rgba(25,25,45,1)"),
            zaxis=dict(title="Z (m)", range=[0, wall_height],
                       showbackground=True,
                       backgroundcolor="rgba(15,15,30,1)"),
            aspectmode="manual",
            aspectratio=dict(x=x_size / max_xy, y=y_size / max_xy, z=aspect_z),
            camera=dict(
                eye=dict(x=1.4, y=-1.8, z=1.2),
                up=dict(x=0, y=0, z=1),
                center=dict(x=0, y=0, z=-0.1),
            ),
            bgcolor="rgba(15,15,25,1)",
        ),
        paper_bgcolor="rgba(15,15,25,1)",
        plot_bgcolor="rgba(15,15,25,1)",
        font=dict(color="white"),
        margin=dict(l=0, r=0, b=0, t=50),
        legend=dict(
            yanchor="top", y=0.99, xanchor="left", x=0.01,
            bgcolor="rgba(30,30,40,0.85)",
            font=dict(size=10),
        ),
        height=700,
    )
    return fig


def generate_3d_dashboard_osm(
    parse_result: Dict[str, Any],
    trajectory: Optional[List[Dict[str, float]]] = None,
    bundle: Optional[Dict[str, Any]] = None,
    violation_nodes: Optional[List[Dict[str, Any]]] = None,
    min_distance_m: Optional[float] = None,
    wall_height: float = 3.2,
) -> Tuple["go.Figure", Optional["go.Figure"]]:
    """
    OSM 专用仪表盘：
      fig_scene — OSM 3D 房间场景图（含轨迹，若提供）
      fig_rho   — 鲁棒度曲线（若提供 trajectory + bundle）

    与 generate_3d_dashboard() 的区别：
      - 房间地板来自真实 OSM 多边形，而非 occupancy grid 等值面
      - 通路网络为独立图层，可单独切换显示
    """
    fig_scene = generate_osm_3d_scene(
        parse_result,
        wall_height=wall_height,
        title="OSM Indoor Scene — SMDSL",
    )

    # 若提供了轨迹，叠加到 OSM 场景上
    if trajectory and len(trajectory) > 0:
        xs = [float(p.get("x", 0)) for p in trajectory]
        ys = [float(p.get("y", 0)) for p in trajectory]
        traj_z = wall_height * 0.3

        clearance_labels = []
        for i, pt in enumerate(trajectory):
            t = pt.get("t", i)
            lbl = f"t={t:.2f}s<br>x={xs[i]:.3f}<br>y={ys[i]:.3f}"
            if bundle is not None:
                d = _sample_distance_at(xs[i], ys[i], bundle)
                lbl += f"<br>clearance={d:.4f}m"
            clearance_labels.append(lbl)

        fig_scene.add_trace(go.Scatter3d(
            x=xs, y=ys, z=[traj_z] * len(xs),
            mode="lines+markers",
            line=dict(color="cyan", width=5),
            marker=dict(size=4, color="cyan"),
            name="Trajectory",
            text=clearance_labels,
            hoverinfo="text",
        ))
        if violation_nodes:
            vx = [float(v.get("x", 0)) for v in violation_nodes]
            vy = [float(v.get("y", 0)) for v in violation_nodes]
            vhover = [
                f"<b>VIOLATION</b><br>t={v.get('t','?')}s<br>"
                f"rho={float(v.get('rho',0)):.4f}m"
                for v in violation_nodes
            ]
            fig_scene.add_trace(go.Scatter3d(
                x=vx, y=vy, z=[traj_z] * len(vx),
                mode="markers",
                marker=dict(size=12, color="red", symbol="circle"),
                name=f"Violations ({len(violation_nodes)})",
                text=vhover,
                hoverinfo="text",
            ))

    fig_rho = None
    if trajectory and bundle is not None:
        fig_rho = _generate_robustness_curve(
            trajectory=trajectory,
            bundle=bundle,
            min_distance_m=min_distance_m,
            title="STL Robustness ρ(t) — OSM Scene",
        )

    return fig_scene, fig_rho


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
