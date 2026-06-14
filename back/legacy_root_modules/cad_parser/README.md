# CAD 2D Spatial Parsing — Demo 1 (RoboIR Zone 1 感知层)

> 弃用 U-Net，使用矢量栅格化 + 距离场 + 安全代价 A\* 的确定性图论流水线。

## 输入 / 输出

| 项目 | 说明 |
|------|------|
| **输入** | FloorplanQA 矢量平面图 JSON（`room_boundary` + `walls` + `objects` + `openings.doors/windows`） |
| **处理** | 栅格化 → 距离场（EDT）→ 带 R_robot 防穿透的 A\* 漫水 |
| **输出** | `topology.json` — 含 path / obstacle / inflated_buffer / loading 标签 + 元数据 |

数据集主用 `data/cad_samples/floorplanqa/layouts/{kitchen,bedroom,living_room,hssd}/`（共 1,981 份矢量 JSON）。`FloorPlanCAD` 的 PNG raster 子集留作视觉对照。

## 四步流水线

1. **栅格化与二值化** — `rasterize_to_grid`
   - boundary 内 = 1（自由空间），boundary 外 = 0（隐式障碍物）
   - walls（线段，加粗到 `wall_thickness`）+ objects + windows → 0
   - doors（矩形多边形）→ 1（覆盖墙截断，允许 A\* 进入）
2. **距离场生成** — `compute_distance_field`
   - 使用 `scipy.ndimage.distance_transform_edt`
   - 每个自由像素的值 = 到最近墙的欧氏距离（像素）
3. **安全代价 A\* 漫水** — `safety_aware_astar_flood`
   - 多源 Dijkstra（h ≡ 0；漫水阶段无单一目标）
   - 节点代价：`g(parent) + step_cost + safety_weight / max(1, distance_field[n])`
   - 若 `distance_field[n] < R_robot_px` → 视为 +∞，绝对不可通行
4. **拓扑图元提取** — `extract_topology_json`
   - A\* 频繁经过的连通区域 → `CLASS_PATH (2)`
   - 墙 / 边界外 / 家具 / 窗 → `CLASS_OBSTACLE (3)`
   - 自由但 R\<R_robot → `CLASS_INFLATED (4)` (可视但不可走的缓冲带)
   - 从任一种子出发 A\* 都到不了的孤立自由区域 → `CLASS_UNKNOWN (0)`
   - `mark_loading_zone` 预留 `CLASS_LOADING (1)` 接口

## 快速开始

```bash
# 单房间最小命令
python -m cad_parser.main \
    --cad_path data/cad_samples/floorplanqa/layouts/kitchen/room_0.json \
    --output out/demo1/kitchen.topology.json \
    --visualize out/demo1/kitchen.png

# 带自定义机器人半径 + 高分辨率
python -m cad_parser.main \
    --cad_path data/cad_samples/floorplanqa/layouts/living_room/room_0.json \
    --resolution 0.02 --robot_radius 0.25 \
    --output out/demo1/living_room.topology.json \
    --visualize out/demo1/living_room.png

# 手工指定种子（世界坐标，米）
python -m cad_parser.main --cad_path ... \
    --seeds_world "1.0,0.3;2.5,4.0;0.5,5.5"
```

CLI 主要开关：

| Flag | 默认 | 含义 |
|---|---|---|
| `--resolution` | `0.05` | 米/像素（典型移动机器人 5cm/px） |
| `--robot_radius` | `0.30` | 机器人物理半径（米） |
| `--wall_thickness` | `0.10` | 墙体物理厚度（米） |
| `--padding` | `0.5` | bbox 外延填充（米） |
| `--safety_weight` | `0.5` | 1/distance 惩罚权重，置 0 即纯 Dijkstra |
| `--seeds_world` | （门洞自动） | 手动种子点 `"x1,y1;x2,y2"` |
| `--no_grid` | off | 不嵌入 H×W 标签矩阵，仅写元数据 |
| `--visualize` | off | 生成 2×2 四联图 PNG |

## 输出 JSON 结构

```jsonc
{
  "metadata": {
    "width_pixels": 240, "height_pixels": 340,
    "resolution_m_per_px": 0.02,
    "origin_world": {"x": -0.5, "y": -0.5},
    "bbox_world": [-0.5, -0.5, 4.3, 6.3],
    "robot_radius_m": 0.25,
    "wall_thickness_m": 0.10,
    "padding_m": 0.5
  },
  "classes": {
    "0": {"name": "unknown",         "id": 0},
    "1": {"name": "loading_zone",    "id": 1},
    "2": {"name": "path",            "id": 2},
    "3": {"name": "obstacle",        "id": 3},
    "4": {"name": "inflated_buffer", "id": 4}
  },
  "summary": {
    "class_pixels": {"path": 13607, "obstacle": 51724, "inflated_buffer": 15573, "unknown": 696},
    "total_a_star_visits": 13607,
    "n_seeds": 1,
    "min_clearance_px": 1.0,
    "max_clearance_px": 45.8,
    "max_clearance_m": 0.916
  },
  "seeds_pixel": [{"row": 37, "col": 72}],
  "grid":         [[2, 2, ...], ...],   // 仅当未传 --no_grid
  "heatmap_log_x1000": [[..], ...]      // 同上
}
```

## 为什么 A\* 优于 U-Net？

| 维度 | U-Net | 安全代价 A\* |
|------|------|------|
| 对 CAD 的适配性 | ❌ 无视觉纹理 → 实例分割崩 | ✅ 直接在矢量数据 + 占据栅格上工作 |
| 训练需求 | ❌ 需大量标注 + GPU | ✅ 零训练，纯几何 |
| 可解释性 | ❌ 黑盒 mask | ✅ 每像素的标签都可追溯到距离场与代价函数 |
| 物理一致性 | ❌ 无 R_robot 概念，输出 mask 仍需后处理才能给规划器 | ✅ 距离场 < R_robot 直接禁通行，输出可直接消费 |

## 子模块

| 文件 | 职责 |
|---|---|
| `astar_topology.py` | 核心算法实现 — 5 步函数 + `run_pipeline` 便捷封装 |
| `main.py` | CLI 入口；调用 `run_pipeline` 并落盘 JSON / PNG |
| `visualize.py` | matplotlib 2×2 四联图（栅格 / 距离场 / 访问热力 / 标签） |
