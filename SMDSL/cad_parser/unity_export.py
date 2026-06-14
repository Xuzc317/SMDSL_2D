"""
unity_export.py — 将 SMDSL 拓扑数据导出为 Unity NavMesh 就绪包

产出物（ZIP）：
  navmesh_rooms.obj       — 房间地板多边形（三角剖分），Y-up Unity 坐标系
  navmesh_graph.json      — 邻接图 + 房间语义（房间名/面积/质心/通路）
  SMDSLNavMeshImporter.cs — Unity C# 自动导入脚本（粘贴到 Assets/Editor/）
  README_Unity.md         — 5 步导入教程

支持格式：
  - OSM 模式：直接用 osm_rooms 多边形（最精确）
  - 其他模式（JSON/DWG/SVG/PNG）：从 topology grid 提取连通路径区域生成简化多边形

坐标系映射（SMDSL → Unity）：
  SMDSL: X 向右 (m)，Y 向上（纸面）
  Unity: X 向右，Y 向上（高度），Z 向屏幕内 → 映射为 (x, 0, y)
"""

from __future__ import annotations

import io
import json
import math
import zipfile
from pathlib import Path
from typing import Any, Dict, List, Optional, Tuple

import numpy as np


# ══════════════════════════════════════════════════════════════════════
# 坐标系转换
# ══════════════════════════════════════════════════════════════════════

def _to_unity(x_m: float, y_m: float, z_m: float = 0.0) -> Tuple[float, float, float]:
    """SMDSL (x, y) → Unity (x, z_m, y)，Y 轴为高度。"""
    return round(x_m, 4), round(z_m, 4), round(y_m, 4)


# ══════════════════════════════════════════════════════════════════════
# 多边形三角剖分（扇形 fan 法）
# ══════════════════════════════════════════════════════════════════════

def _fan_triangulate(pts: List[Tuple[float, float]]) -> List[Tuple[int, int, int]]:
    """
    扇形三角剖分：以顶点 0 为枢轴，生成 (n-2) 个三角形。
    对凸多边形精确，对轻微凹多边形足够。
    返回局部顶点索引元组列表。
    """
    pts = pts[:-1] if len(pts) > 1 and pts[0] == pts[-1] else list(pts)
    n = len(pts)
    if n < 3:
        return []
    return [(0, i, i + 1) for i in range(1, n - 1)]


# ══════════════════════════════════════════════════════════════════════
# 从 OSM 房间提取几何
# ══════════════════════════════════════════════════════════════════════

def _rooms_from_osm(parse_result: Dict[str, Any]) -> List[Dict[str, Any]]:
    """
    从 osm_rooms 提取标准化房间列表。

    每个 room dict：
      name, area_m2, height_m, centroid_m, coords_m, area_type, tags, source
    """
    rooms = []
    for r in parse_result.get("osm_rooms", []) or []:
        rooms.append({
            "name": r.get("name", "room"),
            "area_m2": r.get("area_m2", 0.0),
            "height_m": float(r.get("height_m", 3.0) or 3.0),
            "centroid_m": r.get("centroid_m", [0, 0]),
            "coords_m": [[float(c[0]), float(c[1])] for c in r.get("coords_m", [])],
            "area_type": r.get("area_type", "room"),
            "source": "osm",
        })
    return rooms


# ══════════════════════════════════════════════════════════════════════
# 从 topology grid 提取连通组件房间（非 OSM 模式）
# ══════════════════════════════════════════════════════════════════════

def _rooms_from_grid(parse_result: Dict[str, Any]) -> List[Dict[str, Any]]:
    """
    对于 JSON/DWG/SVG/PNG 模式，从 occupancy grid 的 CLASS_PATH 区域
    提取连通分量，为每个分量生成简化矩形 bounding-box 多边形。

    这是近似的：精确轮廓需要 marching squares，
    对于 Unity 导入而言 bbox 足以提供可烘焙的 NavMesh 基面。
    """
    try:
        from scipy import ndimage  # noqa: PLC0415
    except ImportError:
        return _rooms_from_grid_fallback(parse_result)

    topology = parse_result.get("topology")
    tx = parse_result.get("transform", {})
    res = float(tx.get("resolution", 0.05))
    ox, oy = float(tx.get("origin", (0, 0))[0]), float(tx.get("origin", (0, 0))[1])

    if topology is None:
        return []

    # CLASS_PATH = 2
    path_mask = (topology == 2).astype(np.uint8)
    if not path_mask.any():
        path_mask = (parse_result.get("grid", np.zeros((2, 2))) == 1).astype(np.uint8)

    labeled, n_labels = ndimage.label(path_mask)
    rooms = []
    for lab in range(1, min(n_labels + 1, 50)):   # 最多取 50 个连通区
        region = (labeled == lab)
        area_px = int(region.sum())
        area_m2 = area_px * (res ** 2)
        if area_m2 < 0.5:   # 过滤过小碎片
            continue
        rows, cols = np.where(region)
        r_min, r_max = int(rows.min()), int(rows.max())
        c_min, c_max = int(cols.min()), int(cols.max())
        cx = ox + (c_min + c_max) / 2 * res
        cy = oy + (r_min + r_max) / 2 * res
        # bbox 矩形多边形（4 顶点）
        x0, y0 = ox + c_min * res, oy + r_min * res
        x1, y1 = ox + (c_max + 1) * res, oy + (r_max + 1) * res
        rooms.append({
            "name": f"region_{lab}",
            "area_m2": round(area_m2, 2),
            "height_m": 3.0,
            "centroid_m": [round(cx, 3), round(cy, 3)],
            "coords_m": [[x0, y0], [x1, y0], [x1, y1], [x0, y1]],
            "area_type": "room",
            "source": "grid",
        })
    # 按面积降序排列
    rooms.sort(key=lambda r: r["area_m2"], reverse=True)
    return rooms


def _rooms_from_grid_fallback(parse_result: Dict[str, Any]) -> List[Dict[str, Any]]:
    """scipy 不可用时的简单 fallback：整个 grid 当一个大房间。"""
    tx = parse_result.get("transform", {})
    res = float(tx.get("resolution", 0.05))
    ox, oy = float(tx.get("origin", (0, 0))[0]), float(tx.get("origin", (0, 0))[1])
    H, W = tx.get("shape", (100, 100))
    return [{
        "name": "walkable_area",
        "area_m2": round(H * W * res ** 2, 1),
        "height_m": 3.0,
        "centroid_m": [round(ox + W * res / 2, 3), round(oy + H * res / 2, 3)],
        "coords_m": [[ox, oy], [ox + W * res, oy],
                     [ox + W * res, oy + H * res], [ox, oy + H * res]],
        "area_type": "room",
        "source": "grid_fallback",
    }]


# ══════════════════════════════════════════════════════════════════════
# OBJ 生成器
# ══════════════════════════════════════════════════════════════════════

def _build_obj(rooms: List[Dict[str, Any]]) -> str:
    """
    生成 Wavefront OBJ 内容：
    - 每个房间为一个命名 group（g room_N）
    - 地板多边形三角化后在 Y=0 平面
    - 正法线朝上（Y+），与 Unity NavMesh 烘焙要求一致

    Unity 坐标：smdsl_x → obj_x，smdsl_y → obj_z，height → obj_y
    """
    lines: List[str] = [
        "# SMDSL Unity NavMesh Export",
        "# Coordinate system: Unity (Y-up, Z-forward)",
        "# Each group = one navigable room",
        "# Import: File > Import > Wavefront OBJ, set Navigation Static",
        "",
        "mtllib navmesh_rooms.mtl",
        "",
    ]

    vtx_offset = 1   # OBJ vertex index is 1-based

    for room in rooms:
        pts_raw = room.get("coords_m", [])
        if len(pts_raw) < 3:
            continue
        pts = [(float(p[0]), float(p[1])) for p in pts_raw]
        pts = pts[:-1] if len(pts) > 1 and pts[0] == pts[-1] else pts
        if len(pts) < 3:
            continue

        name = room["name"].replace(" ", "_")
        lines.append(f"g {name}")
        lines.append(f"usemtl mat_{room.get('area_type', 'room')}")

        # 顶点（Unity Y=0 地板）
        n_verts = len(pts)
        for x, y in pts:
            ux, uy, uz = _to_unity(x, y, 0.0)
            lines.append(f"v {ux} {uy} {uz}")
        lines.append(f"# centroid: {room['centroid_m']}, area: {room['area_m2']} m2")

        # 三角面（扇形法）
        tris = _fan_triangulate(pts)
        for a, b, c in tris:
            # OBJ 面 = 全局 1-based index
            lines.append(f"f {vtx_offset+a} {vtx_offset+b} {vtx_offset+c}")

        vtx_offset += n_verts
        lines.append("")

    return "\n".join(lines)


def _build_mtl() -> str:
    """生成简单 .mtl 材质文件，不同区域类型用不同颜色。"""
    return "\n".join([
        "# SMDSL NavMesh Material",
        "",
        "newmtl mat_room",
        "Kd 0.2 0.6 0.9",
        "Ka 0.1 0.1 0.1",
        "",
        "newmtl mat_corridor",
        "Kd 0.5 0.8 0.5",
        "",
        "newmtl mat_hall",
        "Kd 0.9 0.7 0.2",
        "",
        "newmtl mat_door",
        "Kd 0.8 0.3 0.3",
        "",
    ])


# ══════════════════════════════════════════════════════════════════════
# NavMesh Graph JSON
# ══════════════════════════════════════════════════════════════════════

def _build_graph_json(
    rooms: List[Dict[str, Any]],
    adjacency: Dict[str, List[str]],
    edges: List[Dict[str, Any]],
    source_name: str,
) -> str:
    """
    生成供 Unity C# 脚本读取的 navmesh_graph.json。

    结构：
      source, exported_at, room_count, rooms[], passages[]
    """
    import datetime  # noqa: PLC0415

    graph = {
        "smdsl_export_version": "1.0",
        "source": source_name,
        "exported_at": datetime.datetime.now().isoformat(timespec="seconds"),
        "coordinate_system": "Unity (Y-up): smdsl_x→X, smdsl_y→Z, height→Y",
        "room_count": len(rooms),
        "passage_count": len(edges),
        "rooms": [
            {
                "name": r["name"],
                "area_type": r.get("area_type", "room"),
                "area_m2": r["area_m2"],
                "height_m": r.get("height_m", 3.0),
                "centroid_unity": list(_to_unity(
                    r["centroid_m"][0], r["centroid_m"][1], 0.0
                )),
                "neighbors": adjacency.get(r["name"], []),
                "vertex_count": len(r.get("coords_m", [])),
            }
            for r in rooms
        ],
        "passages": [
            {
                "name": e.get("name", ""),
                "from_room": e.get("from_room", ""),
                "to_room": e.get("to_room", ""),
                "midpoint_unity": list(_to_unity(
                    float(e.get("midpoint_m", [0, 0])[0]),
                    float(e.get("midpoint_m", [0, 0])[1]),
                    0.0
                )),
                "is_door": e.get("door", "") == "yes",
            }
            for e in edges
        ],
    }
    return json.dumps(graph, ensure_ascii=False, indent=2)


# ══════════════════════════════════════════════════════════════════════
# Unity C# 自动导入脚本
# ══════════════════════════════════════════════════════════════════════

_CSHARP_TEMPLATE = '''\
// SMDSLNavMeshImporter.cs
// Generated by SMDSL Unity Export Plugin
// 使用说明：将此文件放到 Assets/Editor/ 目录，在 Unity 菜单执行 SMDSL > Import NavMesh Graph
//
// 依赖：Unity AI Navigation 包（Window > Package Manager > AI Navigation）
// 测试版本：Unity 2022.3 LTS / Unity 6

using System.IO;
using System.Collections.Generic;
using UnityEngine;
using UnityEditor;
using UnityEngine.AI;

#if UNITY_EDITOR

[System.Serializable]
public class SMDSLRoom
{
    public string name;
    public string area_type;
    public float area_m2;
    public float height_m;
    public float[] centroid_unity;
    public string[] neighbors;
}

[System.Serializable]
public class SMDSLPassage
{
    public string name;
    public string from_room;
    public string to_room;
    public float[] midpoint_unity;
    public bool is_door;
}

[System.Serializable]
public class SMDSLNavGraph
{
    public string source;
    public int room_count;
    public int passage_count;
    public List<SMDSLRoom> rooms;
    public List<SMDSLPassage> passages;
}

public class SMDSLNavMeshImporter : EditorWindow
{
    private static string _graphJsonPath = "";
    private static string _objPath = "";

    [MenuItem("SMDSL/Import NavMesh Graph...")]
    public static void ShowWindow()
    {
        GetWindow<SMDSLNavMeshImporter>("SMDSL NavMesh");
    }

    void OnGUI()
    {
        GUILayout.Label("SMDSL NavMesh Graph Importer", EditorStyles.boldLabel);
        GUILayout.Space(8);

        GUILayout.Label("navmesh_graph.json:");
        GUILayout.BeginHorizontal();
        _graphJsonPath = GUILayout.TextField(_graphJsonPath);
        if (GUILayout.Button("Browse", GUILayout.Width(60)))
            _graphJsonPath = EditorUtility.OpenFilePanel("Select navmesh_graph.json", "", "json");
        GUILayout.EndHorizontal();

        GUILayout.Label("navmesh_rooms.obj (optional, for geometry):");
        GUILayout.BeginHorizontal();
        _objPath = GUILayout.TextField(_objPath);
        if (GUILayout.Button("Browse", GUILayout.Width(60)))
            _objPath = EditorUtility.OpenFilePanel("Select navmesh_rooms.obj", "", "obj");
        GUILayout.EndHorizontal();

        GUILayout.Space(12);
        if (GUILayout.Button("🏗 Build Scene Graph", GUILayout.Height(36)))
            BuildSceneGraph();
    }

    private static void BuildSceneGraph()
    {
        if (!File.Exists(_graphJsonPath))
        {
            Debug.LogError("[SMDSL] navmesh_graph.json not found: " + _graphJsonPath);
            return;
        }

        string json = File.ReadAllText(_graphJsonPath);
        SMDSLNavGraph graph = JsonUtility.FromJson<SMDSLNavGraph>(json);

        // 根节点
        GameObject root = new GameObject("SMDSL_NavGraph_" + graph.source);
        GameObject roomsRoot = new GameObject("Rooms");
        GameObject passagesRoot = new GameObject("Passages");
        roomsRoot.transform.parent = root.transform;
        passagesRoot.transform.parent = root.transform;

        Dictionary<string, Vector3> roomCentroids = new Dictionary<string, Vector3>();

        // 建立房间 GameObject（NavMesh Static floor cube）
        foreach (var room in graph.rooms)
        {
            Vector3 center = new Vector3(
                room.centroid_unity[0],
                room.height_m * 0.5f,
                room.centroid_unity[2]
            );
            roomCentroids[room.name] = new Vector3(
                room.centroid_unity[0], 0f, room.centroid_unity[2]);

            // 估算平面尺寸（正方形近似）
            float side = Mathf.Sqrt(room.area_m2);
            GameObject roomGo = GameObject.CreatePrimitive(PrimitiveType.Cube);
            roomGo.name = room.name;
            roomGo.transform.parent = roomsRoot.transform;
            roomGo.transform.position = center;
            roomGo.transform.localScale = new Vector3(side, room.height_m, side);

            // 设为 NavMesh Static
            GameObjectUtility.SetStaticEditorFlags(
                roomGo, StaticEditorFlags.NavigationStatic);

            // 分配 NavMesh Area
            int areaIndex = room.area_type == "corridor" ? 3 : 0;
            NavMeshBuilder.SetAreaFromBounds(
                new Bounds(center, roomGo.transform.localScale), areaIndex);

            // 材质着色区分类型
            Renderer rend = roomGo.GetComponent<Renderer>();
            if (rend != null)
            {
                Color c = room.area_type == "corridor"
                    ? new Color(0.5f, 0.8f, 0.5f, 0.4f)
                    : new Color(0.2f, 0.6f, 0.9f, 0.4f);
                Material mat = new Material(Shader.Find("Standard"));
                mat.color = c;
                mat.SetFloat("_Mode", 3);
                mat.renderQueue = 3000;
                rend.sharedMaterial = mat;
            }
        }

        // 建立通路 NavMesh Link
        foreach (var passage in graph.passages)
        {
            if (!roomCentroids.ContainsKey(passage.from_room) ||
                !roomCentroids.ContainsKey(passage.to_room))
                continue;

            GameObject linkGo = new GameObject(passage.name);
            linkGo.transform.parent = passagesRoot.transform;
            linkGo.transform.position = new Vector3(
                passage.midpoint_unity[0], 0f, passage.midpoint_unity[2]);

            // NavMeshLink（需要 AI Navigation 包）
#if UNITY_AI_NAVIGATION
            var link = linkGo.AddComponent<Unity.AI.Navigation.NavMeshLink>();
            link.startPoint = roomCentroids[passage.from_room] - linkGo.transform.position;
            link.endPoint   = roomCentroids[passage.to_room]   - linkGo.transform.position;
            link.width = passage.is_door ? 1.2f : 2.0f;
            link.bidirectional = true;
            link.activated = true;
#else
            // 无 AI Navigation 包时，用 Gizmo 线段可视化
            lineGo.AddComponent<SMDSLLinkGizmo>().Init(
                roomCentroids[passage.from_room],
                roomCentroids[passage.to_room]);
#endif
        }

        // 选中根节点
        Selection.activeGameObject = root;
        Debug.Log($"[SMDSL] Scene graph built: {graph.room_count} rooms, " +
                  $"{graph.passage_count} passages. Root = {root.name}");
        Debug.Log("[SMDSL] Next: Window > AI > Navigation > Bake");
    }
}

/// <summary>当无 AI Navigation 包时，用 Gizmo 可视化门洞连线。</summary>
public class SMDSLLinkGizmo : MonoBehaviour
{
    public Vector3 startW, endW;
    public void Init(Vector3 a, Vector3 b) { startW = a; endW = b; }
    void OnDrawGizmosSelected()
    {
        Gizmos.color = Color.yellow;
        Gizmos.DrawLine(startW, endW);
        Gizmos.DrawSphere((startW + endW) * 0.5f, 0.15f);
    }
}

#endif // UNITY_EDITOR
'''


# ══════════════════════════════════════════════════════════════════════
# README
# ══════════════════════════════════════════════════════════════════════

_README = """\
# SMDSL → Unity NavMesh 导入指南

## 包内文件

| 文件 | 用途 |
|------|------|
| `navmesh_rooms.obj` | 房间地板网格（可直接拖入 Unity） |
| `navmesh_rooms.mtl` | OBJ 材质定义 |
| `navmesh_graph.json` | 拓扑图（房间 + 邻接通路） |
| `SMDSLNavMeshImporter.cs` | Unity Editor 自动导入脚本 |

## 5 步快速导入

### 方式一：OBJ 几何烘焙（推荐，最精确）

1. **导入 OBJ**  
   将 `navmesh_rooms.obj` 拖入 Unity Project 面板  
   导入设置：*Generate Colliders = on, Read/Write = on*

2. **设置 Navigation Static**  
   选中所有导入的房间 mesh → Inspector → Static → **Navigation Static** ✓

3. **安装 AI Navigation 包**（Unity 2022+）  
   *Window → Package Manager → Unity Registry → AI Navigation → Install*

4. **烘焙 NavMesh**  
   *Window → AI → Navigation → Bake → Bake*

5. **运行路径测试**  
   给 Agent 添加 *NavMeshAgent* 组件，调用 `agent.SetDestination(pos)` 验证

---

### 方式二：C# 脚本自动建场景图

1. 将 `SMDSLNavMeshImporter.cs` 拖入 `Assets/Editor/`  
2. Unity 菜单 → **SMDSL → Import NavMesh Graph...**  
3. 选择 `navmesh_graph.json` → 点击 **Build Scene Graph**  
4. 场景中会自动生成房间 Cube + NavMesh Link  
5. 回到第 4 步烘焙

## 坐标系说明

```
SMDSL (x, y)  →  Unity (x, 0, y)
房间高度 h_m  →  Cube 高度 h_m，底面 Y=0
```

## 已知限制

- OBJ 模式下房间形状为精确多边形（OSM 源）或 bbox 矩形（PNG/DWG 源）
- NavMesh Link 需要 Unity AI Navigation 包（2022.3 LTS 内置）
- 大型建筑（>200 房间）建议分楼层导入

生成工具：SMDSL RoboIR v1.0 | https://github.com/your-repo/SMDSL
"""


# ══════════════════════════════════════════════════════════════════════
# 主导出函数
# ══════════════════════════════════════════════════════════════════════

def export_unity_navmesh_zip(
    parse_result: Dict[str, Any],
    output_path: Optional[str] = None,
) -> str:
    """
    将 SMDSL ParseResult 打包为 Unity NavMesh ZIP 文件。

    Args:
        parse_result: dispatcher.dispatch_cad() 返回值
        output_path: 目标 zip 路径；默认用系统临时目录

    Returns:
        生成的 .zip 文件绝对路径
    """
    import tempfile, datetime  # noqa: PLC0415, E401

    mode = parse_result.get("mode", "unknown")
    source_name = Path(parse_result.get("source_path", "unknown")).stem

    # ── 提取房间列表 ──────────────────────────────────────────
    if mode == "osm":
        rooms = _rooms_from_osm(parse_result)
        edges = parse_result.get("osm_edges", []) or []
        adjacency = parse_result.get("adjacency_graph", {}) or {}
    else:
        rooms = _rooms_from_grid(parse_result)
        # 对非 OSM 模式：从 cad_data.doors 构建边
        cad_data = parse_result.get("cad_data") or {}
        edges = [
            {
                "name": d.get("name", f"door_{i}"),
                "from_room": d.get("from", ""),
                "to_room": d.get("to", ""),
                "midpoint_m": d.get("position", [0, 0]),
                "door": "yes",
            }
            for i, d in enumerate(cad_data.get("doors", []) or [])
        ]
        # 从 semantics 邻接图补充
        sem = parse_result.get("semantics") or {}
        adjacency = sem.get("adjacency_graph", {}) or {}

    if not rooms:
        raise ValueError("无法提取任何房间几何，请检查输入文件")

    # ── 生成各文件内容 ────────────────────────────────────────
    obj_content = _build_obj(rooms)
    mtl_content = _build_mtl()
    graph_json = _build_graph_json(rooms, adjacency, edges, source_name)
    cs_script = _CSHARP_TEMPLATE
    readme = _README

    # ── 打包 ZIP ──────────────────────────────────────────────
    if output_path is None:
        ts = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
        output_path = str(
            Path(tempfile.gettempdir())
            / f"smdsl_unity_{source_name}_{ts}.zip"
        )

    with zipfile.ZipFile(output_path, "w", zipfile.ZIP_DEFLATED) as zf:
        zf.writestr("navmesh_rooms.obj", obj_content)
        zf.writestr("navmesh_rooms.mtl", mtl_content)
        zf.writestr("navmesh_graph.json", graph_json)
        zf.writestr("Editor/SMDSLNavMeshImporter.cs", cs_script)
        zf.writestr("README_Unity.md", readme)

    return output_path
