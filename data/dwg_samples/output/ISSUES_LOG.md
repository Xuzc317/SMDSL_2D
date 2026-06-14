# DWG Pipeline Issues Log

Generated: 2026-05-15 | Branch: `feature/dwg-industrial-parsing` | Commit: `0331850`

---

## Issue #1: fish_processing_plant.dwg — dwgread JSON 输出溢出

**严重程度**: 高
**影响文件**: `data/dwg_samples/autodesk_official/fish_processing_plant.dwg`

**现象**:
```
dwgread 输出非合法 JSON：Expecting value: line 12255351 column 9 (char 320317982)
```

- JSON 输出达到 ~320MB，行数超过 1200 万行
- dwgread 写入临时文件时被截断，导致 JSON 解析失败
- 该 DWG 是工业鱼加工厂布局，实体密度极大

**根因分析**:
1. `parse_dwg_to_json()` 使用 `subprocess.run()` 将 stdout 重定向到临时文件
2. 临时文件写入无大小限制，但 dwgread 在处理超大文件时可能输出被操作系统 pipe buffer 截断
3. 320MB JSON 单次解析也接近 Python `json.loads()` 的内存舒适边界

**建议修复**:
- 对实体数 > 50000 的 DWG 启用流式解析（`ijson`）或分块写入
- 增加 `dwgread` subprocess 的 `--max-entities` 限制选项（如果 LibreDWG 支持）
- 对大文件自动降级为几何-only 模式（跳过非必要实体类型）
- 短期方案：将 `MAX_DWG_SIZE_BYTES` 从 50MB 降到 10MB 以过滤此类超大文件，返回明确错误提示

---

## Issue #2: 双线墙体导致区域碎片化（Wall Fragmentation）

**严重程度**: 中
**影响文件**: 所有 DWG 建筑图纸（尤其是 osmAG 语料中的 museum/hotel/office）

**现象**:
- `historical_museum.dwg` 经管线产出 **87 个 PATH 连通分量，仅 4 条边**
- 大部分区域面积 < 2m²（墙腔/线间夹缝被误识别为房间）
- 主要区域 0/1/4 面积分别为 818、280、284 m²，彼此不连通

**根因分析**:
1. DWG 建筑图纸通常使用**双线表示墙体**（两条平行 LINE 表示墙厚）
2. 当前 `bridge_thin_walls(kernel_px=2)` 闭运算核半径太小，无法跨越双线间距
3. 双线间距在 0.02m/px 分辨率下约为 5-10px（对应 10-20cm 墙厚），2px 闭运算只能弥合单像素裂缝
4. DWG 中门的表示方式（间隙/短线）在栅格化后产生 1-3px 墙体裂缝

**建议修复**:
- DWG 路径使用 `bridge_thin_walls(kernel_px=5)`（当前为 2）
- 或在 `_parse_dwg()` 中 pre-process wall segments：合并间距 < 30cm 的平行线段为单一厚墙
- 参考 osmAG-from-cad 的 `dxf_filter.py` 做法：在矢量层面过滤非墙体图层后再栅格化
- 添加自适应 `kernel_px` 计算：根据 `wall_thickness_m / resolution` 自动设定

---

## Issue #3: SPLINE / ELLIPSE / HATCH 实体未处理

**严重程度**: 中
**影响文件**: 跨语料统计到 14,226 SPLINE + 3,068 ELLIPSE + 1,482 HATCH

**现象**:
- `cad_parser/dispatcher.py:_extract_dwg_geometry()` 仅处理 LINE、LWPOLYLINE、POLYLINE、CIRCLE、ARC
- SPLINE（样条曲线）、ELLIPSE（椭圆）、HATCH（填充图案）直接跳过
- 导致墙体/区域边界不完整，尤其影响弧形建筑和图案填充区域

**根因分析**:
```
_dispatcher.py 当前支持的实体集：
  LINE / LWPOLYLINE / POLYLINE2D / POLYLINE3D / CIRCLE / ARC

统计到的未处理实体（cross-file aggregate）：
  SPLINE:   14,226   ← 样条曲线，常见于有机形态建筑
  ELLIPSE:   3,068   ← 椭圆，常见于圆形大厅/体育场
  HATCH:     1,482   ← 填充，常见于墙体/区域标识
  SOLID:       151   ← 实体填充面
  REGION:       17   ← 二维区域
  3DFACE:       16   ← 三维面
  XLINE:         8   ← 无限构造线
  RAY:           8   ← 射线
```

**建议修复**:
- SPLINE → 采样为折线段（与 CIRCLE→24 段多边形同理），采样密度由 `resolution` 控制
- ELLIPSE → 参数方程采样为多边形（同 CIRCLE 逻辑）
- HATCH → 可选：提取其 boundary loop 并当做墙体线段处理；或跳过（填充通常覆盖已有墙线）
- 在 `_extract_dwg_geometry()` 中添加 `spline_samples` 参数

---

## Issue #4: 实体类型码映射不完整

**严重程度**: 低
**影响文件**: `cad_parser/dispatcher.py` — `_DWG_TYPE_MAP`

**现象**:
- 当前 `_DWG_TYPE_MAP` 仅覆盖 8 种类型码（1, 7, 18, 19, 21, 22, 23, 44, 45）
- LibreDWG JSON 的 OBJECTS 中频繁出现未映射的数值类型码：
  - `48` — 出现频率极高，可能是 DIMENSION 或 LEADER 相关
  - `50` — 中等频率
  - `52` — 中等频率
  - `42` — 76 次（example_2000.dwg）
  - `79` — 264 次，频率最高
  - `507`, `509` — 分别 24、17 次

**根因分析**:
LibreDWG 的 `entity` 字段有两种编码方式：
1. 字符串名（`"LINE"`, `"CIRCLE"`, `"MTEXT"` 等）
2. 数值 type code（对应 DWG 内部对象类型枚举）

当前 `_find_entities()` 对没有字符串 `entity` 字段的对象尝试用 `_DWG_TYPE_MAP` 映射，但映射表不完整，导致大量对象被静默丢弃。

**建议修复**:
- 查阅 LibreDWG 源码中的 `DWG_TYPE_*` 枚举（`dwg.h`）补全映射表
- 对于非图形实体（Object DB 条目、样式表、字典等），显式标记为 `_DWG_SKIP_TYPES` 并跳过
- 添加 `--debug-entities` 模式打印未映射的类型码及出现次数

---

## Issue #5: Windows 终端 GBK 编码错误

**严重程度**: 低（仅影响 CLI 输出显示）
**影响文件**: 所有打印 Unicode 字符的脚本

**现象**:
```
UnicodeEncodeError: 'gbk' codec can't encode character '✓' in position 2
```
- ✓ (U+2713) / ✗ (U+2717) / — (U+2014) 等 Unicode 字符在 Windows GBK 终端无法渲染
- 输出中出现乱码 `��`

**建议修复**:
- 所有脚本统一使用 ASCII-safe 状态标识（PASS/FAIL/SKIP 替代 ✓/✗/—）
- 或在脚本开头设置 `sys.stdout.reconfigure(encoding='utf-8')` (Python 3.7+)
- `run_full_pipeline.py` 和 `batch_dwgread.py` 已修复，`test_area_graph.py` 已修复

---

## Issue #6: hotel-ground-floor-plan.dwg 超大文件内存压力

**严重程度**: 中
**影响文件**: `data/dwg_samples/autodesk_official/hotel-ground-floor-plan.dwg`

**现象**:
- 127,945 实体，94 图层
- dwgread JSON 输出 157MB
- `json.loads()` 解析耗时显著，内存占用 ~500MB+
- 后续 `rasterize_to_grid()` 对 127K 墙线逐条 bresenham 栅格化，耗时数秒

**根因分析**:
- 酒店首层平面图包含大量细节（家具、卫浴、细线标注）
- 当前管线将所有 LINE 实体都当做潜在墙体参与栅格化
- 没有按图层过滤：家具层（furniture）、标注层（dimension）、文字层的 LINE 也被栅格化为墙

**建议修复**:
- 在 `_parse_dwg()` 中添加图层过滤参数 `wall_layers`，仅栅格化指定图层的线段
- 利用 `_extract_dwg_semantics()` 产出的 `layers` 列表，在 UI 中让用户选择墙体图层
- 对超大实体集启用下采样：合并共线且相邻的 LINE 段（减少 bresenham 调用）
- 参考 osmAG-from-cad 的 `dxf_filter.py` 图层关键词过滤策略
