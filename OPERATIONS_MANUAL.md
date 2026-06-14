# SMDSL_2D 操作手册

> 面向开发者和使用者的完整操作指南。最后更新：2026-06-15

---

## 1. 环境准备

### 1.1 系统要求

- Python 3.10+
- Windows 10+ / Linux / macOS
- 8GB+ RAM（处理大型 DWG 文件建议 16GB）
- Git

### 1.2 安装依赖

```bash
cd D:\Code\SMDSL_demo
pip install -r requirements.txt

# 可选：真实路网基准测试
pip install osmnx
```

### 1.3 配置 API Key（仅 Demo 2/3 需要 LLM）

```powershell
$env:DEEPSEEK_API_KEY = "sk-..."
$env:NO_PROXY = "localhost,127.0.0.1"
```

### 1.4 验证安装

```bash
# 运行全部测试
python -m pytest SMDSL/tests/ -v

# 预期输出：38 passed
```

---

## 2. 快速启动

### 2.1 启动 Gradio 调试面板

```bash
cd D:\Code\SMDSL_demo\SMDSL
python -m smdsl_demo.app
```

浏览器打开 `http://127.0.0.1:7860/`

面板包含三个 Tab：
- **Demo 1 — 环境感知**：加载 CAD 文件，查看距离场/拓扑，规划路径
- **Demo 2 — 语义编译**：自然语言 → RoboIR 编译
- **Demo 3 — 物理求解**：STL 鲁棒度验证 + 结构化反馈

### 2.2 支持的 CAD 格式

| 格式 | 扩展名 | 说明 |
|------|--------|------|
| FloorplanQA JSON | `.json` | 结构化的房间布局，含墙/门/家具 |
| AutoCAD DWG | `.dwg` | 工业 CAD 图纸（需 LibreDWG）|
| OpenStreetMap | `.osm` | 原生拓扑（多边形+邻接图）|
| SVG | `.svg` | 矢量图形 |
| PNG/JPG | `.png`, `.jpg` | 栅格图像（二值化为占据图）|

---

## 3. 数据集管理

### 3.1 生成合成基准数据

```bash
cd D:\Code\SMDSL_demo

# 生成 200 个合成房间布局（用于基准测试）
python data/generate_synthetic_layouts.py \
    --n_layouts 200 \
    --output data/datasets/synthetic_benchmark \
    --seed 42
```

生成的数据结构：
```
data/datasets/synthetic_benchmark/
├── bedroom/        # 卧室布局
├── living_room/    # 客厅布局
├── kitchen/        # 厨房布局
├── bathroom/       # 浴室布局
├── corridor/       # 走廊布局
├── office/         # 办公室布局
├── warehouse/      # 仓库布局
└── manifest.json   # 数据集元信息
```

### 3.2 下载真实路网数据

> **网络要求**: OSMnx 需要访问 `nominatim.openstreetmap.org` (443端口)。
> 在中国大陆网络环境下需要配置 HTTPS 代理或 VPN 才能正常下载。

```bash
# 如使用代理，先设置环境变量
$env:HTTPS_PROXY = "http://127.0.0.1:7890"

# 下载指定城市路网
python data/download_osm_networks.py \
    --city "Beijing, China" \
    --n_networks 5

# 使用预设批量下载
python data/download_osm_networks.py \
    --preset mixed \
    --n_networks 20 \
    --output data/datasets/osm_networks

# 预设选项: campus | residential | industrial | mixed | all
```

### 3.3 数据集目录约定

```
data/datasets/
├── synthetic_benchmark/    # 合成布局（生成）
├── osm_networks/           # OSM 真实路网（下载）
└── README.md               # 数据集说明
```

---

## 4. 运行基准测试

### 4.1 EDT vs Costmap 对比基准

```bash
# 合成数据快速测试
python benchmark/bench_edt_vs_costmap.py

# 完整基准：200 布局 × 5 起终点对
python benchmark/bench_edt_vs_costmap.py \
    --data_dir data/datasets/synthetic_benchmark \
    --n_layouts 200 \
    --n_pairs 5 \
    --output benchmark/results/edt_vs_costmap_200.json

# OSM 真实路网基准
python benchmark/bench_edt_vs_costmap.py \
    --data_dir data/datasets/osm_networks \
    --n_layouts 30 \
    --n_pairs 5 \
    --output benchmark/results/edt_vs_costmap_osm.json
```

### 4.2 解读基准输出

```
================================================================================
  EDT vs Costmap — Comparison Results
================================================================================
room_type       | method     | clearance_min(mm)  | clearance_mean(mm)  | time(ms)
----------------+------------+--------------------+---------------------+----------
bedroom         | EDT        |            524.9 |             797.8 |      7.2
                | costmap    |            476.4 |             695.8 |      2.7
...
================================================================================
  Summary: EDT clearance_min improvement over costmap: 12.1%
```

- **clearance_min**: 路径上距离障碍物的最小距离（关键安全指标）
- **clearance_mean**: 路径平均安全距离
- **time(ms)**: 规划耗时

---

## 5. 开发工作流

### 5.1 分支策略

```
main          ← 受保护，禁止直接 push
├── fix/*     ← P0 修复
├── opt/*     ← P1 优化
├── refactor/* ← 重构
├── bench/*   ← 基准测试
└── test/*    ← 测试补充
```

### 5.2 提交规范

```
[Phase X.Y] 简短描述（文件:函数）

- 具体改动点
- 影响范围
```

### 5.3 运行测试

```bash
# 全部单元测试
python -m pytest SMDSL/tests/ -v

# 特定测试文件
python -m pytest SMDSL/tests/test_z_axis.py -v

# 集成测试（需要 DEEPSEEK_API_KEY）
python SMDSL/tests/test_closed_loop_recovery.py
```

### 5.4 代码检查

```bash
# 语法检查
python -m py_compile SMDSL/smdsl_demo/app.py

# 导入验证
cd SMDSL && python -c "from smdsl_demo.trajectory_smoother import smooth_path_to_trajectory"
```

---

## 6. 常见问题

### Q: 启动 Gradio 报错 `ModuleNotFoundError`
**A**: 确保在 `SMDSL/` 目录下运行，或将 `SMDSL/` 加入 `PYTHONPATH`。

### Q: DWG 文件加载失败
**A**: DWG 需要 LibreDWG。先确认文件路径正确，尝试用 JSON 格式文件测试。

### Q: 基准测试报 `No module named 'cad_parser'`
**A**: 在项目根目录 `D:\Code\SMDSL_demo` 下运行，不要在子目录运行。

### Q: OSM 下载超时
**A**: 网络问题。减少 `--n_networks` 数量，或使用 `--city` 指定单个位置重试。

---

## 7. 项目结构速查

```
SMDSL/                          # 主代码目录
├── cad_parser/                 # Zone 1: 几何/拓扑解析
│   ├── astar_topology.py       # EDT + A* + 梯度微调
│   ├── dispatcher.py           # 多格式 CAD 入口
│   └── ...
├── smdsl_demo/                 # Zone 2-4 + UI
│   ├── app.py                  # Gradio 主 UI
│   ├── vlm_parser.py           # Zone 2: NL→RoboIR
│   ├── spatial_api_stub.py     # Zone 3: STL 物理求解
│   ├── metrics.py              # Zone 4: 结构化反馈
│   ├── trajectory_smoother.py  # 样条+梯形 Profile 轨迹合成
│   ├── ui_theme.py             # CSS + 主题
│   └── ui_common.py            # 共享 UI 工具
├── tests/                      # 测试目录 (38 tests)
├── data/                       # 数据目录
└── benchmark/                  # 基准测试
```
