# SMDSL 方向一 修改方案 v1

> 版本：v1 · 基于方向一 2D 极致打磨 + 交互测试反馈
> 状态：已完成审查，待按 Phase 顺序执行

---

## 一、当前项目状态总览

### 1.1 已完成（本次会议）

| # | 改动 | 文件 | 行数 |
|---|------|------|------|
| ✅ | Z 轴硬断言（Fix 1） | `smdsl_demo/spatial_api_stub.py` | ~13 行 |
| ✅ | 架构图 4-Zone 可视化 | `smdsl_demo/architecture_viz.py` | 142 行 |
| ✅ | 演示录制（Playwright） | `smdsl_demo/demo_recorder.py` | 243 行 |
| ✅ | 梯形速度 Profile | `smdsl_demo/motion_profile.py` | 129 行 |
| ✅ | 扩展启动器（架构+录制+运动沙箱） | `smdsl_demo/launch_smdsl.py` | 237 行 |
| ✅ | 独立工具启动器 | `smdsl_demo/app_tabs.py` | 149 行 |
| ✅ | 执行 Workflow 提示词 | `SMDSL_Direction1_Workflow.md` | 16 KB |

### 1.2 未完成（待执行队列）

| Phase | 改动 | 文件 | 预估 |
|-------|------|------|------|
| 1.2 | RoboIR diff 硬校验 | `tests/test_closed_loop_recovery.py` | 1.5h |
| 1.3 | DWG 实体丢弃率告警 | `cad_parser/dispatcher.py` + `app.py` | 2h |
| 2.1 | 轨迹合成接入 | `trajectory_smoother.py` → `app.py` | 1h |
| 2.2 | 距离场梯度微调 | `cad_parser/astar_topology.py` → `app.py` | 2h |
| 2.3 | EDT vs Costmap 基准 | `benchmark/bench_edt_vs_costmap.py` | 3h |
| 3.1 | app.py 模块拆分 | `ui_theme.py` + `ui_common.py` | 1.5h |
| 4 | 测试 | 多个测试文件 | 3h |
| 5 | 基准运行 | `benchmark/` | 2h |
| 6 | 文档 | `README.md` + `CHANGELOG.md` | 1h |

### 1.3 已有但未审查的文件

| 文件 | 大小 | 说明 |
|------|------|------|
| `trajectory_smoother.py` | 4.6 KB | 已实现三次样条 + 梯形 Profile，文档注明 Phase 2.1 |
| `ui_theme.py` | 20.8 KB | 完整的 Gradio 主题系统（Apple 设计语言） |
| `ui_common.py` | 3.2 KB | 共享 UI 组件 |

⚠️ **注意**：`motion_profile.py`（本会议新建）与 `trajectory_smoother.py`（预置）功能重叠，需决定去留。

---

## 二、交互测试发现的问题

### 问题 A：Tab 1 环境感知页文字过冗

**位置**：`app.py` L1837~L1880 附近
**症状**："这一步在做什么" 区域有 40+ 行的 Markdown 说明，但用户只需要知道功能区按钮的流程。
**修改**：将长篇说明精简为 5 行以内，只说"按什么按钮 → 做什么操作"。

示例精简版：
```
### 操作流程
1. 选择/上传 CAD 文件 → 系统自动解析
2. 解析完成后查看 4 联图（栅格/距离场/A* 热力/拓扑标签）
3. 从下拉列表选择起终点 → 点击"规划路径" → 查看路径和轨迹
4. 轨迹自动同步至 Demo 3，可进行 STL 验证
```

### 问题 B：文件浏览路径错误

**位置**：`app.py` L72
**当前值**：
```python
_DATA_ROOT = str(_REPO_ROOT / "data" / "cad_samples")
```
解析后为 `D:\Code\SMDSL_demo\data\cad_samples`。

**正确值**：
```python
_DATA_ROOT = str(_REPO_ROOT / "SMDSL" / "data" / "cad_samples")
```
解析后为 `D:\Code\SMDSL_demo\SMDSL\data\cad_samples`。

**影响**：文件浏览器（L1930）的 `root_dir` 和预设样本路径（L81~L82、L1733）均使用 `_REPO_ROOT` 拼接，修正 `_DATA_ROOT` 后需确认所有相关路径一致。

### 问题 C：快速预览路径未修正

**位置**：`app.py` L1886~L1898
**症状**：示例文件路径硬编码为 `D:\code_backup_dwg\data\cad_samples\...`，在现有环境不存在。
**修改**：将预设路径改为相对于 `_REPO_ROOT` 的动态路径，或统一使用 `_DATA_ROOT` 拼接。

### 问题 D：机器人半径 (m) slider 是否保留

**位置**：`app.py` L1975~L1980
**当前**：`gr.Slider(label="机器人半径 (m)", ...)`，默认 0.15m，附带说明文本。
**建议**：**保留**。因为它是直接影响 A* 搜索代价和距离场安全检查的参数，是 2D 路径规划的核心参数。但可以将附带说明简化，只说"影响路径与障碍物的最小距离"。

### 问题 E：3D 拓扑白模预览 → 删除

**位置**：`app.py` L1995~L1997（Accordion "🌐 3D 拓扑白模预览" + Button "🏗 生成 3D 白模预览"）
**对应的函数**：`demo1_3d_preview`（L1145~L1163）
**对应的回调**：`d1_3d_btn.click`（L2158~L2161）
**修改**：删除该 Accordion + Button + Plot + 对应的回调绑定 + 对应的 handler 函数。
**原因**：方向一聚焦 2D，3D 白模预览与方向一目标无关。

### 问题 F：3D 空间场景图 → 删除

**位置**：`app.py` L1999~L2012（Accordion "🌆 3D 空间场景图" + Markdown 说明 + Button "🏙 生成 3D 空间场景图"）
**对应的函数**：`demo1_scene_graph_3d`（L1166~L1227）
**对应的回调**：`d1_scene3d_btn.click`（L2165~L2172）
**修改**：删除该 Accordion + Markdown + Button + Plot + 对应的回调绑定 + handler 函数。
**原因**：方向一聚焦 2D。其中 OSM 模式的 3D 房间图与此无关。

### 问题 G：Demo 3 的 3D Trajectory Sandbox → 改为 2D

**位置**：`app.py` L2463~L2466（Plot label "📊 3D Trajectory Sandbox"）
**对应的函数**：`demo3_run` 调用 `generate_3d_dashboard`（L1698~L1700）
**当前行为**：生成 `fig_3d`（Plotly 3D 场景）+ `fig_rho`（ρ 曲线）
**修改**：
- 将 `fig_3d` 从 3D Plotly 场景切换为 2D matplotlib 路径叠加图
- 保留 `fig_rho`（ρ 曲线是一维时间序列，本身就是 2D）
- 标签从 "3D Trajectory Sandbox" 改为 "2D 轨迹叠加图"

---

## 三、累积修改清单（按文件分组）

### app.py（最多改动）

| 行号 | 改动类型 | 内容 |
|------|---------|------|
| L72 | 修改 | `_DATA_ROOT` 路径修正 |
| L81~L87 | 修改 | 预设样本路径修正 |
| L1837~L1880 | 替换 | 长说明 → 精简操作流程 |
| L1886~L1898 | 修改 | 预设样本示例路径修正 |
| L1905 | 追加 | 窗口文件选择器：默认根目录设为 `_DATA_ROOT` / "SMDSL" / "data" / "cad_samples" |
| L1975~L1980 | 保留 | 机器人半径 slider（说明简化） |
| L1995~L2012 | 删除 | 3D 拓扑白模预览 + 3D 空间场景图区块 |
| L2465 | 修改 | Label 从 "3D Trajectory Sandbox" → "2D 轨迹叠加图" |
| L2158~L2172 | 删除 | 3D 相关的回调绑定 |
| L1698~L1700 | 修改 | `generate_3d_dashboard` → 改用 2D 叠加图 |

**对应的 handler 和 import 清理**：
- 删除 `demo1_3d_preview`（L1145~L1163）
- 删除 `demo1_scene_graph_3d`（L1166~L1227）
- import 中移除 `generate_3d_dashboard`，替换为 2D 版本

### 其他文件

| 文件 | 改动 |
|------|------|
| `trajectory_smoother.py` | 确认功能完整性，与 `motion_profile.py` 合并 |
| `motion_profile.py` | 视合并结果决定保留或删除 |
| `cad_parser/astar_topology.py` | 新增 `compute_field_gradient` + `refine_path_via_gradient`（Phase 2.2） |
| `cad_parser/dispatcher.py` | DWG 实体统计 + note 字段（Phase 1.3） |
| `tests/test_closed_loop_recovery.py` | RoboIR diff 校验（Phase 1.2） |
| `benchmark/bench_edt_vs_costmap.py` | 新建（Phase 2.3） |
| `README.md` | 更新方向一进展 |
| `CHANGELOG.md` | 新建 |

---

## 四、执行顺序建议

```
Phase 1.2 (RoboIR diff)        ─── P0 安全修复
    ↓
Phase 1.3 (DWG 丢弃告警)       ─── P0 可见性修复
    ↓
app.py UI 修正 ① 路径/文案/删除3D  ─── 交互问题修复
    ↓
trajectory_smoother.py 审查
    ↓
Phase 2.1 (轨迹合成接入)       ─── P1 优化
    ↓
Phase 2.2 (梯度微调)          ─── P1 优化
    ↓
Phase 3.1 (模块拆分)          ─── 重构
    ↓
Phase 4 (测试)                ─── 质量保障
    ↓
Phase 2.3 (EDT vs Costmap)    ─── 基准（核心交付物）
    ↓
Phase 5+6 (基准运行 + 文档)   ─── 收尾
```

---

## 五、建议记录

1. **`motion_profile.py` vs `trajectory_smoother.py`**：优先审查 `trajectory_smoother.py`，若它已完整实现所需功能，则删除 `motion_profile.py`，将我这里实现的 `trapezoidal_velocity_profile` 函数合并过去。
2. **`generate_3d_dashboard` 的 2D 替代**：可以直接用 `visualize_demo3.py` 中的 `render_trajectory_overlay`（虽然标记为 deprecated）或者用 matplotlib 新建一个简单的 2D 路径叠加图，去掉 Plotly 3D 依赖。
3. **文件选择器 UI**：`gr.FileExplorer` 配合 `_DATA_ROOT` 路径。修正后需测试是否可正确浏览到实际数据目录。
