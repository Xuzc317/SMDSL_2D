# CAD Samples

Gradio UI 预设样本目录。每个子目录对应一种 CAD 格式：

| 子目录 | 格式 | 说明 |
|--------|------|------|
| `floorplanqa/layouts/` | JSON | 程序化生成的房间布局（synthetic_benchmark 子集） |
| `svg_samples/` | SVG | SVG 平面图（待添加） |
| `raster_samples/` | PNG/JPG | 栅格化平面图（待添加） |

## 添加新样本

将文件放入对应子目录后，在 `SMDSL/smdsl_demo/app.py` 的 `_PRESET_CANDIDATES` 和 UI 预设列表中注册路径即可。

DWG 格式样本请使用 `data/dwg_samples/` 目录中的文件（已追踪）。
