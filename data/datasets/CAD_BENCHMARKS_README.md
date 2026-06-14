# CAD 基准数据集（`data/`）

本目录汇总 SMDSL Demo 1 可用的三类工业/导航 CAD 测试数据。

## 1. osmAG-from-cad（已就绪，跳过重复下载）

| 项 | 值 |
|---|---|
| 来源 | https://github.com/jiajiezhang7/osmAG-from-cad |
| 本地路径 | `data/osmAG-from-cad/cad2osm/data/web-cad/dwg/` |
| 规模 | 32× DWG + 32× DXF |
| 用途 | 双线墙、拓扑破碎、房间漫水不全等极端导航测试 |

`fish_processing_plant.dwg` 亦在此语料中（约 14 MB，高密度工业图元）。

## 2. 鞋类制造仓库（Mendeley，已下载）

| 项 | 值 |
|---|---|
| 来源 | https://data.mendeley.com/datasets/pf2w725pw3/1 |
| 本地路径 | `data/footwear_manufacturing_warehouse/` |
| 许可 | CC BY 4.0 |
| 关键文件 | `Full layout.dwg`、多层 `Layout_Z*.svg/pdf`、CSV 库位/波次/脚本 |

自动下载端点：`https://data.mendeley.com/public-api/zip/pf2w725pw3/download/1`

## 3. Autodesk Factory Design Utilities（部分就绪）

| 项 | 值 |
|---|---|
| 官方教程包 | http://www.autodesk.com/fdu-tutorials1-4_download |
| 本地路径 | `data/dwg_samples/autodesk_official/` |
| 已有 | `fish_processing_plant.dwg`（与 osmAG 语料同源） |
| 待手动 | `Factory Footprint.dwg`（需从 FDU 教程 ZIP 解压，CDN 对脚本 403） |

手动步骤：浏览器打开官方链接 → 下载并解压到 `data/autodesk_fdu_tutorials/` → 将 `Factory Footprint.dwg` 复制到 `data/dwg_samples/autodesk_official/`。

## 一键刷新

```powershell
python data/datasets/download_cad_datasets.py
```

清单：`data/datasets/cad_benchmark_manifest.json`
