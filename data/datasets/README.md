# Future Dataset Integration: ArchCAD-400K & ResPlan

This directory is reserved for large-scale CAD datasets used for fine-tuning
language models (LLMs) or training Vision-Language-Action (VLA) models.

## ArchCAD-400K

**Description**: Large-scale dataset of CAD floor plans with paired annotations.
Suitable for training semantic extraction models on architectural drawings.

**Status**: Not yet downloaded.

**Download**: (URL to be confirmed — check official repository)
- Expected format: DWG/DXF + JSON annotations
- Expected size: ~400K samples, ~50-100 GB

**Schema Mapping** (to our `cad_data` dict):

| ArchCAD Field | Our `cad_data` Key | Notes |
|---------------|-------------------|-------|
| `floor_boundary` | `boundary` | May need coordinate normalization |
| `wall_segments` | `walls` | Already compatible |
| `room_labels` | `objects[*].label` | Text annotations → LLM training data |
| `door_positions` | `doors` | Polygon representation |
| `layer_metadata` | Track B semantics | For `semantic_extractor.py` |

**Batch Processing**: Use `data/dwg_samples/run_full_pipeline.py` with
`--resolution 0.02 --min-region-area 2.0`. For 400K files, distribute across
multiple workers using `cad_parser/sweep.py` architecture as reference.

**Integration Point**: The `cad_parser/dispatcher.py:dispatch_cad()` function
already handles extension-based routing. Add a `archcad` mode in
`dispatch_cad()` that maps ArchCAD's schema onto our `cad_data` dict before
passing to `rasterize_to_grid()`.

## ResPlan

**Description**: Residential floor plan dataset with structural element
annotations. Good for training room-type classification and wall detection.

**Status**: Not yet downloaded.

**Download**: (URL to be confirmed)
- Expected format: PNG/JPG images + JSON segmentation masks
- Expected size: TBD

**Schema Mapping**:

| ResPlan Field | Our Pipeline Input | Processing |
|---------------|-------------------|------------|
| `image` (PNG) | `dispatch_cad(png_path)` | Uses `_parse_raster()` path |
| `segmentation_mask` | Compare against `classify_topology_global()` | Room segmentation accuracy |
| `room_type_labels` | `semantic_extractor` training data | LLM fine-tuning |

## VLA Training Pipeline (Conceptual)

For Vision-Language-Action models that consume spatial topology:

```
DWG/PDF → cad_parser pipeline → AreaGraph JSON → SMDSL compiler → RoboIR constraints
                                                         ↓
                                              VLA training sample
```

Each training sample would contain:
1. **Vision**: Rendered topology grid / distance field as input image
2. **Language**: SMDSL constraint text (e.g., "从 Loading_Zone_A 移动到 Unloading_Zone_B，保持与墙壁 > 0.3m 距离")
3. **Action**: A* trajectory waypoints + velocity profile from `astar_shortest_path()`

## Notes

- All datasets are gitignored. Use `data/datasets/.gitkeep` to preserve the directory structure.
- For batch processing, ensure LibreDWG (`dwgread`) is installed for DWG-based datasets.
- The `DEEPSEEK_API_KEY` environment variable is required for `semantic_extractor.py` LLM node injection.
