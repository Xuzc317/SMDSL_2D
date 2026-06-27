任务：重新生成正确的 results.csv

当前状况：
  仓库中 results.csv（38fb03c）是 --quick 模式生成的，只有 10 张地图。
  它只是一个管线烟雾测试（pipeline smoke test），不是论文可用的实验数据。

要求（论文级实验数据）：

1. 数量：--n-maps 10000

2. 难度分层：
   当前 --difficulty mixed 是随机分布，我们需要等量分层。
   在 main_benchmark.py 的 _derive_map_config 中，改成在 easy/medium/hard/extreme 之间轮流分配（每张地图顺序轮换），而不是随机选择。
   这样保证 10,000 张地图 = 2,500 easy + 2,500 medium + 2,500 hard + 2,500 extreme。

3. 参数校准（重要）：
   当前 quick test 数据显示 EDT clearance_min 全部是 50mm（1 像素），costmap 9/10 失败。
   这说明 narrow_width / robot_radius 的比例需要校准，否则数据没有区分度。
   调整 DIFFICULTY_PRESETS 中的窄通道宽度：
     easy:    narrow_width=18, wide_width=60   → 两方法都有合理路径
     medium:  narrow_width=12, wide_width=50   → EDT 路径居中，Costmap 开始贴边
     hard:    narrow_width=8,  wide_width=42   → Costmap 被膨胀区堵塞
     extreme: narrow_width=5,  wide_width=36   → 极限通道，仅 EDT 可通过
   参考 robot_radius_px=6。

4. 执行：
   调整参数后，运行全量 benchmark：
     cd D:\Code\SMDSL_demo\SMDSL\benchmark
     python main_benchmark.py --n-maps 10000 --workers 8

   等待完成后，确认 results.csv 有 20,002 行（表头 + 20,000 数据行）。
   确认数据分布：easy/medium/hard/extreme 各 ~5,000 行（含 EDT + Costmap 对）。
   确认后提交到 git。
