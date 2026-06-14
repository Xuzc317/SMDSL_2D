# SMDSL（RoboIR）Demo 开发历程手记

> 本文记录从基线核查到当前 Gradio 三 Tab 集成过程中，**踩过的坑、为什么那样改、改完啥效果**。写法偏口语，但关键概念会点到为止，方便后来人或测试同学对齐上下文。

---

## 一、项目一开始在干啥

我们要做的是「四大执行区」拆开：**感知 / 语义编译 / 约束求解 / 结构化反馈** 不要糊在一坨里。Demo 1 管 CAD→栅格→距离场→拓扑；Demo 2 管自然语言→RoboIR（大模型只当翻译官，别算坐标）；Demo 3 管轨迹+距离场算 ρ、 violation、给机器看的 JSON。

基线核查时发现：数据路径、桩代码、和「解耦原则」对不齐的地方一堆，得先收拾再谈功能。

---

## 二、Demo 1：从「能跑」到「别被外面那片白骗了」

### 2.1 单源漫水 + 整图自由空间 = 外部泛洪（Exterior Flooding）

**现象：** 工厂/户型图外面往往有一大块「自由像素」（padding、白底、或者 SVG 初始全 1 再画墙）。A* / 多源 Dijkstra 从门洞种子一漫水，**最大连通域经常落在外圈**，图上绿一大片都在建筑外，屋里反而像配角。

**原因（说人话）：** 墙内墙外在栅格里都是「1=可走」，算法又不知道哪边是「房间」；谁面积大、谁先从边缘和种子连上，谁就像主路。这不是模型傻，是**拓扑语义没进栅格**。

**改法：** 在 `rasterize_to_grid` 出栅格之后、算 EDT 之前，加 **`remove_exterior_freespace`**：从四条边界的自由像素做连通域（或等价 flood），把「贴边连出去」的那一整坨标成外部，**强制改成墙（0）**。顺带对障碍层做了小核闭运算，堵 1～2 像素的墙缝，避免外圈从缝渗进室内。

**效果：** 绿区基本回到建筑轮廓里；JSON 户型 + padding 场景改善明显。PNG 线稿若本身没有「封闭边界」，一度会被删光，后面又加了**护栏**：若删完自由像素几乎没了，就**回退**不切，避免误杀整张图。

### 2.2 门洞太细、膨胀半径一大 = 室内碎成好多岛

**现象：** 门/窗在栅格里就一两像素宽，再叠一个 `robot_radius` 的「可走阈值」，房间之间**不连通**了；单源漫水只能覆盖一个岛，别的屋永远灰着。

**原因：** 这是**拓扑碎片化**，不是「A* 写错了」；几何上就是窄通道被膨胀吃掉或 CAD 线太细。

**改法：**  
1. **`bridge_thin_walls`**：对自由区域做形态学闭运算，专门「啃」掉特别细的门框线/标注线，让相邻房间在栅格上重新连起来。  
2. **拓扑不再依赖单源 BFS 漫水界定可走区**：改成 **`classify_topology_global`**——在整张内部掩码上，直接用距离场和 `robot_radius` 做矩阵阈值：哪里离墙够远就是绿（path），不够就是橙（inflated），墙就是黑。这样**不跟种子赌运气**。  
3. UI 上把 **连通域个数、主区占比** 打出来，一眼能看出「还有没有孤岛」。

**效果：** 大部分 FloorplanQA 房间主区能合并到 1～2 个连通域；性能还是毫秒级，因为全是向量化，没有 Python 里逐像素队列漫水。

### 2.3 数据路径：FloorPlanCAD vs FloorplanQA

**现象：** 一开始按名字找「FloorPlanCAD」，发现有的目录只有 PNG，没有和 Demo 1 矢量管线匹配的 JSON。

**改法：** 实际批量跑通用的是 **`floorplanqa/layouts` 下的 JSON**；别的数据该归档归档，在 `INVENTORY` 里写清楚。

**效果：** 测试和演示有稳定样本，不再「随机找个文件夹就 ERROR」。

### 2.4 PNG / SVG 解析翻车

**PNG：** 透明底 RGBA 转灰度时，透明被当成 0，整图几乎全黑，二值化直接报「全黑或全白」。**改法：** 先白底合成再灰度；固定阈值不行再 **Otsu** 兜底。

**SVG：** 真实文件大量用 `<path>`，老代码只认 line/polyline/polygon。**改法：** 补了轻量 `d` 解析（M/L/H/V/Z 等）和 `<rect>`。

**效果：** 演示里常见的 PNG、Inkscape 类 SVG 能进同一套管线。

### 2.5 交互选点、轨迹和图上「对不齐」

**现象：** 用户点的 A/B 和画出来的路径端点看着差一截。

**原因：** 点击坐标要 snap 到安全栅格；可视化标题若仍用「原始点击」就会和心理预期打架。

**改法：** 路径信息里统一用 **snap 后的世界坐标**；snap 半径加大 + 全局最近安全点兜底。

**效果：** 文案和轨迹一致，投诉少一截。

---

## 三、Demo 2 & 3：集成、大模型、和「别在聊天里贴 key」

### 3.1 Demo 2：真接 DeepSeek，别 mock 了

**问题：** 桩返回固定 JSON，测不出 prompt、重试、JSON 被 markdown 包一层就 parse 挂。

**改法：** `openai` SDK + `base_url` 指 DeepSeek；**3 次重试**；正则剥 ```json；**Few-shot +「禁止算坐标」** 护栏；`infer_local_context` / `summarize_roboir` / `validate_roboir_references` / `summarize_diagnostic`。

**效果：** Tab 2 能当真编译器用；Tab 3 诊断能吃到结构化结果。密钥**只走环境变量**，文档里反复提醒别往仓库和截图里塞。

### 3.2 Demo 3：距离场、ρ、和 UI 别各说各话

**问题：** Tab 1 没跑时 Tab 3 静默用默认厨房轨迹，看起来像「成功了其实假的」。

**改法：** 加载轨迹时若 state 空，**明确警告**；Pose 表在没距离约束时也要能看 clearance； violation 在叠加图和 ρ 曲线上画出来（`visualize_demo3.py`）。

**效果：** 测试同学能分清「真用 Tab 1 场」还是「默认样例」。

---

## 四、Gradio 这一层：全是「工程细节」但一样要命

### 4.1 502、按钮没反应

**原因：** 系统代理把 `localhost` 也代理了；`gr.State` 写在 `Blocks` 外面会直接 AttributeError。

**改法：** 启动前设 `NO_PROXY`；State 全部挪进 `with gr.Blocks`。

### 4.2 缩进、引号、中文 f-string

**原因：** 大块 Markdown 里混英文双引号，和 Python 字符串打架；某次合并把 `with gr.Row` 缩进弄断，整文件 parse 不过。

**改法：** 长文案用中文书名号或拆字符串；改完 **`ast.parse` 扫一眼**。

### 4.3 `numpy` 忘了 import

**原因：** 拓扑改成 `classify_topology_global` 后写了 `np.uint32`，但 `app.py` 顶部从没 `import numpy`。

**改法：** 补一行 `import numpy as np`。

### 4.4 Gradio 6 主题：`font` 不能全是字符串

**原因：** `launch(theme=...)` 里主题比较会访问 `Font.name`，元组里纯字符串会 `AttributeError`。

**改法：** `font=[gr.themes.GoogleFont("Inter"), ...]` 等形式，满足 Gradio 类型要求。

### 4.5 文档站风格 CSS

**诉求：** 视觉对齐 Claude Code 文档那种米白 + 克制强调色。

**注意：** 自定义 CSS 用 `!important` 较多，升级 Gradio 时要防选择器失效——属于**维护债**，测试可记一条「大版本升级后扫一遍 UI」。

---

## 五、按时间顺序一句话串起来（给忙人看的）

先 **audit + 数据归位** → Demo1 **外部泛洪** 用边缘连通域切掉 → **室内碎片化** 用破壁 + **全局矩阵分类** 换掉单源漫水定绿区 → **PNG/SVG** 各种真实文件格式补洞 → **Tab234 打通 + DeepSeek 真连** → **Gradio 代理/State/缩进/np/主题** 一轮轮擦屁股 → **文案护栏（FloorplanQA 来源、摘要不等于执行依据）** → 写 **QA_TEST_PLAN** 给测试。

---

## 六、还没完的事（诚实记录）

- 极稀疏线稿 PNG 仍可能「外部剔除护栏触发 → 整图当一大块自由空间」，和真实「有墙围住的 occupancy」体验不同——**输入质量**仍是上限。  
- 若未来要「工厂 CNC 专用矢量数据集」，还要再换或再标一批数据，当前仍以 FloorplanQA 为主演示。  
- Git 若从未初始化，**没有**「自动找回每一版历史」；需要的话从当下起建仓库 + CHANGELOG 往前写叙事（本文档可当素材）。

---

## 七、相关文件（想深挖代码时从这里跳）

| 主题 | 文件 |
|------|------|
| Demo 1 栅格 / EDT / 外部剔除 / 破壁 / 全局分类 | `cad_parser/astar_topology.py` |
| 多格式入口、PNG/SVG 修复 | `cad_parser/dispatcher.py` |
| Demo 1 可视化 | `cad_parser/visualize.py` |
| Demo 3 图 | `smdsl_demo/visualize_demo3.py` |
| 大模型与摘要/诊断 | `smdsl_demo/vlm_parser.py` |
| 三 Tab UI | `smdsl_demo/app.py` |
| 测试执行清单 | `smdsl_demo/QA_TEST_PLAN.md` |
| 架构原则 | `smdsl_demo/PROJECT_CONTEXT.md` |

---

*文档版本：随仓库迭代可改标题日期；有重大算法或 UI 变更时请在本文件追加一节「修订记录」。*
