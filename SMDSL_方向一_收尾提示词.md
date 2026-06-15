# SMDSL 方向一 收尾提示词

方向一代码已全部完成。剩余一个路径清理 + Git 推送，然后就可以做交互测试了。

---

## 1. 清理 benchmark 旧路径

benchmark 文件在 git 中被跟踪在 `SMDSL/benchmark/` 下，但 `benchmark/` 路径下还有一个旧副本。删掉它：

```bash
rmdir /s /q D:\Code\SMDSL_demo\benchmark
```

验证：
```bash
# 只剩 SMDSL/benchmark/ 下的版本
dir D:\Code\SMDSL_demo\SMDSL\benchmark
# 旧路径已删除
dir D:\Code\SMDSL_demo\benchmark   # 应报错"找不到路径"
```

---

## 2. Git 提交并推送

```bash
cd D:\Code\SMDSL_demo
git add .
git commit -m "[方向一 v3] 清理: 统一 benchmark 路径到 SMDSL/benchmark/"
git push origin main
```

验证：
```bash
git log --oneline -3
```

---

## 3. 启动 app

```bash
cd D:\Code\SMDSL_demo
set DEEPSEEK_API_KEY=your_key_here
python -m SMDSL.smdsl_demo.app
```

如果遇到依赖缺失（如 `gradio`、`numpy`、`scipy`、`openai`）：
```bash
pip install gradio numpy scipy openai matplotlib plotly
```

---

## 4. 交互测试清单

| Tab | 测试项 | 预期 |
|-----|--------|------|
| 环境感知 | 选择一个 `.json` 或 `.png` 文件 | 自动解析，4 联图出现 |
| 环境感知 | 选起终点 → 规划路径 | 路径显示，轨迹自动同步到 Demo 3 |
| 架构图 | 切换到 Architecture Tab | 4-Zone 架构图完整渲染 |
| 操作说明 | 环境感知页顶部 | 简洁的"操作流程"4 行，无长说明 |
| Demo 3 | 加载轨迹 → STL 验证 | 验证结果 + ρ 曲线显示 |

---

## 5. 完成标准

- [ ] app.py 启动无报错
- [ ] Tab 1 文件选择可浏览到实际数据目录
- [ ] 环境感知可正常解析 CAD 文件
- [ ] Architecture Tab 显示架构图
- [ ] Demo Recording Tab 存在且 UI 完整
- [ ] 旧 benchmark 目录已清理
- [ ] Git 已推送
