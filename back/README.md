# back/ — SMDSL 项目归档目录

> **永久规则**：任何被淘汰的旧模块、废弃的冗余数据或旧版架构，必须移动到此目录，
> **禁止直接删除**，以保留历史可追溯性。

## 目录结构

```
back/
├── legacy_root_modules/        # SMDSL 旧版根目录副本 (已被 SMDSL/ 取代)
│   ├── cad_parser/             #   → 归档时间: 2026-05-18
│   └── smdsl_demo/             #   → 归档原因: SMDSL/ 成为唯一正权威来源
└── scratch_scripts/            # 开发过程中的一次性调试/批处理脚本
    ├── _batch_dwg_test.py
    ├── _fix_test.py
    └── _run_app.py
```

> **注意**：`factory-robot-projects/` 和 `ros_flutter_gui_app/` 是并列于 SMDSL
> 的独立工程，保留在 `D:\code\` 根目录，**不归入 back/**。

## 归档记录

| 日期       | 归档内容                | 归档原因                                |
|------------|------------------------|----------------------------------------|
| 2026-05-18 | `legacy_root_modules/` | 根目录双副本合并，SMDSL/ 成为唯一权威来源 |
| 2026-05-18 | `scratch_scripts/`     | 一次性调试脚本，已被 gitignore 规则覆盖  |
