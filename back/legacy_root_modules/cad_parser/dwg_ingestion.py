"""
dwg_ingestion.py — 工业 DWG 图纸解析入口 (LibreDWG 包装器 + 安全沙箱)

安全措施：
  1. 文件大小上限 50 MB（R2004 格式额外限制 10 MB）
  2. DWG 文件头魔数校验 (ACxxxx)
  3. subprocess 严格 timeout (默认 30s / R2004 15s)
  4. CVE-2025-61154 缓解：R2004 压缩元数据堆溢出特殊处理
  5. 临时文件隔离，避免路径注入

依赖：
  - LibreDWG 工具链 (dwgread)：https://www.gnu.org/software/libredwg/
  - 若未安装，所有函数返回结构化错误信息而非崩溃
"""

from __future__ import annotations

import json
import os
import subprocess
import tempfile
from pathlib import Path
from typing import Any, Dict, Optional


# ══════════════════════════════════════════════════════════════════════
# 安全常量
# ══════════════════════════════════════════════════════════════════════

MAX_DWG_SIZE_BYTES = 50 * 1024 * 1024       # 50 MB
DWGREAD_TIMEOUT_S = 30                       # subprocess 默认超时

# DWG 文件头：前 6 字节标识版本
# AC1012=R13, AC1014=R14, AC1015=R2000, AC1018=R2004,
# AC1021=R2007, AC1024=R2010, AC1027=R2013, AC1032=R2018
DWG_MAGIC_PREFIX = b"AC"
DWG_R2004_VERSION = b"AC1018"                # CVE-2025-61154 受影响

# R2004 额外限制
R2004_MAX_SIZE = 10 * 1024 * 1024            # 10 MB
R2004_TIMEOUT_S = 15


# ══════════════════════════════════════════════════════════════════════
# 文件头校验
# ══════════════════════════════════════════════════════════════════════

def _check_dwg_header(file_path: str) -> Dict[str, Any]:
    """校验 DWG 文件头魔数并返回版本信息。"""
    try:
        with open(file_path, "rb") as f:
            header = f.read(6)
    except OSError as e:
        return {
            "valid": False, "version": "unknown",
            "is_r2004": False, "message": f"无法读取文件：{e}",
        }

    if len(header) < 6 or header[:2] != DWG_MAGIC_PREFIX:
        return {
            "valid": False, "version": "unknown",
            "is_r2004": False,
            "message": f"文件头魔数不匹配：期望 ACxxxx，实际 {header[:6]!r}",
        }

    version_str = header[:6].decode("ascii", errors="replace")
    is_r2004 = header[:6] == DWG_R2004_VERSION
    extra = ""
    if is_r2004:
        extra = " (R2004 — CVE-2025-61154 风险，已启用额外防护)"

    return {
        "valid": True, "version": version_str,
        "is_r2004": is_r2004,
        "message": f"DWG {version_str}{extra}",
    }


# ══════════════════════════════════════════════════════════════════════
# 查找 dwgread
# ══════════════════════════════════════════════════════════════════════

def _find_dwgread() -> Optional[str]:
    """查找 dwgread 可执行文件。按 PATH → 常见安装路径搜索。"""
    import os as _os
    local_appdata = _os.environ.get("LOCALAPPDATA", "")
    candidates = [
        "dwgread",
        r"C:\Program Files\LibreDWG\bin\dwgread.exe",
        r"C:\Program Files (x86)\LibreDWG\bin\dwgread.exe",
        "/usr/bin/dwgread",
        "/usr/local/bin/dwgread",
    ]
    if local_appdata:
        candidates.insert(1, f"{local_appdata}\\LibreDWG\\bin\\dwgread.exe")
    for c in candidates:
        try:
            result = subprocess.run(
                [c, "--version"], capture_output=True, timeout=5,
            )
            combined = result.stderr + result.stdout
            if result.returncode == 0 or b"dwgread" in combined.lower():
                return c
        except (subprocess.TimeoutExpired, FileNotFoundError, PermissionError):
            continue
    return None


# ══════════════════════════════════════════════════════════════════════
# 主入口：DWG → JSON
# ══════════════════════════════════════════════════════════════════════

def parse_dwg_to_json(file_path: str) -> Dict[str, Any]:
    """
    调用本机 dwgread -Ojson 将 .dwg 文件转换为结构化 JSON。

    安全沙箱执行顺序：
      1. 文件存在性检查
      2. 扩展名白名单 (.dwg / .dxf)
      3. 文件大小上限检查
      4. DWG 文件头魔数校验
      5. CVE-2025-61154 缓解（R2004 额外限制）
      6. dwgread 可用性检查
      7. subprocess + timeout 执行转换
      8. JSON 解析与验证

    Returns:
        {"status": "ok"|"error", "dwg_version": str,
         "json": dict|None, "message": str}
    """
    path = Path(file_path)

    # 1. 存在性
    if not path.exists():
        return _err("unknown", f"文件不存在：{file_path}")

    # 2. 扩展名白名单
    ext = path.suffix.lower()
    if ext not in (".dwg", ".dxf"):
        return _err("unknown", f"不支持扩展名 {ext}，期望 .dwg 或 .dxf")

    # 3. 文件大小
    file_size = path.stat().st_size
    if file_size > MAX_DWG_SIZE_BYTES:
        return _err(
            "unknown",
            f"文件过大：{file_size / 1_048_576:.1f} MB，"
            f"上限 {MAX_DWG_SIZE_BYTES / 1_048_576:.0f} MB",
        )

    # 4. 魔数校验（仅 DWG；DXF 是文本格式无需校验）
    header_info = _check_dwg_header(str(path))
    if ext == ".dwg" and not header_info["valid"]:
        return _err(header_info["version"], f"DWG 文件头校验失败：{header_info['message']}")

    # 5. CVE-2025-61154 缓解
    is_r2004 = header_info["is_r2004"]
    effective_timeout = R2004_TIMEOUT_S if is_r2004 else DWGREAD_TIMEOUT_S

    if is_r2004 and file_size > R2004_MAX_SIZE:
        return _err(
            header_info["version"],
            f"R2004 格式受 CVE-2025-61154 影响，大小限制 {R2004_MAX_SIZE // 1_048_576} MB。"
            f"当前 {file_size / 1_048_576:.1f} MB。请用 AutoCAD R2007+ 另存。",
        )

    # 6. 查找 dwgread
    dwgread = _find_dwgread()
    if dwgread is None:
        return _err(
            header_info["version"],
            "未找到 LibreDWG (dwgread)。"
            "安装指南：https://www.gnu.org/software/libredwg/",
        )

    # 7. 执行转换
    try:
        tmp_fd, tmp_path = tempfile.mkstemp(suffix=".json", prefix="dwgread_")
        os.close(tmp_fd)

        creationflags = 0
        if os.name == "nt":
            creationflags = subprocess.CREATE_NO_WINDOW  # type: ignore[attr-defined]

        proc = subprocess.run(
            [dwgread, "-Ojson", str(path)],
            stdout=open(tmp_path, "w", encoding="utf-8"),
            stderr=subprocess.PIPE,
            timeout=effective_timeout,
            creationflags=creationflags,
        )

        stderr_text = proc.stderr.decode("utf-8", errors="replace") if proc.stderr else ""

        if proc.returncode != 0:
            Path(tmp_path).unlink(missing_ok=True)
            return _err(
                header_info["version"],
                f"dwgread 退出码 {proc.returncode}：{stderr_text[:400]}",
            )

        # 8. 读取并解析（容错编码：dwgread 可能输出 Latin-1 字符）
        raw_bytes = open(tmp_path, "rb").read()
        Path(tmp_path).unlink(missing_ok=True)

        if len(raw_bytes) < 10:
            return _err(
                header_info["version"],
                "dwgread 输出为空（可能是不支持的版本或损坏文件）。",
            )

        # 编码回退链：UTF-8 → Latin-1 → cp1252
        raw_text = None
        for enc in ("utf-8", "latin-1", "cp1252"):
            try:
                raw_text = raw_bytes.decode(enc)
                break
            except UnicodeDecodeError:
                continue
        if raw_text is None:
            return _err(header_info["version"], "无法解码 dwgread 输出（编码不兼容）")

        dwg_json = json.loads(raw_text)
        raw_size = len(raw_text)

        # ── 大文件预警：超过 50 MB 时下游将强制开启图层白名单过滤 ──
        large_json_threshold = 50 * 1024 * 1024  # 50 MB
        large_warning = ""
        if raw_size > large_json_threshold:
            large_warning = (
                f"（WARNING: JSON {raw_size / 1_048_576:.0f} MB > "
                f"{large_json_threshold // 1_048_576} MB 阈值，"
                f"将强制图层过滤以避免 OOM）"
            )

        return {
            "status": "ok",
            "dwg_version": header_info["version"],
            "json": dwg_json,
            "is_r2004": is_r2004,
            "raw_size_bytes": raw_size,
            "message": f"成功解析 {header_info['version']}"
                       f"{'（R2004 安全模式）' if is_r2004 else ''}，"
                       f"JSON {raw_size // 1024} KB{large_warning}",
        }

    except subprocess.TimeoutExpired:
        Path(tmp_path).unlink(missing_ok=True)
        return _err(
            header_info["version"],
            f"dwgread 超时（{effective_timeout}s）。文件可能过大或损坏。",
        )
    except json.JSONDecodeError as e:
        Path(tmp_path).unlink(missing_ok=True)
        return _err(header_info["version"], f"dwgread 输出非合法 JSON：{e}")
    except Exception as e:
        Path(tmp_path).unlink(missing_ok=True)
        return _err(header_info["version"], f"解析异常：{type(e).__name__}: {e}")


# ══════════════════════════════════════════════════════════════════════
# 快速预检（不上传/不完整解析）
# ══════════════════════════════════════════════════════════════════════

def get_dwg_metadata(file_path: str) -> Dict[str, Any]:
    """读取 DWG 头信息，用于上传前的安全预检。"""
    path = Path(file_path)
    if not path.exists():
        return {"exists": False, "message": "文件不存在"}

    header = _check_dwg_header(str(path))
    size = path.stat().st_size
    return {
        "exists": True,
        "path": str(path),
        "size_bytes": size,
        "size_mb": round(size / 1_048_576, 2),
        "dwg_version": header["version"],
        "is_valid_dwg": header["valid"],
        "is_r2004": header["is_r2004"],
        "cve_2025_61154_risk": header["is_r2004"],
        "message": header["message"],
    }


# ══════════════════════════════════════════════════════════════════════
# 工具函数
# ══════════════════════════════════════════════════════════════════════

def _err(version: str, msg: str) -> Dict[str, Any]:
    return {"status": "error", "dwg_version": version, "json": None, "message": msg}
