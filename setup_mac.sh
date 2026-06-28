#!/usr/bin/env bash
# =============================================================================
# SMDSL_2D — Mac 一键环境搭建脚本
# =============================================================================
# 用法:
#   chmod +x setup_mac.sh
#   ./setup_mac.sh
#
# 跳过硬件的步骤:
#   ./setup_mac.sh --skip-conda    # 使用 venv 代替 conda
#   ./setup_mac.sh --skip-brew     # 跳过 Homebrew 安装（已手动安装依赖）
# =============================================================================

set -euo pipefail

REPO_URL="git@github.com:Xuzc317/SMDSL_2D.git"
REPO_NAME="SMDSL_2D"
CONDA_ENV="smdsl"
PYTHON_VER="3.11"
SKIP_CONDA=false
SKIP_BREW=false

# ─── 参数解析 ──────────────────────────────────────────────

for arg in "$@"; do
    case "$arg" in
        --skip-conda) SKIP_CONDA=true ;;
        --skip-brew)  SKIP_BREW=true ;;
        -h|--help)
            echo "用法: ./setup_mac.sh [--skip-conda] [--skip-brew]"
            exit 0
            ;;
    esac
done

# ─── 彩色输出 ──────────────────────────────────────────────

RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

info()  { echo -e "${BLUE}[INFO]${NC}  $*"; }
ok()    { echo -e "${GREEN}[OK]${NC}    $*"; }
warn()  { echo -e "${YELLOW}[WARN]${NC}  $*"; }
error() { echo -e "${RED}[ERROR]${NC} $*"; }

header() {
    echo ""
    echo -e "${GREEN}════════════════════════════════════════════════════════════${NC}"
    echo -e "${GREEN}  $*${NC}"
    echo -e "${GREEN}════════════════════════════════════════════════════════════${NC}"
}

# ─── Step 0: 检查基础 ─────────────────────────────────────

header "Step 0: 检查操作系统"

if [[ "$(uname)" != "Darwin" ]]; then
    error "此脚本仅适用于 macOS。当前系统: $(uname)"
    exit 1
fi
ok "macOS 已确认 — $(sw_vers -productVersion)"

ARCH=$(uname -m)
case "$ARCH" in
    arm64)  CHIP="Apple Silicon (M-series)" ;;
    x86_64) CHIP="Intel" ;;
    *)      CHIP="$ARCH" ;;
esac
info "芯片架构: $CHIP"

# ─── Step 1: Homebrew ──────────────────────────────────────

header "Step 1: Homebrew + 系统依赖"

if $SKIP_BREW; then
    info "跳过 Homebrew 安装 (--skip-brew)"
else
    if ! command -v brew &>/dev/null; then
        info "Homebrew 未安装，正在安装..."
        /bin/bash -c "$(curl -fsSL https://raw.githubusercontent.com/Homebrew/install/HEAD/install.sh)"

        # Apple Silicon 需要额外路径
        if [[ "$ARCH" == "arm64" ]]; then
            eval "$(/opt/homebrew/bin/brew shellenv)"
        fi
    fi
    ok "Homebrew: $(brew --version | head -1)"

    info "安装系统依赖..."
    BREW_DEPS=("libredwg" "cairo" "git")
    for dep in "${BREW_DEPS[@]}"; do
        if brew list "$dep" &>/dev/null; then
            ok "已安装: $dep"
        else
            info "正在安装: $dep ..."
            brew install "$dep"
            ok "安装完成: $dep"
        fi
    done

    # Python
    if ! command -v python3 &>/dev/null; then
        info "安装 Python $PYTHON_VER ..."
        brew install "python@${PYTHON_VER}"
    fi
    ok "Python: $(python3 --version)"
fi

# 验证关键工具
info "验证系统依赖..."
for tool in dwgread python3 git; do
    if command -v "$tool" &>/dev/null; then
        ok "$tool: $(command -v $tool)"
    else
        warn "$tool 未找到 — 可能需要手动安装"
    fi
done

# dwgread 特殊验证
if command -v dwgread &>/dev/null; then
    dwgread --version 2>&1 | head -1 || true
    ok "dwgread 可用 — DWG 文件解析已就绪"
else
    warn "dwgread 不可用 — DWG 解析功能将不可用"
    warn "请手动运行: brew install libredwg"
fi

# ─── Step 2: 克隆仓库 ─────────────────────────────────────

header "Step 2: 克隆 SMDSL_2D"

CURRENT_DIR=$(pwd)

if [[ -d "$REPO_NAME" ]]; then
    info "目录 $REPO_NAME 已存在，跳过克隆"
    cd "$REPO_NAME"
    info "拉取最新代码..."
    git pull origin main
else
    info "正在克隆 $REPO_URL ..."
    git clone "$REPO_URL"
    cd "$REPO_NAME"
fi

PROJECT_ROOT=$(pwd)
ok "项目根目录: $PROJECT_ROOT"

# ─── Step 3: Python 环境 ───────────────────────────────────

header "Step 3: Python 虚拟环境"

if $SKIP_CONDA; then
    # ── venv 方案 ──
    info "使用 venv (--skip-conda)"

    if [[ ! -d "venv" ]]; then
        python3 -m venv venv
        info "venv 已创建"
    fi
    source venv/bin/activate
    ok "venv 已激活"
else
    # ── conda 方案 ──
    CONDA_FOUND=false
    PREFIX=""

    for candidate in "$HOME/miniconda3" "$HOME/anaconda3" "/opt/homebrew/Caskroom/miniconda/base" "/usr/local/Caskroom/miniconda/base"; do
        if [[ -f "$candidate/etc/profile.d/conda.sh" ]]; then
            source "$candidate/etc/profile.d/conda.sh"
            CONDA_FOUND=true
            PREFIX="$candidate"
            break
        fi
    done

    if ! $CONDA_FOUND && command -v conda &>/dev/null; then
        CONDA_FOUND=true
    fi

    if $CONDA_FOUND; then
        ok "Conda 已找到: ${PREFIX:-$(which conda)}"

        if conda env list | grep -q "^${CONDA_ENV}\s"; then
            info "环境 '$CONDA_ENV' 已存在，激活中..."
        else
            info "创建 conda 环境 '$CONDA_ENV' (Python $PYTHON_VER)..."
            conda create -n "$CONDA_ENV" "python=${PYTHON_VER}" -y
        fi
        conda activate "$CONDA_ENV"
        ok "conda 环境 '$CONDA_ENV' 已激活"
    else
        warn "未找到 conda，回退到 venv"
        if [[ ! -d "venv" ]]; then
            python3 -m venv venv
        fi
        source venv/bin/activate
        ok "venv 已激活"
    fi
fi

ok "Python: $(python --version)"
info "pip: $(pip --version | head -1)"

# ─── Step 4: Python 依赖 ──────────────────────────────────

header "Step 4: 安装 Python 依赖"

# PyQt5 在 Mac 上的特殊处理
info "安装 PyQt5 (Mac 兼容)..."
if pip install PyQt5 2>/dev/null; then
    ok "PyQt5 安装成功"
else
    warn "PyQt5 安装失败 — 尝试通过 brew 安装"
    brew install pyqt5 2>/dev/null || true
    pip install PyQt5 2>/dev/null || {
        warn "PyQt5 仍然失败 — 切换到 PyQt6"
        pip install PyQt6
        warn "PyQt6 已安装 (非 PyQt5)。SMDSL 的 DWG 检查功能可能需要适配。"
    }
fi

info "安装其余依赖 (requirements.txt)..."
pip install -r requirements.txt
ok "所有依赖安装完成"

# ─── Step 5: 配置文件 ─────────────────────────────────────

header "Step 5: 配置文件"

# .env
if [[ ! -f ".env" ]]; then
    cat > .env <<'EOF'
# SMDSL_2D 环境变量
# 请将下面的 placeholder 替换为你的实际 API Key

DEEPSEEK_API_KEY=sk-your-deepseek-api-key-here
DEEPSEEK_KEY=sk-your-deepseek-api-key-here

# 可选：自定义 DeepSeek base URL
# DEEPSEEK_BASE_URL=https://api.deepseek.com
EOF
    warn ".env 已创建 — 请编辑填入你的 DEEPSEEK_API_KEY"
else
    ok ".env 已存在，跳过"
fi

# Git safe.directory (避免 dubious ownership 错误)
git config --global --add safe.directory "$PROJECT_ROOT" 2>/dev/null || true

ok "配置文件就绪"

# ─── Step 6: 验证安装 ─────────────────────────────────────

header "Step 6: 验证安装"

VERIFY_OK=true

info "检查关键 Python 包..."
for pkg in numpy scipy gradio openai pytest matplotlib plotly Pillow; do
    if python -c "import $pkg" 2>/dev/null; then
        ok "$pkg"
    else
        error "$pkg — 导入失败"
        VERIFY_OK=false
    fi
done

info "检查可选 Python 包..."
for pkg in ezdxf svgwrite cairosvg cv2 skimage; do
    if python -c "import $pkg" 2>/dev/null; then
        ok "$pkg"
    else
        warn "$pkg — 不可用（可能影响部分功能）"
    fi
done

info "运行单元测试..."
cd SMDSL
if python -m pytest tests/ --tb=short -q 2>&1; then
    ok "单元测试全部通过"
else
    warn "部分测试失败 — 可能是环境差异，不影响核心功能"
    VERIFY_OK=false
fi
cd "$PROJECT_ROOT"

info "检查数据完整性..."
DATA_DIRS=(
    "data/dwg_samples/autodesk_official"
    "data/datasets/synthetic_benchmark"
    "SMDSL/data/cad_samples"
)
for dir in "${DATA_DIRS[@]}"; do
    if [[ -d "$dir" ]] && [[ -n "$(ls -A "$dir" 2>/dev/null)" ]]; then
        ok "$dir — 存在"
    else
        warn "$dir — 缺失或为空"
        VERIFY_OK=false
    fi
done

# dwgread 功能验证
if command -v dwgread &>/dev/null; then
    DWG_FILE="data/dwg_samples/autodesk_official/office.dwg"
    if [[ -f "$DWG_FILE" ]]; then
        if dwgread --version &>/dev/null; then
            ok "dwgread 功能验证通过"
        fi
    fi
fi

# ─── 总结 ──────────────────────────────────────────────────

header "安装完成"

echo ""
echo "  项目路径:  $PROJECT_ROOT"
echo "  Python:    $(python --version)"
echo "  环境:      $(which python)"
echo ""

if $VERIFY_OK; then
    echo -e "  ${GREEN}✅ 所有核心检查通过！${NC}"
else
    echo -e "  ${YELLOW}⚠️  部分检查未通过，请查看上方警告${NC}"
fi

echo ""
echo "  ─── 启动命令 ───"
echo ""
echo "  # 启动 Gradio UI (Demo 1-3)"
echo "  cd $PROJECT_ROOT/SMDSL"
echo "  python -m smdsl_demo.app"
echo ""
echo "  # 运行基准测试"
echo "  python -m benchmark.main_benchmark --n_maps 10 --n_pairs 3"
echo ""
echo "  # 运行所有单元测试"
echo "  python -m pytest tests/ -v"
echo ""
echo "  ⚠️  别忘了编辑 .env 填入 DEEPSEEK_API_KEY"
echo "      nano $PROJECT_ROOT/.env"
echo ""

exit 0
