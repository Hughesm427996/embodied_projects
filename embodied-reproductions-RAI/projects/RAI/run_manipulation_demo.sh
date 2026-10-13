#!/usr/bin/env bash
# 若被 sh/dash 调用，自动用 bash 重新执行，避免 bash 专用语法报错
if [ -z "${BASH_VERSION:-}" ]; then
    exec bash "$0" "$@"
fi
#
# RAI 机械臂（Manipulation / Franka Panda）演示 —— 一键启动脚本（带命令行回显）
#
# 用法：  ./run_manipulation_demo.sh          # 在终端运行
#         双击 / 右键“作为程序运行”也可        # 无终端时输出写入 log/demo_manipulation/
# 停止：  在运行窗口按 Ctrl+C；无终端时执行 ./cleanup_demo.sh
#
# 说明：  仿真、MoveIt、robotic_manipulation、感知服务都由本界面首次打开时
#         自动拉起（见 o3de_config.yaml 里的 binary_path），无需手动启动。
#
# 演示节奏控制：
#   STEP_DELAY=1.5   每条命令回显后停顿秒数（默认 1.5，设 0 不停顿）
#   STEP_WAIT=1      每条命令回显后等待你按回车再执行（手把手演示用）
#
# 其它：
#   PORT=8501             Streamlit 端口
#   AGENT_VERSION=v2      v2（默认，新服务名）或 v1（旧服务名）
#   OPENAI_API_KEY=sk-..  覆盖 key（默认从 ~/.bash_history 读取）
#   OPEN_BROWSER=0        不自动打开浏览器
#   HF_ENDPOINT=...       HuggingFace 镜像（默认 https://hf-mirror.com）
#
set -euo pipefail

REPO_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$REPO_DIR"

GAME_LAUNCHER="demo_assets/manipulation/RAIManipulationDemo/RAIManipulationDemo.GameLauncher"
PORT="${PORT:-8501}"
LOG_DIR="$REPO_DIR/log/demo_manipulation"
mkdir -p "$LOG_DIR"

export DISPLAY="${DISPLAY:-:0}"
export AGENT_VERSION="${AGENT_VERSION:-v2}"

# 国内直连 huggingface.co 不通，感知服务(grounding_dino/grounded_sam)会卡在下载
# tokenizer，最终报 “ROS2 stack is not ready in time”。默认走 hf-mirror.com，
# 需要直连时用 HF_ENDPOINT 覆盖。
export HF_ENDPOINT="${HF_ENDPOINT:-https://hf-mirror.com}"
export HF_HUB_DISABLE_TELEMETRY=1

STEP_DELAY="${STEP_DELAY:-1.5}"
STEP_WAIT="${STEP_WAIT:-0}"

if [ -t 1 ]; then
    C_RED=$'\033[31m'; C_GREEN=$'\033[32m'; C_YELLOW=$'\033[33m'; C_CYAN=$'\033[36m'; C_BOLD=$'\033[1m'; C_RESET=$'\033[0m'
else
    C_RED=""; C_GREEN=""; C_YELLOW=""; C_CYAN=""; C_BOLD=""; C_RESET=""
fi
info() { echo "${C_CYAN}[Manip]${C_RESET} $*"; }
ok()   { echo "${C_GREEN}[Manip]${C_RESET} $*"; }
warn() { echo "${C_YELLOW}[Manip]${C_RESET} $*"; }
err()  { echo "${C_RED}[Manip]${C_RESET} $*" >&2; }

section() {
    echo
    echo "${C_BOLD}${C_CYAN}================================================================${C_RESET}"
    echo "${C_BOLD}${C_CYAN}  $*${C_RESET}"
    echo "${C_BOLD}${C_CYAN}================================================================${C_RESET}"
}

pause() {
    if [ "$STEP_WAIT" = "1" ]; then
        read -r -p "  ${C_YELLOW}>>> 按回车继续执行${C_RESET}" _ || true
    elif [ "$STEP_DELAY" != "0" ]; then
        sleep "$STEP_DELAY"
    fi
}

show_cmd() {
    printf '%s$ %s%s\n' "$C_GREEN" "$*" "$C_RESET"
    pause
}

# 无终端运行（双击“作为程序运行”）时，把输出写入日志文件，便于事后排查。
if [ ! -t 1 ]; then
    SESSION_LOG="$LOG_DIR/session_$(date +%Y%m%d_%H%M%S).log"
    exec >>"$SESSION_LOG" 2>&1
else
    SESSION_LOG=""
fi

if [ ! -x "$GAME_LAUNCHER" ]; then
    err "找不到仿真程序：$GAME_LAUNCHER"
    err "请先运行：./scripts/download_demo.sh manipulation"
    exit 1
fi

if [ -z "${OPENAI_API_KEY:-}" ]; then
    OPENAI_API_KEY="$(grep -o 'sk-[A-Za-z0-9]\{20,\}' "$HOME/.bash_history" 2>/dev/null | tail -1 || true)"
fi
if [ -z "${OPENAI_API_KEY:-}" ]; then
    err "未找到 OPENAI_API_KEY。请先执行： export OPENAI_API_KEY=你的key"
    exit 1
fi
export OPENAI_API_KEY

# 记录需要关闭的后台进程（浏览器打开器等）
BG_PIDS=()

cleanup() {
    trap - EXIT INT TERM
    warn "正在停止演示进程..."
    set +e
    for p in "${BG_PIDS[@]:-}"; do
        [ -n "$p" ] && kill "$p" 2>/dev/null
    done
    pkill -f "manipulation-demo-streamlit.py" 2>/dev/null
    pkill -f "RAIManipulationDemo.GameLauncher" 2>/dev/null
    pkill -f "run_perception_services.py" 2>/dev/null
    pkill -f "robotic_manipulation" 2>/dev/null
    pkill -f "panda_moveit" 2>/dev/null
    pkill -f "manipulation-demo.launch.py" 2>/dev/null
    pkill -f "move_group" 2>/dev/null
    ok "已全部停止。（如需彻底清理残留：./cleanup_demo.sh）"
}

report_failure() {
    local rc="$1"
    case "$rc" in
        0|130|143) return ;;
    esac
    err "演示异常退出（退出码 $rc）。"
    if [ -n "$SESSION_LOG" ]; then
        err "完整日志：$SESSION_LOG"
        if command -v zenity >/dev/null 2>&1; then
            zenity --error --width=580 --title="RAI 机械臂演示启动失败" \
                --text="演示启动失败（退出码 $rc）。\n\n日志文件：\n$SESSION_LOG\n\n可执行 ./cleanup_demo.sh 清理残留进程。" 2>/dev/null &
        fi
    elif [ -t 1 ]; then
        echo
        read -r -p "  按回车关闭窗口..." _ || true
    fi
}

on_exit() {
    local rc=$?
    cleanup
    report_failure "$rc"
    trap - EXIT INT TERM
    exit "$rc"
}
trap on_exit EXIT
trap 'exit 130' INT TERM

UV_BIN="$(command -v uv || echo "$HOME/.local/bin/uv")"
ensure_python_deps() {
    # Streamlit 页面依赖 rai_bench（inflect/plotly 等）。缺失通常是没同步可选的
    # simbench/perception 依赖组，页面会直接报 ModuleNotFoundError。
    if ! python -c "import inflect, plotly" >/dev/null 2>&1; then
        warn "缺少演示所需的 Python 依赖（inflect/plotly），正在执行 uv sync ..."
        if ! "$UV_BIN" sync --group simbench --group perception; then
            err "uv sync 失败，请检查网络后重试。"
            exit 1
        fi
    fi
}

section "RAI 机械臂（Manipulation）演示 —— 一键启动"
info "仓库目录：$REPO_DIR"
[ -n "$SESSION_LOG" ] && info "会话日志：$SESSION_LOG"

section "步骤 0：清理上一次残留进程"
show_cmd "pkill -f RAIManipulationDemo.GameLauncher ; pkill -f manipulation-demo-streamlit.py ; pkill -f run_perception_services.py"
pkill -f "manipulation-demo-streamlit.py" 2>/dev/null || true
pkill -f "RAIManipulationDemo.GameLauncher" 2>/dev/null || true
pkill -f "run_perception_services.py" 2>/dev/null || true
pkill -f "robotic_manipulation" 2>/dev/null || true
pkill -f "panda_moveit" 2>/dev/null || true
sleep 2
ok "残留清理完成"

section "步骤 1：加载 ROS 2 与项目环境"
show_cmd "source /opt/ros/jazzy/setup.bash"
set +u
# shellcheck disable=SC1091
source /opt/ros/jazzy/setup.bash
export SHELL=/bin/bash
show_cmd "source $REPO_DIR/setup_shell.sh"
# shellcheck disable=SC1091
source "$REPO_DIR/setup_shell.sh"
set -u
ok "环境已加载（ROS_DISTRO=$ROS_DISTRO）"

show_cmd "python -c 'import inflect, plotly' || uv sync --group simbench --group perception"
ensure_python_deps
ok "Python 依赖检查通过"

info "HuggingFace 镜像：$HF_ENDPOINT（感知服务依赖）"

section "步骤 2：启动 Streamlit 界面（内部会自动拉起仿真与机械臂栈）"
info "配置：AGENT_VERSION=$AGENT_VERSION"
info "首次打开页面后需要等待约 1~2 分钟完成仿真与 MoveIt 初始化。"
echo
ok "${C_BOLD}演示地址：${C_RESET} http://localhost:$PORT"
if [ "${OPEN_BROWSER:-1}" = "1" ] && command -v xdg-open >/dev/null 2>&1; then
    ( for _ in $(seq 1 40); do
          if (exec 3<>"/dev/tcp/127.0.0.1/$PORT") 2>/dev/null; then
              exec 3>&- 2>/dev/null || true
              xdg-open "http://localhost:$PORT" >/dev/null 2>&1 || true
              break
          fi
          sleep 1
      done ) &
    BG_PIDS+=("$!")
fi
ok "按 Ctrl+C 结束演示（会一并关闭仿真与机械臂栈）"
echo

show_cmd "streamlit run examples/manipulation-demo-streamlit.py --server.port $PORT --server.headless true"
streamlit run examples/manipulation-demo-streamlit.py \
    --server.port "$PORT" \
    --server.address 0.0.0.0 \
    --server.headless true
