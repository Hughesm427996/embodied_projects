#!/usr/bin/env bash
# 若被 sh/dash 调用，自动用 bash 重新执行，避免 bash 专用语法报错
if [ -z "${BASH_VERSION:-}" ]; then
    exec bash "$0" "$@"
fi
#
# RAI 演示环境一键复原脚本
#
# 作用：
#   1. 结束所有演示相关进程（仿真 / nav2 / 感知 / MoveIt / streamlit / ros2 launch …）
#   2. 删除仿真运行期产生的残留文件（Game.log、Metrics、lockfile、imgui.ini）
#   3. 清理 ROS 2 运行日志（~/.ros/log）
#
# 用法：  ./cleanup_demo.sh
#
# 可用环境变量：
#   CLEAN_ROS_LOGS=0   不清理 ~/.ros/log
#   CLEAN_SCRIPT_LOGS=0 不删除 ./log/demo_rosbot、./log/demo_manipulation
#
# 说明：本脚本不会改动 demo_assets 里的可执行文件/资产，也不会还原代码改动。
#
set -uo pipefail

REPO_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$REPO_DIR"

CLEAN_ROS_LOGS="${CLEAN_ROS_LOGS:-1}"
CLEAN_SCRIPT_LOGS="${CLEAN_SCRIPT_LOGS:-1}"

if [ -t 1 ]; then
    C_RED=$'\033[31m'; C_GREEN=$'\033[32m'; C_YELLOW=$'\033[33m'; C_CYAN=$'\033[36m'; C_BOLD=$'\033[1m'; C_RESET=$'\033[0m'
else
    C_RED=""; C_GREEN=""; C_YELLOW=""; C_CYAN=""; C_BOLD=""; C_RESET=""
fi
info() { echo "${C_CYAN}[清理]${C_RESET} $*"; }
ok()   { echo "${C_GREEN}[清理]${C_RESET} $*"; }
warn() { echo "${C_YELLOW}[清理]${C_RESET} $*"; }

section() {
    echo
    echo "${C_BOLD}${C_CYAN}================================================================${C_RESET}"
    echo "${C_BOLD}${C_CYAN}  $*${C_RESET}"
    echo "${C_BOLD}${C_CYAN}================================================================${C_RESET}"
}

PATTERNS=(
    "RAIROSBotXLDemo.GameLauncher"
    "RAIManipulationDemo.GameLauncher"
    "RAIAgricultureDemo.GameLauncher"
    "run_perception_services.py"
    "run_perception_agents.py"
    "manipulation-demo-streamlit.py"
    "streamlit run examples/rosbot-xl-demo.py"
    "rosbot-xl.launch.py"
    "manipulation-demo.launch.py"
    "robotic_manipulation"
    "panda_moveit"
    "move_group"
    "bt_navigator"
    "controller_server"
    "planner_server"
    "slam_toolbox"
    "behavior_server"
    "waypoint_follower"
    "velocity_smoother"
    "robot_state_publisher"
    "ros2 launch"
    "rviz2"
)

kill_all() {
    local sig="$1" pat
    for pat in "${PATTERNS[@]}"; do
        pkill "-$sig" -f -- "$pat" 2>/dev/null || true
    done
}

section "RAI 演示环境一键复原"

section "步骤 1：结束演示相关进程"
info "发送 SIGTERM ..."
kill_all TERM
sleep 2
info "发送 SIGKILL 兜底 ..."
kill_all KILL
sleep 1
ok "进程结束指令已发送"

section "步骤 2：检查是否仍有残留进程"
REMAIN="$(pgrep -af "GameLauncher|run_perception_services|run_perception_agents|robotic_manipulation|move_group|bt_navigator|controller_server|slam_toolbox|manipulation-demo-streamlit.py|rosbot-xl-demo.py" 2>/dev/null || true)"
if [ -n "$REMAIN" ]; then
    warn "仍检测到以下进程："
    echo "$REMAIN" | sed 's/^/    /'
else
    ok "无演示进程残留"
fi

section "步骤 3：删除仿真运行期残留文件"
for demo_dir in "$REPO_DIR"/demo_assets/*/*/; do
    [ -d "$demo_dir" ] || continue
    name="$(basename "$demo_dir")"

    if [ -d "${demo_dir}user/log" ]; then
        find "${demo_dir}user/log" -type f -delete 2>/dev/null || true
        info "$name: 已清空 user/log/"
    fi
    if [ -d "${demo_dir}user/Metrics" ]; then
        find "${demo_dir}user/Metrics" -type f -delete 2>/dev/null || true
        info "$name: 已清空 user/Metrics/"
    fi
    if [ -f "${demo_dir}user/Cache/lockfile.txt" ]; then
        rm -f "${demo_dir}user/Cache/lockfile.txt"
        info "$name: 已删除 user/Cache/lockfile.txt"
    fi
    if [ -f "${demo_dir}imgui.ini" ]; then
        rm -f "${demo_dir}imgui.ini"
        info "$name: 已删除 imgui.ini"
    fi
done
if [ -f "$REPO_DIR/imgui.ini" ]; then
    rm -f "$REPO_DIR/imgui.ini"
    info "已删除仓库根目录 imgui.ini"
fi

section "步骤 4：清理日志"
if [ "$CLEAN_ROS_LOGS" = "1" ] && [ -d "$HOME/.ros/log" ]; then
    find "$HOME/.ros/log" -mindepth 1 -maxdepth 1 -exec rm -rf {} + 2>/dev/null || true
    ok "已清空 ~/.ros/log"
else
    info "跳过 ~/.ros/log"
fi
if [ "$CLEAN_SCRIPT_LOGS" = "1" ]; then
    rm -rf "$REPO_DIR/log/demo_rosbot" "$REPO_DIR/log/demo_manipulation"
    ok "已删除脚本运行日志 log/demo_rosbot、log/demo_manipulation"
fi

section "复原完成"
ok "现在可以干净地重新运行演示脚本："
echo "    ./run_rosbot_demo.sh"
echo "    ./run_manipulation_demo.sh"
echo
