#!/usr/bin/env bash
# 文件：run_bot.sh；用途：兼容旧的主控单独启动命令，不启动裁判、AI或RViz。
set -eo pipefail
ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
TEAM="${1:-A}"
MODE="${2:-wait}"
ACTION_TICKS="${3:-100}"
source /opt/ros/humble/setup.bash
if [[ ! -f "$ROOT/install/setup.bash" ]]; then
  echo '请先运行 bash scripts/build.sh' >&2
  exit 1
fi
source "$ROOT/install/setup.bash"
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-27}"
export ROS_LOCALHOST_ONLY="${ROS_LOCALHOST_ONLY:-0}"
export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_fastrtps_cpp}"
export ROS_LOG_DIR="$ROOT/runtime/ros"
mkdir -p "$ROOT/runtime/ros" "$ROOT/runtime/bot"
LOG_PATH="$ROOT/runtime/bot/$(date +%Y%m%d-%H%M%S)-${TEAM}-$$.jsonl"
exec ros2 run rmtc_connection connection_node --ros-args \
  -p "team:=$TEAM" -p "mode:=$MODE" -p "action_ticks:=$ACTION_TICKS" -p "log_file:=$LOG_PATH"
