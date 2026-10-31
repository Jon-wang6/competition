#!/usr/bin/env bash
set -eo pipefail
HERE="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
TEAM="${1:-A}"
case "$TEAM" in A|B|C|D) ;; *) echo '席位须为 A/B/C/D' >&2; exit 2;; esac
source /opt/ros/humble/setup.bash
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-27}"
export ROS_LOCALHOST_ONLY="${ROS_LOCALHOST_ONLY:-0}"
export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_fastrtps_cpp}"
export ROS_LOG_DIR="${ROS_LOG_DIR:-$HOME/.rmtc-practice/bot-logs}"
export RMTC_TEAM_TOKEN="${RMTC_TEAM_TOKEN:-practice-$TEAM}"
mkdir -p "$ROS_LOG_DIR"
exec "$HERE/bin/robot_node" --ros-args -p "team:=$TEAM" -p "team_name:=${2:-练习车$TEAM}" -p "ai_gateway:=${AI_GATEWAY:-http://127.0.0.1:8080/v1/chat/completions}"
