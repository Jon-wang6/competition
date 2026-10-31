#!/usr/bin/env bash
# 本地v1.2启动入口。直接运行也经过C++网页/AI网关。
# launch设置PRACTICE_BACKEND_ONLY=1时仅启动原Python进程管理器，防止递归启动。
set -eo pipefail
HERE="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
PROJECT="$(cd -- "$HERE/../../.." && pwd)"
source /opt/ros/humble/setup.bash
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-27}"
export ROS_LOCALHOST_ONLY="${ROS_LOCALHOST_ONLY:-0}"
export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_fastrtps_cpp}"
export ROS_LOG_DIR="${ROS_LOG_DIR:-$PROJECT/runtime/ros}"
mkdir -p "$ROS_LOG_DIR"
if [[ "${PRACTICE_BACKEND_ONLY:-0}" == "1" ]]; then
    exec /usr/bin/python3 "$HERE/practice.py" "$@"
fi
source "$PROJECT/install/local_setup.bash"
params=("stage:=2" "referee_dir:=$HERE" "domain_id:=$ROS_DOMAIN_ID" "ros_log_dir:=$ROS_LOG_DIR")
if [[ -n "${RMTC_INTERNAL_DOMAIN_ID:-}" ]]; then params+=("internal_domain_id:=$RMTC_INTERNAL_DOMAIN_ID"); fi
while (($#)); do
    if (($# < 2)); then echo "参数缺少值：$1" >&2; exit 2; fi
    case "$1" in
        --stage) name=stage;;
        --port) name=port;;
        --tick-ms) name=tick_ms;;
        --max-ticks) name=max_ticks;;
        --bots) name=bots;;
        --config) name=config_file;;
        --data-dir) name=data_dir;;
        --ai-mode) name=ai_mode;;
        *) echo "未知参数：$1" >&2; exit 2;;
    esac
    if [[ -n "$2" ]]; then params+=("$name:=$2"); fi
    shift 2
done
exec ros2 launch rmtc_connection referee.launch.py "${params[@]}"
