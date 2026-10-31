#!/usr/bin/env bash
# 文件：build.sh；用途：兼容旧编译命令，统一输出到competition/build和install。
set -eo pipefail
ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
source /opt/ros/humble/setup.bash
# 与全部launch共用根目录build/install，避免编译到旧工作区却运行另一份程序。
cd "$ROOT"
exec colcon build --packages-select rmtc_connection
