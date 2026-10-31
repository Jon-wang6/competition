#!/usr/bin/env bash
set -eo pipefail
ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
export ROS_LOG_DIR="$ROOT/runtime/ros"
mkdir -p "$ROOT/runtime/ros" "$ROOT/runtime/referee"
cd "$ROOT/referee/local_v1.2/rmtc_practice"
exec ./start_referee.sh --data-dir "$ROOT/runtime/referee" "$@"
