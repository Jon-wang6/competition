#!/usr/bin/env bash
# 文件：run_core_audit.sh
# 用途：编译C++规则探针，并在隔离域191/192回放全部裁判构造场景。
# 不要与其他使用191/192的审计同时运行；不会启动选手或使用AI服务。
set -eo pipefail
test_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
project_dir="$(cd -- "$test_dir/../../.." && pwd)"
source /opt/ros/humble/setup.bash
set -u
audit_dir="$project_dir/runtime/referee-v12-audit"
cmake -S "$test_dir" -B "$audit_dir/build"
cmake --build "$audit_dir/build" -j 4
export ROS_DOMAIN_ID=192
export ROS_LOCALHOST_ONLY=1
export ROS_LOG_DIR="$audit_dir/ros"
mkdir -p "$ROS_LOG_DIR"
result_file="$audit_dir/core-results.jsonl"
: > "$result_file"
failed=0
for fixture in "$test_dir"/cases/*.json; do
    name="$(basename -- "$fixture" .json)"
    if ! "$audit_dir/build/simulator_probe" "$fixture" \
        "$project_dir/referee/local_v1.2/rmtc_practice/bin/simulator_node" \
        "$audit_dir/$name" >> "$result_file"; then
        failed=$((failed+1))
    fi
done
cat "$result_file"
exit "$failed"
