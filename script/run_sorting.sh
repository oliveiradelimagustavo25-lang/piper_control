#!/usr/bin/env bash
# 加载 ROS 与工作区后，把运行流程交给 Python。
set -e

workspace=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)
source /opt/ros/humble/setup.bash
source "$workspace/install/setup.bash"
cd "$workspace"
exec python3 -u test/sorting_run.py "$@"
