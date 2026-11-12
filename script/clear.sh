#!/usr/bin/env bash
# 按当前 ROS 域清理本工作区的孤立 launch 与节点；识别逻辑见 build/clear_processes.py。
set -e
root=$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd -P)
exec python3 "$root/script/build/clear_processes.py" "$@"
