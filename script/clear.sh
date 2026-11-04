#!/usr/bin/env bash
# 清理从本工作区启动、但启动进程已经退出的已知 ROS 进程。

root=$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd -P)
dry_run=false
if [[ ${1:-} == --dry-run && $# == 1 ]]; then
    dry_run=true
elif [[ $# != 0 ]]; then
    echo "用法: $0 [--dry-run]" >&2
    exit 2
fi

pids=()
starts=()
for proc in /proc/[0-9]*; do
    [[ -O "$proc" && -r "$proc/cmdline" && -r "$proc/stat" ]] || continue
    pid=${proc##*/}
    read -r _ _ state parent _ < "$proc/stat" || continue
    [[ $state != Z ]] || continue

    # 只处理被 init 或用户 systemd 接管的进程，保留仍有启动进程管理的节点。
    if [[ $parent != 1 && $(cat "/proc/$parent/comm" 2>/dev/null) != systemd ]]; then
        continue
    fi
    cwd=$(readlink -f "$proc/cwd" 2>/dev/null) || continue
    case "$cwd" in "$root"|"$root"/*) ;; *) continue ;; esac

    mapfile -d '' -t args < "$proc/cmdline"
    executable=${args[0]:-}
    if [[ $executable == */python* ]]; then
        executable=${args[1]:-}
    fi
    case "$executable" in
        */lib/robot_state_publisher/robot_state_publisher|\
        */lib/mujoco_ros2_control/ros2_control_node|\
        */lib/controller_manager/spawner|\
        */lib/moveit_ros_move_group/move_group|\
        */lib/my_robot_commander/pick_place_executor|\
        */lib/my_robot_perception/color_detector|\
        */lib/my_robot_perception/depth_localizer)
            pids+=("$pid")
            starts+=("$(awk '{print $22}' "$proc/stat")")
            printf '残留进程 PID=%s: %s\n' "$pid" "$executable"
            ;;
    esac
done

if (( ${#pids[@]} == 0 )); then
    echo '没有发现符合条件的残留进程。'
    exit 0
fi
if $dry_run; then
    echo '仅显示结果，没有发送信号。'
    exit 0
fi

# 确认 PID 仍属于同一个进程，依次尝试正常退出和强制终止。
same_process() {
    local stat="/proc/${pids[$1]}/stat"
    [[ -r $stat && $(awk '{print $22}' "$stat") == "${starts[$1]}" ]]
}
for signal in INT TERM KILL; do
    alive=false
    for i in "${!pids[@]}"; do
        if same_process "$i"; then
            kill -s "$signal" "${pids[$i]}" 2>/dev/null || true
            alive=true
        fi
    done
    $alive || break
    if [[ $signal == KILL ]]; then sleep 1; else sleep 2; fi
done

for i in "${!pids[@]}"; do
    if same_process "$i"; then
        echo "PID ${pids[$i]} 尚未消失，请检查进程状态。" >&2
        exit 1
    fi
done
echo '残留进程已清理。'
