#!/usr/bin/env bash
# 启动完整仿真，在 pose_1 观察后依次执行三色抓放。
set -eo pipefail

root=$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd -P)
ros_domain=${ROS_DOMAIN_ID:-227}
headless=false
show_window=true
strategy=color_order
planner_config="$root/install/my_robot_planner/share/my_robot_planner/config/task_planner.yaml"
launch_args=()

usage() {
    cat <<'HELP'
用法: ./script/pick_three_colors.sh [选项] [-- launch参数...]

  --ros-domain-id N       ROS 域（默认继承环境，否则使用 227）
  --headless              隐藏 MuJoCo 窗口
  --no-debug-window       隐藏颜色调试窗口
  --strategy NAME         color_order（默认）或 nearest
  --planner-config FILE   任务规划器配置文件
  -h, --help              显示帮助

例如: ./script/pick_three_colors.sh --ros-domain-id 227 -- sim_speed_factor:=0.5
Ctrl+C 停止本脚本启动的仿真和任务节点。
HELP
}

while (( $# )); do
    case "$1" in
        --ros-domain-id|--strategy|--planner-config)
            if (( $# < 2 )); then echo "$1 需要一个值" >&2; exit 2; fi
            case "$1" in
                --ros-domain-id) ros_domain=$2 ;;
                --strategy) strategy=$2 ;;
                --planner-config) planner_config=$2 ;;
            esac
            shift 2 ;;
        --headless) headless=true; shift ;;
        --no-debug-window) show_window=false; shift ;;
        -h|--help) usage; exit 0 ;;
        --) shift; launch_args=("$@"); break ;;
        *) echo "未知选项: $1" >&2; usage; exit 2 ;;
    esac
done
if [[ $strategy != color_order && $strategy != nearest ]]; then
    echo '策略只能是 color_order 或 nearest。' >&2
    exit 2
fi

if [[ ! -f $planner_config ]]; then
    echo "找不到任务规划器配置: $planner_config" >&2
    exit 1
fi
planner_config=$(realpath "$planner_config")

# 加载本工作区的 ROS 环境，所有子进程继承同一 ROS 域。
source /opt/ros/humble/setup.bash
source "$root/install/setup.bash"
export ROS_DOMAIN_ID=$ros_domain
export PYTHONUNBUFFERED=1
cd "$root"

# 同域节点可能与仿真控制器或机器人描述冲突，启动前先检查。
existing_nodes=$(timeout 8 ros2 node list --no-daemon) || {
    echo '无法检查 ROS 域，请确认 ROS 环境和域号。' >&2
    exit 1
}
if [[ -n $existing_nodes ]]; then
    printf 'ROS 域 %s 已有节点，请选择空闲域：\n%s\n' "$ROS_DOMAIN_ID" "$existing_nodes" >&2
    exit 1
fi

world_pid=''
planner_pid=''
action_pid=''
tail_pid=''
result_file=$(mktemp)
cleanup() {
    # 只停止本脚本创建的进程组，不使用按名称全局清理。
    for pid in "$action_pid" "$planner_pid" "$world_pid"; do
        if [[ -n $pid ]] && kill -0 "$pid" 2>/dev/null; then
            kill -INT -- "-$pid" 2>/dev/null || true
        fi
    done
    for _ in {1..10}; do
        alive=false
        for pid in "$action_pid" "$planner_pid" "$world_pid"; do
            if [[ -n $pid ]] && kill -0 "$pid" 2>/dev/null; then alive=true; fi
        done
        $alive || break
        sleep 1
    done
    for pid in "$action_pid" "$planner_pid" "$world_pid"; do
        if [[ -n $pid ]] && kill -0 "$pid" 2>/dev/null; then
            kill -KILL -- "-$pid" 2>/dev/null || true
        fi
        [[ -z $pid ]] || wait "$pid" 2>/dev/null || true
    done
    if [[ -n $tail_pid ]]; then
        kill "$tail_pid" 2>/dev/null || true
        wait "$tail_pid" 2>/dev/null || true
    fi
    rm -f "$result_file"
}
trap cleanup EXIT
trap 'exit 130' INT
trap 'exit 143' TERM

echo "ROS 域: $ROS_DOMAIN_ID；正在启动 MuJoCo。"
setsid ros2 launch my_robot_bringup mujoco.launch.py \
    "headless:=$headless" "show_window:=$show_window" "${launch_args[@]}" &
world_pid=$!

# 控制器、执行 Action 和相机必须就绪，才发送会让机械臂运动的任务。
deadline=$((SECONDS + 120))
ready=false
while (( SECONDS < deadline )) && kill -0 "$world_pid" 2>/dev/null; do
    controllers=$(timeout 5 ros2 control list_controllers 2>/dev/null | sed -E 's/\x1b\[[0-9;]*m//g' || true)
    if [[ $(grep -Ec '^(arm_controller|gripper_controller|joint_state_broadcaster)[[:space:]].*[[:space:]]active([[:space:]]|$)' <<< "$controllers") != 3 ]]; then
        sleep 1
        continue
    fi
    pick_action=$(timeout 5 ros2 param get --no-daemon --hide-type /pick_place_executor action_name 2>/dev/null || true)
    named_action=$(timeout 5 ros2 param get --no-daemon --hide-type /pick_place_executor move_to_named_action 2>/dev/null || true)
    image_topic=$(timeout 5 ros2 param get --no-daemon --hide-type /color_detector image_topic 2>/dev/null || true)
    objects_topic=$(timeout 5 ros2 param get --no-daemon --hide-type /depth_localizer objects_topic 2>/dev/null || true)
    actions=$(timeout 5 ros2 action list 2>/dev/null || true)
    if [[ -n $pick_action && -n $named_action && -n $image_topic && -n $objects_topic ]] &&
       grep -Fxq "$pick_action" <<< "$actions" && grep -Fxq "$named_action" <<< "$actions" &&
       timeout 8 ros2 topic echo --once --qos-reliability best_effort "$image_topic" >/dev/null 2>&1; then
        ready=true
        break
    fi
    sleep 1
done
if ! $ready; then
    echo '仿真未就绪：请检查控制器、执行 Action 和相机图像。' >&2
    exit 1
fi

echo '仿真已就绪；启动任务规划器并前往 pose_1。'
# 读取节点的实际接口名称，保持 launch 覆盖参数与任务规划器一致。
setsid ros2 run my_robot_planner task_planner --ros-args --params-file "$planner_config" \
    -p use_sim_time:=true -p observation_target:=pose_1 \
    -p "objects_topic:=$objects_topic" -p "pick_place_action:=$pick_action" \
    -p "move_to_named_action:=$named_action" &
planner_pid=$!
deadline=$((SECONDS + 30))
sort_action=''
while (( SECONDS < deadline )) && kill -0 "$planner_pid" 2>/dev/null && kill -0 "$world_pid" 2>/dev/null; do
    sort_action=$(timeout 5 ros2 param get --no-daemon --hide-type /task_planner action_name 2>/dev/null || true)
    actions=$(timeout 5 ros2 action list 2>/dev/null || true)
    if [[ -n $sort_action ]] && grep -Fxq "$sort_action" <<< "$actions"; then
        break
    fi
    sort_action=''
    sleep 1
done
if [[ -z $sort_action ]]; then
    echo '任务规划器的分拣 Action 未就绪。' >&2
    exit 1
fi

# ros2 action send_goal 在任务被拒绝或失败时也可能返回 0，因此检查结果内容。
setsid ros2 action send_goal --feedback "$sort_action" \
    my_robot_interface/action/SortObjects \
    "{strategy: $strategy, color_order: [1, 2, 3]}" >"$result_file" 2>&1 &
action_pid=$!
tail -n +1 -f --pid="$action_pid" "$result_file" &
tail_pid=$!
while kill -0 "$action_pid" 2>/dev/null; do
    if ! kill -0 "$world_pid" 2>/dev/null || ! kill -0 "$planner_pid" 2>/dev/null; then
        echo '仿真或任务规划器已退出，停止当前任务。' >&2
        exit 1
    fi
    sleep 1
done
wait "$action_pid" || true
wait "$tail_pid" || true
action_pid=''
tail_pid=''
if grep -Fq 'Goal finished with status: SUCCEEDED' "$result_file" &&
   grep -Eq '^[[:space:]]*success: true' "$result_file"; then
    echo '三色抓放动作已完成，请检查物块的实际放置结果。'
    result=0
else
    echo '三色任务未成功完成，请检查上方结果和节点日志。' >&2
    result=1
fi

echo '仿真保持运行，按 Ctrl+C 退出。'
wait "$world_pid" || true
exit "$result"
