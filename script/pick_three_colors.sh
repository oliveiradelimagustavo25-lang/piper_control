#!/usr/bin/env bash
# 启动三色仿真，到达观察姿态后执行一次分拣任务。
set -e

root=$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd -P)
ros_domain=${ROS_DOMAIN_ID:-227}
gui=''
show_window=''
launch_config="$root/config/gazebo_launch.yaml"
planner_config=''

usage() {
    cat <<'HELP'
用法: ./script/pick_three_colors.sh [选项] [-- launch参数...]

  --ros-domain-id N       ROS 域（默认继承环境，否则使用 227）
  --headless              隐藏 Gazebo 和 RViz 窗口
  --no-debug-window       隐藏颜色调试窗口
  --config FILE           总启动配置（默认 config/gazebo_launch.yaml）
  --planner-config FILE   任务规划器配置文件
  -h, --help              显示帮助

例如: ./script/pick_three_colors.sh --ros-domain-id 227 -- gui:=false
启动后自动移动到 pose_1，并执行一次三色分拣任务。
HELP
}

while (( $# )); do
    case "$1" in
        --ros-domain-id|--planner-config|--config)
            if (( $# < 2 )); then echo "$1 需要一个值" >&2; exit 2; fi
            case "$1" in
                --ros-domain-id) ros_domain=$2 ;;
                --planner-config) planner_config=$(realpath -e "$2") ;;
                --config) launch_config=$(realpath -e "$2") ;;
            esac
            shift 2 ;;
        --headless) gui=false; shift ;;
        --no-debug-window) show_window=false; shift ;;
        -h|--help) usage; exit 0 ;;
        --) shift; break ;;
        *:=*) break ;;
        *) echo "未知选项: $1" >&2; usage; exit 2 ;;
    esac
done

source /opt/ros/humble/setup.bash
source "$root/install/setup.bash"
export ROS_DOMAIN_ID=$ros_domain
export ROS_LOCALHOST_ONLY=${ROS_LOCALHOST_ONLY:-1}
# Keep the script's ROS clients in the same domain as the launch arguments.
for argument in "$@"; do
    case "$argument" in
        launch_config:=*) launch_config=${argument#launch_config:=} ;;
        ros_domain_id:=*) export ROS_DOMAIN_ID=${argument#ros_domain_id:=} ;;
        localhost_only:=*) export ROS_LOCALHOST_ONLY=${argument#localhost_only:=} ;;
    esac
done
if [[ ${RMW_IMPLEMENTATION:-} == rmw_cyclonedds_cpp && -z ${CYCLONEDDS_URI:-} && $ROS_LOCALHOST_ONLY == 1 ]]; then
    export CYCLONEDDS_URI="file://$root/src/my_robot_bringup/config/cyclonedds_local.xml"
fi
export PYTHONUNBUFFERED=1
cd "$root"

# 避免与同一 ROS 域中其他控制器及机器人描述混用。
if ! existing_nodes=$(timeout 8 ros2 node list --no-daemon); then
    echo '无法检查 ROS 域中的节点，请检查通信环境。' >&2
    exit 1
fi
if [[ -n $existing_nodes ]]; then
    printf 'ROS 域 %s 已有节点，请选择空闲域：\n%s\n' "$ROS_DOMAIN_ID" "$existing_nodes" >&2
    exit 1
fi

args=("launch_config:=$launch_config" "ros_domain_id:=$ROS_DOMAIN_ID" "localhost_only:=$ROS_LOCALHOST_ONLY")
if [[ -n $gui ]]; then args+=("gui:=$gui" "use_rviz:=$gui"); fi
if [[ -n $show_window ]]; then args+=("show_window:=$show_window"); fi
if [[ -n $planner_config ]]; then args+=("planner_config:=$planner_config"); fi
printf 'ROS 域: %s；总配置: %s\n' "$ROS_DOMAIN_ID" "$launch_config"
if [[ -n $gui ]]; then printf '命令行覆盖 gui/use_rviz: %s\n' "$gui"; fi
if [[ -n $show_window ]]; then printf '命令行覆盖 show_window: %s\n' "$show_window"; fi
if [[ -n $planner_config ]]; then printf '规划器配置: %s\n' "$planner_config"; fi
if (( $# )); then printf '额外启动参数:'; printf ' %q' "$@"; printf '\n'; fi
# 将 launch 和动作客户端放入独立进程组，只清理由本脚本启动的进程。
launch_pid=''
action_pid=''
tail_pid=''
result_file=$(mktemp)
cleanup() {
    for pid in "$action_pid" "$launch_pid"; do
        if [[ -n $pid ]] && kill -0 "$pid" 2>/dev/null; then
            kill -INT -- "-$pid" 2>/dev/null || true
        fi
    done
    for _ in {1..10}; do
        alive=false
        for pid in "$action_pid" "$launch_pid"; do
            if [[ -n $pid ]] && kill -0 "$pid" 2>/dev/null; then alive=true; fi
        done
        $alive || break
        sleep 1
    done
    for pid in "$action_pid" "$launch_pid"; do
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

setsid ros2 launch my_robot_bringup gazebo.launch.py "${args[@]}" "$@" &
launch_pid=$!
echo '等待机械臂控制器与命名姿态 Action 就绪……'

# 读取执行器的实际 Action 名称，以兼容 YAML 和命令行覆盖。
deadline=$((SECONDS + 120))
named_action=''
while (( SECONDS < deadline )) && kill -0 "$launch_pid" 2>/dev/null; do
    controllers=$(timeout 5 ros2 control list_controllers 2>/dev/null | sed -E 's/\x1b\[[0-9;]*m//g' || true)
    if ! grep -Eq '^arm_controller[[:space:]].*[[:space:]]active([[:space:]]|$)' <<< "$controllers" ||
       ! grep -Eq '^gripper_controller[[:space:]].*[[:space:]]active([[:space:]]|$)' <<< "$controllers" ||
       ! grep -Eq '^joint_state_broadcaster[[:space:]].*[[:space:]]active([[:space:]]|$)' <<< "$controllers"; then
        sleep 1
        continue
    fi
    named_action=$(timeout 5 ros2 param get --no-daemon --hide-type \
        /pick_place_executor move_to_named_action 2>/dev/null || true)
    actions=$(timeout 5 ros2 action list 2>/dev/null || true)
    if [[ -n $named_action ]] && grep -Fxq "$named_action" <<< "$actions"; then
        break
    fi
    named_action=''
    sleep 1
done
if [[ -z $named_action ]] || ! kill -0 "$launch_pid" 2>/dev/null; then
    echo '命名姿态 Action 未就绪，请检查 MoveIt、控制器及 motion 层。' >&2
    exit 1
fi

# Action 结果必须同时检查状态和 success：ROS 2 CLI 在失败时也可能返回 0。
send_action() {
    local action=$1 type=$2 goal=$3 label=$4
    : > "$result_file"
    setsid ros2 action send_goal --feedback "$action" "$type" "$goal" >"$result_file" 2>&1 &
    action_pid=$!
    tail -n +1 -f --pid="$action_pid" "$result_file" &
    tail_pid=$!
    while kill -0 "$action_pid" 2>/dev/null; do
        if ! kill -0 "$launch_pid" 2>/dev/null; then
            echo '仿真已退出，停止当前动作。' >&2
            return 1
        fi
        sleep 1
    done
    wait "$action_pid" || true
    wait "$tail_pid" || true
    action_pid=''
    tail_pid=''
    if ! grep -Fq 'Goal finished with status: SUCCEEDED' "$result_file" ||
       ! grep -Eq '^[[:space:]]*success: true' "$result_file"; then
        echo "$label 失败，请检查上面的 Action 结果。" >&2
        return 1
    fi
}

echo "正在通过 $named_action 移动到 pose_1……"
send_action "$named_action" my_robot_interface/action/MoveToNamed '{target: pose_1}' '移动到 pose_1'
echo '已到达 pose_1，准备开始分拣。'

# 任务规划器和执行器的接口名称由它们实际运行的配置决定。
deadline=$((SECONDS + 60))
sort_action=''
while (( SECONDS < deadline )) && kill -0 "$launch_pid" 2>/dev/null; do
    sort_action=$(timeout 5 ros2 param get --no-daemon --hide-type \
        /task_planner action_name 2>/dev/null || true)
    pick_action=$(timeout 5 ros2 param get --no-daemon --hide-type \
        /pick_place_executor action_name 2>/dev/null || true)
    actions=$(timeout 5 ros2 action list 2>/dev/null || true)
    if [[ -n $sort_action && -n $pick_action ]] &&
       grep -Fxq "$sort_action" <<< "$actions" && grep -Fxq "$pick_action" <<< "$actions"; then
        break
    fi
    sort_action=''
    sleep 1
done
if [[ -z $sort_action ]] || ! kill -0 "$launch_pid" 2>/dev/null; then
    echo '分拣或抓放 Action 未就绪，请检查任务规划器和执行器。' >&2
    exit 1
fi

echo "正在通过 $sort_action 开始分拣；策略和颜色顺序使用任务规划器配置。"
send_action "$sort_action" my_robot_interface/action/SortObjects '{}' '三色分拣'
echo '分拣动作序列已完成；物块最终位置仍需视觉确认。'
echo '仿真保持运行，按 Ctrl+C 退出。'
if ! wait "$launch_pid"; then
    echo '仿真启动进程异常退出，请检查上方日志。' >&2
    exit 1
fi
