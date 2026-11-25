#!/usr/bin/env bash
# GraspNet 候选抓取入口：在 pick_three_colors.sh 基础上启动 grasp_candidate_publisher。
#
# 数据驱动降级：publisher 在线 = 候选可用，执行器优先尝试候选姿态；
# publisher 未启动、推理失败或候选过期 = 自动回退传统深度链路，无需模式开关。
#
# 用法: ./script/pick_graspnet.sh [选项] [-- launch参数...]
# 仿真选项（--headless / --ros-domain-id / --config 等）原样透传给 pick_three_colors.sh。
set -e

root=$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd -P)
conda_python=${GRASPNET_PYTHON:-/home/ltt/miniconda3/envs/graspnet/bin/python}
weights="$root/tools/graspnet/models/artifacts/checkpoint-rs.tar"
predict_script="$root/tools/graspnet/scripts/predict.py"
top_k=5
min_interval=5.0
ros_domain=${ROS_DOMAIN_ID:-227}
dry_run=false
passthrough=()
launch_extra=()
# 第一轮/通用模型的类别不同，默认用包含八类项目物体的第二轮模型。
default_launch=(detector:=yolo "yolo_model:=$root/tools/yolo/models/artifacts/round_02_e050_20261114_152654.onnx")

usage() {
    cat <<'HELP'
用法: ./script/pick_graspnet.sh [选项] [-- launch参数...]

GraspNet 选项:
  --conda-python PATH   GraspNet conda 解释器（默认 $GRASPNET_PYTHON 或
                        /home/ltt/miniconda3/envs/graspnet/bin/python）
  --weights FILE        模型权重（默认 tools/graspnet/models/artifacts/checkpoint-rs.tar）
  --top-k N             每个物体发布的候选数（默认 5）
  --min-interval S      最小推理间隔秒（默认 5.0）

仿真选项（透传给 pick_three_colors.sh）:
  --headless            隐藏 MuJoCo 窗口
  --no-debug-window     隐藏颜色调试窗口
  --dry-run             仅检查仿真启动配置
  --ros-domain-id N     ROS 域（默认 227）
  --rviz                打开 MoveIt RViz
  --config FILE         总启动配置
  -- detector:=cv       显式改用 CV；yolo_model:=PATH 可覆盖默认模型
  -h, --help            显示帮助

例如: ./script/pick_graspnet.sh --headless --top-k 3
      ./script/pick_graspnet.sh --ros-domain-id 230 -- sim_speed_factor:=0.5
HELP
}

while (( $# )); do
    case "$1" in
        --conda-python|--weights|--top-k|--min-interval)
            if (( $# < 2 )); then echo "$1 需要一个值" >&2; exit 2; fi
            case "$1" in
                --conda-python) conda_python=$2 ;;
                --weights) weights=$(realpath -e "$2") ;;
                --top-k) top_k=$2 ;;
                --min-interval) min_interval=$2 ;;
            esac
            shift 2 ;;
        --ros-domain-id)
            if (( $# < 2 )); then echo "$1 需要一个值" >&2; exit 2; fi
            ros_domain=$2
            passthrough+=("$1" "$2")
            shift 2 ;;
        --dry-run) dry_run=true; passthrough+=("$1"); shift ;;
        -h|--help) usage; exit 0 ;;
        --) shift; break ;;
        *) passthrough+=("$1"); shift ;;
    esac
done

# 启动参数放在脚本选项之后，显式参数可覆盖 GraspNet 的入口默认值。
for argument in "$@"; do
    case "$argument" in
        detector:=*) default_launch[0]=$argument ;;
        yolo_model:=*) default_launch[1]=$argument ;;
        *) launch_extra+=("$argument") ;;
    esac
done
passthrough+=(-- "${default_launch[@]}" "${launch_extra[@]}")

if $dry_run; then
    exec "$root/script/pick_three_colors.sh" "${passthrough[@]}"
fi

source /opt/ros/humble/setup.bash
source "$root/install/setup.bash"
export ROS_DOMAIN_ID=$ros_domain

# 前置检查：conda 解释器与权重必须存在；CUDA 可用性仅警告（推理失败时自动降级）。
if [[ ! -x $conda_python ]]; then
    echo "GraspNet conda 解释器不存在: $conda_python" >&2
    echo '候选发布不可用；仿真将以传统链路运行。' >&2
    exec "$root/script/pick_three_colors.sh" "${passthrough[@]}"
fi
if [[ ! -f $weights ]]; then
    echo "模型权重不存在: $weights" >&2
    echo '候选发布不可用；仿真将以传统链路运行。' >&2
    exec "$root/script/pick_three_colors.sh" "${passthrough[@]}"
fi
if ! "$conda_python" -c 'import torch; assert torch.cuda.is_available()' 2>/dev/null; then
    echo '警告: GraspNet 环境 CUDA 不可用，推理将失败并自动降级为传统链路。' >&2
fi

publisher_pid=''
sim_pid=''
cleanup() {
    # 给 publisher 15 s 优雅退出：强杀正在推理的 CUDA 进程可能触发 GPU MMU Fault。
    if [[ -n $publisher_pid ]] && kill -0 "$publisher_pid" 2>/dev/null; then
        kill -INT "$publisher_pid" 2>/dev/null || true
        for _ in {1..15}; do
            kill -0 "$publisher_pid" 2>/dev/null || break
            sleep 1
        done
        kill -KILL "$publisher_pid" 2>/dev/null || true
        wait "$publisher_pid" 2>/dev/null || true
    fi
    if [[ -n $sim_pid ]] && kill -0 "$sim_pid" 2>/dev/null; then
        kill -INT "$sim_pid" 2>/dev/null || true
        wait "$sim_pid" 2>/dev/null || true
    fi
}
trap cleanup EXIT
trap 'exit 130' INT
trap 'exit 143' TERM

# 启动仿真（pick_three_colors.sh 负责域检查、launch、就绪、pose_1、分拣）。
"$root/script/pick_three_colors.sh" "${passthrough[@]}" &
sim_pid=$!

# 等待仿真节点出现后再启动 publisher，避免触发 pick_three_colors.sh 的域占用检查。
echo '等待仿真节点就绪后启动 GraspNet 候选发布……'
deadline=$((SECONDS + 120))
while (( SECONDS < deadline )) && kill -0 "$sim_pid" 2>/dev/null; do
    if timeout 5 ros2 node list --no-daemon 2>/dev/null | grep -q mujoco_ros2_control_node; then
        break
    fi
    sleep 2
done
if ! kill -0 "$sim_pid" 2>/dev/null; then
    echo '仿真已退出，候选发布取消。' >&2
    exit 1
fi

ros2 run my_robot_perception grasp_candidate_publisher --ros-args \
    -p "conda_python:=$conda_python" \
    -p "predict_script:=$predict_script" \
    -p "weights:=$weights" \
    -p "min_interval:=$min_interval" \
    -p "top_k:=$top_k" -p use_sim_time:=true &
publisher_pid=$!
echo "GraspNet 候选发布已启动 (top_k=$top_k, interval=${min_interval}s)。"

wait "$sim_pid" || true
