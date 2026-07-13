#!/usr/bin/env bash
# 启动 RM65 MoveIt fake hardware demo，等就绪后运行仿真安全测试。
# 用法（容器外宿主机）：bash scripts/run_sim_move_test.sh
# 用法（容器内）：       bash /workspace/RM/scripts/run_sim_move_test.sh

set -eo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"

# 容器内路径优先，宿主机直接运行时用当前路径
if [[ -d /workspace/RM ]]; then
    WS_ROOT="/workspace/RM"
else
    WS_ROOT="${REPO_ROOT}"
fi

MAIN_WS="${WS_ROOT}/workspaces/rm_yolo_ros2_ws"
TEST_SCRIPT="${WS_ROOT}/scripts/test_rm_move_safe_sim.py"

if [[ ! -f /opt/ros/humble/setup.bash ]]; then
    echo "[ERROR] 未找到 ROS 2 Humble，请在容器内运行。"
    exit 1
fi

source /opt/ros/humble/setup.bash
source "${MAIN_WS}/install/setup.bash"

# --- 检查 demo 是否已在运行 ---
if ros2 node list 2>/dev/null | grep -q "move_group"; then
    echo "[INFO] 检测到 move_group 已在运行，跳过启动 demo。"
else
    echo "[INFO] 启动 rm_65_config demo.launch.py（后台）…"
    ros2 launch rm_65_config demo.launch.py &
    DEMO_PID=$!

    echo "[INFO] 等待 move_group 就绪（最多 30 秒）…"
    for i in $(seq 1 30); do
        sleep 1
        if ros2 node list 2>/dev/null | grep -q "move_group"; then
            echo "[INFO] move_group 已就绪。"
            break
        fi
        if [[ $i -eq 30 ]]; then
            echo "[ERROR] 超时：move_group 未启动，请检查日志。"
            kill "${DEMO_PID}" 2>/dev/null || true
            exit 1
        fi
    done
fi

echo ""
echo "[INFO] 运行仿真安全测试…"
echo "=============================="
python3 "${TEST_SCRIPT}"
