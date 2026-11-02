#!/usr/bin/env bash
# ==============================================================================
# Script tiện ích: Khởi chạy demo quy hoạch đường đi toàn cục A* với RViz2
# ==============================================================================

set -e

# Xác định đường dẫn thư mục hiện tại
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# 1. Source môi trường ROS 2 Humble
if [ -f "/opt/ros/humble/setup.bash" ]; then
    source "/opt/ros/humble/setup.bash"
else
    echo "LỖI: Không tìm thấy cài đặt ROS 2 Humble tại /opt/ros/humble/setup.bash!"
    exit 1
fi

# 2. Source workspace hiện tại nếu đã build
if [ -f "${SCRIPT_DIR}/install/setup.bash" ]; then
    source "${SCRIPT_DIR}/install/setup.bash"
fi

# 3. Kiểm tra các gói công cụ dòng lệnh thường thiếu trong ROS 2
MISSING_PKGS=()

if ! dpkg -l | grep -q "ros-humble-ros2launch"; then
    MISSING_PKGS+=("ros-humble-ros2launch")
fi

if ! dpkg -l | grep -q "ros-humble-ros2run"; then
    MISSING_PKGS+=("ros-humble-ros2run")
fi

if ! dpkg -l | grep -q "ros-humble-rviz2"; then
    MISSING_PKGS+=("ros-humble-rviz2")
fi

if [ ${#MISSING_PKGS[@]} -gt 0 ]; then
    echo "====================================================================="
    echo "LƯU Ý: Hệ thống đang thiếu một số tiện ích dòng lệnh CLI của ROS 2:"
    echo "  ${MISSING_PKGS[*]}"
    echo "Để dùng lệnh chuẩn 'ros2 launch' và các lệnh khác, bạn nên cài đặt:"
    echo "  sudo apt update && sudo apt install -y ros-humble-ros2launch ros-humble-ros2run ros-humble-ros2topic ros-humble-ros2action ros-humble-rviz2 ros-humble-nav2-rviz-plugins"
    echo "====================================================================="
fi

echo "====================================================================="
echo "KHỞI CHẠY DEMO NAV2 A* GLOBAL PATH PLANNING"
echo "  - Map: ${SCRIPT_DIR}/maps/sample_map.yaml"
echo "  - Global Planner: nav2_navfn_planner/NavfnPlanner (A* enabled)"
echo "  - Giao diện RViz2: ${SCRIPT_DIR}/rviz/astar_demo.rviz"
echo "====================================================================="

# 4. Kiểm tra xem lệnh 'ros2 launch' có sẵn không
if ros2 launch --help &> /dev/null; then
    echo "[INFO] Sử dụng trình khởi chạy chuẩn: ros2 launch..."
    exec ros2 launch nav2_astar_demo astar_demo.launch.py "$@"
else
    echo "[INFO] Đang khởi chạy tự động qua python launch_demo.py (LaunchService)..."
    exec python3 "${SCRIPT_DIR}/launch_demo.py" "$@"
fi
