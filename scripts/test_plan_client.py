#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Script kiểm thử CLI: Gửi yêu cầu tính toán đường đi tới planner_server qua action ComputePathToPose.
Tự động lấy điểm Start và Goal mặc định từ file cấu hình config/nav2_params.yaml của bản đồ.
Ví dụ:
  python3 scripts/test_plan_client.py                 # Tự động chạy với điểm mặc định và plugin mặc định
  python3 scripts/test_plan_client.py --planner SCSO   # Chạy thuật toán bầy đàn Mèo cát (SCSO)
  python3 scripts/test_plan_client.py --planner GridBased # Chạy thuật toán A*
"""

import os
import sys
import yaml
import argparse
import rclpy
from rclpy.action import ActionClient
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import ComputePathToPose


def get_default_coords():
    """Tự động đọc tọa độ Start, Goal và planner_id mặc định từ nav2_params.yaml"""
    yaml_path = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', 'config', 'nav2_params.yaml'))
    start_x, start_y = 6.5, -6.03
    goal_x, goal_y = -6.31, 6.24
    planner_id = 'GridBased'

    if os.path.isfile(yaml_path):
        try:
            with open(yaml_path, 'r', encoding='utf-8') as f:
                data = yaml.safe_load(f)
                bridge = data.get('goal_pose_bridge', {}).get('ros__parameters', {})
                start_x = bridge.get('start_x', start_x)
                start_y = bridge.get('start_y', start_y)
                goal_x = bridge.get('goal_x', goal_x)
                goal_y = bridge.get('goal_y', goal_y)
                planner_id = bridge.get('planner_id', planner_id)
        except Exception as e:
            print(f"[CẢNH BÁO] Không thể đọc nav2_params.yaml: {e}")

    return f"{start_x} {start_y}", f"{goal_x} {goal_y}", planner_id


def parse_args():
    def_start, def_goal, def_planner = get_default_coords()
    parser = argparse.ArgumentParser(description='Kiểm thử tìm đường Nav2 Global Planner (A* / SCSO) qua CLI')
    parser.add_argument(
        '--start', type=str, default=def_start,
        help=f'Tọa độ điểm bắt đầu "X Y" (mặc định lấy từ cấu hình bản đồ: "{def_start}")'
    )
    parser.add_argument(
        '--goal', type=str, default=def_goal,
        help=f'Tọa độ điểm đích "X Y" (mặc định lấy từ cấu hình bản đồ: "{def_goal}")'
    )
    parser.add_argument(
        '--planner', type=str, default=def_planner,
        help=f'Tên plugin planner_id ("GridBased" cho A*, "SCSO" cho SCSO, mặc định: "{def_planner}")'
    )
    return parser.parse_args()


def calculate_length(path):
    length = 0.0
    poses = path.poses
    for i in range(len(poses) - 1):
        dx = poses[i + 1].pose.position.x - poses[i].pose.position.x
        dy = poses[i + 1].pose.position.y - poses[i].pose.position.y
        length += (dx * dx + dy * dy) ** 0.5
    return length


def main():
    args = parse_args()

    try:
        start_x, start_y = map(float, args.start.split())
        goal_x, goal_y = map(float, args.goal.split())
    except ValueError:
        print('Lỗi định dạng tọa độ. Vui lòng truyền chuỗi dạng "X Y", ví dụ: --start "6.5 -6.03"')
        sys.exit(1)

    planner_name = args.planner.strip()
    if not planner_name:
        planner_name = 'GridBased'

    rclpy.init()
    node = rclpy.create_node('benchmark_plan_client')
    action_client = ActionClient(node, ComputePathToPose, '/compute_path_to_pose')

    print('=' * 65)
    print('  NAV2 GLOBAL PATH PLANNER BENCHMARK CLIENT')
    print('=' * 65)
    print(f'  - Thuật toán (Planner ID) : [{planner_name}]')
    print(f'  - Điểm xuất phát (Start)   : ({start_x:.2f}, {start_y:.2f}) [Mặc định từ bản đồ]')
    print(f'  - Điểm đích đến (Goal)     : ({goal_x:.2f}, {goal_y:.2f}) [Mặc định từ bản đồ]')
    print('=' * 65)

    print('Đang kết nối tới Action Server "/compute_path_to_pose"...')
    if not action_client.wait_for_server(timeout_sec=10.0):
        print('\n[LỖI] Không thể kết nối tới planner_server!')
        print('Hãy chắc chắn bạn đã khởi động hệ thống bằng lệnh:')
        print('  ./run_demo.sh')
        print('ở một cửa sổ Terminal khác trước khi chạy script này!')
        sys.exit(1)

    goal_msg = ComputePathToPose.Goal()
    goal_msg.use_start = True

    goal_msg.start.header.frame_id = 'map'
    goal_msg.start.header.stamp = node.get_clock().now().to_msg()
    goal_msg.start.pose.position.x = start_x
    goal_msg.start.pose.position.y = start_y
    goal_msg.start.pose.orientation.w = 1.0

    goal_msg.goal.header.frame_id = 'map'
    goal_msg.goal.header.stamp = node.get_clock().now().to_msg()
    goal_msg.goal.pose.position.x = goal_x
    goal_msg.goal.pose.position.y = goal_y
    goal_msg.goal.pose.orientation.w = 1.0

    goal_msg.planner_id = planner_name

    print(f'Gửi yêu cầu tới planner [{planner_name}]...')
    send_goal_future = action_client.send_goal_async(goal_msg)
    rclpy.spin_until_future_complete(node, send_goal_future, timeout_sec=5.0)
    goal_handle = send_goal_future.result()

    if not goal_handle.accepted:
        print(f'[LỖI] Yêu cầu bị từ chối bởi planner_server! (Kiểm tra lại plugin_id "{planner_name}")')
        sys.exit(1)

    print(f'Đang tính toán đường đi với thuật toán [{planner_name}]...')
    result_future = goal_handle.get_result_async()
    rclpy.spin_until_future_complete(node, result_future, timeout_sec=30.0)

    result = result_future.result().result
    time_ms = result.planning_time.sec * 1000.0 + result.planning_time.nanosec / 1e6

    poses_count = len(result.path.poses)
    if poses_count > 0:
        total_len = calculate_length(result.path)
        print('\n' + '=' * 65)
        print(f'  KẾT QUẢ TÍNH TOÁN ĐƯỜNG ĐI [{planner_name}]: THÀNH CÔNG!')
        print('=' * 65)
        print(f'  - Thời gian xử lý (Runtime)       : {time_ms:.2f} ms')
        print(f'  - Chiều dài đường đi (Path Length): {total_len:.3f} m')
        print(f'  - Số lượng điểm mốc (Waypoints)   : {poses_count} poses')
        print(f'  - Đường đi đã được phát lên topic /plan để RViz2 hiển thị trực quan!')
        print('=' * 65 + '\n')
    else:
        print('\n' + '=' * 65)
        print(f'  [THẤT BẠI] Không tìm thấy đường đi!')
        print('  Điểm Start hoặc Goal có thể nằm trong vùng cản hoặc bị bao vây hoàn toàn.')
        print('=' * 65 + '\n')

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
