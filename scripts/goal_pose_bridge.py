#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Node cầu nối tương tác RViz2 -> Planner Server (ComputePathToPose).
Đặc điểm:
1. ĐÁNH DẤU HÌNH TRÒN CỰC KỲ RÕ NÉT & DỄ NHÌN TRÊN RVIZ2:
   - Điểm đầu (START): Hình tròn phẳng MÀU XANH LÁ (Green) nổi bật, có viền tương phản.
   - Điểm cuối (GOAL): Hình tròn phẳng MÀU ĐỎ (Red) nổi bật, có viền tương phản.
   - Kích thước: Mặc định đường kính 0.22m (~1/3 chiều rộng vật cản cột 0.50m).
   - Chiều cao Z được nâng cao lên 0.08m để nổi hẳn lên trên lớp bản đồ và costmap (không bị chìm hay mờ).
2. TỰ ĐỘNG LẬP ĐƯỜNG ĐI (auto_plan):
   - Tự động gọi A* Planner ngay khi hệ thống sẵn sàng.
3. HỖ TRỢ ĐỔI ĐIỂM LINH HOẠT TRÊN RVIZ2:
   - "2D Pose Estimate" để đổi điểm đầu (Start) -> Hình tròn Xanh lá nhảy theo.
   - "2D Goal Pose" để đổi điểm cuối (Goal) -> Hình tròn Đỏ nhảy theo.
"""

import math
import sys
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy, QoSHistoryPolicy
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from nav2_msgs.action import ComputePathToPose
from visualization_msgs.msg import Marker, MarkerArray


class GoalPoseBridge(Node):

    def __init__(self):
        super().__init__('goal_pose_bridge')

        # ----------------------------------------------------------------------
        # Khai báo các tham số cấu hình
        # ----------------------------------------------------------------------
        self.declare_parameter('planner_id', 'GridBased')
        self.declare_parameter('auto_plan', True)
        self.declare_parameter('start_x', 6.5)
        self.declare_parameter('start_y', -6.03)
        self.declare_parameter('start_yaw', 0.0)
        self.declare_parameter('goal_x', -6.31)
        self.declare_parameter('goal_y', 6.24)
        self.declare_parameter('goal_yaw', 0.0)
        self.declare_parameter('publish_markers', True)

        # Kích thước hình tròn: 0.60m (~1/3 chiều rộng vật cản cột 0.50m)
        self.declare_parameter('marker_diameter', 0.60)
        self.declare_parameter('marker_height', 0.20)

        self.planner_id = self.get_parameter('planner_id').get_parameter_value().string_value
        if not self.planner_id:
            self.planner_id = 'GridBased'
        self.auto_plan = self.get_parameter('auto_plan').get_parameter_value().bool_value
        self.publish_markers = self.get_parameter('publish_markers').get_parameter_value().bool_value

        start_x = self.get_parameter('start_x').get_parameter_value().double_value
        start_y = self.get_parameter('start_y').get_parameter_value().double_value
        start_yaw = self.get_parameter('start_yaw').get_parameter_value().double_value

        goal_x = self.get_parameter('goal_x').get_parameter_value().double_value
        goal_y = self.get_parameter('goal_y').get_parameter_value().double_value
        goal_yaw = self.get_parameter('goal_yaw').get_parameter_value().double_value

        self.marker_diameter = self.get_parameter('marker_diameter').get_parameter_value().double_value
        self.marker_height = self.get_parameter('marker_height').get_parameter_value().double_value

        # Khởi tạo điểm Start cố định
        self.start_pose = PoseStamped()
        self.start_pose.header.frame_id = 'map'
        self.start_pose.pose.position.x = start_x
        self.start_pose.pose.position.y = start_y
        self.start_pose.pose.orientation.z = math.sin(start_yaw / 2.0)
        self.start_pose.pose.orientation.w = math.cos(start_yaw / 2.0)

        # Khởi tạo điểm Goal cố định
        self.goal_pose = PoseStamped()
        self.goal_pose.header.frame_id = 'map'
        self.goal_pose.pose.position.x = goal_x
        self.goal_pose.pose.position.y = goal_y
        self.goal_pose.pose.orientation.z = math.sin(goal_yaw / 2.0)
        self.goal_pose.pose.orientation.w = math.cos(goal_yaw / 2.0)

        # Action client kết nối tới planner_server
        self._action_client = ActionClient(self, ComputePathToPose, '/compute_path_to_pose')

        # Publisher xuất bản Marker hình tròn lên RViz2
        self._marker_pub = self.create_publisher(MarkerArray, '/planning_markers', 10)

        # Subscriber nhận điểm bắt đầu từ RViz "2D Pose Estimate"
        self._initial_pose_sub = self.create_subscription(
            PoseWithCovarianceStamped,
            '/initialpose',
            self._initial_pose_callback,
            10
        )

        # Subscriber nhận điểm đích từ RViz "2D Goal Pose"
        self._goal_pose_sub = self.create_subscription(
            PoseStamped,
            '/goal_pose',
            self._goal_pose_callback,
            10
        )

        # Timer xuất bản markers liên tục 2Hz để luôn hiển thị rõ nét trên RViz2
        self._marker_timer = self.create_timer(0.5, self._publish_markers)

        # Cờ đánh dấu đã tự động gửi kế hoạch ban đầu chưa
        self._initial_plan_sent = False
        if self.auto_plan:
            self._auto_plan_timer = self.create_timer(0.5, self._check_and_trigger_initial_plan)

        self.get_logger().info('=====================================================================')
        self.get_logger().info('ĐÃ KHỞI CHẠY ĐIỀU KHIỂN & ĐÁNH DẤU HÌNH TRÒN (GoalPoseBridge)!')
        self.get_logger().info(f'  - Điểm đầu (START - Hình tròn XANH LÁ): ({start_x:.2f}, {start_y:.2f})')
        self.get_logger().info(f'  - Điểm cuối (GOAL - Hình tròn ĐỎ):       ({goal_x:.2f}, {goal_y:.2f})')
        self.get_logger().info(f'  - Đường kính hình tròn:                  {self.marker_diameter:.2f} m (~1/3 vật cản)')
        self.get_logger().info(f'  - Tự động lập đường đi A*:               {"BẬT" if self.auto_plan else "TẮT"}')
        self.get_logger().info('=====================================================================')

    def _check_and_trigger_initial_plan(self):
        """Tự động gọi A* planner khi action server sẵn sàng"""
        if self._initial_plan_sent:
            return

        if self._action_client.server_is_ready():
            self._initial_plan_sent = True
            self.get_logger().info('Action server sẵn sàng! Đang tự động tìm đường từ Hình tròn Xanh -> Hình tròn Đỏ...')
            self._request_plan()
            self._auto_plan_timer.cancel()

    def _initial_pose_callback(self, msg: PoseWithCovarianceStamped):
        """Cập nhật điểm đầu khi người dùng click 2D Pose Estimate trên RViz2"""
        self.start_pose.header = msg.header
        self.start_pose.pose = msg.pose.pose
        self.get_logger().info(
            f'Cập nhật Điểm Đầu mới (Hình tròn Xanh lá): ({self.start_pose.pose.position.x:.2f}, {self.start_pose.pose.position.y:.2f})'
        )
        self._publish_markers()
        self._request_plan()

    def _goal_pose_callback(self, msg: PoseStamped):
        """Cập nhật điểm cuối khi người dùng click 2D Goal Pose trên RViz2"""
        self.goal_pose = msg
        self.get_logger().info(
            f'Cập nhật Điểm Cuối mới (Hình tròn Đỏ): ({self.goal_pose.pose.position.x:.2f}, {self.goal_pose.pose.position.y:.2f})'
        )
        self._publish_markers()
        self._request_plan()

    def _request_plan(self):
        """Gửi yêu cầu ComputePathToPose tới planner_server"""
        if not self._action_client.wait_for_server(timeout_sec=1.0):
            self.get_logger().warn('Chưa thể kết nối tới planner_server action!')
            return

        x_s = self.start_pose.pose.position.x
        y_s = self.start_pose.pose.position.y
        x_g = self.goal_pose.pose.position.x
        y_g = self.goal_pose.pose.position.y

        self.get_logger().info(f'Đang tính toán đường đi với plugin [{self.planner_id}] từ ({x_s:.2f}, {y_s:.2f}) -> ({x_g:.2f}, {y_g:.2f})...')

        goal_msg = ComputePathToPose.Goal()
        goal_msg.use_start = True
        goal_msg.start = self.start_pose
        goal_msg.start.header.stamp = self.get_clock().now().to_msg()
        goal_msg.goal = self.goal_pose
        goal_msg.goal.header.stamp = self.get_clock().now().to_msg()
        goal_msg.planner_id = self.planner_id

        send_future = self._action_client.send_goal_async(goal_msg)
        send_future.add_done_callback(self._goal_response_callback)

    def _goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warn('Yêu cầu tìm đường bị planner_server từ chối!')
            return

        res_future = goal_handle.get_result_async()
        res_future.add_done_callback(self._result_callback)

    def _result_callback(self, future):
        result = future.result().result
        path = result.path
        time_ms = result.planning_time.sec * 1000.0 + result.planning_time.nanosec / 1e6

        if len(path.poses) > 0:
            self.get_logger().info(
                f'LẬP ĐƯỜNG ĐI THÀNH CÔNG! Số điểm waypoints: {len(path.poses)} | Thời gian: {time_ms:.2f} ms.'
            )
        else:
            self.get_logger().warn('Không tìm thấy đường đi khả thi (bị chặn bởi vật cản hoặc ngoài bản đồ)!')

    def _publish_markers(self):
        """Xuất bản visual markers hình tròn nổi bật: START (Xanh lá) và GOAL (Đỏ)"""
        if not self.publish_markers:
            return

        marker_array = MarkerArray()
        now = self.get_clock().now().to_msg()

        sx = self.start_pose.pose.position.x
        sy = self.start_pose.pose.position.y

        gx = self.goal_pose.pose.position.x
        gy = self.goal_pose.pose.position.y

        # Độ cao Z đặt ở 0.08m để nổi hẳn lên trên mặt sàn map và costmap (tránh bị che khuất)
        z_pos = 0.08
        z_thickness = self.marker_height  # 0.08m

        # ======================================================================
        # 1. ĐIỂM ĐẦU (START) - HÌNH TRÒN XANH LÁ RỰC RỠ CÓ VIỀN TƯƠNG PHẢN
        # ======================================================================

        # 1.1 Viền ngoài tương phản màu xanh đậm (đường kính lớn hơn 30%)
        m_start_border = Marker()
        m_start_border.header.frame_id = 'map'
        m_start_border.header.stamp = now
        m_start_border.ns = 'planning_markers'
        m_start_border.id = 0
        m_start_border.type = Marker.CYLINDER
        m_start_border.action = Marker.ADD
        m_start_border.pose.position.x = sx
        m_start_border.pose.position.y = sy
        m_start_border.pose.position.z = z_pos - 0.02
        m_start_border.pose.orientation.w = 1.0
        m_start_border.scale.x = self.marker_diameter * 1.35
        m_start_border.scale.y = self.marker_diameter * 1.35
        m_start_border.scale.z = z_thickness * 0.7
        m_start_border.color.r = 0.0
        m_start_border.color.g = 0.35
        m_start_border.color.b = 0.0
        m_start_border.color.a = 0.95
        marker_array.markers.append(m_start_border)

        # 1.2 Lõi hình tròn chính màu Xanh Lá Rực Rỡ (Pure Bright Neon Green)
        m_start_core = Marker()
        m_start_core.header.frame_id = 'map'
        m_start_core.header.stamp = now
        m_start_core.ns = 'planning_markers'
        m_start_core.id = 1
        m_start_core.type = Marker.CYLINDER
        m_start_core.action = Marker.ADD
        m_start_core.pose.position.x = sx
        m_start_core.pose.position.y = sy
        m_start_core.pose.position.z = z_pos
        m_start_core.pose.orientation.w = 1.0
        m_start_core.scale.x = self.marker_diameter
        m_start_core.scale.y = self.marker_diameter
        m_start_core.scale.z = z_thickness
        m_start_core.color.r = 0.0
        m_start_core.color.g = 1.0
        m_start_core.color.b = 0.0
        m_start_core.color.a = 1.0
        marker_array.markers.append(m_start_core)

        # ======================================================================
        # 2. ĐIỂM CUỐI (GOAL) - HÌNH TRÒN ĐỎ RỰC RỠ CÓ VIỀN TƯƠNG PHẢN
        # ======================================================================

        # 2.1 Viền ngoài tương phản màu đỏ đậm (đường kính lớn hơn 30%)
        m_goal_border = Marker()
        m_goal_border.header.frame_id = 'map'
        m_goal_border.header.stamp = now
        m_goal_border.ns = 'planning_markers'
        m_goal_border.id = 2
        m_goal_border.type = Marker.CYLINDER
        m_goal_border.action = Marker.ADD
        m_goal_border.pose.position.x = gx
        m_goal_border.pose.position.y = gy
        m_goal_border.pose.position.z = z_pos - 0.02
        m_goal_border.pose.orientation.w = 1.0
        m_goal_border.scale.x = self.marker_diameter * 1.35
        m_goal_border.scale.y = self.marker_diameter * 1.35
        m_goal_border.scale.z = z_thickness * 0.7
        m_goal_border.color.r = 0.45
        m_goal_border.color.g = 0.0
        m_goal_border.color.b = 0.0
        m_goal_border.color.a = 0.95
        marker_array.markers.append(m_goal_border)

        # 2.2 Lõi hình tròn chính màu Đỏ Tươi Rực Rỡ (Vivid Crimson Red)
        m_goal_core = Marker()
        m_goal_core.header.frame_id = 'map'
        m_goal_core.header.stamp = now
        m_goal_core.ns = 'planning_markers'
        m_goal_core.id = 3
        m_goal_core.type = Marker.CYLINDER
        m_goal_core.action = Marker.ADD
        m_goal_core.pose.position.x = gx
        m_goal_core.pose.position.y = gy
        m_goal_core.pose.position.z = z_pos
        m_goal_core.pose.orientation.w = 1.0
        m_goal_core.scale.x = self.marker_diameter
        m_goal_core.scale.y = self.marker_diameter
        m_goal_core.scale.z = z_thickness
        m_goal_core.color.r = 1.0
        m_goal_core.color.g = 0.0
        m_goal_core.color.b = 0.0
        m_goal_core.color.a = 1.0
        marker_array.markers.append(m_goal_core)

        self._marker_pub.publish(marker_array)


def main(args=None):
    rclpy.init(args=args)
    bridge_node = GoalPoseBridge()
    try:
        rclpy.spin(bridge_node)
    except KeyboardInterrupt:
        pass
    finally:
        bridge_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
