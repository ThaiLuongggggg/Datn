#include "custom_planner_example/custom_planner.hpp"
#include "pluginlib/class_list_macros.hpp"

namespace custom_planner_example
{

void CustomAStarPlanner::configure(
  const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
  std::string name,
  std::shared_ptr<tf2_ros::Buffer> tf,
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
{
  auto node = parent.lock();
  name_ = name;
  tf_ = tf;
  costmap_ = costmap_ros->getCostmap();
  clock_ = node->get_clock();
  logger_ = node->get_logger();

  RCLCPP_INFO(
    logger_,
    "[CustomAStarPlanner] Cấu hình thành công plugin: %s",
    name_.c_str());
}

void CustomAStarPlanner::cleanup()
{
  RCLCPP_INFO(
    logger_,
    "[CustomAStarPlanner] Giải phóng tài nguyên plugin: %s",
    name_.c_str());
  costmap_ = nullptr;
}

void CustomAStarPlanner::activate()
{
  RCLCPP_INFO(
    logger_,
    "[CustomAStarPlanner] Kích hoạt (Activate) plugin: %s",
    name_.c_str());
}

void CustomAStarPlanner::deactivate()
{
  RCLCPP_INFO(
    logger_,
    "[CustomAStarPlanner] Hủy kích hoạt (Deactivate) plugin: %s",
    name_.c_str());
}

nav_msgs::msg::Path CustomAStarPlanner::createPlan(
  const geometry_msgs::msg::PoseStamped & start,
  const geometry_msgs::msg::PoseStamped & goal)
{
  nav_msgs::msg::Path global_path;
  global_path.poses.clear();
  global_path.header.stamp = clock_->now();
  global_path.header.frame_id = goal.header.frame_id;

  if (start.header.frame_id != goal.header.frame_id) {
    RCLCPP_ERROR(
      logger_,
      "[CustomAStarPlanner] Frame của start (%s) và goal (%s) không khớp nhau!",
      start.header.frame_id.c_str(), goal.header.frame_id.c_str());
    return global_path;
  }

  RCLCPP_INFO(
    logger_,
    "[CustomAStarPlanner] Tìm đường từ (%.2f, %.2f) đến (%.2f, %.2f)",
    start.pose.position.x, start.pose.position.y,
    goal.pose.position.x, goal.pose.position.y);

  // ---------------------------------------------------------------------------
  // TRIỂN KHAI THUẬT TOÁN A* TẠI ĐÂY:
  // 1. Chuyển đổi tọa độ thế giới (x, y) sang tọa độ ô lưới (mx, my) của Costmap:
  //    unsigned int start_mx, start_my, goal_mx, goal_my;
  //    costmap_->worldToMap(start.pose.position.x, start.pose.position.y, start_mx, start_my);
  //    costmap_->worldToMap(goal.pose.position.x, goal.pose.position.y, goal_mx, goal_my);
  //
  // 2. Chạy hàng đợi ưu tiên (Priority Queue) của A* với hàm chi phí f(n) = g(n) + h(n).
  //    Chi phí ô lưới lấy qua: costmap_->getCost(mx, my).
  //
  // 3. Truy vết đường đi (reconstruct path) và chuyển ngược ô lưới sang tọa độ thế giới:
  //    double wx, wy;
  //    costmap_->mapToWorld(mx, my, wx, wy);
  // ---------------------------------------------------------------------------

  // Dưới đây là ví dụ minh họa nội suy đường đi mẫu giữa start và goal:
  const int num_steps = 25;
  for (int i = 0; i <= num_steps; ++i) {
    double factor = static_cast<double>(i) / num_steps;
    geometry_msgs::msg::PoseStamped pose;
    pose.header = global_path.header;
    pose.pose.position.x = start.pose.position.x + factor * (goal.pose.position.x - start.pose.position.x);
    pose.pose.position.y = start.pose.position.y + factor * (goal.pose.position.y - start.pose.position.y);
    pose.pose.position.z = 0.0;
    pose.pose.orientation.w = 1.0;
    global_path.poses.push_back(pose);
  }

  RCLCPP_INFO(
    logger_,
    "[CustomAStarPlanner] Tìm kiếm hoàn tất! Tổng số điểm trên đường đi: %zu",
    global_path.poses.size());

  return global_path;
}

}  // namespace custom_planner_example

// Đăng ký lớp với pluginlib để Nav2 có thể tải động (dynamic loading)
PLUGINLIB_EXPORT_CLASS(custom_planner_example::CustomAStarPlanner, nav2_core::GlobalPlanner)
