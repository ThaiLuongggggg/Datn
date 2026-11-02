#ifndef CUSTOM_PLANNER_EXAMPLE__CUSTOM_PLANNER_HPP_
#define CUSTOM_PLANNER_EXAMPLE__CUSTOM_PLANNER_HPP_

#include <memory>
#include <string>
#include "rclcpp/rclcpp.hpp"
#include "nav2_core/global_planner.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include "nav2_costmap_2d/costmap_2d.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/path.hpp"

namespace custom_planner_example
{

/**
 * @class CustomAStarPlanner
 * @brief Lớp triển khai Custom Global Planner kế thừa từ nav2_core::GlobalPlanner
 */
class CustomAStarPlanner : public nav2_core::GlobalPlanner
{
public:
  CustomAStarPlanner() = default;
  virtual ~CustomAStarPlanner() = default;

  /**
   * @brief Cấu hình plugin khi planner_server chuyển sang trạng thái INACTIVE (on_configure)
   */
  void configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
    std::string name, std::shared_ptr<tf2_ros::Buffer> tf,
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) override;

  /**
   * @brief Dọn dẹp tài nguyên khi node tắt hoặc reset (on_cleanup)
   */
  void cleanup() override;

  /**
   * @brief Kích hoạt plugin khi node chuyển sang ACTIVE (on_activate)
   */
  void activate() override;

  /**
   * @brief Hủy kích hoạt plugin khi node chuyển về INACTIVE (on_deactivate)
   */
  void deactivate() override;

  /**
   * @brief Hàm cốt lõi: Tính toán chuỗi điểm đường đi (Path) từ điểm xuất phát tới đích
   * @param start Điểm xuất phát của robot (geometry_msgs::msg::PoseStamped)
   * @param goal Điểm đích cần tới (geometry_msgs::msg::PoseStamped)
   * @return nav_msgs::msg::Path Đường đi được lập
   */
  nav_msgs::msg::Path createPlan(
    const geometry_msgs::msg::PoseStamped & start,
    const geometry_msgs::msg::PoseStamped & goal) override;

private:
  std::shared_ptr<tf2_ros::Buffer> tf_;
  nav2_costmap_2d::Costmap2D * costmap_{nullptr};
  std::string name_;
  rclcpp::Logger logger_{rclcpp::get_logger("CustomAStarPlanner")};
  rclcpp::Clock::SharedPtr clock_;
};

}  // namespace custom_planner_example

#endif  // CUSTOM_PLANNER_EXAMPLE__CUSTOM_PLANNER_HPP_
