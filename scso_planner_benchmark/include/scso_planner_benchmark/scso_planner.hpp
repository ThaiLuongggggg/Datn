#ifndef SCSO_PLANNER_BENCHMARK__SCSO_PLANNER_HPP_
#define SCSO_PLANNER_BENCHMARK__SCSO_PLANNER_HPP_

#include <memory>
#include <string>
#include <vector>
#include <random>

#include "rclcpp/rclcpp.hpp"
#include "nav2_core/global_planner.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include "nav2_costmap_2d/costmap_2d.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "nav_msgs/msg/path.hpp"

namespace scso_planner_benchmark
{

/**
 * @struct Point2D
 * @brief Tọa độ 2D thực (mét)
 */
struct Point2D
{
  double x{0.0};
  double y{0.0};

  Point2D() = default;
  Point2D(double px, double py) : x(px), y(py) {}
};

/**
 * @struct SandCat
 * @brief Đại diện cho một cá thể mèo cát trong không gian tìm kiếm d chiều
 */
struct SandCat
{
  std::vector<double> position;       // Vector vị trí hiện tại X_c: d = 2*(m-2)
  std::vector<double> best_position;  // Vị trí tốt nhất lịch sử của cá thể
  double fitness{std::numeric_limits<double>::infinity()};
  double best_fitness{std::numeric_limits<double>::infinity()};
};

/**
 * @class ScsoPlanner
 * @brief Plugin Global Planner cho ROS 2 Nav2 sử dụng thuật toán Sand Cat Swarm Optimization (SCSO) truyền thống
 *        kết hợp với Hàm chi phí Chuẩn hóa Không thứ nguyên (Dimensionless Cost Function) và Beta thích nghi.
 */
class ScsoPlanner : public nav2_core::GlobalPlanner
{
public:
  ScsoPlanner() = default;
  virtual ~ScsoPlanner() = default;

  void configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
    std::string name, std::shared_ptr<tf2_ros::Buffer> tf,
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) override;

  void cleanup() override;
  void activate() override;
  void deactivate() override;

  nav_msgs::msg::Path createPlan(
    const geometry_msgs::msg::PoseStamped & start,
    const geometry_msgs::msg::PoseStamped & goal) override;

private:
  // Giải mã chuỗi waypoint đầy đủ P = {P_1, P_2, ..., P_m} từ vector nghiệm X
  std::vector<Point2D> decodeTrajectory(
    const std::vector<double> & X,
    const Point2D & start_pt,
    const Point2D & goal_pt) const;

  // Tính toán hàm chi phí không thứ nguyên F = w1*J1 + w2*J2 + w3*J3
  double evaluateFitness(
    const std::vector<Point2D> & waypoints,
    double L_min,
    double beta,
    int K_total,
    double & out_J1,
    double & out_J2,
    double & out_J3,
    bool & out_is_feasible) const;

  // Tính hệ số phạt beta tự thích nghi
  double calculateAdaptiveBeta(
    double L_min,
    double D_map,
    int K_total) const;

  // Truy vấn khoảng cách từ điểm (wx, wy) tới chướng ngại vật gần nhất (mét)
  double getObstacleDistance(double wx, double wy, bool & is_out_of_bounds) const;

  // Chuyển chuỗi Waypoints thành nav_msgs::msg::Path nội suy thẳng đều cho Nav2 Controller
  nav_msgs::msg::Path generateDensePath(
    const std::vector<Point2D> & waypoints,
    const geometry_msgs::msg::PoseStamped & start,
    const geometry_msgs::msg::PoseStamped & goal) const;

  // Thành viên dữ liệu ROS 2
  std::shared_ptr<tf2_ros::Buffer> tf_;
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros_;
  nav2_costmap_2d::Costmap2D * costmap_{nullptr};
  std::string name_;
  std::string global_frame_;
  rclcpp::Logger logger_{rclcpp::get_logger("ScsoPlanner")};
  rclcpp::Clock::SharedPtr clock_;

  // Tham số thuật toán SCSO truyền thống
  int max_iterations_{150};           // t_max
  int population_size_{50};           // N
  int num_waypoints_{7};              // m (tổng số điểm nút kể cả Start và Goal)
  int samples_per_segment_{15};       // N_s (số điểm lấy mẫu nội suy trên mỗi đoạn thẳng)
  double max_sensitivity_range_{2.0}; // S_M (tần số thính giác khởi tạo, chuẩn SCSO là 2.0)
  int seed_{0};

  // Tham số hàm chi phí không thứ nguyên
  double w1_{1.0};                    // Trọng số độ dài J1
  double w2_{1.5};                    // Trọng số độ mượt J2
  double w3_{1.0};                    // Trọng số an toàn J3
  double kappa_{1.5};                 // Hệ số an toàn phạt vi phạm
  double robot_radius_{0.22};         // r_robot
  double safety_margin_{0.05};        // delta_margin
  double d_min_{0.27};                // d_min = r_robot + delta_margin
  double d_safe_{0.675};              // d_safe = 2.5 * d_min

  // Bộ sinh số ngẫu nhiên
  mutable std::mt19937 rng_;
};

}  // namespace scso_planner_benchmark

#endif  // SCSO_PLANNER_BENCHMARK__SCSO_PLANNER_HPP_
