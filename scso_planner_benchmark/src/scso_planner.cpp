#include "scso_planner_benchmark/scso_planner.hpp"

#include <cmath>
#include <string>
#include <memory>
#include <vector>
#include <random>
#include <algorithm>
#include <limits>

#include "nav2_util/node_utils.hpp"
#include "pluginlib/class_list_macros.hpp"

namespace scso_planner_benchmark
{

void ScsoPlanner::configure(
  const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
  std::string name,
  std::shared_ptr<tf2_ros::Buffer> tf,
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
{
  tf_ = tf;
  name_ = name;
  costmap_ros_ = costmap_ros;
  costmap_ = costmap_ros_->getCostmap();
  global_frame_ = costmap_ros_->getGlobalFrameID();

  auto node = parent.lock();
  if (!node) {
    throw std::runtime_error{"[ScsoPlanner] Không thể khóa node cha trong hàm configure"};
  }

  clock_ = node->get_clock();
  logger_ = node->get_logger();

  // Khai báo các tham số thuật toán SCSO truyền thống
  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".max_iterations", rclcpp::ParameterValue(150));
  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".population_size", rclcpp::ParameterValue(50));
  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".num_waypoints", rclcpp::ParameterValue(7));
  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".samples_per_segment", rclcpp::ParameterValue(15));
  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".max_sensitivity_range", rclcpp::ParameterValue(2.0));
  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".seed", rclcpp::ParameterValue(0));

  // Khai báo các tham số hàm chi phí chuẩn hóa không thứ nguyên
  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".w1", rclcpp::ParameterValue(1.0));
  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".w2", rclcpp::ParameterValue(1.5));
  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".w3", rclcpp::ParameterValue(1.0));
  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".kappa", rclcpp::ParameterValue(1.5));
  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".robot_radius", rclcpp::ParameterValue(0.22));
  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".safety_margin", rclcpp::ParameterValue(0.05));

  // Đọc các giá trị tham số
  node->get_parameter(name_ + ".max_iterations", max_iterations_);
  node->get_parameter(name_ + ".population_size", population_size_);
  node->get_parameter(name_ + ".num_waypoints", num_waypoints_);
  node->get_parameter(name_ + ".samples_per_segment", samples_per_segment_);
  node->get_parameter(name_ + ".max_sensitivity_range", max_sensitivity_range_);

  node->get_parameter(name_ + ".w1", w1_);
  node->get_parameter(name_ + ".w2", w2_);
  node->get_parameter(name_ + ".w3", w3_);
  node->get_parameter(name_ + ".kappa", kappa_);
  node->get_parameter(name_ + ".robot_radius", robot_radius_);
  node->get_parameter(name_ + ".safety_margin", safety_margin_);

  // Ràng buộc số waypoint tối thiểu
  if (num_waypoints_ < 3) {
    RCLCPP_WARN(logger_, "[ScsoPlanner] num_waypoints phải >= 3. Đang tự đặt lại thành 3.");
    num_waypoints_ = 3;
  }

  // Thiết lập các ngưỡng an toàn
  d_min_ = robot_radius_ + safety_margin_;
  d_safe_ = 2.5 * d_min_;

  // Khởi tạo bộ sinh số ngẫu nhiên
  node->get_parameter(name_ + ".seed", seed_);
  if (seed_ <= 0) {
    rng_.seed(std::random_device{}());
  } else {
    rng_.seed(static_cast<unsigned int>(seed_));
  }

  RCLCPP_INFO(
    logger_,
    "[ScsoPlanner] Cấu hình thành công: %s | Pop: %d | Iter: %d | Waypoints: %d | d_min: %.3fm | d_safe: %.3fm",
    name_.c_str(), population_size_, max_iterations_, num_waypoints_, d_min_, d_safe_);
}

void ScsoPlanner::cleanup()
{
  RCLCPP_INFO(logger_, "[ScsoPlanner] Giải phóng tài nguyên plugin: %s", name_.c_str());
  costmap_ = nullptr;
}

void ScsoPlanner::activate()
{
  RCLCPP_INFO(logger_, "[ScsoPlanner] Kích hoạt plugin: %s", name_.c_str());
}

void ScsoPlanner::deactivate()
{
  RCLCPP_INFO(logger_, "[ScsoPlanner] Hủy kích hoạt plugin: %s", name_.c_str());
}

std::vector<Point2D> ScsoPlanner::decodeTrajectory(
  const std::vector<double> & X,
  const Point2D & start_pt,
  const Point2D & goal_pt) const
{
  std::vector<Point2D> waypoints;
  waypoints.reserve(num_waypoints_);

  // P_1 = P_start
  waypoints.push_back(start_pt);

  // m-2 điểm nút trung gian
  int intermediate_count = num_waypoints_ - 2;
  for (int k = 0; k < intermediate_count; ++k) {
    waypoints.emplace_back(X[2 * k], X[2 * k + 1]);
  }

  // P_m = P_goal
  waypoints.push_back(goal_pt);

  return waypoints;
}

double ScsoPlanner::getObstacleDistance(double wx, double wy, bool & is_out_of_bounds) const
{
  is_out_of_bounds = false;
  unsigned int mx, my;
  if (!costmap_->worldToMap(wx, wy, mx, my)) {
    is_out_of_bounds = true;
    return 0.0;
  }

  unsigned char cost = costmap_->getCost(mx, my);

  if (cost == nav2_costmap_2d::NO_INFORMATION) {
    return d_safe_;
  }

  // Nếu ô nằm trong vùng cấm robot (Lethal hoặc Inscribed)
  if (cost >= nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE) {
    if (cost == nav2_costmap_2d::LETHAL_OBSTACLE) {
      return 0.0;
    }
    return 0.5 * d_min_;
  }

  // Nếu ô nằm hoàn toàn ở vùng tự do
  if (cost == nav2_costmap_2d::FREE_SPACE) {
    return d_safe_ + 1.0;
  }

  // Nếu ô nằm trong vùng suy giảm (Inflation zone: 1 đến 252)
  double factor = 1.0 - (static_cast<double>(cost) / 253.0);
  return d_min_ + factor * (d_safe_ - d_min_);
}

double ScsoPlanner::calculateAdaptiveBeta(
  double L_min,
  double D_map,
  int K_total) const
{
  double worst_length_ratio = ((num_waypoints_ - 1) * D_map) / std::max(L_min, 1e-6);
  double bracket = (w1_ / w3_) * worst_length_ratio + (w2_ / w3_) + 1.0;
  return kappa_ * static_cast<double>(K_total) * bracket;
}

double ScsoPlanner::evaluateFitness(
  const std::vector<Point2D> & waypoints,
  double L_min,
  double beta,
  int K_total,
  double & out_J1,
  double & out_J2,
  double & out_J3,
  bool & out_is_feasible) const
{
  out_is_feasible = true;
  int m = static_cast<int>(waypoints.size());

  // --- THÀNH PHẦN J1: CHI PHÍ ĐỘ DÀI CHUẨN HÓA ---
  std::vector<double> l(m - 1);
  std::vector<Point2D> vec_u(m - 1);
  double total_length = 0.0;

  for (int k = 0; k < m - 1; ++k) {
    vec_u[k].x = waypoints[k + 1].x - waypoints[k].x;
    vec_u[k].y = waypoints[k + 1].y - waypoints[k].y;
    l[k] = std::hypot(vec_u[k].x, vec_u[k].y);
    total_length += l[k];
  }
  out_J1 = total_length / std::max(L_min, 1e-9);

  // --- THÀNH PHẦN J2: CHI PHÍ ĐỘ MƯỢT (KHÔNG DÙNG ACOS) ---
  double sum_smooth = 0.0;
  for (int k = 0; k < m - 2; ++k) {
    double dot = vec_u[k].x * vec_u[k + 1].x + vec_u[k].y * vec_u[k + 1].y;
    double denom = l[k] * l[k + 1] + 1e-9;
    double cos_theta = std::clamp(dot / denom, -1.0, 1.0);
    sum_smooth += (1.0 - cos_theta) * 0.5;
  }
  out_J2 = sum_smooth / static_cast<double>(m - 2);

  // --- THÀNH PHẦN J3: AN TOÀN VÀ PHẠT VA CHẠM NỘI SUY TUYẾN TÍNH ---
  double sum_penalty = 0.0;
  for (int k = 0; k < m - 1; ++k) {
    for (int p = 0; p < samples_per_segment_; ++p) {
      double t = static_cast<double>(p) / samples_per_segment_;
      double sample_x = waypoints[k].x + t * vec_u[k].x;
      double sample_y = waypoints[k].y + t * vec_u[k].y;

      bool is_out = false;
      double d_j = getObstacleDistance(sample_x, sample_y, is_out);

      double c_j = 0.0;
      if (is_out || d_j <= d_min_) {
        out_is_feasible = false;
        c_j = beta * (1.0 + (d_min_ - d_j) / d_min_);
      } else if (d_j <= d_safe_) {
        double term1 = (d_safe_ - d_j) / (d_safe_ - d_min_);
        double term2 = d_min_ / d_j;
        double ratio = term1 * term2;
        c_j = ratio * ratio;
      }
      sum_penalty += c_j;
    }
  }

  // Kiểm tra điểm đích cuối cùng
  bool is_goal_out = false;
  double d_goal = getObstacleDistance(waypoints.back().x, waypoints.back().y, is_goal_out);
  if (is_goal_out || d_goal <= d_min_) {
    out_is_feasible = false;
    sum_penalty += beta * (1.0 + (d_min_ - d_goal) / d_min_);
  } else if (d_goal <= d_safe_) {
    double term1 = (d_safe_ - d_goal) / (d_safe_ - d_min_);
    double term2 = d_min_ / d_goal;
    double ratio = term1 * term2;
    sum_penalty += ratio * ratio;
  }

  out_J3 = sum_penalty / static_cast<double>(K_total);

  return w1_ * out_J1 + w2_ * out_J2 + w3_ * out_J3;
}

nav_msgs::msg::Path ScsoPlanner::generateDensePath(
  const std::vector<Point2D> & waypoints,
  const geometry_msgs::msg::PoseStamped & start,
  const geometry_msgs::msg::PoseStamped & goal) const
{
  nav_msgs::msg::Path path;
  path.header.stamp = clock_->now();
  path.header.frame_id = global_frame_;

  if (waypoints.size() < 2) {
    return path;
  }

  double step_size = costmap_->getResolution(); // Khoảng cách nội suy đều giữa các pose
  if (step_size <= 0.0) step_size = 0.05;

  for (size_t k = 0; k < waypoints.size() - 1; ++k) {
    double dx = waypoints[k + 1].x - waypoints[k].x;
    double dy = waypoints[k + 1].y - waypoints[k].y;
    double seg_len = std::hypot(dx, dy);
    int num_steps = std::max(1, static_cast<int>(std::ceil(seg_len / step_size)));

    double yaw = std::atan2(dy, dx);
    double qz = std::sin(yaw / 2.0);
    double qw = std::cos(yaw / 2.0);

    for (int s = 0; s < num_steps; ++s) {
      double t = static_cast<double>(s) / num_steps;
      geometry_msgs::msg::PoseStamped pose;
      pose.header = path.header;
      pose.pose.position.x = waypoints[k].x + t * dx;
      pose.pose.position.y = waypoints[k].y + t * dy;
      pose.pose.position.z = 0.0;
      pose.pose.orientation.z = qz;
      pose.pose.orientation.w = qw;
      path.poses.push_back(pose);
    }
  }

  // Điểm cuối cùng nhận orientation từ Pose goal gốc
  geometry_msgs::msg::PoseStamped final_pose = goal;
  final_pose.header = path.header;
  path.poses.push_back(final_pose);

  // Đảm bảo điểm bắt đầu giữ đúng orientation từ start
  if (!path.poses.empty()) {
    path.poses.front().pose.orientation = start.pose.orientation;
  }

  return path;
}

nav_msgs::msg::Path ScsoPlanner::createPlan(
  const geometry_msgs::msg::PoseStamped & start,
  const geometry_msgs::msg::PoseStamped & goal)
{
  nav_msgs::msg::Path global_path;
  global_path.header.stamp = clock_->now();
  global_path.header.frame_id = global_frame_;

  if (start.header.frame_id != global_frame_ || goal.header.frame_id != global_frame_) {
    RCLCPP_ERROR(
      logger_,
      "[ScsoPlanner] Frame của start (%s) hoặc goal (%s) không khớp với costmap frame (%s)",
      start.header.frame_id.c_str(), goal.header.frame_id.c_str(), global_frame_.c_str());
    return global_path;
  }

  Point2D start_pt(start.pose.position.x, start.pose.position.y);
  Point2D goal_pt(goal.pose.position.x, goal.pose.position.y);

  // Kiểm tra điểm xuất phát và đích trong Costmap
  unsigned int smx, smy, gmx, gmy;
  if (!costmap_->worldToMap(start_pt.x, start_pt.y, smx, smy) ||
      !costmap_->worldToMap(goal_pt.x, goal_pt.y, gmx, gmy)) {
    RCLCPP_ERROR(logger_, "[ScsoPlanner] Start hoặc Goal nằm ngoài vùng ô lưới của Costmap!");
    return global_path;
  }

  if (costmap_->getCost(smx, smy) >= nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE ||
      costmap_->getCost(gmx, gmy) >= nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE) {
    RCLCPP_ERROR(logger_, "[ScsoPlanner] Điểm Start hoặc Goal nằm trong vùng chướng ngại vật!");
    return global_path;
  }

  // 1. Tính toán các đặc trưng hình học
  double L_min = std::hypot(goal_pt.x - start_pt.x, goal_pt.y - start_pt.y);
  if (L_min < 1e-4) {
    RCLCPP_WARN(logger_, "[ScsoPlanner] Start và Goal trùng nhau!");
    global_path.poses.push_back(start);
    return global_path;
  }

  double map_w_m = costmap_->getSizeInMetersX();
  double map_h_m = costmap_->getSizeInMetersY();
  double D_map = std::hypot(map_w_m, map_h_m);

  int intermediate_count = num_waypoints_ - 2;
  int d = 2 * intermediate_count; // Số chiều tìm kiếm của bài toán
  int K_total = (num_waypoints_ - 1) * samples_per_segment_ + 1;

  // 2. Chốt hệ số phạt Beta thích nghi tự động
  double adaptive_beta = calculateAdaptiveBeta(L_min, D_map, K_total);

  // 3. Giới hạn không gian tìm kiếm trong phạm vi bản đồ
  double min_x = costmap_->getOriginX() + d_min_;
  double max_x = costmap_->getOriginX() + map_w_m - d_min_;
  double min_y = costmap_->getOriginY() + d_min_;
  double max_y = costmap_->getOriginY() + map_h_m - d_min_;

  std::uniform_real_distribution<double> rand_unit(0.0, 1.0);
  std::uniform_real_distribution<double> rand_angle(0.0, 2.0 * M_PI);

  // 4. Khởi tạo quần thể N mèo cát
  int N = population_size_;
  std::vector<SandCat> population(N);

  std::vector<double> global_best_position(d);
  double global_best_fitness = std::numeric_limits<double>::infinity();

  for (int i = 0; i < N; ++i) {
    population[i].position.resize(d);
    population[i].best_position.resize(d);

    for (int k = 0; k < intermediate_count; ++k) {
      double t = static_cast<double>(k + 1) / (num_waypoints_ - 1);
      double line_x = start_pt.x + t * (goal_pt.x - start_pt.x);
      double line_y = start_pt.y + t * (goal_pt.y - start_pt.y);

      if (i == 0) {
        // Cá thể 0 khởi tạo ngay trên đường thẳng nối Start-Goal
        population[i].position[2 * k] = std::clamp(line_x, min_x, max_x);
        population[i].position[2 * k + 1] = std::clamp(line_y, min_y, max_y);
      } else {
        // Phân tán ngẫu nhiên xung quanh đường cơ sở
        double perturb_range = 0.3 * L_min;
        double px = line_x + (2.0 * rand_unit(rng_) - 1.0) * perturb_range;
        double py = line_y + (2.0 * rand_unit(rng_) - 1.0) * perturb_range;
        population[i].position[2 * k] = std::clamp(px, min_x, max_x);
        population[i].position[2 * k + 1] = std::clamp(py, min_y, max_y);
      }
    }

    auto waypoints = decodeTrajectory(population[i].position, start_pt, goal_pt);
    double j1, j2, j3;
    bool feasible;
    double fit = evaluateFitness(waypoints, L_min, adaptive_beta, K_total, j1, j2, j3, feasible);

    population[i].fitness = fit;
    population[i].best_fitness = fit;
    population[i].best_position = population[i].position;

    if (fit < global_best_fitness) {
      global_best_fitness = fit;
      global_best_position = population[i].position;
    }
  }

  // 5. Vòng lặp tiến hóa SCSO Truyền Thống
  double S_M = max_sensitivity_range_; // Mặc định 2.0 theo phương trình (3)

  for (int iter = 0; iter < max_iterations_; ++iter) {
    // Phương trình (3): r_G giảm tuyến tính từ S_M (2.0) về 0
    double r_G = S_M - (S_M * static_cast<double>(iter)) / static_cast<double>(max_iterations_);

    for (int i = 0; i < N; ++i) {
      // Phương trình (2): r = r_G * rand
      double r = r_G * rand_unit(rng_);

      // Phương trình (6): R = 2 * r_G * rand - r_G
      double R = 2.0 * r_G * rand_unit(rng_) - r_G;

      std::vector<double> new_pos(d);

      for (int j = 0; j < d; ++j) {
        double X_c = population[i].position[j];
        double X_b = global_best_position[j];
        double rand_val = rand_unit(rng_);

        if (std::abs(R) > 1.0) {
          // --- THĂM DÒ TOÀN CỤC (GLOBAL EXPLORATION) - Phương trình (1) & (7) ---
          // X(t+1) = r * (X_b - rand * X_c)
          new_pos[j] = r * (X_b - rand_val * X_c);
        } else {
          // --- KHAI THÁC CỤC BỘ / TẤN CÔNG (LOCAL EXPLOITATION) - Phương trình (4), (5) & (7) ---
          // X_md = |rand * X_b - X_c|
          // X(t+1) = X_b - r * X_md * cos(theta)
          double X_md = std::abs(rand_val * X_b - X_c);
          double theta = rand_angle(rng_);
          new_pos[j] = X_b - r * X_md * std::cos(theta);
        }

        // Giới hạn biên bản đồ
        if (j % 2 == 0) {
          new_pos[j] = std::clamp(new_pos[j], min_x, max_x);
        } else {
          new_pos[j] = std::clamp(new_pos[j], min_y, max_y);
        }
      }

      // Đánh giá fitness cho vị trí mới
      auto new_waypoints = decodeTrajectory(new_pos, start_pt, goal_pt);
      double j1, j2, j3;
      bool feasible;
      double new_fit = evaluateFitness(new_waypoints, L_min, adaptive_beta, K_total, j1, j2, j3, feasible);

      population[i].position = new_pos;
      population[i].fitness = new_fit;

      // Cập nhật kỷ lục cá nhân
      if (new_fit < population[i].best_fitness) {
        population[i].best_fitness = new_fit;
        population[i].best_position = new_pos;
      }

      // Cập nhật con mồi tối ưu toàn đàn
      if (new_fit < global_best_fitness) {
        global_best_fitness = new_fit;
        global_best_position = new_pos;
      }
    }
  }

  // 6. Trích xuất đường đi tối ưu toàn đàn và đánh giá chi tiết
  auto best_waypoints = decodeTrajectory(global_best_position, start_pt, goal_pt);
  double final_J1, final_J2, final_J3;
  bool final_feasible;
  double final_fitness = evaluateFitness(
    best_waypoints, L_min, adaptive_beta, K_total, final_J1, final_J2, final_J3, final_feasible);

  RCLCPP_INFO(
    logger_,
    "[ScsoPlanner] Hoàn thành tối ưu: Best Fitness: %.4f | J1: %.4f | J2: %.4f | J3: %.4f | Feasible: %s | Beta: %.1f",
    final_fitness, final_J1, final_J2, final_J3, final_feasible ? "TRUE" : "FALSE (Va chạm)", adaptive_beta);

  // 7. Tạo đường đi nav_msgs::msg::Path nội suy thẳng đều (không B-Spline)
  global_path = generateDensePath(best_waypoints, start, goal);

  return global_path;
}

}  // namespace scso_planner_benchmark

PLUGINLIB_EXPORT_CLASS(scso_planner_benchmark::ScsoPlanner, nav2_core::GlobalPlanner)
