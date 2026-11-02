# Minimal ROS 2 Nav2 A* Global Path Planning Demo

Dự án ROS 2 (Humble) tối giản nhằm mục đích minh họa và thử nghiệm thuật toán quy hoạch đường đi toàn cục **A\*** (Global Path Planning) trực quan trên **RViz2**.

---

## 1. Điểm nổi bật & Kiến trúc tối giản

Hệ thống được thiết kế theo cấu hình tinh gọn nhất có thể, **không sử dụng mô phỏng robot phức tạp (Gazebo)** và **không chạy local controller / BT navigator**. Chỉ tập trung vào quy hoạch đường đi toàn cục với 4 node chính:

1. **`map_server` (`nav2_map_server`)**: Đọc bản đồ dạng lưới ô từ file `.yaml` và `.pgm` rồi xuất bản lên topic `/map`.
2. **`planner_server` (`nav2_planner`)**: Chạy bản đồ chi phí toàn cục (`global_costmap`) và thực thi giải thuật tìm đường **A\*** (mặc định cấu hình `nav2_navfn_planner/NavfnPlanner` với tham số `use_astar: true`).
3. **`lifecycle_manager` (`nav2_lifecycle_manager`)**: Tự động quản lý và kích hoạt trạng thái vòng đời (`unconfigured` -> `inactive` -> `active`) cho `map_server` và `planner_server`.
4. **`rviz2`**: Giao diện trực quan hóa 2D/3D hiển thị bản đồ tĩnh, lớp chi phí costmap thổi phồng vật cản, và đường đi toàn cục tối ưu (`/plan`) vẽ bằng màu xanh lá cây.
5. *(Phụ trợ)* **`static_transform_publisher` (`tf2_ros`)**: Cung cấp transform tĩnh giữa frame `map` và `base_link` để `global_costmap` khởi tạo hoàn chỉnh mà không cần robot thực hoặc simulator.
6. *(Phụ trợ tương tác)* **`goal_pose_bridge`**: Lắng nghe sự kiện người dùng click **"2D Goal Pose"** và **"2D Pose Estimate"** trên RViz2 để gọi Action `/compute_path_to_pose`, vẽ đường đi ngay tức thì.

---

## 2. Cấu trúc thư mục dự án

```text
Test/
├── CMakeLists.txt                  # Cấu hình biên dịch ROS 2 ament_cmake & export pluginlib
├── package.xml                     # Khai báo metadata và các package phụ thuộc
├── run_demo.sh                     # Script bash tiện ích khởi chạy nhanh demo
├── launch_demo.py                  # Script Python khởi chạy trực tiếp LaunchService
├── README.md                       # Hướng dẫn chi tiết dự án (bằng Tiếng Việt)
├── CONFIG_POINTS.md                # Hướng dẫn chi tiết cấu hình điểm Start & Goal cố định
├── ADJUST_MARKER.md                # Hướng dẫn điều chỉnh kích thước & màu sắc hình tròn đánh dấu
├── launch/
│   └── astar_demo.launch.py        # File launch chính (hỗ trợ tham số map & planner_plugin)
├── config/
│   └── nav2_params.yaml            # Cấu hình chi tiết tham số Nav2, costmap, A* planner
├── maps/
│   ├── sample_map.yaml             # Metadata bản đồ (resolution, origin, thresholds)
│   └── sample_map.pgm              # Ảnh bản đồ nhị phân (mê cung phòng & vật cản 15x15m)
├── rviz/
│   └── astar_demo.rviz             # Cấu hình hiển thị RViz2 tối ưu (Grid, Map, Costmap, Path)
├── scripts/
│   ├── goal_pose_bridge.py         # Node cầu nối click RViz2 -> Action ComputePathToPose
│   └── test_plan_client.py         # Script kiểm thử CLI gửi tọa độ start & goal
└── custom_planner_example/         # Mã nguồn mẫu cho Custom Global Planner plugin
    ├── include/custom_planner_example/
    │   └── custom_planner.hpp      # Header kế thừa nav2_core::GlobalPlanner
    ├── src/
    │   └── custom_planner.cpp      # Implementation C++ & đăng ký PLUGINLIB_EXPORT_CLASS
    └── custom_planner_plugins.xml  # File XML khai báo plugin với Nav2
```

---

## 3. Cài đặt các gói phụ thuộc (Prerequisites)

Dự án chạy trên nền tảng **ROS 2 Humble** (Ubuntu 22.04 LTS). Trước khi bắt đầu, hãy đảm bảo hệ thống đã cài đặt đầy đủ các gói:

```bash
sudo apt update
sudo apt install -y \
  ros-humble-navigation2 \
  ros-humble-nav2-bringup \
  ros-humble-ros2cli \
  ros-humble-ros2launch \
  ros-humble-ros2run \
  ros-humble-ros2topic \
  ros-humble-ros2action \
  ros-humble-rviz2 \
  ros-humble-nav2-rviz-plugins
```

---

## 4. Biên dịch và Khởi chạy

### Cách 1: Biên dịch bằng `colcon` (Khuyên dùng chuẩn ROS 2)

```bash
# 1. Di chuyển vào thư mục dự án
cd ~/Test

# 2. Source môi trường ROS 2 Humble
source /opt/ros/humble/setup.bash

# 3. Biên dịch dự án
colcon build --symlink-install

# 4. Source gói vừa biên dịch
source install/setup.bash

# 5. Khởi chạy demo
ros2 launch nav2_astar_demo astar_demo.launch.py
```

### Cách 2: Khởi chạy nhanh bằng script `run_demo.sh`

```bash
cd ~/Test
./run_demo.sh
```

---

## 5. Hướng dẫn thử nghiệm trên RViz2 & Điểm Cố Định

### Chế độ Điểm Cố Định & Tự Động Lập Đường Đi (Mới cập nhật):
Mặc định khi chạy, hệ thống đã cài đặt sẵn **cặp tọa độ cố định**:
- **Start**: `(-3.0, -3.0)` (Marker màu xanh lá cây trên RViz2).
- **Goal**: `(3.0, 3.0)` (Marker màu đỏ trên RViz2).
- Thuật toán **A\*** sẽ **tự động tính toán đường đi ngay lập tức** khi khởi động mà bạn không cần click chuột.

> [!TIP]
> Xem hướng dẫn chi tiết cách tự thay đổi tọa độ điểm cố định trong file [`CONFIG_POINTS.md`](CONFIG_POINTS.md).

### Thử nghiệm tương tác thủ công trên RViz2:
Nếu muốn thử nghiệm điểm tùy ý khác:
1. **Đổi điểm xuất phát (Start)**: Chọn **"2D Pose Estimate"** (phím `P`) rồi click vào bản đồ.
2. **Đổi điểm đích đến (Goal)**: Chọn **"2D Goal Pose"** (phím `G`) rồi click vào bản đồ.
3. Marker và đường đi A* màu xanh lá (`/plan`) sẽ tự động cập nhật ngay lập tức.

### Thử nghiệm tìm đường độc lập qua dòng lệnh (CLI):

Bạn có thể chạy script client mà không cần click RViz:

```bash
source install/setup.bash
python3 scripts/test_plan_client.py --start "-3.0 -3.0" --goal "3.0 3.0"
```

---

## 6. Hướng dẫn thay đổi bản đồ (Swap the Map)

Dự án cung cấp 2 cách linh hoạt để thay thế bản đồ theo nhu cầu:

### Cách A: Cấu hình qua Launch Argument (Nhanh chóng, không cần sửa code)

Chỉ cần truyền đường dẫn file `.yaml` của bản đồ mới qua tham số `map:=...`:

```bash
ros2 launch nav2_astar_demo astar_demo.launch.py map:=/duong_dan_tuyet_doi/toi/ban_do_moi.yaml
```

*Ví dụ:*
```bash
ros2 launch nav2_astar_demo astar_demo.launch.py map:=/home/luong/Test/maps/sample_map.yaml
```

### Cách B: Cấu hình cố định trong file `config/nav2_params.yaml`

Mở file `config/nav2_params.yaml`, tìm đến khối `map_server`:

```yaml
map_server:
  ros__parameters:
    use_sim_time: false
    yaml_filename: "/duong_dan_tuyet_doi/toi/ban_do_cua_ban.yaml"
```

### Lưu ý về định dạng bản đồ trong ROS 2:
Một bản đồ chuẩn bao gồm:
1. File ảnh `.pgm` hoặc `.png`: Ảnh xám thể hiện không gian tự do (trắng, giá trị 254), vật cản (đen, giá trị 0) và vùng chưa biết (xám, giá trị 205).
2. File thông số `.yaml`:
   ```yaml
   image: ban_do.pgm
   resolution: 0.05               # Kích thước thực tế của 1 pixel (mét)
   origin: [-10.0, -10.0, 0.0]    # Tọa độ thế giới thực của góc dưới bên trái ảnh
   negate: 0
   occupied_thresh: 0.65          # Ngưỡng xem là vật cản
   free_thresh: 0.25              # Ngưỡng xem là vùng trống
   ```

---

## 7. Hướng dẫn viết và nạp Custom `nav2_core::GlobalPlanner` Plugin

Nav2 sử dụng thư viện **`pluginlib`** của ROS 2 để tải động các thuật toán lập kế hoạch toàn cục lúc runtime mà không cần biên dịch lại bộ mã nguồn gốc của Nav2.

Dưới đây là quy trình từng bước để phát triển và tích hợp một plugin thuật toán của riêng bạn (ví dụ: A* cải tiến, Dijkstra, Theta*, Jump Point Search):

### Bước 1: Kế thừa lớp cơ sở `nav2_core::GlobalPlanner`

Tạo file header C++ (ví dụ: `include/my_planner/my_planner.hpp`):

```cpp
#pragma once
#include <memory>
#include <string>
#include "rclcpp/rclcpp.hpp"
#include "nav2_core/global_planner.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/path.hpp"

namespace my_planner
{

class MyAStarPlanner : public nav2_core::GlobalPlanner
{
public:
  MyAStarPlanner() = default;
  virtual ~MyAStarPlanner() = default;

  // Khởi tạo tham số và nhận con trỏ costmap từ planner_server
  void configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
    std::string name, std::shared_ptr<tf2_ros::Buffer> tf,
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) override;

  // Giải phóng tài nguyên
  void cleanup() override;

  // Kích hoạt khi node chuyển sang ACTIVE
  void activate() override;

  // Hủy kích hoạt khi node chuyển sang INACTIVE
  void deactivate() override;

  // Phương thức quan trọng nhất: Tính toán đường đi từ Start đến Goal
  nav_msgs::msg::Path createPlan(
    const geometry_msgs::msg::PoseStamped & start,
    const geometry_msgs::msg::PoseStamped & goal) override;

private:
  nav2_costmap_2d::Costmap2D * costmap_{nullptr};
  rclcpp::Clock::SharedPtr clock_;
  rclcpp::Logger logger_{rclcpp::get_logger("MyAStarPlanner")};
};

}  // namespace my_planner
```

### Bước 2: Triển khai mã nguồn C++ & Đăng ký Pluginlib

Trong file `src/my_planner.cpp`:

```cpp
#include "my_planner/my_planner.hpp"
#include "pluginlib/class_list_macros.hpp"

namespace my_planner
{

void MyAStarPlanner::configure(
  const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
  std::string name, std::shared_ptr<tf2_ros::Buffer> tf,
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
{
  auto node = parent.lock();
  costmap_ = costmap_ros->getCostmap();
  clock_ = node->get_clock();
  RCLCPP_INFO(logger_, "Cấu hình MyAStarPlanner thành công!");
}

void MyAStarPlanner::cleanup() { costmap_ = nullptr; }
void MyAStarPlanner::activate() { RCLCPP_INFO(logger_, "Kích hoạt MyAStarPlanner!"); }
void MyAStarPlanner::deactivate() { RCLCPP_INFO(logger_, "Hủy kích hoạt MyAStarPlanner!"); }

nav_msgs::msg::Path MyAStarPlanner::createPlan(
  const geometry_msgs::msg::PoseStamped & start,
  const geometry_msgs::msg::PoseStamped & goal)
{
  nav_msgs::msg::Path path;
  path.header.stamp = clock_->now();
  path.header.frame_id = goal.header.frame_id;

  // 1. Chuyển đổi tọa độ thế giới sang tọa độ ô lưới của Costmap:
  unsigned int mx_start, my_start, mx_goal, my_goal;
  if (!costmap_->worldToMap(start.pose.position.x, start.pose.position.y, mx_start, my_start) ||
      !costmap_->worldToMap(goal.pose.position.x, goal.pose.position.y, mx_goal, my_goal)) {
    RCLCPP_ERROR(logger_, "Điểm xuất phát hoặc điểm đích nằm ngoài phạm vi bản đồ!");
    return path;
  }

  // 2. Chạy thuật toán tìm kiếm đường đi (A*, Dijkstra, v.v.):
  // Lấy chi phí ô lưới: costmap_->getCost(mx, my);

  // 3. Đưa các điểm tọa độ tìm được vào path.poses:
  // geometry_msgs::msg::PoseStamped p;
  // path.poses.push_back(p);

  return path;
}

}  // namespace my_planner

// ĐĂNG KÝ VỚI PLUGINLIB:
PLUGINLIB_EXPORT_CLASS(my_planner::MyAStarPlanner, nav2_core::GlobalPlanner)
```

### Bước 3: Tạo file mô tả plugin XML (`my_planner_plugins.xml`)

```xml
<library path="my_planner">
  <class type="my_planner::MyAStarPlanner" base_class_type="nav2_core::GlobalPlanner">
    <description>Plugin thuật toán tìm kiếm đường đi toàn cục tùy biến cho Nav2.</description>
  </class>
</library>
```

### Bước 4: Export plugin trong `CMakeLists.txt`

Thêm dòng sau vào file `CMakeLists.txt` của bạn:

```cmake
# Khai báo thư viện chia sẻ (.so)
add_library(my_planner SHARED src/my_planner.cpp)
ament_target_dependencies(my_planner rclcpp rclcpp_lifecycle nav2_core nav2_costmap_2d geometry_msgs nav_msgs pluginlib)

# Xuất bản file XML mô tả plugin cho nav2_core
pluginlib_export_plugin_description_file(nav2_core my_planner_plugins.xml)

# Cài đặt thư viện
install(TARGETS my_planner ARCHIVE DESTINATION lib LIBRARY DESTINATION lib RUNTIME DESTINATION bin)
```

### Bước 5: Nạp plugin vào hệ thống Nav2

Sau khi build với `colcon build`:

- **Cách 1: Nạp qua tham số dòng lệnh Launch**:
  ```bash
  ros2 launch nav2_astar_demo astar_demo.launch.py planner_plugin:=my_planner::MyAStarPlanner
  ```

- **Cách 2: Nạp qua file `config/nav2_params.yaml`**:
  ```yaml
  planner_server:
    ros__parameters:
      expected_planner_frequency: 10.0
      use_sim_time: false
      planner_plugins: ["GridBased"]
      GridBased:
        plugin: "my_planner::MyAStarPlanner"
  ```

> [!TIP]
> Trong dự án này, một plugin mẫu hoàn chỉnh đã được xây dựng sẵn tại thư mục `custom_planner_example/` và được cấu hình sẵn trong `CMakeLists.txt`. Bạn có thể thử nghiệm nạp ngay plugin mẫu này bằng lệnh:
> ```bash
> ros2 launch nav2_astar_demo astar_demo.launch.py planner_plugin:=custom_planner_example::CustomAStarPlanner
> ```

---

## 8. Chuyển đổi giữa các thuật toán có sẵn trong Nav2

Ngoài plugin tùy biến, Nav2 Humble cung cấp sẵn 2 thuật toán A* mạnh mẽ mà bạn có thể cấu hình nhanh:

1. **`nav2_navfn_planner/NavfnPlanner` (A\*)**:
   - Mặc định của dự án (`use_astar: true`).
   - Dùng heuristic khoảng cách để tăng tốc tìm kiếm đường đi trên lưới 2D.
2. **`nav2_smac_planner/SmacPlanner2D` (A\* 2D chuyên sâu)**:
   - Thuật toán A* 2D tối ưu hóa tốc độ cao của Nav2.
   - Chạy với launch argument:
     ```bash
     ros2 launch nav2_astar_demo astar_demo.launch.py planner_plugin:=nav2_smac_planner/SmacPlanner2D
     ```
