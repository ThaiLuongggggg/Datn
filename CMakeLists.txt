cmake_minimum_required(VERSION 3.8)
project(nav2_astar_demo)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Tìm kiếm các gói phụ thuộc bắt buộc
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(nav2_core REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(nav2_util REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(pluginlib REQUIRED)
find_package(tf2_ros REQUIRED)

# Cấu hình include directories cho custom planner example và scso planner
include_directories(
  custom_planner_example/include
  scso_planner_benchmark/include
)

# Biên dịch plugin mẫu CustomAStarPlanner thành dynamic library (.so)
add_library(custom_planner_example SHARED
  custom_planner_example/src/custom_planner.cpp
)
target_include_directories(custom_planner_example PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/custom_planner_example/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(custom_planner_example
  rclcpp
  rclcpp_lifecycle
  nav2_core
  nav2_costmap_2d
  geometry_msgs
  nav_msgs
  pluginlib
  tf2_ros
)

# Biên dịch plugin ScsoPlanner (SCSO Benchmark Plugin)
add_library(scso_planner_benchmark SHARED
  scso_planner_benchmark/src/scso_planner.cpp
)
target_include_directories(scso_planner_benchmark PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/scso_planner_benchmark/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(scso_planner_benchmark
  rclcpp
  rclcpp_lifecycle
  nav2_core
  nav2_costmap_2d
  nav2_util
  geometry_msgs
  nav_msgs
  pluginlib
  tf2_ros
)

# Export file mô tả plugin để pluginlib nhận dạng khi nạp động
pluginlib_export_plugin_description_file(nav2_core custom_planner_example/custom_planner_plugins.xml)
pluginlib_export_plugin_description_file(nav2_core scso_planner_benchmark/scso_planner_plugins.xml)

# Cài đặt thư viện plugin
install(
  TARGETS 
    custom_planner_example
    scso_planner_benchmark
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# Cài đặt headers
install(
  DIRECTORY custom_planner_example/include/
  DESTINATION include
)
install(
  DIRECTORY scso_planner_benchmark/include/
  DESTINATION include
)

# Cài đặt các thư mục launch, config, maps, rviz vào share directory
install(
  DIRECTORY launch config maps rviz
  DESTINATION share/${PROJECT_NAME}
)

# Cài đặt các python script vào lib directory và share
install(
  PROGRAMS
    launch_demo.py
    scripts/goal_pose_bridge.py
    scripts/test_plan_client.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
