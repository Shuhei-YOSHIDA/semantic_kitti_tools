cmake_minimum_required(VERSION 3.8)
project(semantic_kitti_tools)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# To set -std=c++17 flag to explicitly to compile_commands.json
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(tf2_msgs REQUIRED)
find_package(PCL REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(rosbag2_cpp REQUIRED)

add_executable(ground_truth_map
    src/ground_truth_map.cpp
    src/semantic_kitti_tools/util.cpp
)
target_include_directories(ground_truth_map
  PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>/include
  $<BUILD_INTERFACE:${EIGEN3_INCLUDE_DIR}>
  $<BUILD_INTERFACE:${PCL_INCLUDE_DIR}>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(ground_truth_map
  rclcpp
  geometry_msgs
  sensor_msgs
  nav_msgs
  visualization_msgs
  pcl_conversions
  pcl_ros
  tf2_eigen
)
install(TARGETS
  ground_truth_map
  DESTINATION lib/${PROJECT_NAME})

add_executable(make_rosbag2
    src/make_rosbag2.cpp
    src/semantic_kitti_tools/util.cpp
)
target_include_directories(make_rosbag2
  PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>/include>
  $<BUILD_INTERFACE:${EIGEN3_INCLUDE_DIR}>
  $<BUILD_INTERFACE:${PCL_INCLUDE_DIR}>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
  include)
ament_target_dependencies(make_rosbag2
  rclcpp
  geometry_msgs
  sensor_msgs
  nav_msgs
  visualization_msgs
  tf2_msgs
  pcl_conversions
  pcl_ros
  tf2_eigen
  rosbag2_cpp
)
install(TARGETS
  make_rosbag2
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY include/ DESTINATION include/${PROJECT_NAME})



if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
