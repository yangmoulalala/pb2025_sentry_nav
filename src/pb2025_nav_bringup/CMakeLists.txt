cmake_minimum_required(VERSION 3.8)
project(pb2025_nav_bringup)

find_package(ament_cmake_auto REQUIRED)

# 查找依赖包
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rm_interfaces REQUIRED)

if(BUILD_TESTING)
find_package(ament_lint_auto REQUIRED)
  list(APPEND AMENT_LINT_AUTO_EXCLUDE
    ament_cmake_uncrustify
    ament_cmake_cpplint
    ament_cmake_flake8
  )
  ament_lint_auto_find_test_dependencies()
endif()

# 添加include目录
include_directories(
  include
)

# 添加可执行文件
add_executable(target_navigation_node src/target_navigation_node.cpp)
ament_target_dependencies(target_navigation_node
  rclcpp
  rclcpp_action
  geometry_msgs
  nav_msgs
  nav2_msgs
  std_msgs
  rm_interfaces
)

# 安装头文件
install(DIRECTORY include/
  DESTINATION include/${PROJECT_NAME}
)

# 安装可执行文件
install(TARGETS
  target_navigation_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_auto_package(
  INSTALL_TO_SHARE
  launch
  map
  pcd
  config
  behavior_trees
  rviz
)