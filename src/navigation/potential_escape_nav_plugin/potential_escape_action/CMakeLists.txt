cmake_minimum_required(VERSION 3.5)
project(potential_escape_action CXX)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

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

include_directories(
  include
)

# potential_escape_action
ament_auto_add_library(nav2_escape_action_bt_node SHARED src/potential_escape_action.cpp)

# Link dependencies via ament
ament_target_dependencies(nav2_escape_action_bt_node
  rclcpp
  rclcpp_action
  rclcpp_lifecycle
  sensor_msgs
  std_msgs
  std_srvs
  tf2
  tf2_ros
  geometry_msgs
  nav_msgs
  nav2_msgs
  nav2_util
  behaviortree_cpp
)

# Add compile definition for plugin export
target_compile_definitions(nav2_escape_action_bt_node PRIVATE BT_PLUGIN_EXPORT)

install(
  TARGETS nav2_escape_action_bt_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(FILES nav2_tree_nodes.xml DESTINATION share/${PROJECT_NAME})

install(DIRECTORY include/
  DESTINATION include/
)

ament_export_include_directories(
  include
)

ament_package()
