cmake_minimum_required(VERSION 3.8)
project(potential_escape CXX)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(nav2_common REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav2_behavior_tree REQUIRED)
find_package(nav2_util REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(nav2_core REQUIRED)
find_package(pluginlib REQUIRED)
find_package(nav2_behaviors REQUIRED)

nav2_package()

include_directories(
  include
)

pluginlib_export_plugin_description_file(nav2_core behavior_plugin.xml)



# potential_escape
set(dependencies
  rclcpp
  rclcpp_action
  rclcpp_lifecycle
  rclcpp_components
  std_msgs
  nav2_util
  nav2_behavior_tree
  nav2_msgs
  nav_msgs
  tf2
  tf2_geometry_msgs
  geometry_msgs
  nav2_costmap_2d
  nav2_core
  pluginlib
)

add_library(nav2_escape_behavior SHARED
  src/potential_escape.cpp
)

ament_target_dependencies(nav2_escape_behavior
  ${dependencies}
)

rclcpp_components_register_nodes(nav2_escape_behavior "behavior_server::BehaviorServer")

install(TARGETS nav2_escape_behavior
                nav2_escape_behavior
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)


install(TARGETS ${executable_name}
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include/
)

install(FILES behavior_plugin.xml
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY 
  DESTINATION share/${PROJECT_NAME}/plugins/
)




ament_export_include_directories(include)
ament_export_libraries(${library_name}
  nav2_escape_behavior
)
ament_export_dependencies(${dependencies})
ament_package()
