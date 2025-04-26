# cmake_minimum_required(VERSION 3.8)
# project(small_gicp_localization)

# if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
#   add_compile_options(-Wall -Wextra -Wpedantic)
# endif()

# # find dependencies
# find_package(ament_cmake REQUIRED)
# find_package(ament_cmake_auto REQUIRED)
# ament_auto_find_build_dependencies()

# ament_auto_add_library(${PROJECT_NAME} SHARED
#   DIRECTORY src
# )

# target_link_libraries(${PROJECT_NAME} OpenMP::OpenMP_CXX)

# rclcpp_components_register_node(${PROJECT_NAME}
#   PLUGIN small_gicp_localization::SmallGicpLocalization
#   EXECUTABLE ${PROJECT_NAME}_node
# )
# # uncomment the following section in order to fill in
# # further dependencies manually.
# # find_package(<dependency> REQUIRED)

# if(BUILD_TESTING)
#   find_package(ament_lint_auto REQUIRED)
#   # the following line skips the linter which checks for copyrights
#   # comment the line when a copyright and license is added to all source files
#   set(ament_cmake_copyright_FOUND TRUE)
#   # the following line skips cpplint (only works in a git repo)
#   # comment the line when this package is in a git repo and when
#   # a copyright and license is added to all source files
#   set(ament_cmake_cpplint_FOUND TRUE)
#   ament_lint_auto_find_test_dependencies()
# endif()

# ament_auto_package(
#   INSTALL_TO_SHARE
#   launch
# )
cmake_minimum_required(VERSION 3.8)
project(small_gicp_localization)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

# Find OpenMP
find_package(OpenMP REQUIRED)

# Find Eigen3
find_package(Eigen3 REQUIRED)

# Find PCL
find_package(PCL REQUIRED COMPONENTS common io)

ament_auto_add_library(${PROJECT_NAME} SHARED
  DIRECTORY src
)

# Enable OpenMP compile options
target_compile_options(${PROJECT_NAME} PRIVATE $<$<COMPILE_LANGUAGE:CXX>:${OpenMP_CXX_FLAGS}>)

# Link OpenMP library
target_link_libraries(${PROJECT_NAME} OpenMP::OpenMP_CXX)

# Link Eigen3 (include directories only, as Eigen3 is header-only)
target_include_directories(${PROJECT_NAME} PRIVATE ${EIGEN3_INCLUDE_DIR})

# Link PCL libraries
target_link_libraries(${PROJECT_NAME} ${PCL_LIBRARIES})
target_include_directories(${PROJECT_NAME} PRIVATE ${PCL_INCLUDE_DIRS})

rclcpp_components_register_node(${PROJECT_NAME}
  PLUGIN small_gicp_localization::SmallGicpLocalization
  EXECUTABLE ${PROJECT_NAME}_node
)

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

ament_auto_package(
  INSTALL_TO_SHARE
  launch
)