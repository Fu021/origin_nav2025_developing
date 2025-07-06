cmake_minimum_required(VERSION 3.8)
project(rviz_tf_shift)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 查找依赖
find_package(ament_cmake REQUIRED)
find_package(rviz_common REQUIRED)
find_package(Qt5 COMPONENTS Widgets Quick Qml QuickWidgets REQUIRED)
find_package(ament_index_cpp REQUIRED)


# MOC 文件（用于 QObject 继承类）
qt5_wrap_cpp(MOC_FILES
  include/tf_shift_panel.h
)

# 源文件
set(SRC_FILES
  src/tf_shift_panel.cpp
)

# 头文件包含路径
include_directories(
  ${PROJECT_SOURCE_DIR}/include
)

# 构建共享库（rviz 插件要求）
add_library(${PROJECT_NAME} SHARED
  ${SRC_FILES}
  ${MOC_FILES}
)

# 链接 Qt 和 ROS 所需库
target_link_libraries(${PROJECT_NAME}
  Qt5::Widgets
  Qt5::Quick
  Qt5::Qml
  Qt5::QuickWidgets
)

ament_target_dependencies(${PROJECT_NAME}
  rviz_common
  ament_index_cpp
)

# 安装插件库
install(
  TARGETS ${PROJECT_NAME}
  DESTINATION lib/${PROJECT_NAME}
)

# 安装插件描述文件
pluginlib_export_plugin_description_file(rviz_common plugin_description.xml)
ament_package()
