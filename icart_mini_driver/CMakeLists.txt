cmake_minimum_required(VERSION 3.8)
project(icart_mini_driver)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 必要な依存を検索
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_auto REQUIRED)
find_package(controller_interface REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(diff_drive_controller REQUIRED)
find_package(joint_state_broadcaster REQUIRED)

find_library(YPSpur_LIBRARIES NAMES ypspur PATHS /usr/local/lib /usr/lib)
find_path(YPSpur_INCLUDE_DIRS NAMES ypspur.h PATHS /usr/local/include /usr/include)

if(NOT YPSpur_LIBRARIES OR NOT YPSpur_INCLUDE_DIRS)
  message(FATAL_ERROR "YPSpur library not found. Please install it.")
endif()

ament_auto_find_build_dependencies()

# プラグインとしてビルド
ament_auto_add_library(icart_mini_driver SHARED
  src/icart_mini_driver_node.cpp
)

# インクルードディレクトリ
include_directories(
  ${YPSpur_INCLUDE_DIRS}
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# リンクするライブラリ
target_link_libraries(icart_mini_driver
  ${YPSpur_LIBRARIES}
)

# pluginlib 用 XML ファイルをエクスポート
pluginlib_export_plugin_description_file(hardware_interface icart_mini_driver_plugins.xml)

# 依存関係のエクスポート
ament_export_dependencies(
  hardware_interface
  pluginlib
  rclcpp
  tf2
  tf2_geometry_msgs
  diff_drive_controller
  joint_state_broadcaster
)

# インストール設定
install(
  DIRECTORY config scripts
  DESTINATION share/${PROJECT_NAME}
)

ament_auto_package(
  INSTALL_TO_SHARE config scripts
)
