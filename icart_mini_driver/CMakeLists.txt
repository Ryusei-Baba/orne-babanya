cmake_minimum_required(VERSION 3.8)
project(icart_mini_driver)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_auto REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

find_library(YPSpur_LIBRARIES NAMES ypspur PATHS /usr/local/lib /usr/lib)
find_path(YPSpur_INCLUDE_DIRS NAMES ypspur.h PATHS /usr/local/include /usr/include)

if(NOT YPSpur_LIBRARIES OR NOT YPSpur_INCLUDE_DIRS)
  message(FATAL_ERROR "YPSpur library not found. Please install it.")
endif()

ament_auto_find_build_dependencies()

ament_auto_add_library(icart_mini_driver SHARED
  src/icart_mini_driver_node.cpp
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_dependencies(
  ament_cmake_auto
  rclcpp
  tf2
  tf2_geometry_msgs
)

include_directories(
  ${YPSpur_INCLUDE_DIRS}
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# Link necessary libraries
target_link_libraries(icart_mini_driver
  rclcpp::rclcpp
  tf2::tf2
  tf2_geometry_msgs::tf2_geometry_msgs
  ${YPSpur_LIBRARIES}
)

install(
  DIRECTORY config scripts
  DESTINATION share/${PROJECT_NAME}
)

ament_auto_package(
  INSTALL_TO_SHARE config scripts
)