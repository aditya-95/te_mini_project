cmake_minimum_required(VERSION 3.8)
project(marker)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(interactive_markers REQUIRED)
find_package(visualization_msgs REQUIRED)

add_executable(simple_marker src/simple_marker.cpp)
ament_target_dependencies(simple_marker interactive_markers rclcpp visualization_msgs)

install(TARGETS
  simple_marker
  DESTINATION lib/${PROJECT_NAME})

ament_package()
