cmake_minimum_required(VERSION 3.8)
project(ik)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()


# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(interactive_markers REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(orocos_kdl REQUIRED)
find_package(urdf REQUIRED)
find_package(kdl_parser REQUIRED)

include_directories(${orocos_kdl_INCLUDE_DIR})

add_executable(ik_joint_state_publisher src/ik_joint_state_publisher.cpp)
target_link_libraries(ik_joint_state_publisher ${orocos_kdl_LIBRARIES})
ament_target_dependencies(ik_joint_state_publisher rclcpp sensor_msgs visualization_msgs 
                          geometry_msgs tf2 tf2_ros urdf kdl_parser)

add_executable(serial src/serial.cpp)
ament_target_dependencies(serial rclcpp sensor_msgs)

add_executable(simple_marker src/simple_marker.cpp)
ament_target_dependencies(simple_marker rclcpp interactive_markers visualization_msgs)

add_executable(basic_control src/basic_control.cpp)
ament_target_dependencies(basic_control interactive_markers rclcpp tf2 tf2_ros tf2_geometry_msgs visualization_msgs)

install(TARGETS
  ik_joint_state_publisher
  serial
  simple_marker
  basic_control
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch
    DESTINATION share/${PROJECT_NAME})

install(DIRECTORY urdf
    DESTINATION share/${PROJECT_NAME})

install(DIRECTORY rviz2
    DESTINATION share/${PROJECT_NAME})

ament_package()
