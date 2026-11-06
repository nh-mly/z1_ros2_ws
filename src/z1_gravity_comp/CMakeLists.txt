cmake_minimum_required(VERSION 3.8)
project(z1_gravity_comp)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(kdl_parser REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)

add_executable(gravity_comp src/gravity_comp_node.cpp)
ament_target_dependencies(gravity_comp
  kdl_parser
  rclcpp
  sensor_msgs
  std_msgs
  std_srvs)

install(TARGETS gravity_comp
  DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME})
# 验证/示教脚本(经 ros2 run z1_gravity_comp <name>.py 调用)
install(PROGRAMS
  scripts/gc_forward_test.py
  scripts/gc_hold.py
  DESTINATION lib/${PROJECT_NAME})

ament_package()
