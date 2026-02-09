cmake_minimum_required(VERSION 3.8)
project(hello_moveit)

if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(geometry_msgs REQUIRED)

add_executable(hello_moveit_node src/hello_moveit.cpp)

# 使用 ament 依赖管理，自动处理包含路径和链接
ament_target_dependencies(hello_moveit_node
  rclcpp
  moveit_ros_planning_interface
  geometry_msgs
)

install(TARGETS hello_moveit_node
  DESTINATION lib/${PROJECT_NAME})
  
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})  

ament_package()
