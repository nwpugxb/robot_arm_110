cmake_minimum_required(VERSION 3.8)
project(so101_hardware)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(rclcpp REQUIRED)
find_package(pluginlib REQUIRED)

# 包含头文件路径
include_directories(external/SCServo)

# 编译插件库 - 这里的源文件必须与你 ls 出来的完全一致
add_library(so101_hardware SHARED
  src/so101_interface.cpp
  external/SCServo/SCSerial.cpp
  external/SCServo/SCS.cpp
  external/SCServo/SMS_STS.cpp
  external/SCServo/SCSCL.cpp   # 虽然你主要用 STS，但包含它也没关系
)

target_include_directories(so101_hardware PRIVATE 
  include 
  external/SCServo
)

ament_target_dependencies(so101_hardware
  hardware_interface
  pluginlib
  rclcpp
)

# 核心：这里一定要对应你实际的文件名 so101_hardware.xml
pluginlib_export_plugin_description_file(hardware_interface so101_hardware.xml)

install(TARGETS so101_hardware
  DESTINATION lib
)

install(DIRECTORY include/
  DESTINATION include
)

ament_package()
