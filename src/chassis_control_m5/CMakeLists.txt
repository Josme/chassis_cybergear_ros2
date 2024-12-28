cmake_minimum_required(VERSION 3.8)
project(chassis_control_m5)

set(CMAKE_CXX_STANDARD 17)
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# add_definitions(-DCG_DEBUG)  
# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(can_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(nav_msgs REQUIRED)

add_library(cybergear_m5_driver
  # src/motor/cybergear_bridge_packet.cpp
  # src/motor/cybergear_bridge.cpp
  src/motor/cybergear_can_interface_ros2.cpp
  src/motor/cybergear_controller.cpp
  src/motor/cybergear_driver.cpp
)

ament_target_dependencies(cybergear_m5_driver
  rclcpp
  can_msgs
)
include_directories(cybergear_m5_driver PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include>"
  ${rclcpp_INCLUDE_DIRS}
  ./include/chassis_control_m5/motor
)


add_executable(chassis_control_m5_node
  src/main.cpp
  src/chassis_control.cpp
)

include_directories(chassis_control_m5_node PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include>"
  ${rclcpp_INCLUDE_DIRS}
  ./include
)
ament_target_dependencies(chassis_control_m5_node
  rclcpp
  geometry_msgs
  sensor_msgs
  nav_msgs
  tf2_ros
)
target_link_libraries(chassis_control_m5_node
  
  cybergear_m5_driver
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


install(TARGETS
chassis_control_m5_node
  cybergear_m5_driver
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY
  launch
  config
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
