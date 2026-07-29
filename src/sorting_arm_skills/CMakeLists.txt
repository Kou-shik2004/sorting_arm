cmake_minimum_required(VERSION 3.8)
project(sorting_arm_skills)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wconversion -Werror)
endif()

find_package(ament_cmake REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(rclcpp REQUIRED)

add_executable(commander src/commander.cpp)
ament_target_dependencies(commander rclcpp moveit_ros_planning_interface)

add_executable(moveit_check src/moveit_ready_check.cpp)
ament_target_dependencies(moveit_check moveit_ros_planning_interface rclcpp)

install( TARGETS 
  moveit_check
  commander
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
