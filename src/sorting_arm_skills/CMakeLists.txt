cmake_minimum_required(VERSION 3.8)
project(sorting_arm_skills)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# -Wpedantic/-Wnon-virtual-dtor/-Woverloaded-virtual dropped: this package has no
# virtual dispatch left and pedantic never caught a real bug here either.
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(
    -Wall
    -Wextra
    -Wshadow
    -Werror
  )
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(moveit_core REQUIRED)
find_package(shape_msgs REQUIRED)
find_package(control_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(sorting_arm_interfaces REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)

add_library(sorting_arm_skills
  src/helpers.cpp
  src/motion_commander.cpp
  src/scene_manager.cpp
  src/gripper_commander.cpp
  src/skill_state.cpp
  src/pick_server.cpp
  src/place_server.cpp
  src/home_server.cpp
)
target_include_directories(sorting_arm_skills PUBLIC
  ${CMAKE_CURRENT_SOURCE_DIR}/include
)

# rclcpp/rclcpp_action/the message packages appear in this library's public
# headers, so skill_server_node and motion_demo need them too.
# moveit_ros_planning_interface is PUBLIC too — MoveGroupInterface/
# PlanningSceneInterface are included straight in motion_commander.hpp and
# scene_manager.hpp now, no forward-declaration pimpl hiding them anymore.
ament_target_dependencies(sorting_arm_skills PUBLIC
  rclcpp
  rclcpp_action
  geometry_msgs
  moveit_msgs
  shape_msgs
  control_msgs
  sensor_msgs
  sorting_arm_interfaces
)
target_link_libraries(sorting_arm_skills PUBLIC
  moveit_core::moveit_collision_detection
  moveit_ros_planning_interface::moveit_move_group_interface
  moveit_ros_planning_interface::moveit_planning_scene_interface
  moveit_ros_planning_interface::moveit_common_planning_interface_objects
)

add_executable(skill_server_node src/skill_server_main.cpp)
target_link_libraries(skill_server_node PRIVATE sorting_arm_skills)

add_executable(motion_demo src/motion_demo_main.cpp)
target_link_libraries(motion_demo PRIVATE sorting_arm_skills)

# Pre-existing hand-run MoveIt scratch executable — unrelated to this slice,
# left exactly as it was.

# Hand-run verification harness (Step 11's own actions, no orchestration
# layer) — links sorting_arm_skills only for its already-public rclcpp_action/
# sorting_arm_interfaces deps, not for any of its MoveIt/skills logic.
add_executable(sequence_demo src/sequence_demo_main.cpp)
target_link_libraries(sequence_demo PRIVATE sorting_arm_skills)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(AMENT_LINT_AUTO_EXCLUDE
    ament_cmake_copyright
    ament_cmake_cpplint
    ament_cmake_uncrustify
  )
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_cpplint REQUIRED)
  ament_cpplint(
    FILTERS "-legal/copyright" "-build/header_guard"
    MAX_LINE_LENGTH 120
  )
endif()

install(
  DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)
install(
  TARGETS sorting_arm_skills
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
)
install(
  TARGETS skill_server_node motion_demo sequence_demo
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
