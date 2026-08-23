cmake_minimum_required(VERSION 3.8)
project(sorting_arm_skills)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# no virtual dispatch here, so -Wpedantic/-Wnon-virtual-dtor/-Woverloaded-virtual are dropped
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
find_package(control_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(moveit_core REQUIRED)
find_package(shape_msgs REQUIRED)
find_package(sorting_arm_interfaces REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(moveit_task_constructor_core REQUIRED)

add_library(sorting_arm_skills STATIC
  src/types.cpp
  src/scene_manager.cpp
  src/skill_state.cpp
  src/gripper_commander.cpp
  src/mtc_pick_place.cpp
  src/sort_server.cpp
  src/home_server.cpp
)
target_include_directories(sorting_arm_skills PUBLIC
  ${CMAKE_CURRENT_SOURCE_DIR}/include
)

# PUBLIC: these appear in the library's public headers, so consumers get them too
ament_target_dependencies(sorting_arm_skills PUBLIC
  rclcpp
  rclcpp_action
  control_msgs
  geometry_msgs
  moveit_msgs
  sorting_arm_interfaces
)
# SYSTEM (-isystem) shields -Werror from MTC's own headers; MTC types stay in mtc_pick_place.cpp
ament_target_dependencies(sorting_arm_skills SYSTEM PUBLIC moveit_task_constructor_core)
target_link_libraries(sorting_arm_skills PRIVATE shape_msgs::shape_msgs)
target_link_libraries(sorting_arm_skills PUBLIC
  moveit_core::moveit_collision_detection
  moveit_ros_planning_interface::moveit_move_group_interface
  moveit_ros_planning_interface::moveit_planning_scene_interface
  moveit_ros_planning_interface::moveit_common_planning_interface_objects
)

add_executable(skill_server_node src/skill_server_main.cpp)
target_link_libraries(skill_server_node PRIVATE sorting_arm_skills)

# hand-run harness; links its ROS deps directly, uses no skills code
add_executable(sequence_demo src/sequence_demo_main.cpp)
ament_target_dependencies(sequence_demo
  rclcpp
  rclcpp_action
  geometry_msgs
  shape_msgs
  sorting_arm_interfaces
)

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
    FILTERS "-legal/copyright" "-build/header_guard" "-build/include_order"
    MAX_LINE_LENGTH 125
  )
endif()

install(
  DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)
install(
  TARGETS skill_server_node sequence_demo
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
