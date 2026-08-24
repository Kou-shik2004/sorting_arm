cmake_minimum_required(VERSION 3.8)
project(sorting_arm_executive)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(
    -Wall
    -Wextra
    -Wpedantic
    -Wshadow
    -Wnon-virtual-dtor
    -Woverloaded-virtual
    -Werror
  )
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(behaviortree_cpp REQUIRED)
find_package(behaviortree_ros2 REQUIRED)
find_package(controller_manager_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(sorting_arm_interfaces REQUIRED)

add_executable(executive_node
  src/bt_nodes.cpp
  src/executive_node.cpp
)
target_include_directories(executive_node PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)
ament_target_dependencies(executive_node PUBLIC
  ament_index_cpp
  behaviortree_ros2
  controller_manager_msgs
  geometry_msgs
  rclcpp
  rclcpp_action
  sorting_arm_interfaces
)
target_link_libraries(executive_node PUBLIC behaviortree_cpp::behaviortree_cpp)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(AMENT_LINT_AUTO_EXCLUDE
    ament_cmake_copyright
    ament_cmake_cpplint
    ament_cmake_uncrustify
  )
  # cppcheck 2.13 cant parse a function def in a namespace followed by an
  # unknown macro (TEST/TEST_F) - false positive, not our bug, skip test/
  set(AMENT_LINT_AUTO_FILE_EXCLUDE test/*)
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_cpplint REQUIRED)
  ament_cpplint(
    FILTERS "-legal/copyright" "-build/header_guard" "-build/include_order"
    MAX_LINE_LENGTH 125
  )

endif()

install(DIRECTORY behavior_trees config DESTINATION share/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)
install(TARGETS executive_node DESTINATION lib/${PROJECT_NAME})

ament_package()
