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
find_package(controller_manager_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(shape_msgs REQUIRED)
find_package(sorting_arm_interfaces REQUIRED)

add_library(sorting_arm_executive_core STATIC
  src/assignment_planner.cpp
  src/bt_nodes.cpp
  src/config.cpp
  src/executive_node.cpp
)
target_include_directories(sorting_arm_executive_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(sorting_arm_executive_core PUBLIC
  ament_index_cpp
  controller_manager_msgs
  geometry_msgs
  rclcpp
  rclcpp_action
  shape_msgs
  sorting_arm_interfaces
)
target_link_libraries(sorting_arm_executive_core PUBLIC behaviortree_cpp::behaviortree_cpp)

add_executable(executive_node src/executive_main.cpp)
target_link_libraries(executive_node PRIVATE sorting_arm_executive_core)

add_executable(fixed_object_provider src/fixed_object_provider_main.cpp)
target_link_libraries(fixed_object_provider PRIVATE sorting_arm_executive_core)

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

  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_assignment_planner test/test_assignment_planner.cpp)
  target_link_libraries(test_assignment_planner sorting_arm_executive_core)

  ament_add_gtest(test_tree_policy test/test_tree_policy.cpp)
  target_compile_definitions(test_tree_policy PRIVATE
    SORTING_TREE_PATH="${CMAKE_CURRENT_SOURCE_DIR}/behavior_trees/sorting_cycle.xml"
  )
  target_link_libraries(test_tree_policy sorting_arm_executive_core)

  ament_add_gtest(test_bt_transport test/test_bt_transport.cpp)
  target_link_libraries(test_bt_transport sorting_arm_executive_core)

  ament_add_gtest(test_executive_lifecycle test/test_executive_lifecycle.cpp)
  target_link_libraries(test_executive_lifecycle sorting_arm_executive_core)

  set_tests_properties(test_bt_transport test_executive_lifecycle PROPERTIES RUN_SERIAL TRUE)
endif()

install(DIRECTORY behavior_trees config DESTINATION share/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)
install(TARGETS executive_node fixed_object_provider DESTINATION lib/${PROJECT_NAME})

ament_package()
