cmake_minimum_required(VERSION 3.8)
project(sorting_arm_skills)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(
    -Wall
    -Wextra
    -Wpedantic
    -Wconversion
    -Wshadow
    -Wnon-virtual-dtor
    -Woverloaded-virtual
    -Werror
  )
endif()

find_package(ament_cmake REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(rclcpp REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(AMENT_LINT_AUTO_EXCLUDE
    ament_cmake_copyright
    ament_cmake_cpplint
  )
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_cpplint REQUIRED)
  ament_cpplint(
    FILTERS "-legal/copyright"
    MAX_LINE_LENGTH 120
  )
endif()

add_executable(commander src/commander.cpp)
ament_target_dependencies(commander rclcpp moveit_ros_planning_interface)

install( TARGETS
  commander
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
