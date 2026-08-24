cmake_minimum_required(VERSION 3.8)
project(sorting_arm_bringup)

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
find_package(rclcpp REQUIRED)
find_package(ros_gz_interfaces REQUIRED)

# one-shot node that populates the world with the run's random cubes
add_executable(cube_spawner src/cube_spawner.cpp)
ament_target_dependencies(cube_spawner rclcpp ros_gz_interfaces)
install(TARGETS cube_spawner DESTINATION lib/${PROJECT_NAME})

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

install(DIRECTORY
  launch config rviz worlds models
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
