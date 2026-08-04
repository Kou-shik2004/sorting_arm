cmake_minimum_required(VERSION 3.8)
project(sorting_arm_perception)

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
find_package(builtin_interfaces REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(image_geometry REQUIRED)
find_package(message_filters REQUIRED)
find_package(OpenCV REQUIRED COMPONENTS core highgui imgproc)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(shape_msgs REQUIRED)
find_package(sorting_arm_interfaces REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(tf2_ros REQUIRED)

add_executable(camera_object_provider
  src/perception_main.cpp
  src/perception_node.cpp
  src/rgbd_detector.cpp
)
target_include_directories(camera_object_provider PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(camera_object_provider
  builtin_interfaces
  cv_bridge
  geometry_msgs
  image_geometry
  message_filters
  rclcpp
  sensor_msgs
  shape_msgs
  sorting_arm_interfaces
  std_msgs
  tf2
  tf2_eigen
  tf2_ros
)
target_link_libraries(camera_object_provider Eigen3::Eigen ${OpenCV_LIBS})

add_executable(perception_viewer src/perception_viewer.cpp)
ament_target_dependencies(perception_viewer
  cv_bridge
  rclcpp
  sensor_msgs
)
target_link_libraries(perception_viewer ${OpenCV_LIBS})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(AMENT_LINT_AUTO_EXCLUDE
    ament_cmake_copyright
    ament_cmake_cpplint
    ament_cmake_uncrustify
  )
  ament_lint_auto_find_test_dependencies()
endif()

install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)
install(TARGETS camera_object_provider perception_viewer DESTINATION lib/${PROJECT_NAME})
ament_package()
