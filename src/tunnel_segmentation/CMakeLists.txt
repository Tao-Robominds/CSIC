cmake_minimum_required(VERSION 3.8)
project(tunnel_segmentation)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(PCL REQUIRED COMPONENTS common io filters segmentation)
find_package(pcl_conversions REQUIRED)

include_directories(
  include
  ${PCL_INCLUDE_DIRS}
)

link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

# Point Cloud Converter Node
add_executable(point_cloud_converter src/point_cloud_converter.cpp)
ament_target_dependencies(point_cloud_converter
  rclcpp
  sensor_msgs
  pcl_conversions
)
target_link_libraries(point_cloud_converter ${PCL_LIBRARIES})

# Tunnel Segmentation Node
add_executable(tunnel_segmentation src/tunnel_segmentation.cpp)
ament_target_dependencies(tunnel_segmentation
  rclcpp
  sensor_msgs
  pcl_conversions
)
target_link_libraries(tunnel_segmentation ${PCL_LIBRARIES})

install(TARGETS
  point_cloud_converter
  tunnel_segmentation
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files
install(DIRECTORY
  launch
  config
  DESTINATION share/${PROJECT_NAME}
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

ament_package()
