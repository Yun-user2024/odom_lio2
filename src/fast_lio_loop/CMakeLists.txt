cmake_minimum_required(VERSION 3.8)
project(fast_lio_loop)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3 -fopenmp")

set(CMAKE_POSITION_INDEPENDENT_CODE ON)

# ROS2
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(pcl_conversions REQUIRED)

# Third-party
find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED COMPONENTS common io registration)
find_package(GTSAM REQUIRED)

set(dependencies
  rclcpp
  geometry_msgs
  nav_msgs
  sensor_msgs
  visualization_msgs
  std_msgs
  std_srvs
  tf2_geometry_msgs
  pcl_ros
  pcl_conversions
)

include_directories(
  include
  ${PCL_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
)

# ScanContext library
add_library(scan_context
  src/scan_context.cpp
)
target_include_directories(scan_context PUBLIC include)
target_link_libraries(scan_context ${PCL_LIBRARIES} Eigen3::Eigen)

# PoseGraph library (GTSAM wrapper)
add_library(pose_graph
  src/pose_graph.cpp
)
target_include_directories(pose_graph PUBLIC include)
target_link_libraries(pose_graph gtsam ${PCL_LIBRARIES})

# Main node
add_executable(loop_closure_node
  src/loop_closure_node.cpp
)
target_include_directories(loop_closure_node PUBLIC include)
target_link_libraries(loop_closure_node
  scan_context
  pose_graph
  ${PCL_LIBRARIES}
  gtsam
)
ament_target_dependencies(loop_closure_node ${dependencies})

# Install
install(TARGETS loop_closure_node scan_context pose_graph
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
