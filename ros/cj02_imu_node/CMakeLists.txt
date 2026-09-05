cmake_minimum_required(VERSION 3.0.2)
project(cj02_imu_node)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  geometry_msgs
  std_msgs
  tf2
  tf2_geometry_msgs
)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp sensor_msgs geometry_msgs std_msgs tf2 tf2_geometry_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ../sdk/cpp/include
)

add_executable(cj02_imu_node src/cj02_imu_node.cpp)
target_link_libraries(cj02_imu_node ${catkin_LIBRARIES})

install(TARGETS cj02_imu_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
