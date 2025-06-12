cmake_minimum_required(VERSION 3.0.2)
project(vins)

## Set build type
set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g -Werror")

## Find required packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  geometry_msgs
  nav_msgs
  tf
  cv_bridge
  image_transport
)

find_package(OpenCV REQUIRED)
find_package(Ceres REQUIRED)
find_package(Eigen3 REQUIRED)

## Include directories
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${CERES_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
)

## Declare catkin package
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES vins_lib
  CATKIN_DEPENDS roscpp std_msgs geometry_msgs nav_msgs tf cv_bridge image_transport
  DEPENDS OpenCV Ceres Eigen3
)

## Source files from all subdirectories
file(GLOB_RECURSE SRC_FILES src/*.cpp)

## Create a shared library (if applicable)
add_library(vins_lib ${SRC_FILES})
target_link_libraries(vins_lib ${catkin_LIBRARIES} ${OpenCV_LIBS} ${CERES_LIBRARIES})

## Optional: Create executables separately if needed
add_executable(vins_node src/vins_estimator/node_ros.cpp)
target_link_libraries(vins_node vins_lib)

add_executable(kitti_odom_test src/vins_estimator/node_kitti_odom.cpp)
target_link_libraries(kitti_odom_test vins_lib)

add_executable(kitti_gps_test src/vins_estimator/node_kitti_gps.cpp)
target_link_libraries(kitti_gps_test vins_lib)
