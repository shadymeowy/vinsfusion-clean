cmake_minimum_required(VERSION 3.0.2)
project(vins)

## Set build type
set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -g")
set(CMAKE_POSITION_INDEPENDENT_CODE ON) # TODO: Is this necessary?

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

## Source files
file(GLOB_RECURSE SRC_FILES_VINS_ESTIMATOR src/vins_estimator/*.cpp)
file(GLOB_RECURSE SRC_FILES_GLOBAL_FUSION src/global_fusion/*.cpp)
file(GLOB_RECURSE SRC_FILES_LOOP_FUSION src/loop_fusion/*.cpp)

file(GLOB_RECURSE SRC_FILES_DBOW src/DBoW/*.cpp)
file(GLOB_RECURSE SRC_FILES_DUTILS src/DUtils/*.cpp)
file(GLOB_RECURSE SRC_FILES_DVISION src/DVision/*.cpp)

file(GLOB_RECURSE SRC_FILES_GEOGRAPHICLIB src/GeographicLib/*.cpp)
file(GLOB_RECURSE SRC_FILES_CAMODOCAL src/camodocal/*.cc)

## Create a combined static library for DBoW, DUtils, DVision
add_library(dvision_lib STATIC
  ${SRC_FILES_DBOW}
  ${SRC_FILES_DUTILS}
  ${SRC_FILES_DVISION}
)

## Create a static library for GeographicLib
add_library(geographiclib STATIC
  ${SRC_FILES_GEOGRAPHICLIB}
)

## Create a static library for camodocal
add_library(camodocal STATIC
  ${SRC_FILES_CAMODOCAL}
)

target_link_libraries(camodocal
  ${OpenCV_LIBS}
)

## Create a shared library for vins estimator
add_library(vins_estimator_lib ${SRC_FILES_VINS_ESTIMATOR})
target_link_libraries(vins_estimator_lib
  ${catkin_LIBRARIES}
  ${OpenCV_LIBS}
  ${CERES_LIBRARIES}
  dvision_lib
  geographiclib
  camodocal
)

## Create a shared library for global fusion
add_library(global_fusion_lib ${SRC_FILES_GLOBAL_FUSION})
target_link_libraries(global_fusion_lib
  ${catkin_LIBRARIES}
  ${OpenCV_LIBS}
  ${CERES_LIBRARIES}
  dvision_lib
  geographiclib
  camodocal
)

## Create a shared library for loop fusion
add_library(loop_fusion_lib ${SRC_FILES_LOOP_FUSION})
target_link_libraries(loop_fusion_lib
  ${catkin_LIBRARIES}
  ${OpenCV_LIBS}
  ${CERES_LIBRARIES}
  dvision_lib
  geographiclib
  camodocal
)

## Optional: Create executables separately if needed
add_executable(vins_node src/node/node_ros.cpp)
target_link_libraries(vins_node vins_estimator_lib)

add_executable(kitti_odom_test src/node/node_kitti_odom.cpp)
target_link_libraries(kitti_odom_test vins_estimator_lib)

add_executable(kitti_gps_test src/node/node_kitti_gps.cpp)
target_link_libraries(kitti_gps_test vins_estimator_lib)

add_executable(loop_fusion_node src/node/node_loop_fusion.cpp)
target_link_libraries(loop_fusion_node loop_fusion_lib)

add_executable(global_fusion_node src/node/node_global_fusion.cpp)
target_link_libraries(global_fusion_node global_fusion_lib)