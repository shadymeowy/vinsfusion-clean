cmake_minimum_required(VERSION 3.0.2)
project(vins)

## Set build type
set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -g")
cmake_policy(SET CMP0069 NEW)
set(CMAKE_INTERPROCEDURAL_OPTIMIZATION TRUE)

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

## Map Ceres and Eigen3 variables for catkin_package
# Ceres
if (CERES_INCLUDE_DIRS)
  set(Ceres_INCLUDE_DIRS ${CERES_INCLUDE_DIRS})
endif()
if (CERES_LIBRARIES)
  set(Ceres_LIBRARIES ${CERES_LIBRARIES})
endif()

# Eigen3
if (EIGEN3_INCLUDE_DIR)
  set(Eigen3_INCLUDE_DIRS ${EIGEN3_INCLUDE_DIR})
endif()
if (EIGEN3_LIBS)
  set(Eigen3_LIBRARIES ${EIGEN3_LIBS})
endif()

## Declare catkin package
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES vins_lib
  CATKIN_DEPENDS roscpp std_msgs geometry_msgs nav_msgs tf cv_bridge image_transport
  DEPENDS OpenCV Ceres Eigen3
)

## Create executables separately if needed
add_executable(
  vins_node
  src/node/node_ros.cpp
  src/vins_estimator/estimator/parameters.cpp
  src/vins_estimator/estimator/estimator.cpp
  src/vins_estimator/estimator/feature_manager.cpp
  src/vins_estimator/factor/pose_local_parameterization.cpp
  src/vins_estimator/factor/projectionTwoFrameOneCamFactor.cpp
  src/vins_estimator/factor/projectionTwoFrameTwoCamFactor.cpp
  src/vins_estimator/factor/projectionOneFrameTwoCamFactor.cpp
  src/vins_estimator/factor/marginalization_factor.cpp
  src/vins_estimator/utility/utility.cpp
  src/vins_estimator/utility/visualization.cpp
  src/vins_estimator/utility/CameraPoseVisualization.cpp
  src/vins_estimator/initial/solve_5pts.cpp
  src/vins_estimator/initial/initial_aligment.cpp
  src/vins_estimator/initial/initial_sfm.cpp
  src/vins_estimator/initial/initial_ex_rotation.cpp
  src/vins_estimator/featureTracker/feature_tracker.cpp
  src/vins_estimator/featureTracker/feature_tracker_log.cpp
  src/vins_estimator/featureTracker/feature_tracker_mono.cpp
  src/vins_estimator/featureTracker/klt_tracker.cpp
  src/camodocal/camera_models/Camera.cc
  src/camodocal/camera_models/CameraFactory.cc
  src/camodocal/camera_models/PinholeCamera.cc
  src/camodocal/camera_models/PinholeFullCamera.cc
  src/camodocal/camera_models/CataCamera.cc
  src/camodocal/camera_models/EquidistantCamera.cc
  src/camodocal/camera_models/ScaramuzzaCamera.cc
)

set_target_properties(vins_node PROPERTIES
  BUILD_WITH_INSTALL_RPATH TRUE
  INSTALL_RPATH "/root/catkin_ws/src/VINS-Fusion/pyloader/build"
)

target_link_libraries(vins_node
  ${catkin_LIBRARIES}
  ${OpenCV_LIBS}
  ${CERES_LIBRARIES}
  "/root/catkin_ws/src/VINS-Fusion/pyloader/build/libpyloader.so"
)

add_executable(
  loop_fusion_node
  src/node/node_loop_fusion.cpp
  src/loop_fusion/pose_graph.cpp
  src/loop_fusion/keyframe.cpp
  src/loop_fusion/parameters.cpp
  src/loop_fusion/utility/CameraPoseVisualization.cpp
  src/DBoW/BowVector.cpp
  src/DBoW/FBrief.cpp
  src/DBoW/FeatureVector.cpp
  src/DBoW/QueryResults.cpp
  src/DBoW/ScoringObject.cpp
  src/DUtils/Random.cpp
  src/DUtils/Timestamp.cpp
  src/DVision/BRIEF.cpp
  src/DBoW/VocabularyBinary.cpp
  src/camodocal/camera_models/Camera.cc
  src/camodocal/camera_models/CameraFactory.cc
  src/camodocal/camera_models/PinholeCamera.cc
  src/camodocal/camera_models/PinholeFullCamera.cc
  src/camodocal/camera_models/CataCamera.cc
  src/camodocal/camera_models/EquidistantCamera.cc
  src/camodocal/camera_models/ScaramuzzaCamera.cc
)

target_link_libraries(loop_fusion_node
  ${catkin_LIBRARIES}
  ${OpenCV_LIBS}
  ${CERES_LIBRARIES}
)

add_executable(
  global_fusion_node
  src/node/node_global_fusion.cpp
  src/global_fusion/globalOpt.cpp
  src/GeographicLib/LocalCartesian.cpp
  src/GeographicLib/Geocentric.cpp
  src/GeographicLib/Math.cpp
)

target_link_libraries(global_fusion_node
  ${catkin_LIBRARIES}
  ${OpenCV_LIBS}
  ${CERES_LIBRARIES}
)