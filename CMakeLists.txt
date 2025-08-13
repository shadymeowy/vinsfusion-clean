cmake_minimum_required(VERSION 3.0.2)
project(vins)

## Set build type
set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -g")
cmake_policy(SET CMP0069 NEW)
cmake_policy(SET CMP0074 NEW)
set(CMAKE_INTERPROCEDURAL_OPTIMIZATION TRUE)

# check for cuda flag USE_CUDA
if (NOT DEFINED USE_CUDA)
  set(USE_CUDA OFF)
endif()

if (USE_CUDA)
  if(CMAKE_VERSION VERSION_LESS 3.17)
    find_package(CUDA REQUIRED)

    add_library(CUDA::cudart   INTERFACE IMPORTED)
    add_library(CUDA::cublas   INTERFACE IMPORTED)
    add_library(CUDA::cublasLt INTERFACE IMPORTED)
    add_library(CUDA::cusolver INTERFACE IMPORTED)
    add_library(CUDA::cusparse INTERFACE IMPORTED)

    # Use legacy vars; fall back to find_library if any are empty
    if(NOT CUDA_CUDART_LIBRARY)
      find_library(CUDA_CUDART_LIBRARY cudart HINTS "${CUDA_TOOLKIT_ROOT_DIR}/lib64" "/usr/local/cuda/lib64")
    endif()
    if(NOT CUDA_CUBLAS_LIBRARIES)
      find_library(CUDA_CUBLAS_LIBRARIES cublas HINTS "${CUDA_TOOLKIT_ROOT_DIR}/lib64" "/usr/local/cuda/lib64")
    endif()
    if(NOT CUDA_CUBLASLT_LIBRARY)
      find_library(CUDA_CUBLASLT_LIBRARY cublasLt HINTS "${CUDA_TOOLKIT_ROOT_DIR}/lib64" "/usr/local/cuda/lib64")
    endif()
    if(NOT CUDA_cusolver_LIBRARY)
      find_library(CUDA_cusolver_LIBRARY cusolver HINTS "${CUDA_TOOLKIT_ROOT_DIR}/lib64" "/usr/local/cuda/lib64")
    endif()
    if(NOT CUDA_cusparse_LIBRARY)
      find_library(CUDA_cusparse_LIBRARY cusparse HINTS "${CUDA_TOOLKIT_ROOT_DIR}/lib64" "/usr/local/cuda/lib64")
    endif()

    set_target_properties(CUDA::cudart   PROPERTIES INTERFACE_LINK_LIBRARIES "${CUDA_CUDART_LIBRARY}")
    set_target_properties(CUDA::cublas   PROPERTIES INTERFACE_LINK_LIBRARIES "${CUDA_CUBLAS_LIBRARIES}")
    set_target_properties(CUDA::cublasLt PROPERTIES INTERFACE_LINK_LIBRARIES "${CUDA_CUBLASLT_LIBRARY}")
    set_target_properties(CUDA::cusolver PROPERTIES INTERFACE_LINK_LIBRARIES "${CUDA_cusolver_LIBRARY}")
    set_target_properties(CUDA::cusparse PROPERTIES INTERFACE_LINK_LIBRARIES "${CUDA_cusparse_LIBRARY}")
  else()
    find_package(CUDAToolkit REQUIRED COMPONENTS cublas cublasLt cudart cusolver cusparse)
  endif()
endif()

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

# ONNX Runtime
set(ONNXRUNTIME_DIR /opt/netvlad/onnxruntime-linux-x64-gpu-1.22.0)

## Include directories
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${CERES_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
  ${ONNXRUNTIME_DIR}/include
)

link_directories(${ONNXRUNTIME_DIR}/lib)

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
  src/camodocal/camera_models/Camera.cc
  src/camodocal/camera_models/CameraFactory.cc
  src/camodocal/camera_models/PinholeCamera.cc
  src/camodocal/camera_models/PinholeFullCamera.cc
  src/camodocal/camera_models/CataCamera.cc
  src/camodocal/camera_models/EquidistantCamera.cc
  src/camodocal/camera_models/ScaramuzzaCamera.cc
)

target_link_libraries(vins_node
  ${catkin_LIBRARIES}
  ${OpenCV_LIBS}
  ${CERES_LIBRARIES}
)

add_executable(
  loop_fusion_node
  src/node/node_loop_fusion.cpp
  src/loop_fusion/pose_graph.cpp
  src/loop_fusion/keyframe.cpp
  src/loop_fusion/parameters.cpp
  src/loop_fusion/utility/CameraPoseVisualization.cpp
  src/loop_fusion/netvladdb.cpp
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
  onnxruntime
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