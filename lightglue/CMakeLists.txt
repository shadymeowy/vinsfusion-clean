cmake_minimum_required(VERSION 3.15)
project(onnx_db_example CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Adjust these to your installation paths
set(ONNXRUNTIME_DIR /home/tdemirdal/Others/onnxruntime-linux-x64-gpu-1.22.0)

include_directories(${ONNXRUNTIME_DIR}/include 
                    ${WORKSPACE_DIR}/)
link_directories(${ONNXRUNTIME_DIR}/lib)

find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

add_executable(lightglue_example matcher.cpp example.cpp)

# Link OpenCV libraries to your target
target_link_libraries(lightglue_example onnxruntime ${OpenCV_LIBS})
