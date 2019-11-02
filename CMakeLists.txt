cmake_minimum_required(VERSION 3.5)
project(ObjectDetect)

set(CMAKE_CXX_STANDARD 11)
#set(CMAKE_CXX_FLAGS "$ENV{CFLAGS} -O3")
find_package(OpenCV REQUIRED)
find_package( PCL 1.8.0 )
find_package( Boost 1.58.0 REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

include_directories(${PCL_INCLUDE_DIRS})

set(TENSORRT_INCLUDE_DIR /home/ibd02/Project/yunzhou/Yolo/TensorRT-5.0.2.6/include)
include_directories (${TENSORRT_INCLUDE_DIR})
#add wrapper
set(TRT_WRAPPER_DIR ${CMAKE_CURRENT_SOURCE_DIR}/tensorRTWrapper/code)
add_subdirectory(${TRT_WRAPPER_DIR})
include_directories(${TRT_WRAPPER_DIR}/include)

include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include)
include_directories(${Boost_INCLUDE_DIRS})

link_directories(${OpenCV_LIBRARIES_DIRS})
file(GLOB SOURCE
        "./*.h"
        "./*.cpp"
        "common/*.h")


add_executable(ObjectDetect ${SOURCE})

target_link_libraries(ObjectDetect TrtNet ${OpenCV_LIBS} ${PCL_LIBRARIES} ${Boost_LIBRARIES}
        -lboost_system -lboost_regex -lshp)