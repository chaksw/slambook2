cmake_minimum_required(VERSION 2.8)

# 添加C++ 11 标准支持
set( CMAKE_CXX_FLAGS "-std=c++11" )

find_package(Pangolin REQUIRED)

# 寻找OpenCV库
find_package(OpenCV REQUIRED)
# 添加头文件
include_directories(${OpenCV_INCLUDES_DIRS})

add_executable(stereoVision stereoVision.cpp)
target_link_libraries(stereoVision ${OpenCV_LIBS} ${Pangolin_LIBRARIES})