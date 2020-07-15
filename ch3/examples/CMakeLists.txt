# 声明要求的cmake最低版本
cmake_minimum_required(VERSION 2.8)

# 声明一个cmake工程
project(example)

# 添加头文件
include_directories("/usr/include/eigen3") 
# 添加一个可执行程序
# 语法：add_executable（程序名 源代码文件）
add_executable(corrdianteTransform corrdianteTransform.cpp)

find_package(Pangolin REQUIRED)
include_directories(${Pangolin_INCLUDE_DIRS})
add_executable(plotTrajectory plotTrajectory.cpp)
target_link_libraries(plotTrajectory ${Pangolin_LIBRARIES})
