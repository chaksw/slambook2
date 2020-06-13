# 声明要求的cmake最低版本
cmake_minimum_required(VERSION 2.8)

set(CMAKE_BUILD_TYPE "Debug")
# 声明一个cmake工程
project(HelloSLAM)

# 添加一个可执行程序
# 语法：add_executable（程序名 源代码文件）
add_executable(helloSLAM helloSLAM.cpp)

# 添加库
# 添加共享库 编译时会生成 libhello_shared.so文件
add_library(hello_shared SHARED libHelloSLAM.cpp)
# 添加静态库 编译时会生成 libhello.a文件
add_library(hello libHelloSLAM.cpp)

# 添加可执行程序调用hello库中函数
add_executable(useHello useHello.cpp)
# 将库文件链接到可执行程序上
target_link_libraries(useHello hello_shared)
