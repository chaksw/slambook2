cmake_minimum_required(VERSION 2.8)

include_directories("/usr/include/eigen3") 
# 为使用sopbus，需要使用find_pacakge命令找到它
find_package( Sophus REQUIRED )
include_directories( ${Sophus_INCLUDE_DIRS} )

add_executable(useSophus useSophus.cpp )
