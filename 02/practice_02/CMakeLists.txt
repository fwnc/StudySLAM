cmake_minimum_required(VERSION 3.15)
project(Matrix100X100)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_BUILD_TYPE "Release")
include_directories("/usr/include/eigen3")
add_executable(Matrix100X100 main.cpp)