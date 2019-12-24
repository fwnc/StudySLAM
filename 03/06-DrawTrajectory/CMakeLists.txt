cmake_minimum_required(VERSION 3.15)
project(DrawTrajectory)
set(CMAKE_CXX_STANDARD 14)
add_executable(DrawTrajectory main.cpp)


include_directories("/usr/include/eigen3")
find_package(Pangolin REQUIRED)
include_directories(${Pangolin_INCLUDE_DIRS})
target_link_libraries(DrawTrajectory ${Pangolin_LIBRARIES})
