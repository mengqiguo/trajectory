cmake_minimum_required(VERSION 3.10)
project(trajectory_smoother)
set(CMAKE_CXX_STANDARD 17)
find_package(Eigen3 REQUIRED)

include_directories(
include(${EIGEN3_INCLUDE_DIR})
)



add_executable(
trajectory_demo
app/main.cpp
src/frenet_converter.cpp
src/qp_smoother.cpp
src/trajectory_smoother.cpp
)
