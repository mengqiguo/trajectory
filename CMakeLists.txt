cmake_minimum_required(VERSION 3.10)
project(FemSmoother)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# 查找 OSQP（已安装后会导出 CMake config）
find_package(osqp CONFIG REQUIRED)

# 源文件
add_executable(fem_smoother
    src/main.cpp
    src/trajectory_smoother.cpp
)

target_include_directories(fem_smoother PRIVATE include)
target_link_libraries(fem_smoother PRIVATE osqp::osqp)

# 如果需要调试信息
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -O2")