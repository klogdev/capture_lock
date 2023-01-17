cmake_minimum_required(VERSION 3.10)

project(capture_lock)

set(CMAKE_CXX_STANDARD 17)
SET(GCC_COVERAGE_COMPILE_FLAGS "-Werror=return-type")

find_package(Eigen3 REQUIRED)
find_package(Ceres REQUIRED)
find_package(gflags REQUIRED)
find_package(COLMAP REQUIRED)
add_subdirectory(workspace)
