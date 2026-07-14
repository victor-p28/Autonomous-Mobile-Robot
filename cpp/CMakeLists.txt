cmake_minimum_required(VERSION 3.10)
project(ekf_test)

set(CMAKE_CXX_STANDARD 17)

find_package(Eigen3 REQUIRED)

add_executable(ekf_test main.cpp ekf.cpp)
target_link_libraries(ekf_test Eigen3::Eigen)