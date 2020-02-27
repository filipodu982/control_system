cmake_minimum_required(VERSION 3.15)
project(control_system)

set(CMAKE_CXX_STANDARD 17)
include_directories(src/control-toolbox/ct_optcon/include)
include_directories(Eigen/)
include_directories(ct/core)
include_directories(optcon)

add_executable(control_system main.cpp ROV.cpp ROV.h)