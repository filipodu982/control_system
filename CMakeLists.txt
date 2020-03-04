cmake_minimum_required(VERSION 3.15)
project(control_system)

set(CMAKE_CXX_STANDARD 17)
include_directories(src/control-toolbox/ct_optcon/include)
include_directories(Eigen/)
include_directories(ct/core)
include_directories(ct/optcon)

include_directories(/usr/local/lib)
add_definitions(-DCT_USE_LAPACK)

add_executable(control_system main.cpp ROV.cpp EigenQP.cpp ROV.h)
target_link_libraries(control_system LINK_PUBLIC lapack)
target_link_libraries(control_system LINK_PUBLIC blas)