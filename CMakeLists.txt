cmake_minimum_required(VERSION 3.15)
project(control_system)

set(CMAKE_CXX_STANDARD 17)

add_executable(control_system main.cpp ROV.cpp ROV.h)