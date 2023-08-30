cmake_minimum_required(VERSION 3.0.0)

project(mybinds)

set (CMAKE_CXX_STANDARD 20)

add_subdirectory(pybind11)

include_directories(bullet)

pybind11_add_module(mybinds myBulletPythonBindsThreaded.cpp)

add_executable(test_binds myBulletPythonBindsThreaded.cpp)
target_compile_definitions(test_binds PUBLIC EXE=1)

