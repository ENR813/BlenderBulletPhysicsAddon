cmake_minimum_required(VERSION 3.0.0)

set(ENV{CMAKE_TOOLCHAIN_FILE} F:/dev/vcpkg/scripts/buildsystems/vcpkg.cmake)

#set(VCPKG_TRACE_FIND_PACKAGE ON)

set(VCPKG_TARGET_TRIPLET x64-windows)

MESSAGE(STATUS "${VCPKG_TARGET_TRIPLET}")

MESSAGE(STATUS "TEST")

IF(DEFINED ENV{CMAKE_TOOLCHAIN_FILE})
	MESSAGE(STATUS "CMAKE_TOOLCHAIN_FILE env seen: --[$ENV{CMAKE_TOOLCHAIN_FILE}]--")
ELSE()
	MESSAGE(STATUS "WTF")
ENDIF()

project(mybinds)

set (CMAKE_CXX_STANDARD 20)

#find_package(unofficial-sqlite3 CONFIG REQUIRED)

#include_directories(F:/dev/vcpkg/installed/x64-windows/include)
#link_directories(F:/dev/vcpkg/installed/x64-windows/lib)

add_subdirectory(pybind11)

#find_package(OpenGL REQUIRED)
#find_package(GLUT REQUIRED)
#find_package(Bullet CONFIG REQUIRED)

#include_directories( ${OPENGL_INCLUDE_DIRS} ${BULLET_INCLUDE_DIRS}) #${GLUT_INCLUDE_DIRS}

include_directories(bullet)

pybind11_add_module(mybinds myBulletPythonBindsThreaded.cpp)
#target_compile_definitions(mybinds PUBLIC LIBRARY)

add_executable(test visual_test.cpp)

add_executable(phys_view bullet_test_vis.cpp)

add_executable(test_binds myBulletPythonBindsThreaded.cpp)
target_compile_definitions(test_binds PUBLIC EXE=1)

#target_link_libraries(mybinds ${OPENGL_LIBRARIES} main PRIVATE ${BULLET_LIBRARIES}) #PRIVATE GLUT::GLUT

#target_link_libraries(mybinds PRIVATE unofficial::sqlite3::sqlite3)