cmake_minimum_required(VERSION 3.13)
project(PLANE_RCNN C CXX)

#####################################################
# version
set(PLANE_RCNN_VERSION_MAJOR 1)
set(PLANE_RCNN_VERSION_MINOR 1)
set(PLANE_RCNN_VERSION_PATCH 1)

#####################################################
# c++17 flag
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++17")

#####################################################
# build system
IF (CMAKE_SYSTEM_NAME MATCHES "Linux")
    MESSAGE(STATUS "platform: Linux")
    # .so option ( shared library )
    add_compile_options(-fPIC)
ENDIF (CMAKE_SYSTEM_NAME MATCHES "Linux")

#####################################################
# set cmake module
set (SOURCE_DIR "${CMAKE_CURRENT_SOURCE_DIR}")
set (CMAKE_MODULE_PATH "${CMAKE_SOURCE_DIR}/cmake" ${CMAKE_MODULE_PATH})
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

#####################################################
# include cmake
include(common/base)

#####################################################
# option
option( BUILD_TEST "Build Test." OFF)


#####################################################
# find package
find_package(OpenGL)
find_package(glm)
find_package(GLEW)
find_package(GLFW QUIET)
if(NOT GLFW_FOUND)
    message(STATUS "GLFW NOT FOUND PKG FOUND...")
    find_package(PkgConfig REQUIRED)
    pkg_check_modules(GLFW REQUIRED glfw3)
endif()

#####################################################
# include headers
include_directories(.)
include_directories(core)
include_directories(src)
include_directories(3rdparty)
#####################################################
# add subdirectory
add_subdirectory(core)
add_subdirectory(src)
add_subdirectory(3rdparty)
#####################################################
# test
if(NOT GTEST_FOUND AND BUILD_TEST)
    message(STATUS "GTest install...")
    include_directories(test)
    add_subdirectory(test)
endif()












