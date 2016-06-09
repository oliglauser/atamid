cmake_minimum_required(VERSION 3.3)
project(rigReducer)

set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR})
set(CMAKE_COLOR_MAKEFILE ON)

find_package(OpenGL REQUIRED)
find_package(EIGEN REQUIRED)
find_package(GLFWH REQUIRED)
find_package(LIBIGL REQUIRED)
#find_package(NANOGUI REQUIRED)

set(CMAKE_CXX_LINK_FLAGS "-framework OpenGL -framework Cocoa -framework IOKit -framework CoreVideo")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -msse4.2")

#manually set at the moment :( sorry for that
SET(NANOGUI_INCLUDE_DIRS
        /Users/oliverglauser/git/libigl/external/nanogui/include
        /Users/oliverglauser/git/libigl/external/nanogui/ext/nanovg/src
        /Users/oliverglauser/git/libigl/external/nanogui/ext/glfw/include
        )

FIND_LIBRARY(NANOGUI_LIBRARY NAMES nanogui PATHS /Users/oliverglauser/git/libigl/external/nanogui/sbuild)

SET(NANOGUI_FOUND "YES")

include_directories( ${NANOGUI_INCLUDE_DIRS} )
include_directories( ../ )

include_directories( ${EIGEN_INCLUDE_DIR})
include_directories( ${LIBIGL_INCLUDE_DIRS})
include_directories( ${GLFW_INCLUDE_DIR})
include_directories(
        /usr/local/include
        /opt/local/include
        ${PROJECT_SOURCE_DIR}
        ${OPENGL_INCLUDE_DIR}
)

link_directories(
        /usr/local/lib
        ${EIGEN_DIRS}
)

#Gurobi Solver
include_directories("/Library/gurobi650/mac64/include")
link_directories("/Library/gurobi650/mac64/lib")

add_definitions(-DIGL_OPENGL_4)

IF(CMAKE_BUILD_TYPE MATCHES RELEASE)
    add_definitions(-DNDEBUG)
ENDIF(CMAKE_BUILD_TYPE MATCHES RELEASE)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wno-deprecated-register")

set(SHARED_LIBRARIES
        ${OPENGL_LIBRARIES}
        ${NANOGUI_LIBRARY}
        ${EIGEN_LIBRARIES}
        ${GLFW_LIBRARIES}
        )

set(SOURCE_FILES main.cpp
        MeshMerger.cpp
        draw_things.cpp
        reducer3.cpp
        reduceModel.cpp
        )

add_executable(${PROJECT_NAME} ${SOURCE_FILES})
target_link_libraries(${PROJECT_NAME} ${SHARED_LIBRARIES} gurobi_c++ gurobi65)