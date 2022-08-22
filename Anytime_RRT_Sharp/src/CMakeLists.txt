cmake_minimum_required(VERSION 3.0)

project(ARRTS VERSION 1.0)

# specify C++ standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED True)

# configure a header file to pass in some config settings
configure_file(ARRTSConfig.h.in ARRTSConfig.h)

# add libraries
add_library(ConfigspaceGraph configspaceGraph.cpp)
add_library(WorkspaceGraph workspaceGraph.cpp)

list(APPEND EXTRA_LIBS ConfigspaceGraph)
list(APPEND EXTRA_LIBS WorkspaceGraph)

# add googletest directory
add_subdirectory(googletest)
include_directories(googletest/inlcude)

# add the executables
add_executable(ARRTS RRT_Sharp.cpp)
add_executable(tests unitTests.cpp)

target_link_libraries(ARRTS PUBLIC ${EXTRA_LIBS})
target_link_libraries(tests PRIVATE gtest)

target_include_directories(ARRTS PUBLIC "${PROJECT_BINARY_DIR}")