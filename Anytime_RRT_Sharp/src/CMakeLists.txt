cmake_minimum_required(VERSION 3.0)

project(ARRTS VERSION 1.0)

# specify C++ standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED True)

# configure a header file to pass in some config settings
configure_file(ARRTSConfig.h.in ARRTSConfig.h)

add_library(ConfigspaceGraph configspaceGraph.cpp)
add_library(WorkspaceGraph workspaceGraph.cpp)

list(APPEND EXTRA_LIBS ConfigspaceGraph)
list(APPEND EXTRA_LIBS WorkspaceGraph)

# add the executable
add_executable(ARRTS RRT_Sharp.cpp)

target_link_libraries(ARRTS PUBLIC ${EXTRA_LIBS})

target_include_directories(ARRTS PUBLIC "${PROJECT_BINARY_DIR}")