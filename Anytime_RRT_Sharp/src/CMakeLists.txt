cmake_minimum_required(VERSION 3.0)

project(RRT_Sharp VERSION 1.0)

# specify C++ standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED True)

# configure a header file to pass in some config settings
configure_file(ARRTSConfig.h.in ARRTSConfig.h)

# add libraries
add_library(ConfigspaceGraph configspaceGraph.cpp)
add_library(WorkspaceGraph workspaceGraph.cpp)
add_library(ARRTS ARRTS.cpp)

list(APPEND EXTRA_LIBS ConfigspaceGraph)
list(APPEND EXTRA_LIBS WorkspaceGraph)
list(APPEND EXTRA_LIBS ARRTS)

list(APPEND TEST_LIBS ARRTS)
list(APPEND TEST_LIBS WorkspaceGraph)
list(APPEND TEST_LIBS gtest)

# add googletest directory
add_subdirectory(googletest)
include_directories(googletest/inlcude)

# include test data
file(COPY test/ DESTINATION test)

# add the executables
add_executable(RRT_Sharp RRT_Sharp.cpp)
add_executable(tests unitTests.cpp)

target_link_libraries(RRT_Sharp PUBLIC ${EXTRA_LIBS})
target_link_libraries(tests PRIVATE ${TEST_LIBS})

target_include_directories(RRT_Sharp PUBLIC "${PROJECT_BINARY_DIR}")