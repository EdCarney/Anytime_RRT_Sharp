cmake_minimum_required(VERSION 3.0)

project(RRT_Sharp VERSION 1.0)

# specify C++ standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED True)

# configure a header file to pass in some config settings
configure_file(ARRTSConfig.h.in ARRTSConfig.h)

# add libraries
add_library(ConfigspaceGraph ConfigspaceGraph.cpp)
add_library(ConfigspaceNode ConfigspaceNode.cpp)
add_library(WorkspaceGraph WorkspaceGraph.cpp)
add_library(Vehicle Vehicle.cpp)
add_library(Obstacle Obstacle.cpp)
add_library(Geometry Geometry.cpp)
add_library(ArrtsService ArrtsService.cpp)

list(APPEND EXTRA_LIBS ConfigspaceGraph)
list(APPEND EXTRA_LIBS ConfigspaceNode)
list(APPEND EXTRA_LIBS WorkspaceGraph)
list(APPEND EXTRA_LIBS Vehicle)
list(APPEND EXTRA_LIBS Obstacle)
list(APPEND EXTRA_LIBS Geometry)
list(APPEND EXTRA_LIBS ArrtsService)

list(APPEND TEST_LIBS ConfigspaceGraph)
list(APPEND TEST_LIBS ConfigspaceNode)
list(APPEND TEST_LIBS WorkspaceGraph)
list(APPEND TEST_LIBS Vehicle)
list(APPEND TEST_LIBS Obstacle)
list(APPEND TEST_LIBS Geometry)
list(APPEND TEST_LIBS ArrtsService)
list(APPEND TEST_LIBS gtest)

# add googletest directory
add_subdirectory(googletest)
include_directories(googletest/inlcude)

# include test data
file(COPY testdata/ DESTINATION test)

# add the executables
add_executable(RRT_Sharp RRT_Sharp.cpp)
add_executable(UnitTests tests/UnitTests.cpp)

target_link_libraries(RRT_Sharp PUBLIC ${EXTRA_LIBS})
target_link_libraries(UnitTests PRIVATE ${TEST_LIBS})

target_include_directories(RRT_Sharp PUBLIC "${PROJECT_BINARY_DIR}")