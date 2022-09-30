cmake_minimum_required(VERSION 3.21)

project(RRT_Sharp VERSION 1.0)

# specify C++ standard
set(CMAKE_OSX_ARCHITECTURES "x86_64" CACHE INTERNAL "" FORCE)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED True)
SET(CMAKE_OSX_DEPLOYMENT_TARGET 10.15)

# configure a header file to pass in some config settings
configure_file(ArrtsServiceConfig.h.in ArrtsServiceConfig.h)

# add libraries
add_library(ConfigspaceGraph ConfigspaceGraph.cpp)
add_library(ConfigspaceNode ConfigspaceNode.cpp)
add_library(WorkspaceGraph WorkspaceGraph.cpp)
add_library(Vehicle Vehicle.cpp)
add_library(Geometry2D Geometry2D.cpp)
add_library(Geometry3D Geometry3D.cpp)
add_library(ArrtsEngine ArrtsEngine.cpp)
add_library(ArrtsParams ArrtsParams.cpp)
add_library(ArrtsService ArrtsService.cpp)

# main libs
list(APPEND EXTRA_LIBS ConfigspaceGraph)
list(APPEND EXTRA_LIBS ConfigspaceNode)
list(APPEND EXTRA_LIBS WorkspaceGraph)
list(APPEND EXTRA_LIBS Vehicle)
list(APPEND EXTRA_LIBS Geometry2D)
list(APPEND EXTRA_LIBS Geometry3D)
list(APPEND EXTRA_LIBS ArrtsEngine)
list(APPEND EXTRA_LIBS ArrtsParams)
list(APPEND EXTRA_LIBS ArrtsService)

# libs for testing
list(APPEND TEST_LIBS ConfigspaceGraph)
list(APPEND TEST_LIBS ConfigspaceNode)
list(APPEND TEST_LIBS WorkspaceGraph)
list(APPEND TEST_LIBS Vehicle)
list(APPEND TEST_LIBS Geometry2D)
list(APPEND TEST_LIBS Geometry3D)
list(APPEND TEST_LIBS ArrtsParams)
list(APPEND TEST_LIBS gtest)

# add googletest directory
add_subdirectory(googletest)
include_directories(googletest/inlcude)

add_subdirectory(Dubins3d)

# include test data
file(COPY testdata/ DESTINATION test)

# add the executables
add_executable(RRT_Sharp RRT_Sharp.cpp)
add_executable(UnitTests tests/UnitTests.cpp)

target_link_libraries(RRT_Sharp PUBLIC ${EXTRA_LIBS})
target_link_libraries(UnitTests PRIVATE ${TEST_LIBS})

target_include_directories(RRT_Sharp PUBLIC "${PROJECT_BINARY_DIR}")