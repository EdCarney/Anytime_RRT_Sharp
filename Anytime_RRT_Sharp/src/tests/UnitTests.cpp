#include <gtest/gtest.h>
#include "ARRTSTests.hpp"
#include "GeometryTests.hpp"
#include "ObstacleTests.hpp"
#include "VehicleTests.hpp"

int main(int argc, char* argv[])
{
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}