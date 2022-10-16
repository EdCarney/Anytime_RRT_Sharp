#include <gtest/gtest.h>
#include "ArrtsParamsTests.hpp"
#include "Geometry2DTests.hpp"
#include "Geometry3DTests.hpp"
#include "ManeuverEngineTests.hpp"
#include "VehicleTests.hpp"

int main(int argc, char* argv[])
{
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}