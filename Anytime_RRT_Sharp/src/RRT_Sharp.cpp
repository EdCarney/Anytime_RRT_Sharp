#include "ArrtsParams.hpp"
#include "ArrtsService.hpp"

int main(int argc, char** argv)
{
    ArrtsService service;
    double uavGoalRadius = 2.5;

    if (argc > 1)
        for (int i = 1; i < argc; ++i)
            service.calculatePath(ArrtsParams(argv[i], uavGoalRadius));
    else
        service.calculatePath(ArrtsParams("./test", uavGoalRadius));

    return 0;
}