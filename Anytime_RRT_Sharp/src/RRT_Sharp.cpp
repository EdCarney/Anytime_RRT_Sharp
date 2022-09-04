#include "ArrtsService.hpp"

using namespace std;
using namespace std::chrono;

int main(int argc, char** argv)
{
    double uavGoalRadius = 2.5;
    int minNodeCount = 10000, goalBiasCount = 100, maxNumNeighbors = 15;
    ArrtsService service;

    if (argc > 1)
    {
        for (int i = 1; i < argc; ++i)
        {
            service.initializeFromDataDirectory(argv[i]);
            service.calculatePath(uavGoalRadius, minNodeCount, goalBiasCount, maxNumNeighbors);
        }
    }
    else
    {
        ArrtsService service("./test");
        service.calculatePath(uavGoalRadius, minNodeCount, goalBiasCount, maxNumNeighbors);
    }

    return 0;
}