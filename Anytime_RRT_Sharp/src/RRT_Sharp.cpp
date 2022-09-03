#include "ARRTS.hpp"

using namespace std;
using namespace std::chrono;

int main()
{
    double uavGoalRadius = 2.5;
    int maxCount = 10000, goalBiasCount = 100, maxNumNeighbors = 15;

    ArrtsService service("./test");
    service.calculatePath(uavGoalRadius, maxCount, goalBiasCount, maxNumNeighbors);

    return 0;
}