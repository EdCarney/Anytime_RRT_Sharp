#include "ARRTS.hpp"

using namespace std;
using namespace std::chrono;

int main()
{
    double uavGoalRadius = 2.5;
    int maxCount = 10000, goalBiasCount = 100, maxNumNeighbors = 15;

    ArrtsService service("./test");

    printf("ObsVol: %f, NumObs: %lu, Freespace: [%f, %f, %f, %f]\n", service.obstacleVolume(), service.obstacles().size(), service.limits().minPoint().x(), service.limits().minPoint().y(), service.limits().maxPoint().x(), service.limits().maxPoint().y());
    printf("UAV Location: %f, %f, %f\n", service.startState().x(), service.startState().y(), service.startState().theta());
    printf("Root Node:    %f, %f, %f\n", service.goalState().x(), service.goalState().y(), service.goalState().theta());

    srand (time(NULL));
    service.calculatePath(uavGoalRadius, maxCount, goalBiasCount, maxNumNeighbors);
    
    return 0;
}