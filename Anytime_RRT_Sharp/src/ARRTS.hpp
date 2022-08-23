#include <cstddef>
#include <cstdlib>
#include <stdexcept>
#include <string>
#include "cppshrhelp.hpp"
#include "Geometry.hpp"
#include "Utility.hpp"

using namespace std;

#ifndef ARRTS_H
#define ARRTS_H

class DLL_EXPORT ArrtsService
{
    private:
        Obstacle* obstacles;
        State startState;
        State goalState;
        Node* vehicleOutline;

        int numObstacles;
        int numVehiclePoints;

    public:
        ArrtsService();

        void SetGoalState(double x, double y, double theta);
        State GetGoalState();

        void SetStartState(double x, double y, double theta);
        State GetStartState();

        void AddObstacle(double x, double y, double r);
        void AddObstacles(const double* x, const double* y, const double* r, int numObs);
        void AddObstaclesFromFile(FILE* file);
        Obstacle* GetObstacles();
        Obstacle GetObstacle(int i);
        int GetNumObstacles();

        void AddVehiclePoint(double x, double y);
        void AddVehiclePoints(const double* x, const double *y, int numPoints);
        void AddVehiclePointsFromFile(FILE* file);
        Node* GetVehiclePoints();
        Node GetVehiclePoint(int i);
        int GetNumVehiclePoints();

        // runs ARRTS algorithm
        State* CalculatePath(double standoffRange, double positionBuffer, double freespaceBuffer);
};

#endif