#include <algorithm>
#include <cstddef>
#include <cstdlib>
#include <stdexcept>
#include <string>
#include "cppshrhelp.hpp"

using namespace std;

#ifndef ARRTS_H
#define ARRTS_H

struct DLL_EXPORT Node { double x, y; };
struct DLL_EXPORT Obstacle : Node { double radius; };
struct DLL_EXPORT Position : Node { double theta; };

class DLL_EXPORT ArrtsService
{
    private:
        Obstacle* obstacles;
        Position startPosition;
        Position goalPosition;
        Node* vehicleOutline;

        int numObstacles;
        int numVehiclePoints;

    public:
        ArrtsService();

        void SetGoalPosition(double x, double y, double theta);
        Position GetGoalPosition();

        void SetStartPosition(double x, double y, double theta);
        Position GetStartPosition();

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
        Position* CalculatePath(double standoffRange, double positionBuffer, double freespaceBuffer);
};

#endif