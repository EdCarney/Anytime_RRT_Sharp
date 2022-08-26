#include <cstddef>
#include <cstdlib>
#include <stdexcept>
#include <string>
#include <vector>
#include "cppshrhelp.hpp"
#include "Geometry.hpp"
#include "Obstacle.hpp"
#include "Utility.hpp"

using namespace std;

#ifndef ARRTS_H
#define ARRTS_H

class DLL_EXPORT ArrtsService
{
    private:
        vector<Obstacle> _obstacles;
        vector<Point> _vehicleOutline;
        State _startState;
        State _goalState;

    public:
        void setGoalState(double x, double y, double theta);
        State goalState();

        void setStartState(double x, double y, double theta);
        State startState();

        void addObstacle(double x, double y, double r);
        void addObstacles(const double* x, const double* y, const double* r, int numObs);
        void addObstaclesFromFile(FILE* file);
        vector<Obstacle> obstacles();
        Obstacle obstacles(int i);

        // runs ARRTS algorithm
        State* calculatePath(double standoffRange, double positionBuffer, double freespaceBuffer);
};

#endif