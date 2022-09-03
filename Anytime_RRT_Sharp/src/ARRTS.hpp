#include <cstddef>
#include <cstdlib>
#include <stdexcept>
#include <string>
#include <vector>
#include "cppshrhelp.hpp"
//#include "ConfigspaceGraph.hpp"
#include "Geometry.hpp"
#include "Obstacle.hpp"
//#include "WorkspaceGraph.hpp"

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
        Rectangle _limits;
        // ConfigspaceGraph _configspaceGraph;
        // WorkspaceGraph _workspaceGraph;

    public:
        State goalState() const;
        void setGoalState(double x, double y, double theta);

        State startState() const;
        void setStartState(double x, double y, double theta);

        Rectangle limits() const;
        void setLimits(Point minPoint, Point maxPoint);
        void setLimits(double minX, double minY, double maxX, double maxY);

        vector<Obstacle> obstacles() const;
        Obstacle obstacles(int i) const;
        void addObstacle(double x, double y, double r);
        void addObstacles(const vector<double>& x, const vector<double>& y, const vector<double>& r);

        void readObstaclesFromFile(FILE* file);
        void readStatesFromFile(FILE* file);
        void readLimitsFromFile(FILE* file);
        void initializeFromDataDirectory(string dataDir);

        // runs ARRTS algorithm
        vector<State> calculatePath(double standoffRange, double positionBuffer, double freespaceBuffer);
};

#endif