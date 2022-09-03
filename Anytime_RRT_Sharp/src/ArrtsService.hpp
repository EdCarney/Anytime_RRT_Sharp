#include <chrono>
#include <cstddef>
#include <cstdlib>
#include <stdexcept>
#include <string>
#include <vector>
#include "ConfigspaceGraph.hpp"
#include "ConfigspaceNode.hpp"
#include "cppshrhelp.hpp"
#include "Geometry.hpp"
#include "Obstacle.hpp"
#include "Vehicle.hpp"
#include "WorkspaceGraph.hpp"

using namespace std;
using namespace std::chrono;

#ifndef ARRTS_SERVICE_H
#define ARRTS_SERVICE_H

#define DEFAULT_STATES_FILE "states.txt"
#define DEFAULT_VEHICLE_FILE "robot.txt"
#define DEFAULT_OBSTACLES_FILE "obstacles.txt"

class DLL_EXPORT ArrtsService
{
    private:
        int _maxCount, _goalBiasCount, _maxNumNeighbors;
        double _dimension = 2; // hard-coded
        double _goalRadius, _obstacleVolume;
        State _startState;
        State _goalState;
        Rectangle _limits;
        Vehicle _vehicle;
        vector<Obstacle> _obstacles;
        vector<State> _path;
        ConfigspaceGraph _configspaceGraph;
        WorkspaceGraph _workspaceGraph;

        void _calculateObstacleVolume();
        void _updateLimitsFromStates();
        void _removeObstaclesNotInLimits();
        void _configureWorkspace();
        void _configureConfigspace();

        void _runAlgorithm();
        void _rewireNodes(vector<ConfigspaceNode>& remainingNodes, ConfigspaceNode& addedNode);
        void _tryConnectToBestNeighbor(vector<ConfigspaceNode>& neighbors, ConfigspaceNode& newNode, ConfigspaceNode& parentNode);
        void _getFinalPath();
        bool _compareNodes(ConfigspaceNode n1, ConfigspaceNode n2);
        ConfigspaceNode _findBestNode();

    public:
        //TEMP
        double obstacleVolume();

        ArrtsService();
        ArrtsService(string dataDirectory);

        State goalState() const;
        void setGoalState(double x, double y, double theta);

        State startState() const;
        void setStartState(double x, double y, double theta);

        Rectangle limits() const;
        void setLimits(Point minPoint, Point maxPoint);
        void setLimits(double minX, double minY, double maxX, double maxY);

        Vehicle vehicle() const;
        void setVehicle(vector<double> x, vector<double> y);

        ConfigspaceGraph& configspaceGraph();
        WorkspaceGraph& workspaceGraph();

        vector<Obstacle> obstacles() const;
        Obstacle obstacles(int i) const;
        void addObstacle(double x, double y, double r);
        void addObstacles(const vector<double>& x, const vector<double>& y, const vector<double>& r);

        void readStatesFromFile(FILE* file);
        void readVehicleFromFile(FILE* file);
        void readObstaclesFromFile(FILE* file);
        void initializeFromDataDirectory(string dataDirectory);

        // runs ARRTS algorithm
        vector<State> calculatePath(double goalRadius, int maxCount, int goalBiasCount, int maxNumNeighbors);
};

#endif