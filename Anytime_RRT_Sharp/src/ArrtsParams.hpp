#include <fstream>
#include <sstream>
#include <string>
#include <vector>
#include "cppshrhelp.hpp"
#include "Geometry2D.hpp"
#include "Geometry3D.hpp"
#include "Vehicle.hpp"

#ifndef ARRTS_PARAMS_H
#define ARRTS_PARAMS_H

#define DEFAULT_STATES_FILE "states.txt"
#define DEFAULT_VEHICLE_FILE "robot.txt"
#define DEFAULT_OBSTACLES_FILE "obstacles.txt"
#define DEFAULT_MIN_NODE_COUNT 20000
#define DEFAULT_MAX_NEIGHBOR_COUNT 15
#define DIMENSION 3

using namespace std;

 class DLL_EXPORT ArrtsParams
 {
   int _minNodeCount, _maxNeighborCount;
   double _goalRadius, _obstacleVolume;
   State _start, _goal;
   Rectangle _limits;
   Vehicle _vehicle;
   vector<Shape3d*> _obstacles;

   void _setLimitsFromStates();
   void _removeObstaclesNotInLimits();
   void _calculateObstacleVolume();
   void _readStatesFromFile(string fileName, bool isOptional = false);
   void _readVehicleFromFile(string fileName, bool isOptional = false);
   void _readObstaclesFromFile(string fileName, bool isOptional = false);

   public:
      ArrtsParams(State start, State goal, vector<Shape3d*> obstacles, double goalRadius, int minNodeCount = DEFAULT_MIN_NODE_COUNT, int maxNieghborCount = DEFAULT_MAX_NEIGHBOR_COUNT);
      ArrtsParams(string dataDirectory, int minNodeCount = DEFAULT_MIN_NODE_COUNT, int maxNieghborCount = DEFAULT_MAX_NEIGHBOR_COUNT);

      int dimension();
      int minNodeCount();
      int maxNeighborCount();
      double goalRadius();
      double obstacleVolume();
      State start();
      State goal();
      Rectangle limits();
      Vehicle vehicle();
      vector<Shape3d*>& obstacles();
      Shape3d* const obstacles(int i);
 };

 #endif //ARRTS_PARAMS_H