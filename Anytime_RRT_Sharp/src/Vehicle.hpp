#include <fstream>
#include <math.h>
#include <sstream>
#include <string>
#include <vector>
#include "cppshrhelp.hpp"
#include "Geometry2D.hpp"
#include "Geometry3D.hpp"

using namespace std;

#ifndef VEHICLE_H
#define VEHICLE_H

class DLL_EXPORT Vehicle
{
    vector<Point> _nodes;       // an array of all nodes for the vehicle
    vector<Point> _offsetNodes; // an array of offset values loaded on vehicle initialization
    Point _centroid;            // centroid of vehicle relative to the offset nodes
    State _state;               // state (x, y, theta) of the vehicle
    double _boundingRadius;     // the radius of a ball circumscribing the vehicle; used for quick collision checking
    void _updateOffsetParams(); // updates centroid and bounding radius from offset nodes
    void _calculateBoundingRadius();
    void _calculateCentroid();
    void _buildVehicle();

public:
    Vehicle();
    Vehicle(vector<double> x, vector<double> y, vector<double> z);
    Vehicle(string fileName);
    void updateState(const State newState);
    void addOffsetNode(double x, double y, double z);
    void addOffsetNodes(vector<double> x, vector<double> y, vector<double> z);
    void addOffsetNodesFromFile(string fileName);
    vector<Point> nodes();
    Point nodes(int i);
    vector<Point> offsetNodes();
    Point offsetNodes(int i);
    State state() const;
    double boundingRadius() const;
};

#endif