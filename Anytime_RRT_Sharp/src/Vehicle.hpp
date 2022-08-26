#include <cstddef>
#include <math.h>
#include <vector>
#include "Geometry.hpp"
#include "Utility.hpp"

#ifndef VEHICLE_H
#define VEHICLE_H

class Vehicle
{
    vector<Point> _nodes;            // an array of all nodes for the vehicle
    vector<Point> _offsetNodes;      // an array of offset values loaded on vehicle initialization
    Point _centroid;                 // centroid of vehicle relative to the offset nodes
    State _state;                    // state (x, y, theta) of the vehicle
    double _boundingRadius;          // the radius of a ball circumscribing the vehicle; used for quick collision checking
    void _updateOffsetParams();      // updates centroid and bounding radius from offset nodes
    void _calculateBoundingRadius();
    void _calculateCentroid();
    void _buildVehicle();

    public:
        Vehicle();
        Vehicle(const double* x, const double *y, int numPoints);
        void updateState(State newState);
        void addOffsetNode(double x, double y);
        void addOffsetNodes(const double* x, const double *y, int numPoints);
        void addOffsetNodesFromFile(FILE* file);
        vector<Point> nodes();
        Point nodes(int i);
        vector<Point> offsetNodes();
        Point offsetNodes(int i);
        State state();
        double boundingRadius();

};

#endif