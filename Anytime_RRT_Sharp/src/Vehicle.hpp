#include <cstddef>
#include <math.h>
#include "Geometry.hpp"
#include "Utility.hpp"

#ifndef VEHICLE_H
#define VEHICLE_H

class Vehicle
{
    int numNodes;                   // number of nodes representing the vehicle
    Node* nodes;                    // an array of all nodes for the vehicle
    Node* offsetNodes;              // an array of offset values loaded on vehicle initialization
    Node centroid;                  // centroid of vehicle relative to the offset nodes
    State state;                    // state (x, y, theta) of the vehicle
    double boundingRadius;          // the radius of a ball circumscribing the vehicle; used for quick collision checking
    void updateOffsetParams();      // updates centroid and bounding radius from offset nodes
    void calculateBoundingRadius();
    void calculateCentroid();
    void buildVehicle();

    public:
        Vehicle();
        Vehicle(const double* x, const double *y, int numPoints);
        void UpdateState(State newState);
        void AddOffsetNode(double x, double y);
        void AddOffsetNodes(const double* x, const double *y, int numPoints);
        void AddOffsetNodesFromFile(FILE* file);
        Node* GetNodes();
        Node GetNode(int i);
        Node* GetOffsetNodes();
        Node GetOffsetNode(int i);
        State GetState();
        int GetNumNodes();
        double GetBoundingRadius();

};

#endif