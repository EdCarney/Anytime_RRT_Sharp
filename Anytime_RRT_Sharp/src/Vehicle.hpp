#include <cstddef>
#include <math.h>
#include "Geometry.hpp"

#ifndef VEHICLE_H
#define VEHICLE_H

class Vehicle
{
    public:
        int numNodes;               // number of nodes representing the vehicle
        Node* nodes;                // an array of all nodes for the vehicle
        Node* offsetNodes;          // an array of offset values loaded on vehicle initialization
        State state;                // state of the vehicle
        double maxPointRadius;      // the radius of a ball circumscribing the vehicle; used for quick collision checking
        Vehicle();
        void updateState(State newState);
};

#endif