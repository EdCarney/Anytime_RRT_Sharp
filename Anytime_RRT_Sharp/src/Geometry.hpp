#include "cppshrhelp.hpp"

#ifndef GEOMETRY_H
#define GEOMETRY_H

struct DLL_EXPORT Point { double x, y; };
struct DLL_EXPORT Circle : Point { double radius; };
struct DLL_EXPORT Rectangle { Point minPoint, maxPoint; };
struct DLL_EXPORT State : Point { double theta; };
struct DLL_EXPORT GoalState : Circle { double theta; };
struct DLL_EXPORT GraphNode: Point { int id, parentNodeId; };

#endif