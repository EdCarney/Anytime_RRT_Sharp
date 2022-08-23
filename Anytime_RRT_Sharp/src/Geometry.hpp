#include "cppshrhelp.hpp"

#ifndef GEOMETRY_H
#define GEOMETRY_H

struct DLL_EXPORT Node { double x, y; };

struct DLL_EXPORT Obstacle : Node { double radius; };
struct DLL_EXPORT Position : Node { double theta; };

struct DLL_EXPORT GraphNode: Node { int id, parentNodeId; };

#endif