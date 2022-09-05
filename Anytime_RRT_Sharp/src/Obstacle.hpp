#include <math.h>
#include "cppshrhelp.hpp"
#include "Geometry.hpp"

#ifndef OBSTACLE_H
#define OBSTACLE_H

class DLL_EXPORT Obstacle : public Circle
{
    using Circle::Circle;

    void _buildObstacle();

    public:
        bool intersects(Point p);
        bool intersects(Line l);
        bool intersects(Circle c);
        bool intersects(Rectangle r);
};

#endif