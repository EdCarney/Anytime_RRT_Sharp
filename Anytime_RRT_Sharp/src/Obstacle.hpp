#include <math.h>
#include "Geometry.hpp"

#ifndef OBSTACLE_H
#define OBSTACLE_H

class Obstacle : public Circle
{
    void _buildObstacle();

    public:
        Obstacle();
        Obstacle(double x, double y, double radius);
        Obstacle(Point p, double radius);
        bool intersects(Point p);
        bool intersects(Line l);
        bool intersects(Circle c);
        bool intersects(Rectangle r);
};

#endif