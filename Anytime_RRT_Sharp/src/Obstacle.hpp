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
        bool Intersects(Point p);
        bool Intersects(Circle c);
        bool Intersects(Rectangle r);
};

#endif