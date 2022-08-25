#include <math.h>
#include "Geometry.hpp"

#ifndef OBSTACLE_H
#define OBSTACLE_H

class Obstacle : public Circle
{
    void buildObstacle();

    public:
        Obstacle();
        Obstacle(double x, double y, double r);
        Obstacle(Point pos, double r);
        bool Intersects(Point position);
        bool Intersects(Circle circle);
        bool Intersects(Rectangle rect);
        double GetX();
        double GetY();
        double GetRadius();

};

#endif