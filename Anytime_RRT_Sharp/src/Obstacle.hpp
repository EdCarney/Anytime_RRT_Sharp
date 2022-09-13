#include <math.h>
#include "cppshrhelp.hpp"
#include "Geometry.hpp"

#ifndef OBSTACLE_H
#define OBSTACLE_H

struct IsObstacle
{
    virtual bool intersects(Point p) const = 0;
    virtual bool intersects(Line l) const = 0;
    virtual bool intersects(Rectangle r) const = 0;
};

class DLL_EXPORT SphereObstacle : public Sphere, public IsObstacle
{
    using Sphere::Sphere;
    void _buildSphereObstacle();

    public:
        bool intersects(Point p) const;
        bool intersects(Line l) const;
        bool intersects(Rectangle r) const;
};

class DLL_EXPORT RectangleObstacle : public Rectangle, public IsObstacle
{
    using Rectangle::Rectangle;
    void _buildRectangleObstacle();

    public:
        bool intersects(Point p) const;
        bool intersects(Line l) const;
        bool intersects(Rectangle r) const;
};

#endif