#include "Obstacle.hpp"

Obstacle::Obstacle()
{
    buildObstacle();
}

Obstacle::Obstacle(double xVal, double yVal, double r)
{
    buildObstacle();
    _x = xVal;
    _y = yVal;
    _radius = r;
}

Obstacle::Obstacle(Point pos, double r)
{
    buildObstacle();
    _x = pos.x();
    _y = pos.y();
    _radius = r;
}

void Obstacle::buildObstacle()
{
    _x = 0.0;
    _y = 0.0;
    _radius = 0.0;
}

bool Obstacle::Intersects(Point point)
{
    double dist = hypot(point.x() - _x, point.y() - _y);
    return dist <= _radius;
}

bool Obstacle::Intersects(Circle circle)
{
    double dist = hypot(circle.x() - _x, circle.y() - _y);
    return dist <= _radius + circle.radius();
}

bool Obstacle::Intersects(Rectangle rect)
{
    double minX = _x - _radius;
    double maxX = _x + _radius;
    double minY = _y - _radius;
    double maxY = _y + _radius;

    bool inXLimits = minX < rect.maxX() && maxX > rect.minX();
    bool inYLimits = minY < rect.maxY() && maxY > rect.minY();

    return inXLimits && inYLimits;
}

double Obstacle::GetX()
{
    return _x;
}

double Obstacle::GetY()
{
    return _y;
}

double Obstacle::GetRadius()
{
    return _radius;
}