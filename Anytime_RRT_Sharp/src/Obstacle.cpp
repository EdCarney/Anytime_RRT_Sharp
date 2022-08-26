#include "Obstacle.hpp"

Obstacle::Obstacle()
{
    _buildObstacle();
}

Obstacle::Obstacle(double x, double y, double radius)
{
    _buildObstacle();
    _x = x;
    _y = y;
    _radius = radius;
}

Obstacle::Obstacle(Point pos, double radius)
{
    _buildObstacle();
    _x = pos.x();
    _y = pos.y();
    _radius = radius;
}

void Obstacle::_buildObstacle()
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