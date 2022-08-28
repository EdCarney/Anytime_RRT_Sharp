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

bool Obstacle::intersects(Point point)
{
    double dist = hypot(point.x() - _x, point.y() - _y);
    return dist <= _radius;
}

bool Obstacle::intersects(Line l)
{
    // get perpendicular distance from circle center
    // to line (https://en.wikipedia.org/wiki/Distance_from_a_point_to_a_line)
    double dist = abs(l.a() * _x + l.b() * _y + l.c()) / sqrt(l.a() * l.a() + l.b() * l.b());
    return dist <= _radius;
}

bool Obstacle::intersects(Circle circle)
{
    double dist = hypot(circle.x() - _x, circle.y() - _y);
    return dist <= _radius + circle.radius();
}

bool Obstacle::intersects(Rectangle rect)
{
    double minX = _x - _radius;
    double maxX = _x + _radius;
    double minY = _y - _radius;
    double maxY = _y + _radius;

    bool inXLimits = minX < rect.maxX() && maxX > rect.minX();
    bool inYLimits = minY < rect.maxY() && maxY > rect.minY();

    return inXLimits && inYLimits;
}