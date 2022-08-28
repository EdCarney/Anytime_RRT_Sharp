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
    // line slope
    double m = (l.p2().y() - l.p1().y()) / (l.p2().x() - l.p1().x());

    // standard form values
    double a = 1;
    double b = -m;
    double c = l.p1().y() - m * l.p1().x();

    // get perpendicular distance from circle center
    // to line (https://en.wikipedia.org/wiki/Distance_from_a_point_to_a_line)
    double dist = abs(a * _x + b * _y + c) / sqrt(a * a + b * b);

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