#include "Obstacle.hpp"

bool approximatelyEqual(double val1, double val2, double delta = 0.001)
{
    return abs(val1 - val2) < delta;
}

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
    double dist = this->distanceTo(point);
    return dist <= _radius;
}

bool Obstacle::intersects(Line line)
{
    //https://mathworld.wolfram.com/Circle-LineIntersection.html
    // center about circle
    double x1 = line.p1().x() - _x;
    double x2 = line.p2().x() - _x;
    double y1 = line.p1().y() - _y;
    double y2 = line.p2().y() - _y;

    double dx = x2 - x1;
    double dy = y2 - y1;
    double dr = hypot(dx, dy);
    double D = x1*y2 - x2*y1;

    double det = _radius*_radius * dr*dr - D*D;

    // no intersection
    if (det < 0)
        return false;

    // get intersection points
    int sgn_dy = dy < 0 ? -1 : 1;

    double xi1 = ((D*dy + sgn_dy*dx*sqrt(det)) / (dr*dr)) + _x;
    double xi2 = ((D*dy - sgn_dy*dx*sqrt(det)) / (dr*dr)) + _x;
    double yi1 = ((-D*dx + abs(dy)*sqrt(det)) / (dr*dr)) + _y;
    double yi2 = ((-D*dx - abs(dy)*sqrt(det)) / (dr*dr)) + _y;

    // detmine if either point lies on line segment
    // https://lucidar.me/en/mathematics/check-if-a-point-belongs-on-a-line-segment/
    double kac1 = line.dotProduct(Line(line.p1(), Point(xi1, yi1)));
    double kac2 = line.dotProduct(Line(line.p1(), Point(xi2, yi2)));
    double kab = line.dotProduct(line);

    if (approximatelyEqual(kac1, 0) || approximatelyEqual(kac2, 0) || approximatelyEqual(kac1, kab) || approximatelyEqual(kac2, kab))
    {
        // at least one intersection point is coincident
        return true;
    }
    else if ((kac1 > 0 && kac1 < kab) || (kac2 > 0 && kac2 < kab))
    {
        // at least one intersection point lies on line segment
        return true;
    }
    
    return false;
}

bool Obstacle::intersects(Circle circle)
{
    double dist = this->distanceTo(circle);
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