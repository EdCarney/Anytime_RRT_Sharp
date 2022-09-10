#include "Obstacle.hpp"

bool Obstacle::intersects(Point point)
{
    double dist = this->distanceTo(point);
    return dist <= _radius;
}

bool Obstacle::intersects(Line line)
{
    // http://paulbourke.net/geometry/circlesphere/index.html#linesphere
    double x1 = line.p1().x();
    double y1 = line.p1().y();
    double z1 = line.p1().z();
    double x2 = line.p2().x();
    double y2 = line.p2().y();
    double z2 = line.p2().z();

    double dx = x2 - x1;
    double dy = y2 - y1;
    double dz = z2 - z1;

    double a = dx*dx + dy*dy + dz*dz;
    double b = 2 * (dx * (x1 - x()) + dy * (y1 - y()) + dz * (z1 - z()));
    double c = x()*x() + y()*y() + z()*z();

    c += x1*x1 + y1*y1 + z1*z1;
    c -= 2 * (x()*x1 + y()*y1 + z()*z1);
    c -= radius()*radius();

    double det = b*b - 4*a*c;

    // no intersection
    if (det < 0)
        return false;

    // get intersection points
    double u1 = (-b + sqrt(det)) / (2 * a);
    double u2 = (-b - sqrt(det)) / (2 * a);

    double xi1 = x1 + u1 * (x2 - x1);
    double yi1 = y1 + u1 * (y2 - y1);
    double zi1 = z1 + u1 * (z2 - z1);

    double xi2 = x1 + u2 * (x2 - x1);
    double yi2 = y1 + u2 * (y2 - y1);
    double zi2 = z1 + u2 * (z2 - z1);

    // detmine if either point lies on line segment
    // https://lucidar.me/en/mathematics/check-if-a-point-belongs-on-a-line-segment/
    Vector kab(x2 - x1, y2 - y1, z2 - z1);
    Vector kac1(xi1 - x1, yi1 - y1, zi1 - z1);
    Vector kac2(xi2 - x1, yi2 - y1, zi2 - z1);

    double dotKab = kab.dot(kab);
    double dotKac1 = kac1.dot(kab);
    double dotKac2 = kac2.dot(kab);

    if ((dotKac1 >= 0 && dotKac1 <= dotKab) || (dotKac2 >= 0 && dotKac2 <= dotKab))
        return true;
    
    return false;
}

bool Obstacle::intersects(Sphere circle)
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