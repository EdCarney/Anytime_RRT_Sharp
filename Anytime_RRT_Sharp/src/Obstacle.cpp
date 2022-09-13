#include "Obstacle.hpp"

bool SphereObstacle::intersects(Point point) const
{
    double dist = this->distanceTo(point);
    return dist <= _radius;
}

bool SphereObstacle::intersects(Line line) const
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

bool SphereObstacle::intersects(Rectangle rect) const
{
    double minX = _x - _radius;
    double maxX = _x + _radius;
    double minY = _y - _radius;
    double maxY = _y + _radius;
    double minZ = _z - _radius;
    double maxZ = _z + _radius;

    bool inXLimits = minX < rect.maxX() && maxX > rect.minX();
    bool inYLimits = minY < rect.maxY() && maxY > rect.minY();
    bool inZLimits = minZ < rect.maxZ() && maxZ > rect.minZ();

    return inXLimits && inYLimits && inZLimits;
}

bool RectangleObstacle::intersects(Point point) const
{
    bool inXLimits = point.x() >= minX() && point.x() <= maxX();
    bool inYLimits = point.y() >= minY() && point.y() <= maxY();
    bool inZLimits = point.z() >= minZ() && point.z() <= maxZ();
    return inXLimits && inYLimits && inZLimits;
}

bool RectangleObstacle::intersects(Line line) const
{
    
}

bool RectangleObstacle::intersects(Rectangle rect) const
{
    bool minInXLimits = (rect.minX() >= minX() && rect.minX() <= maxX());
    bool minInYLimits = (rect.minY() >= minY() && rect.minY() <= maxY());
    bool minInZLimits = (rect.minZ() >= minZ() && rect.minZ() <= maxZ());

    bool maxInXLimits = (rect.maxX() >= minX() && rect.maxX() <= maxX());
    bool maxInYLimits = (rect.maxY() >= minY() && rect.maxY() <= maxY());
    bool maxInZLimits = (rect.maxZ() >= minZ() && rect.maxZ() <= maxZ());

    bool inXLimits = minInXLimits || maxInXLimits;
    bool inYLimits = minInYLimits || maxInYLimits;
    bool inZLimits = minInZLimits || maxInZLimits;

    return inXLimits && inYLimits && inZLimits;
}