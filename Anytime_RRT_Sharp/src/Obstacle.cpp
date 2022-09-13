#include "Obstacle.hpp"

bool SphereObstacle::intersects(Point point) const
{
    double dist = this->distanceTo(point);
    return dist <= _radius;
}

bool SphereObstacle::intersects(Line line) const
{
    // http://paulbourke.net/geometry/circlesphere/index.html#linesphere
    Point p1 = line.p1();
    Vector u = line.tangent();

    double a = u.x()*u.x() + u.y()*u.y() + u.z()*u.z();
    double b = 2 * (u.x() * (p1.x() - x()) + u.y() * (p1.y() - y()) + u.z() * (p1.z() - z()));
    double c = x()*x() + y()*y() + z()*z();

    c += p1.x()*p1.x() + p1.y()*p1.y() + p1.z()*p1.z();
    c -= 2 * (x()*p1.x() + y()*p1.y() + z()*p1.z());
    c -= radius()*radius();

    double det = b*b - 4*a*c;

    // no intersection
    if (det < 0)
        return false;

    // get intersection points
    double mu1 = (-b + sqrt(det)) / (2 * a);
    double mu2 = (-b - sqrt(det)) / (2 * a);

    Point pi1 = line.p1() + u * mu1;
    Point pi2 = line.p1() + u * mu2;

    // detmine if either point lies on line segment
    // https://lucidar.me/en/mathematics/check-if-a-point-belongs-on-a-line-segment/
    Vector kac1 = pi1 - line.p1();
    Vector kac2 = pi2 - line.p1();

    double dotU = u.dot(u);
    double dotKac1 = kac1.dot(u);
    double dotKac2 = kac2.dot(u);

    if ((dotKac1 >= 0 && dotKac1 <= dotU) || (dotKac2 >= 0 && dotKac2 <= dotU))
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
    // https://stackoverflow.com/questions/66523293/intersection-of-line-with-rectangular-prism-python
    Point* p;
    double epsilon = 0.001;
    bool inXLimits, inYLimits, inZLimits;

    for (Plane plane : surfaces())
    {
        *p = plane.getIntersectionPoint(line);

        if (p == NULL)
            break;
        
        inXLimits = p->x() + epsilon >= minX() && p->x() - epsilon <= maxX();
        inYLimits = p->y() + epsilon >= minY() && p->y() - epsilon <= maxY();
        inZLimits = p->z() + epsilon >= minZ() && p->z() - epsilon <= maxZ();

        if (inXLimits && inYLimits && inZLimits)
            return true;
    }

    return false;
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