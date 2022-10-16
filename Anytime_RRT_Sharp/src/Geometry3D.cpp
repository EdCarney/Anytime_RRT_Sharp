#include "Geometry3D.hpp"

Sphere::Sphere()
{
    _x = 0;
    _y = 0;
    _z = 0;
    _radius = 0;
    _area = 0;
}

Sphere::Sphere(Point p, double radius)
{
    _x = p.x();
    _y = p.y();
    _z = p.z();
    _radius = radius;
    _area = _calculateVolume();
}

Sphere::Sphere(double x, double y, double z, double radius)
{
    _x = x;
    _y = y;
    _z = z;
    _radius = radius;
    _area = _calculateVolume();
}

double Sphere::_calculateVolume() const
{
    return (4.0/3.0) * M_PI * pow(radius(), 3);
}

bool Sphere::_intersectsRectanlge(Rectangle rect) const
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

bool Sphere::intersects(const Point& point) const
{
    double dist = this->distanceTo(point);
    return dist <= _radius;
}

bool Sphere::intersects(const Line& line) const
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

bool Sphere::intersects(const Shape3d& shape) const
{
    auto r = dynamic_cast<const Rectangle*>(&shape);
    if (r != nullptr)
        return _intersectsRectanlge(*r);
    throw new runtime_error("Unknown shape type in intersects()");
}

double Sphere::radius() const { return _radius; }

double Sphere::volume() const { return _area; }

Rectangle::Rectangle()
{
    _minPoint = Point();
    _maxPoint = Point();
    _volume = 0;
}

Rectangle::Rectangle(Point minPoint, Point maxPoint)
{
    _minPoint = minPoint;
    _maxPoint = maxPoint;
    _volume = _calculateVolume();
    _surfaces = _calculateSurfaces();
    _points = _calculatePoints();
}

Rectangle::Rectangle(double minX, double minY, double minZ, double maxX, double maxY, double maxZ)
{
    _minPoint = Point(minX, minY, minZ);
    _maxPoint = Point(maxX, maxY, maxZ);
    _volume = _calculateVolume();
    _surfaces = _calculateSurfaces();
    _points = _calculatePoints();
}

double Rectangle::_calculateVolume() const
{
    double xDiff = maxX() - minX();
    double yDiff = maxY() - minY();
    double zDiff = maxZ() - minZ();
    return xDiff * yDiff * zDiff;
}

vector<Plane> Rectangle::_calculateSurfaces() const
{
    vector<Plane> surfaces(6);
    surfaces[0] = Plane(Point(minX(), 0, 0), Vector(1, 0, 0));
    surfaces[1] = Plane(Point(maxX(), 0, 0), Vector(1, 0, 0));
    surfaces[2] = Plane(Point(0, minY(), 0), Vector(0, 1, 0));
    surfaces[3] = Plane(Point(0, maxY(), 0), Vector(0, 1, 0));
    surfaces[4] = Plane(Point(0, 0, minZ()), Vector(0, 0, 1));
    surfaces[5] = Plane(Point(0, 0, maxZ()), Vector(0, 0, 1));
    return surfaces;
}

vector<Point> Rectangle::_calculatePoints() const
{
    vector<Point> points(8);
    points[0] = Point(minX(), minY(), minZ());
    points[1] = Point(maxX(), minY(), minZ());
    points[2] = Point(minX(), maxY(), minZ());
    points[3] = Point(minX(), minY(), maxZ());
    points[4] = Point(maxX(), maxY(), minZ());
    points[5] = Point(minX(), maxY(), maxZ());
    points[6] = Point(maxX(), minY(), maxZ());
    points[7] = Point(maxX(), maxY(), maxZ());
    return points;
}

bool Rectangle::_intersectsRectangle(Rectangle rect) const
{
    for (Point p : points())
        if (rect.intersects(p))
            return true;
    return false;
}

bool Rectangle::intersects(const Point& point) const
{
    bool inXLimits = point.x() >= minX() && point.x() <= maxX();
    bool inYLimits = point.y() >= minY() && point.y() <= maxY();
    bool inZLimits = point.z() >= minZ() && point.z() <= maxZ();
    return inXLimits && inYLimits && inZLimits;
}

bool Rectangle::intersects(const Line& line) const
{
    // https://stackoverflow.com/questions/66523293/intersection-of-line-with-rectangular-prism-python
    Point p;
    double epsilon = 0.001;
    bool inXLimits, inYLimits, inZLimits, inLineLimits;

    for (Plane plane : surfaces())
    {
        p = plane.getIntersectionPoint(line);

        if (&p == NULL)
            break;

        inXLimits = p.x() + epsilon >= minX() && p.x() - epsilon <= maxX();
        inYLimits = p.y() + epsilon >= minY() && p.y() - epsilon <= maxY();
        inZLimits = p.z() + epsilon >= minZ() && p.z() - epsilon <= maxZ();
        inLineLimits = line.p1().distanceTo(p) <= line.length() && line.p2().distanceTo(p) <= line.length();

        if (inXLimits && inYLimits && inZLimits && inLineLimits)
            return true;
    }

    return false;
}

bool Rectangle::intersects(const Shape3d& shape) const
{
    const Rectangle* r = dynamic_cast<const Rectangle*>(&shape);
    if (r)
        return _intersectsRectangle(*r);
    throw new runtime_error("Unknown shape type in intersects()");
}

const vector<Plane>& Rectangle::surfaces() const { return _surfaces; }

const Plane& Rectangle::surfaces(int i) const { return _surfaces[i]; }

const vector<Point>& Rectangle::points() const { return _points; }

const Point& Rectangle::points(int i) const { return _points[i]; }

Point Rectangle::minPoint() const { return _minPoint; }

Point Rectangle::maxPoint() const { return _maxPoint; }

double Rectangle::volume() const { return _volume; }

double Rectangle::minX() const { return _minPoint.x(); }

double Rectangle::minY() const { return _minPoint.y(); }

double Rectangle::minZ() const { return _minPoint.z(); }

double Rectangle::maxX() const { return _maxPoint.x(); }

double Rectangle::maxY() const { return _maxPoint.y(); }

double Rectangle::maxZ() const { return _maxPoint.z(); }