#include <math.h>
#include <vector>
#include "Geometry2D.hpp"
#include "cppshrhelp.hpp"

#ifndef GEOMETRY_3D_H
#define GEOMETRY_3D_H

using namespace std;

struct Shape3d
{
    virtual double volume() const { return 0; };
    virtual bool intersects(Point& p) const { return true; };
    virtual bool intersects(Line& l) const { return true; };
    virtual bool intersects(Shape3d& s) const { return true; };
};

class DLL_EXPORT Rectangle: public Shape3d
{
    protected:
        double _volume;
        Point _minPoint, _maxPoint;
        vector<Plane> _surfaces;
        vector<Point> _points;
        double _calculateVolume() const;
        vector<Plane> _calculateSurfaces() const;
        vector<Point> _calculatePoints() const;
        bool _intersectsRectangle(Rectangle rect) const;

    public:
        Rectangle();
        Rectangle(Point minPoint, Point maxPoint);
        Rectangle(double minX, double minY, double minZ, double maxX, double maxY, double maxZ);
        bool intersects(Point& p) const;
        bool intersects(Line& l) const;
        bool intersects(Shape3d& s) const;
        const vector<Plane>& surfaces() const;
        const Plane& surfaces(int i) const;
        const vector<Point>& points() const;
        const Point& points(int i) const;
        Point minPoint() const;
        Point maxPoint() const;
        double volume() const;
        double minX() const;
        double minY() const;
        double minZ() const;
        double maxX() const;
        double maxY() const;
        double maxZ() const;
};

class DLL_EXPORT Sphere : public Point, public Shape3d
{
    protected:
        double _radius;
        double _area;
        double _calculateVolume() const;
        bool _intersectsRectanlge(Rectangle rect) const;

    public:
        Sphere();
        Sphere(Point p, double radius);
        Sphere(double x, double y, double z, double radius);
        bool intersects(Point& p) const;
        bool intersects(Line& l) const;
        bool intersects(Shape3d& s) const;
        double radius() const;
        double volume() const;
};

#endif //GEOMETRY_3D_H