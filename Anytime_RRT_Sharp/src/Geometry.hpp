#include <float.h>
#include <math.h>
#include <vector>
#include "cppshrhelp.hpp"

#ifndef GEOMETRY_H
#define GEOMETRY_H

using namespace std;

struct HasVolume
{
    virtual double volume() const = 0;
};

class DLL_EXPORT Vector
{
    protected:
        double _x, _y, _z, _magnitude;

    public:
        Vector();
        Vector(double x, double y, double z);
        Vector operator*(double val) const;
        double x() const;
        double y() const;
        double z() const;
        double magnitude() const;
        double dot(const Vector& v) const;
};

class DLL_EXPORT Point
{
    protected:
        double _x, _y, _z;

    public:
        Point();
        Point(double x, double y, double z);
        Point operator-(const Vector& v) const;
        Point operator+(const Vector& v) const;
        Vector operator-(const Point& p) const;
        double x() const;
        double y() const;
        double z() const;
        double distanceTo(Point& p) const;
};

class DLL_EXPORT Line
{
    protected:
        Point _p1, _p2;
        Vector _tangent;
        double _length;

    public:
        Line();
        Line(Point& p1, Point& p2);
        Point p1() const;
        Point p2() const;
        Vector tangent() const;
        double length() const;
};

class DLL_EXPORT Plane
{
    Point _point;
    Vector _normal;

    public:
        Plane();
        Plane(Point p, Vector normal);
        Point point() const;
        Vector normal() const;
        Point getIntersectionPoint(Line line) const;
};

class DLL_EXPORT Sphere : public Point, public HasVolume
{
    protected:
        double _radius;
        double _area;
        double _calculateVolume() const;

    public:
        Sphere();
        Sphere(Point p, double radius);
        Sphere(double x, double y, double z, double radius);
        double radius() const;
        double volume() const;
};

class DLL_EXPORT Rectangle: public HasVolume
{
    protected:
        double _volume;
        Point _minPoint, _maxPoint;
        vector<Plane> _surfaces;
        double _calculateVolume() const;
        vector<Plane> _calculateSurfaces() const;

    public:
        Rectangle();
        Rectangle(Point minPoint, Point maxPoint);
        Rectangle(double minX, double minY, double minZ, double maxX, double maxY, double maxZ);
        Point minPoint() const;
        Point maxPoint() const;
        const vector<Plane>& surfaces() const;
        const Plane& surfaces(int i) const;
        double volume() const;
        double minX() const;
        double minY() const;
        double minZ() const;
        double maxX() const;
        double maxY() const;
        double maxZ() const;
};

class DLL_EXPORT GoalState : public Point
{
    double _theta, _radius;

    public:
        GoalState();
        GoalState(double x, double y, double z, double radius, double theta);
        double theta() const;
        double radius() const;
};

class DLL_EXPORT GraphNode : public Point
{
    protected:
        int _id, _parentId;
        double _theta;
        void _buildGraphNode();
        void _buildGraphNode(GraphNode n);
        void _buildGraphNode(Point p, double theta, int id, int parentId);
        void _buildGraphNode(double x, double y, double z, double theta, int id, int parentId);

    public:
        GraphNode();
        GraphNode(Point p, double theta, int id, int parentId);
        GraphNode(double x, double y, double z, double theta, int id, int parentId);
        double theta() const;
        void setTheta(double theta);
        int id() const;
        void setId(int id);
        int parentId() const;
        void setParentId(int parentId);
};

class DLL_EXPORT State : public Point
{
    protected:
        double _theta;

    public:
        State();
        State(GraphNode node);
        State(Point p, double theta);
        State(double x, double y, double z, double theta);
        double theta() const;
};

class DLL_EXPORT Edge
{
    protected:
        GraphNode _start;
        GraphNode _end;

    public:
        Edge();
        Edge(GraphNode start, GraphNode end);
        GraphNode start();
        GraphNode end();
};

#endif