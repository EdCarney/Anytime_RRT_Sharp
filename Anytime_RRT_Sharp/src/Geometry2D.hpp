#include <math.h>
#include <vector>
#include "cppshrhelp.hpp"

#ifndef GEOMETRY_2D_H
#define GEOMETRY_2D_H

using namespace std;

class DLL_EXPORT Vector
{
    protected:
        double _x, _y, _z, _magnitude;

    public:
        Vector();
        Vector(double x, double y, double z);
        Vector operator*(double val) const;
        Vector operator/(double val) const;
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
        double distanceTo(const Point& p) const;
};

class DLL_EXPORT Line
{
    protected:
        Point _p1, _p2;
        Vector _tangent;
        double _length;

    public:
        Line();
        Line(const Point& p1, const Point& p2);
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
        Point getIntersectionPoint(const Line& line) const;
};

class DLL_EXPORT State : public Point
{
    protected:
        double _theta, _rho;

    public:
        State();
        State(Point p, double theta, double rho);
        State(double x, double y, double z, double theta, double rho);
        double theta() const;
        double rho() const;
};

class DLL_EXPORT GoalState : public State
{
    double _radius;

    public:
        GoalState();
        GoalState(double x, double y, double z, double radius, double theta, double rho);
        double radius() const;
};

class DLL_EXPORT GraphNode : public State
{
    protected:
        unsigned long _id, _parentId;
        void _buildGraphNode();
        void _buildGraphNode(GraphNode n);
        void _buildGraphNode(Point p, double theta, double rho, unsigned long id, unsigned long parentId);
        void _buildGraphNode(double x, double y, double z, double theta, double rho, unsigned long id, unsigned long parentId);

    public:
        GraphNode();
        GraphNode(Point p, double theta, double rho, unsigned long id, unsigned long parentId);
        GraphNode(double x, double y, double z, double theta, double rho, unsigned long id, unsigned long parentId);
        int id() const;
        void setId(int id);
        int parentId() const;
        void setParentId(int parentId);
};

class DLL_EXPORT Edge
{
    protected:
        GraphNode _start;
        GraphNode _end;

    public:
        Edge();
        Edge(GraphNode start, GraphNode end);
        const GraphNode& start() const;
        const GraphNode& end() const;
};

#endif //GEOMETRY_2D_H