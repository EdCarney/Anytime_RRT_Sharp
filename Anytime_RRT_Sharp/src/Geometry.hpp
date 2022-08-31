#include <float.h>
#include <math.h>
#include "cppshrhelp.hpp"

#ifndef GEOMETRY_H
#define GEOMETRY_H

struct HasArea
{
    virtual double area() = 0;
};

class DLL_EXPORT Point
{
    protected:
        double _x, _y;

    public:
        Point();
        Point(double x, double y);
        double x();
        double y();
        double distanceTo(Point p);
};

class DLL_EXPORT Line
{
    protected:
        Point _p1, _p2;
        double _slope, _length;
        // standard form values
        double _a, _b, _c;

    public:
        Line();
        Line(Point p1, Point p2);
        Point p1();
        Point p2();
        double length();
        double dotProduct(Line line);
};

class DLL_EXPORT Circle : public Point, public HasArea
{
    protected:
        double _radius;
        double _area;
        double _calculateArea();

    public:
        Circle();
        Circle(Point p, double radius);
        Circle(double x, double y, double radius);
        double radius();
        double area();
};

class DLL_EXPORT Rectangle
{
    protected:
        Point _minPoint, _maxPoint;

    public:
        Rectangle();
        Rectangle(Point minPoint, Point maxPoint);
        Rectangle(double minX, double minY, double maxX, double maxY);
        Point minPoint();
        Point maxPoint();
        double minX();
        double minY();
        double maxX();
        double maxY();
};

class DLL_EXPORT State : public Point
{
    protected:
        double _theta;

    public:
        State();
        State(Point p, double theta);
        State(double x, double y, double theta);
        double theta();
};

class DLL_EXPORT GoalState : public Point
{
    double _theta, _radius;

    public:
        GoalState();
        GoalState(double x, double y, double radius, double theta);
        double theta();
        double radius();
};

class DLL_EXPORT GraphNode : public Point
{
    protected:
        int _id, _parentId;
        double _theta;
        void _buildGraphNode();
        void _buildGraphNode(GraphNode n);
        void _buildGraphNode(Point p, double theta, int id, int parentId);
        void _buildGraphNode(double x, double y, double theta, int id, int parentId);

    public:
        GraphNode();
        GraphNode(Point p, double theta, int id, int parentId);
        GraphNode(double x, double y, double theta, int id, int parentId);
        double theta();
        void setTheta(double theta);
        int id();
        void setId(int id);
        int parentId();
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
        GraphNode start();
        GraphNode end();
};

#endif