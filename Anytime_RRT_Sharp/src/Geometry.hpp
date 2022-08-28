#include "cppshrhelp.hpp"

#ifndef GEOMETRY_H
#define GEOMETRY_H

class DLL_EXPORT Point
{
    protected:
        double _x, _y;

    public:
        Point();
        Point(double x, double y);
        double x();
        double y();
};

class DLL_EXPORT Line
{
    protected:
        Point _p1, _p2;

    public:
        Line();
        Line(Point p1, Point p2);
        Point p1();
        Point p2();
};

class DLL_EXPORT Circle : public Point
{
    protected:
        double _radius;

    public:
        Circle();
        Circle(Point p, double radius);
        Circle(double x, double y, double radius);
        double radius();
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
        void _buildGraphNode();
        void _buildGraphNode(GraphNode n);
        void _buildGraphNode(Point p, int id, int parentId);
        void _buildGraphNode(double x, double y, int id, int parentId);

    public:
        GraphNode();
        GraphNode(Point p, int id, int parentId);
        GraphNode(double x, double y, int id, int parentId);
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