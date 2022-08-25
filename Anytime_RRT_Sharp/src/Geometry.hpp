#include "cppshrhelp.hpp"

#ifndef GEOMETRY_H
#define GEOMETRY_H

class DLL_EXPORT Point
{
    protected:
        double x, y;

    public:
        Point();
        Point(double xVal, double yVal);
        double GetX();
        double GetY();
};

class DLL_EXPORT Circle : public Point
{
    protected:
        double radius;

    public:
        Circle();
        Circle(Point p, double r);
        Circle(double xVal, double yVal, double rVal);
        double GetRadius();
};

class DLL_EXPORT Rectangle
{
    Point minPoint, maxPoint;

    public:
        Rectangle();
        Rectangle(Point minP, Point maxP);
        Rectangle(double minX, double minY, double maxX, double maxY);
        Point GetMinPoint();
        Point GetMaxPoint();
        double GetMinX();
        double GetMinY();
        double GetMaxX();
        double GetMaxY();
};

class DLL_EXPORT State : public Point
{
    protected:
        double theta;

    public:
        State();
        State(Point p, double thetaVal);
        State(double xVal, double yVal, double thetaVal);
        double GetTheta();
};

class DLL_EXPORT GoalState : public Point
{
    double theta, radius;

    public:
        GoalState();
        GoalState(double xVal, double yVal, double rVal, double thetaVal);
        double GetTheta();
        double GetRadius();
};

class DLL_EXPORT GraphNode : public Point
{
    protected:
        int id, parentNodeId;
        void buildGraphNode();
        void buildGraphNode(Point p, int idVal, int parentId);
        void buildGraphNode(double xVal, double yVal, int idVal, int parentIdVal);

    public:
        GraphNode();
        GraphNode(Point p, int idVal, int parentId);
        GraphNode(double xVal, double yVal, int idVal, int parentIdVal);
        int GetId();
        void SetId(int idVal);
        int GetParentId();
        void SetParentId(int parentIdVal);
};

#endif