#include "Geometry.hpp"

Point::Point()
{
    x = 0;
    y = 0;
}

Point::Point(double xVal, double yVal)
{
    x = xVal;
    y = yVal;
}

double Point::GetX()
{
    return x;
}

double Point::GetY()
{
    return y;
}

Circle::Circle()
{
    x = 0;
    y = 0;
    radius = 0;
}

Circle::Circle(Point p, double r)
{
    x = p.GetX();
    y = p.GetY();
    radius = r;
}

Circle::Circle(double xVal, double yVal, double rVal)
{
    x = xVal;
    y = yVal;
    radius = rVal;
}

double Circle::GetRadius()
{
    return radius;
}

Rectangle::Rectangle()
{
    minPoint = Point();
    maxPoint = Point();
}

Rectangle::Rectangle(Point minP, Point maxP)
{
    minPoint = minP;
    maxPoint = maxP;
}

Rectangle::Rectangle(double minX, double minY, double maxX, double maxY)
{
    minPoint = Point(minX, minY);
    maxPoint = Point(maxX, maxY);
}

Point Rectangle::GetMinPoint()
{
    return minPoint;
}

Point Rectangle::GetMaxPoint()
{
    return maxPoint;
}

double Rectangle::GetMinX()
{
    return minPoint.GetX();
}

double Rectangle::GetMinY()
{
    return minPoint.GetY();
}

double Rectangle::GetMaxX()
{
    return maxPoint.GetX();
}

double Rectangle::GetMaxY()
{
    return maxPoint.GetY();
}

State::State()
{
    x = 0;
    y = 0;
    theta = 0;
}

State::State(Point p, double thetaVal)
{
    x = p.GetX();
    y = p.GetY();
    theta = thetaVal;
}

State::State(double xVal, double yVal, double thetaVal)
{
    x = xVal;
    y = yVal;
    theta = thetaVal;
}

double State::GetTheta()
{
    return theta;
}

GoalState::GoalState()
{
    x = 0;
    y = 0;
    radius = 0;
    theta = 0;
}

GoalState::GoalState(double xVal, double yVal, double rVal, double thetaVal)
{
    x = xVal;
    y = yVal;
    radius = rVal;
    theta = thetaVal;
}

double GoalState::GetTheta()
{
    return theta;
}

double GoalState::GetRadius()
{
    return radius;
}

GraphNode::GraphNode()
{
    buildGraphNode();
}

GraphNode::GraphNode(Point p, int idVal, int parentIdVal)
{
    buildGraphNode(p, idVal, parentIdVal);
}

GraphNode::GraphNode(double xVal, double yVal, int idVal, int parentIdVal)
{
    buildGraphNode(xVal, yVal, idVal, parentIdVal);
}

void GraphNode::buildGraphNode()
{
    x = 0;
    y = 0;
    id = 0;
    parentNodeId = 0;
}

void GraphNode::buildGraphNode(Point p, int idVal, int parentId)
{
    x = p.GetX();
    y = p.GetY();
    id = idVal;
    parentNodeId = parentId;
}

void GraphNode::buildGraphNode(double xVal, double yVal, int idVal, int parentId)
{
    x = xVal;
    y = yVal;
    id = idVal;
    parentNodeId = parentId;
}

int GraphNode::GetId()
{
    return id;
}

int GraphNode::GetParentId()
{
    return parentNodeId;
}

void GraphNode::SetId(int idVal)
{
    id = idVal;
}

void GraphNode::SetParentId(int parentIdVal)
{
    parentNodeId = parentIdVal;
}