#include "Geometry.hpp"

Point::Point()
{
    _x = 0;
    _y = 0;
}

Point::Point(double x, double y)
{
    _x = x;
    _y = y;
}

double Point::x()
{
    return _x;
}

double Point::y()
{
    return _y;
}

Circle::Circle()
{
    _x = 0;
    _y = 0;
    _radius = 0;
}

Circle::Circle(Point p, double radius)
{
    _x = p.x();
    _y = p.y();
    _radius = radius;
}

Circle::Circle(double x, double y, double radius)
{
    _x = x;
    _y = y;
    _radius = radius;
}

double Circle::radius()
{
    return _radius;
}

Rectangle::Rectangle()
{
    _minPoint = Point();
    _maxPoint = Point();
}

Rectangle::Rectangle(Point minPoint, Point maxPoint)
{
    _minPoint = minPoint;
    _maxPoint = maxPoint;
}

Rectangle::Rectangle(double minX, double minY, double maxX, double maxY)
{
    _minPoint = Point(minX, minY);
    _maxPoint = Point(maxX, maxY);
}

Point Rectangle::minPoint()
{
    return _minPoint;
}

Point Rectangle::maxPoint()
{
    return _maxPoint;
}

double Rectangle::minX()
{
    return _minPoint.x();
}

double Rectangle::minY()
{
    return _minPoint.y();
}

double Rectangle::maxX()
{
    return _maxPoint.x();
}

double Rectangle::maxY()
{
    return _maxPoint.y();
}

State::State()
{
    _x = 0;
    _y = 0;
    _theta = 0;
}

State::State(Point p, double theta)
{
    _x = p.x();
    _y = p.y();
    _theta = theta;
}

State::State(double x, double y, double theta)
{
    _x = x;
    _y = y;
    _theta = theta;
}

double State::theta()
{
    return _theta;
}

GoalState::GoalState()
{
    _x = 0;
    _y = 0;
    _radius = 0;
    _theta = 0;
}

GoalState::GoalState(double x, double y, double radius, double theta)
{
    _x = x;
    _y = y;
    _radius = radius;
    _theta = theta;
}

double GoalState::theta()
{
    return _theta;
}

double GoalState::radius()
{
    return _radius;
}

GraphNode::GraphNode()
{
    _buildGraphNode();
}

GraphNode::GraphNode(Point p, int id, int parentId)
{
    _buildGraphNode(p, id, parentId);
}

GraphNode::GraphNode(double x, double y, int id, int parentId)
{
    _buildGraphNode(x, y, id, parentId);
}

void GraphNode::_buildGraphNode()
{
    _x = 0;
    _y = 0;
    _id = 0;
    _parentId = 0;
}

void GraphNode::_buildGraphNode(Point p, int id, int parentId)
{
    _x = p.x();
    _y = p.y();
    _id = id;
    _parentId = parentId;
}

void GraphNode::_buildGraphNode(double x, double y, int id, int parentId)
{
    _x = x;
    _y = y;
    _id = id;
    _parentId = parentId;
}

int GraphNode::id()
{
    return _id;
}

int GraphNode::parentId()
{
    return _parentId;
}

void GraphNode::setId(int id)
{
    _id = id;
}

void GraphNode::setParentId(int parentId)
{
    _parentId = parentId;
}

Edge::Edge()
{
    _start = GraphNode();
    _end = GraphNode();
}

Edge::Edge(GraphNode start, GraphNode end)
{
    _start = start;
    _end = end;
}

GraphNode Edge::start()
{
    return _start;
}

GraphNode Edge::end()
{
    return _end;
}