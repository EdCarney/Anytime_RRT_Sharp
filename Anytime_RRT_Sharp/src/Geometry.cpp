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

double Point::distanceTo(Point p)
{
    return hypot(_x - p.x(), _y - p.y());
}

Line::Line()
{
    _p1 = Point();
    _p2 = Point();
    _slope = 0;
    _a = 0;
    _b = 0;
    _c = 0;
}

Line::Line(Point p1, Point p2)
{
    _p1 = p1;
    _p2 = p2;
    _slope = (p2.y() - p1.y()) / (p2.x() - p1.x());
    _a = 1;
    _b = -_slope;
    _c = p1.y() - _slope * p1.x();
}

Point Line::p1()
{
    return _p1;
}

Point Line::p2()
{
    return _p2;
}

double Line::slope()
{
    return _slope;
}

double Line::a()
{
    return _a;
}

double Line::b()
{
    return _b;
}

double Line::c()
{
    return _c;
}

Circle::Circle()
{
    _x = 0;
    _y = 0;
    _radius = 0;
    _area = 0;
}

Circle::Circle(Point p, double radius)
{
    _x = p.x();
    _y = p.y();
    _radius = radius;
    _area = _calculateArea();
}

Circle::Circle(double x, double y, double radius)
{
    _x = x;
    _y = y;
    _radius = radius;
    _area = _calculateArea();
}

double Circle::_calculateArea()
{
    return M_PI * _radius * _radius;
}

double Circle::radius()
{
    return _radius;
}

double Circle::area()
{
    return _area;
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

void GraphNode::_buildGraphNode(GraphNode n)
{
    _x = n.x();
    _y = n.y();
    _id = n.id();
    _parentId = n.parentId();
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