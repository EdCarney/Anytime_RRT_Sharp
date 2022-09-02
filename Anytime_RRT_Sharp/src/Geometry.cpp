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

double Point::x() const
{
    return _x;
}

double Point::y() const
{
    return _y;
}

double Point::distanceTo(Point p) const
{
    return hypot(_x - p.x(), _y - p.y());
}

Line::Line()
{
    _p1 = Point();
    _p2 = Point();
    _length = 0;
    _slope = 0;
    _a = 0;
    _b = 0;
    _c = 0;
}

Line::Line(Point p1, Point p2)
{
    _p1 = p1;
    _p2 = p2;
    _length = p1.distanceTo(p2);
}

Point Line::p1() const
{
    return _p1;
}

Point Line::p2() const
{
    return _p2;
}

double Line::length() const
{
    return _length;
}

double Line::dotProduct(Line line) const
{
    double dx1 = _p2.x() - _p1.x();
    double dy1 = _p2.y() - _p1.y();
    double dx2 = line.p2().x() - line.p1().x();
    double dy2 = line.p2().y() - line.p1().y();

    return dx1 * dx2 + dy1 * dy2;
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

double Circle::radius() const
{
    return _radius;
}

double Circle::area() const
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

Point Rectangle::minPoint() const
{
    return _minPoint;
}

Point Rectangle::maxPoint() const
{
    return _maxPoint;
}

double Rectangle::minX() const
{
    return _minPoint.x();
}

double Rectangle::minY() const
{
    return _minPoint.y();
}

double Rectangle::maxX() const
{
    return _maxPoint.x();
}

double Rectangle::maxY() const
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

double State::theta() const
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

double GoalState::theta() const
{
    return _theta;
}

double GoalState::radius() const
{
    return _radius;
}

GraphNode::GraphNode()
{
    _buildGraphNode();
}

GraphNode::GraphNode(Point p, double theta, int id, int parentId)
{
    _buildGraphNode(p, theta, id, parentId);
}

GraphNode::GraphNode(double x, double y, double theta, int id, int parentId)
{
    _buildGraphNode(x, y, theta, id, parentId);
}

void GraphNode::_buildGraphNode()
{
    _x = 0;
    _y = 0;
    _theta = 0;
    _id = 0;
    _parentId = 0;
}

void GraphNode::_buildGraphNode(GraphNode n)
{
    _x = n.x();
    _y = n.y();
    _theta = n.theta();
    _id = n.id();
    _parentId = n.parentId();
}

void GraphNode::_buildGraphNode(Point p, double theta, int id, int parentId)
{
    _x = p.x();
    _y = p.y();
    _theta = theta;
    _id = id;
    _parentId = parentId;
}

void GraphNode::_buildGraphNode(double x, double y, double theta, int id, int parentId)
{
    _x = x;
    _y = y;
    _theta = theta;
    _id = id;
    _parentId = parentId;
}

double GraphNode::theta() const
{
    return _theta;
}

int GraphNode::id() const
{
    return _id;
}

int GraphNode::parentId() const
{
    return _parentId;
}

void GraphNode::setTheta(double theta)
{
    _theta = theta;
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