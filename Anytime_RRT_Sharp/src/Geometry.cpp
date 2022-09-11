#include "Geometry.hpp"

Point::Point()
{
    _x = 0;
    _y = 0;
    _z = 0;
}

Point::Point(double x, double y, double z)
{
    _x = x;
    _y = y;
    _z = z;
}

double Point::x() const
{
    return _x;
}

double Point::y() const
{
    return _y;
}

double Point::z() const
{
    return _z;
}

double Point::distanceTo(Point& p) const
{
    double xVal = pow(_x - p.x(), 2);
    double yVal = pow(_y - p.y(), 2);
    double zVal = pow(_z - p.z(), 2);
    return sqrt(xVal + yVal + zVal);
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

Line::Line(Point& p1, Point& p2)
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

Vector::Vector()
{
    _x = 0;
    _y = 0;
    _z = 0;
    _magnitude = 0;
}

Vector::Vector(double x, double y, double z)
{
    _x = x;
    _y = y;
    _z = z;
    _magnitude = sqrt(_x*_x + _y*_y + _z*_z);
}

double Vector::x() const
{
    return _x;
}

double Vector::y() const
{
    return _y;
}

double Vector::z() const
{
    return _z;
}

double Vector::magnitude() const
{
    return _magnitude;
}

double Vector::dot(Vector& v) const
{
    return x() * v.x() + y() * v.y() + z() * v.z();
}

Sphere::Sphere()
{
    _x = 0;
    _y = 0;
    _z = 0;
    _radius = 0;
    _area = 0;
}

Sphere::Sphere(Point p, double radius)
{
    _x = p.x();
    _y = p.y();
    _z = p.z();
    _radius = radius;
    _area = _calculateVolume();
}

Sphere::Sphere(double x, double y, double z, double radius)
{
    _x = x;
    _y = y;
    _z = z;
    _radius = radius;
    _area = _calculateVolume();
}

double Sphere::_calculateVolume() const
{
    return (4.0/3.0) * M_PI * pow(radius(), 3);
}

double Sphere::radius() const
{
    return _radius;
}

double Sphere::volume() const
{
    return _area;
}

Rectangle::Rectangle()
{
    _minPoint = Point();
    _maxPoint = Point();
    _volume = 0;
}

Rectangle::Rectangle(Point minPoint, Point maxPoint)
{
    _minPoint = minPoint;
    _maxPoint = maxPoint;
    _volume = _calculateVolume();
}

Rectangle::Rectangle(double minX, double minY, double minZ, double maxX, double maxY, double maxZ)
{
    _minPoint = Point(minX, minY, minZ);
    _maxPoint = Point(maxX, maxY, maxZ);
    _volume = _calculateVolume();
}

double Rectangle::_calculateVolume() const
{
    double xDiff = maxX() - minX();
    double yDiff = maxY() - minY();
    double zDiff = maxZ() - minZ();
    return xDiff * yDiff * zDiff;
}

Point Rectangle::minPoint() const
{
    return _minPoint;
}

Point Rectangle::maxPoint() const
{
    return _maxPoint;
}

double Rectangle::volume() const
{
    return _volume;
}

double Rectangle::minX() const
{
    return _minPoint.x();
}

double Rectangle::minY() const
{
    return _minPoint.y();
}

double Rectangle::minZ() const
{
    return _minPoint.z();
}

double Rectangle::maxX() const
{
    return _maxPoint.x();
}

double Rectangle::maxY() const
{
    return _maxPoint.y();
}

double Rectangle::maxZ() const
{
    return _maxPoint.z();
}

State::State()
{
    _x = 0;
    _y = 0;
    _z = 0;
    _theta = 0;
}

State::State(GraphNode node)
{
    _x = node.x();
    _y = node.y();
    _z = node.z();
    _theta = node.theta();
}

State::State(Point p, double theta)
{
    _x = p.x();
    _y = p.y();
    _z = p.z();
    _theta = theta;
}

State::State(double x, double y, double z, double theta)
{
    _x = x;
    _y = y;
    _z = z;
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
    _z = 0;
    _radius = 0;
    _theta = 0;
}

GoalState::GoalState(double x, double y, double z, double radius, double theta)
{
    _x = x;
    _y = y;
    _z = z;
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

GraphNode::GraphNode(double x, double y, double z, double theta, int id, int parentId)
{
    _buildGraphNode(x, y, z, theta, id, parentId);
}

void GraphNode::_buildGraphNode()
{
    _x = 0;
    _y = 0;
    _z = 0;
    _theta = 0;
    _id = 0;
    _parentId = 0;
}

void GraphNode::_buildGraphNode(GraphNode n)
{
    _x = n.x();
    _y = n.y();
    _z = n.z();
    _theta = n.theta();
    _id = n.id();
    _parentId = n.parentId();
}

void GraphNode::_buildGraphNode(Point p, double theta, int id, int parentId)
{
    _x = p.x();
    _y = p.y();
    _z = p.z();
    _theta = theta;
    _id = id;
    _parentId = parentId;
}

void GraphNode::_buildGraphNode(double x, double y, double z, double theta, int id, int parentId)
{
    _x = x;
    _y = y;
    _z = z;
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