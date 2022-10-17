#include "Geometry2D.hpp"

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

Vector Vector::operator*(double val) const
{
    return Vector(x() * val, y() * val, z() * val);
}

Vector Vector::operator/(double val) const
{
    return Vector(x() / val, y() / val, z() / val);
}

double Vector::x() const { return _x; }

double Vector::y() const { return _y; }

double Vector::z() const { return _z; }

double Vector::magnitude() const { return _magnitude; }

double Vector::dot(const Vector& v) const
{
    return x() * v.x() + y() * v.y() + z() * v.z();
}

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

Point Point::operator-(const Vector& v) const
{
    return Point(x() - v.x(), y() - v.y(), z() - v.z());
}

Point Point::operator+(const Vector& v) const
{
    return Point(x() + v.x(), y() + v.y(), z() + v.z());
}

Vector Point::operator-(const Point& p) const
{
    double xDiff = x() - p.x();
    double yDiff = y() - p.y();
    double zDiff = z() - p.z();
    return Vector(xDiff, yDiff, zDiff);
}

double Point::x() const { return _x; }

double Point::y() const { return _y; }
 
double Point::z() const { return _z; }

double Point::distanceTo(const Point& p) const {
    double xVal = pow(_x - p.x(), 2);
    double yVal = pow(_y - p.y(), 2);
    double zVal = pow(_z - p.z(), 2);
    return sqrt(xVal + yVal + zVal);
}

Line::Line()
{
    _p1 = Point();
    _p2 = Point();
    _tangent = Vector();
    _length = 0;
}

Line::Line(const Point& p1, const Point& p2)
{
    _p1 = p1;
    _p2 = p2;
    _tangent = (_p2 - _p1) / p1.distanceTo(p2);
    _length = p1.distanceTo(p2);
}

Point Line::p1() const { return _p1; }

Point Line::p2() const { return _p2; }

Vector Line::tangent() const { return _tangent; }

double Line::length() const { return _length; }

Plane::Plane()
{
    _point = Point();
    _normal = Vector();
}

Plane::Plane(Point point, Vector normal)
{
    _point = point;
    _normal = normal;
}

Point Plane::point() const { return _point; }

Vector Plane::normal() const { return _normal; }

Point Plane::getIntersectionPoint(const Line& line) const
{
    Vector u = line.p2() - line.p1();
    Vector w = line.p1() - point();
    Vector n = normal();

    double denom = n.dot(u);
    double numer = -1 * n.dot(w);

    if (denom == 0)
    {
        Point *p = NULL;
        return *p;
    }

    double s = numer / denom;
    return line.p1() + u * s;
}

State::State()
{
    _x = 0;
    _y = 0;
    _z = 0;
    _theta = 0;
    _rho = 0;
}

State::State(Point p, double theta, double rho)
{
    _x = p.x();
    _y = p.y();
    _z = p.z();
    _theta = theta;
    _rho = rho;
}

State::State(double x, double y, double z, double theta, double rho)
{
    _x = x;
    _y = y;
    _z = z;
    _theta = theta;
    _rho = rho;
}

double State::theta() const { return _theta; }

double State::rho() const { return _rho; }

GoalState::GoalState()
{
    _x = 0;
    _y = 0;
    _z = 0;
    _radius = 0;
    _theta = 0;
}

GoalState::GoalState(double x, double y, double z, double radius, double theta, double rho)
{
    _x = x;
    _y = y;
    _z = z;
    _radius = radius;
    _theta = theta;
    _rho = rho;
}

double GoalState::radius() const { return _radius; }

GraphNode::GraphNode() { _buildGraphNode(); }

GraphNode::GraphNode(Point p, double theta, double rho, unsigned long id, unsigned long parentId)
{
    _buildGraphNode(p, theta, rho, id, parentId);
}

GraphNode::GraphNode(double x, double y, double z, double theta, double rho, unsigned long id, unsigned long parentId)
{
    _buildGraphNode(x, y, z, theta, rho, id, parentId);
}

void GraphNode::_buildGraphNode()
{
    _x = 0;
    _y = 0;
    _z = 0;
    _theta = 0;
    _rho = 0;
    _id = 0;
    _parentId = 0;
}

void GraphNode::_buildGraphNode(GraphNode n)
{
    _x = n.x();
    _y = n.y();
    _z = n.z();
    _theta = n.theta();
    _rho = n.rho();
    _id = n.id();
    _parentId = n.parentId();
}

void GraphNode::_buildGraphNode(Point p, double theta, double rho, unsigned long id, unsigned long parentId)
{
    _x = p.x();
    _y = p.y();
    _z = p.z();
    _theta = theta;
    _rho = rho;
    _id = id;
    _parentId = parentId;
}

void GraphNode::_buildGraphNode(double x, double y, double z, double theta, double rho, unsigned long id, unsigned long parentId)
{
    _x = x;
    _y = y;
    _z = z;
    _theta = theta;
    _rho = rho;
    _id = id;
    _parentId = parentId;
}

int GraphNode::id() const { return _id; }

int GraphNode::parentId() const { return _parentId; }

void GraphNode::setId(int id) { _id = id; }

void GraphNode::setParentId(int parentId) { _parentId = parentId; }

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

const GraphNode& Edge::start() const { return _start; }

const GraphNode& Edge::end() const { return _end; }