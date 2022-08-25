#include "Obstacle.hpp"

Obstacle::Obstacle()
{
    buildObstacle();
}

Obstacle::Obstacle(double xVal, double yVal, double r)
{
    buildObstacle();
    x = xVal;
    y = yVal;
    radius = r;
}

Obstacle::Obstacle(Point pos, double r)
{
    buildObstacle();
    x = pos.x;
    y = pos.y;
    radius = r;
}

void Obstacle::buildObstacle()
{
    x = 0.0;
    y = 0.0;
    radius = 0.0;
}

bool Obstacle::Intersects(Point point)
{
    double dist = hypot(point.x - x, point.y - y);
    return dist <= radius;
}

bool Obstacle::Intersects(Circle circle)
{
    double dist = hypot(circle.x - x, circle.y - y);
    return dist <= radius + circle.radius;
}

bool Obstacle::Intersects(Rectangle rect)
{
    double minX = x - radius;
    double maxX = x + radius;
    double minY = y - radius;
    double maxY = y + radius;

    bool inXLimits = minX < rect.maxPoint.x && maxX > rect.minPoint.x;
    bool inYLimits = minY < rect.maxPoint.y && maxY > rect.minPoint.y;

    return inXLimits && inYLimits;
}

double Obstacle::GetX()
{
    return x;
}

double Obstacle::GetY()
{
    return y;
}

double Obstacle::GetRadius()
{
    return radius;
}