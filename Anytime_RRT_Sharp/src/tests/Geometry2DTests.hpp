#include <gtest/gtest.h>
#include <math.h>
#include "../Geometry2D.hpp"


#pragma region Geometry_Point

TEST(Geometry_Point, DefaultInitialize_CheckVals)
{
    Point p;
    GTEST_ASSERT_EQ(p.x(), 0);
    GTEST_ASSERT_EQ(p.y(), 0);
    GTEST_ASSERT_EQ(p.z(), 0);
}

TEST(Geometry_Point, InitializeWithVals_CheckVals)
{
    Point p(1, 2, 3);
    GTEST_ASSERT_EQ(p.x(), 1);
    GTEST_ASSERT_EQ(p.y(), 2);
    GTEST_ASSERT_EQ(p.z(), 3);
}

TEST(Geometry_Point, DistanceTo_NoDiff)
{
    Point p(1, 2, 3);
    GTEST_ASSERT_EQ(p.distanceTo(p), 0);
}

TEST(Geometry_Point, DistanceTo_XDiffOnly)
{
    Point p1(1, 2, 3), p2(4, 2, 3);
    GTEST_ASSERT_EQ(p1.distanceTo(p2), 3);
    GTEST_ASSERT_EQ(p2.distanceTo(p1), 3);
}

TEST(Geometry_Point, DistanceTo_YDiffOnly)
{
    Point p1(1, 2, 3), p2(1, -2, 3);
    GTEST_ASSERT_EQ(p1.distanceTo(p2), 4);
    GTEST_ASSERT_EQ(p2.distanceTo(p1), 4);
}

TEST(Geometry_Point, DistanceTo_ZDiffOnly)
{
    Point p1(1, 2, 1), p2(1, 2, -3);
    GTEST_ASSERT_EQ(p1.distanceTo(p2), 4);
    GTEST_ASSERT_EQ(p2.distanceTo(p1), 4);
}

TEST(Geometry_Point, DistanceTo_XYDiff)
{
    Point p1(1, 2, 3), p2(3, -2, 3);
    double dist = hypot(p1.x() - p2.x(), p1.y() - p2.y());
    GTEST_ASSERT_EQ(p1.distanceTo(p2), dist);
    GTEST_ASSERT_EQ(p2.distanceTo(p1), dist);
}

TEST(Geometry_Point, DistanceTo_XYZDiff)
{
    Point p1(1, 2, 3), p2(3, -2, 4);
    double xDiff = pow(p1.x() - p2.x(), 2);
    double yDiff = pow(p1.y() - p2.y(), 2);
    double zDiff = pow(p1.z() - p2.z(), 2);
    double dist = sqrt(xDiff + yDiff + zDiff);
    GTEST_ASSERT_EQ(p1.distanceTo(p2), dist);
    GTEST_ASSERT_EQ(p2.distanceTo(p1), dist);
}

#pragma endregion //Geometry_Point

#pragma region Geometry_Line

TEST(Geometry_Line, DefaultInitialize_CheckVals)
{
    Line l;
    GTEST_ASSERT_EQ(l.p1().x(), 0);
    GTEST_ASSERT_EQ(l.p1().y(), 0);
    GTEST_ASSERT_EQ(l.p1().z(), 0);
    GTEST_ASSERT_EQ(l.p2().x(), 0);
    GTEST_ASSERT_EQ(l.p2().y(), 0);
    GTEST_ASSERT_EQ(l.p2().z(), 0);
    GTEST_ASSERT_EQ(l.length(), 0);
}

TEST(Geometry_Line, InitializeWithPoints_CheckVals)
{
    Point p1(1, 1, 1), p2(2, 2, 2);
    Line l(p1, p2);
    double xDiff = pow(p1.x() - p2.x(), 2);
    double yDiff = pow(p1.y() - p2.y(), 2);
    double zDiff = pow(p1.z() - p2.z(), 2);
    double dist = sqrt(xDiff + yDiff + zDiff);

    GTEST_ASSERT_EQ(l.p1().x(), 1);
    GTEST_ASSERT_EQ(l.p1().y(), 1);
    GTEST_ASSERT_EQ(l.p1().z(), 1);
    GTEST_ASSERT_EQ(l.p2().x(), 2);
    GTEST_ASSERT_EQ(l.p2().y(), 2);
    GTEST_ASSERT_EQ(l.p2().z(), 2);
    GTEST_ASSERT_EQ(l.length(), dist);
}

#pragma endregion //Geometry_Line

#pragma region Geometry_Circle

TEST(Geometry_Circle, DefaultInitialize_CheckVals)
{
    Sphere c;
    GTEST_ASSERT_EQ(c.x(), 0);
    GTEST_ASSERT_EQ(c.y(), 0);
    GTEST_ASSERT_EQ(c.z(), 0);
    GTEST_ASSERT_EQ(c.radius(), 0);
    GTEST_ASSERT_EQ(c.volume(), 0);
}

TEST(Geometry_Circle, InitializeWithXYRadius_CheckVals)
{
    Sphere c(1, 2, 3, 4);
    GTEST_ASSERT_EQ(c.x(), 1);
    GTEST_ASSERT_EQ(c.y(), 2);
    GTEST_ASSERT_EQ(c.z(), 3);
    GTEST_ASSERT_EQ(c.radius(), 4);
    GTEST_ASSERT_EQ(c.volume(), (4.0 / 3.0) * M_PI * pow(c.radius(), 3));
}

TEST(Geometry_Circle, InitializeWithPointRadius_CheckVals)
{
    Sphere c(Point(1, 2, 3), 3);
    GTEST_ASSERT_EQ(c.x(), 1);
    GTEST_ASSERT_EQ(c.y(), 2);
    GTEST_ASSERT_EQ(c.z(), 3);
    GTEST_ASSERT_EQ(c.radius(), 3);
    GTEST_ASSERT_EQ(c.volume(), (4.0 / 3.0) * M_PI * pow(c.radius(), 3));
}

#pragma endregion //Geometry_Circle

#pragma region Geometry_Vector

TEST(Geometry_Vector, DefaultInitialize_CheckVals)
{
    Vector v;
    GTEST_ASSERT_EQ(v.x(), 0);
    GTEST_ASSERT_EQ(v.y(), 0);
    GTEST_ASSERT_EQ(v.z(), 0);
    GTEST_ASSERT_EQ(v.magnitude(), 0);
}

TEST(Geometry_Vector, InitializeWithVals_CheckVals)
{
    Vector v(1, 2, 3);
    GTEST_ASSERT_EQ(v.x(), 1);
    GTEST_ASSERT_EQ(v.y(), 2);
    GTEST_ASSERT_EQ(v.z(), 3);
    GTEST_ASSERT_EQ(v.magnitude(), sqrt(1 + 4 + 9));
}

TEST(Geometry_Vector, DotProduct_SameVector)
{
    Vector v1(1, 2, 3);
    Vector v2(1, 2, 3);
    GTEST_ASSERT_EQ(v1.dot(v2), 14);
    GTEST_ASSERT_EQ(v2.dot(v1), 14);
}

TEST(Geometry_Vector, DotProduct_OppositeLine)
{
    Vector v1(1, 1, 1);
    Vector v2(-1, -1, -1);
    GTEST_ASSERT_EQ(v1.dot(v2), -3);
    GTEST_ASSERT_EQ(v2.dot(v1), -3);
}

TEST(Geometry_Vector, DotProduct_PerpendicularLine)
{
    Vector v1(1, 0, 0);
    Vector v2(0, 0, 1);
    GTEST_ASSERT_EQ(v1.dot(v2), 0);
    GTEST_ASSERT_EQ(v2.dot(v1), 0);
}

#pragma endregion //Geometry_Vector