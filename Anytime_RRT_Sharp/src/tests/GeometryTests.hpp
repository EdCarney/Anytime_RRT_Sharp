#include <gtest/gtest.h>
#include <math.h>
#include "../Geometry.hpp"


#pragma region Geometry_Point

TEST(Geometry_Point, DefaultInitialize_CheckVals)
{
    Point p;
    GTEST_ASSERT_EQ(p.x(), 0);
    GTEST_ASSERT_EQ(p.y(), 0);
}

TEST(Geometry_Point, InitializeWithVals_CheckVals)
{
    Point p(1, 2);
    GTEST_ASSERT_EQ(p.x(), 1);
    GTEST_ASSERT_EQ(p.y(), 2);
}

TEST(Geometry_Point, DistanceTo_NoDiff)
{
    Point p(1, 2);
    GTEST_ASSERT_EQ(p.distanceTo(p), 0);
}

TEST(Geometry_Point, DistanceTo_XDiffOnly)
{
    Point p1(1, 2), p2(4, 2);
    GTEST_ASSERT_EQ(p1.distanceTo(p2), 3);
    GTEST_ASSERT_EQ(p2.distanceTo(p1), 3);
}

TEST(Geometry_Point, DistanceTo_YDiffOnly)
{
    Point p1(1, 2), p2(1, -2);
    GTEST_ASSERT_EQ(p1.distanceTo(p2), 4);
    GTEST_ASSERT_EQ(p2.distanceTo(p1), 4);
}

TEST(Geometry_Point, DistanceTo_XYDiff)
{
    Point p1(1, 2), p2(3, -2);
    double dist = hypot(p1.x() - p2.x(), p1.y() - p2.y());
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
    GTEST_ASSERT_EQ(l.p2().x(), 0);
    GTEST_ASSERT_EQ(l.p2().y(), 0);
    GTEST_ASSERT_EQ(l.length(), 0);
}

TEST(Geometry_Line, InitializeWithPoints_CheckVals)
{
    Point p1(1, 1), p2(2, 2);
    Line l(p1, p2);
    GTEST_ASSERT_EQ(l.p1().x(), 1);
    GTEST_ASSERT_EQ(l.p1().y(), 1);
    GTEST_ASSERT_EQ(l.p2().x(), 2);
    GTEST_ASSERT_EQ(l.p2().y(), 2);
    GTEST_ASSERT_EQ(l.length(), hypot(p1.x() - p2.x(), p1.y() - p2.y()));
}

TEST(Geometry_Line, DotProduct_SameLine)
{
    Point p1(0, 0), p2(1, 1);
    Point p3(0, 0), p4(1, 1);
    Line l1(p1, p2), l2(p3, p4);
    GTEST_ASSERT_EQ(l1.dotProduct(l2), 2);
    GTEST_ASSERT_EQ(l2.dotProduct(l1), 2);
}

TEST(Geometry_Line, DotProduct_OppositeLine)
{
    Point p1(0, 0), p2(1, 1);
    Point p3(0, 0), p4(-1, -1);
    Line l1(p1, p2), l2(p3, p4);
    GTEST_ASSERT_EQ(l1.dotProduct(l2), -2);
    GTEST_ASSERT_EQ(l2.dotProduct(l1), -2);
}

TEST(Geometry_Line, DotProduct_PerpendicularLine)
{
    Point p1(0, 0), p2(1, 0);
    Point p3(0, 0), p4(0, 1);
    Line l1(p1, p2), l2(p3, p4);
    GTEST_ASSERT_EQ(l1.dotProduct(l2), 0);
    GTEST_ASSERT_EQ(l2.dotProduct(l1), 0);
}

#pragma endregion //Line

#pragma region Geometry_Circle

TEST(Geometry_Circle, DefaultInitialize_CheckVals)
{
    Circle c;
    GTEST_ASSERT_EQ(c.x(), 0);
    GTEST_ASSERT_EQ(c.y(), 0);
    GTEST_ASSERT_EQ(c.radius(), 0);
    GTEST_ASSERT_EQ(c.area(), 0);
}

TEST(Geometry_Circle, InitializeWithXYRadius_CheckVals)
{
    Circle c(1, 2, 3);
    GTEST_ASSERT_EQ(c.x(), 1);
    GTEST_ASSERT_EQ(c.y(), 2);
    GTEST_ASSERT_EQ(c.radius(), 3);
    GTEST_ASSERT_EQ(c.area(), M_PI * 3 * 3);
}

TEST(Geometry_Circle, InitializeWithPointRadius_CheckVals)
{
    Circle c(Point(1, 2), 3);
    GTEST_ASSERT_EQ(c.x(), 1);
    GTEST_ASSERT_EQ(c.y(), 2);
    GTEST_ASSERT_EQ(c.radius(), 3);
    GTEST_ASSERT_EQ(c.area(), M_PI * 3 * 3);
}