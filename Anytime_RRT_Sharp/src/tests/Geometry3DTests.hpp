#include <gtest/gtest.h>
#include "../Geometry3D.hpp"

#pragma region Sphere

TEST(Sphere, DefaultInitialize_CheckVals)
{
    Sphere so;
    GTEST_ASSERT_EQ(so.x(), 0);
    GTEST_ASSERT_EQ(so.y(), 0);
    GTEST_ASSERT_EQ(so.z(), 0);
    GTEST_ASSERT_EQ(so.radius(), 0);
}

TEST(Sphere, InitializeWithPointAndRad_CheckVals)
{
    Point p = { 1, 2, 3 };
    Sphere so(p, 4);
    GTEST_ASSERT_EQ(so.x(), 1);
    GTEST_ASSERT_EQ(so.y(), 2);
    GTEST_ASSERT_EQ(so.z(), 3);
    GTEST_ASSERT_EQ(so.radius(), 4);
}

TEST(Sphere, InitializeWithIndividualVals_CheckVals)
{
    Sphere so(1, 2, 3, 4);
    GTEST_ASSERT_EQ(so.x(), 1);
    GTEST_ASSERT_EQ(so.y(), 2);
    GTEST_ASSERT_EQ(so.z(), 3);
    GTEST_ASSERT_EQ(so.radius(), 4);
}

TEST(Sphere, PointIntersect_DoesIntersect)
{
    Sphere so(1, 1, 1, 5);
    Point p = { 2, 3, 2 };
    EXPECT_TRUE(so.intersects(p));
}

TEST(Sphere, PointIntersect_DoesNotIntersect)
{
    Sphere so(1, 1, 1, 5);
    Point p = { 8, -7, -10 };
    EXPECT_FALSE(so.intersects(p));
}

TEST(Sphere, LineIntersect_DoesIntersect_TwoPoints)
{
    Sphere so(1, 1, 1, 5);
    Point p1(1, 1, 1), p2(2, 2, 8);
    Line l(p1, p2);
    EXPECT_TRUE(so.intersects(l));
}

TEST(Sphere, LineIntersect_DoesIntersect_TwoPoints_Reverse)
{
    Sphere so(1, 1, 1, 5);
    Point p1(1, 1, 1), p2(2, 2, 8);
    Line l(p2, p1);
    EXPECT_TRUE(so.intersects(l));
}

TEST(Sphere, LineIntersect_DoesIntersect_OnePoint)
{
    Sphere so(1, 1, 1, 5);
    Point p1(0, 0, 0), p2(2, 2, 8);
    Line l(p1, p2);
    EXPECT_TRUE(so.intersects(l));
}

TEST(Sphere, LineIntersect_DoesIntersect_OnePoint_Reverse)
{
    Sphere so(1, 1, 1, 5);
    Point p1(0, 0, 0), p2(2, 2, 8);
    Line l(p2, p1);
    EXPECT_TRUE(so.intersects(l));
}

TEST(Sphere, LineIntersect_DoesNotIntersect)
{
    Sphere so(4, 4, 4, 5);
    Point p1(0, 0, 0), p2(-2, -2, -2);
    Line l(p1, p2);
    EXPECT_FALSE(so.intersects(l));
}

TEST(Sphere, LineIntersect_DoesNotIntersect_Reverse)
{
    Sphere so(4, 4, 4, 5);
    Point p1(0, 0, 0), p2(-2, -2, -2);
    Line l(p2, p1);
    EXPECT_FALSE(so.intersects(l));
}

TEST(Sphere, RectagleIntersect_FullyContained)
{
    Sphere so(3, 3, 3, 1);
    Rectangle r = { { 1, 1, 1 }, { 5, 5, 5 } };
    EXPECT_TRUE(so.intersects(r));
}

TEST(Sphere, RectagleIntersect_PointContained)
{
    Sphere so(2, 2, 2, 4);
    Rectangle r = { { 1, 1, 1}, { 5, 5, 5 } };
    EXPECT_TRUE(so.intersects(r));
}

TEST(Sphere, RectagleIntersect_PointNotContainedRadiusContained_Left)
{
    Sphere so(0, 3, 3, 3);
    Rectangle r = { { 1, 1, 1 }, { 5, 5, 5 } };
    EXPECT_TRUE(so.intersects(r));
}

TEST(Sphere, RectagleIntersect_PointNotContainedRadiusContained_Right)
{
    Sphere so(6, 3, 3, 3);
    Rectangle r = { { 1, 1, 1 }, { 5, 5, 5 } };
    EXPECT_TRUE(so.intersects(r));
}

TEST(Sphere, RectagleIntersect_PointNotContainedRadiusContained_Back)
{
    Sphere so(3, 6, 3, 3);
    Rectangle r = { { 1, 1, 1 }, { 5, 5, 5 } };
    EXPECT_TRUE(so.intersects(r));
}

TEST(Sphere, RectagleIntersect_PointNotContainedRadiusContained_Front)
{
    Sphere so(3, 0, 3, 3);
    Rectangle r = { { 1, 1, 1 }, { 5, 5, 5 } };
    EXPECT_TRUE(so.intersects(r));
}

TEST(Sphere, RectagleIntersect_PointNotContainedRadiusContained_Top)
{
    Sphere so(3, 3, 6, 3);
    Rectangle r = { { 1, 1, 1 }, { 5, 5, 5 } };
    EXPECT_TRUE(so.intersects(r));
}

TEST(Sphere, RectagleIntersect_PointNotContainedRadiusContained_Bottom)
{
    Sphere so(3, 3, 0, 3);
    Rectangle r = { { 1, 1, 1 }, { 5, 5, 5 } };
    EXPECT_TRUE(so.intersects(r));
}

TEST(Sphere, RectagleIntersect_DoesNotIntersect)
{
    Sphere so(-3, -3, -3, 3);
    Rectangle r = { { 1, 1, 1 }, { 5, 5, 5 } };
    EXPECT_FALSE(so.intersects(r));
}

#pragma endregion //Sphere

#pragma region Rectangle

TEST(Rectangle, DefaultInitialize_CheckVals)
{
    Rectangle ro;
    GTEST_ASSERT_EQ(ro.minX(), 0);
    GTEST_ASSERT_EQ(ro.minY(), 0);
    GTEST_ASSERT_EQ(ro.minZ(), 0);
    GTEST_ASSERT_EQ(ro.maxX(), 0);
    GTEST_ASSERT_EQ(ro.maxY(), 0);
    GTEST_ASSERT_EQ(ro.maxZ(), 0);
    GTEST_ASSERT_EQ(ro.volume(), 0);
    GTEST_ASSERT_EQ(ro.surfaces().size(), 0);
}

TEST(Rectangle, InitializeWithMinMaxPoints_CheckVals)
{
    Point p1(1, 2, 3), p2(6, 7, 8);
    Rectangle ro(p1, p2);
    GTEST_ASSERT_EQ(ro.minX(), 1);
    GTEST_ASSERT_EQ(ro.minY(), 2);
    GTEST_ASSERT_EQ(ro.minZ(), 3);
    GTEST_ASSERT_EQ(ro.maxX(), 6);
    GTEST_ASSERT_EQ(ro.maxY(), 7);
    GTEST_ASSERT_EQ(ro.maxZ(), 8);
    GTEST_ASSERT_EQ(ro.volume(), 5 * 5 * 5);
    GTEST_ASSERT_EQ(ro.surfaces().size(), 6);
}

TEST(Rectangle, InitializeWithMinMaxPoints_CheckSurfaces)
{
    Point p1(1, 2, 3), p2(6, 7, 8);
    Rectangle ro(p1, p2);

    vector<Vector> normals
    {
        Vector(1, 0, 0),
        Vector(0, 1, 0),
        Vector(0, 0, 1),
        Vector(1, 0, 0),
        Vector(0, 1, 0),
        Vector(0, 0, 1)
    };
    vector<Point> points
    {
        Point(1, 0, 0),
        Point(0, 2, 0),
        Point(0, 0, 3),
        Point(6, 0, 0),
        Point(0, 7, 0),
        Point(0, 0, 8),
    };

    bool normCheck, pointCheck, check;
    Vector n, checkN;
    Point p, checkP;
    for (int i = 0; i < 6; i++)
    {
        check = false;
        n = normals[i];
        p = points[i];
        for (auto surface : ro.surfaces())
        {
            checkN = surface.normal();
            checkP = surface.point();
            normCheck = checkN.x() == n.x() && checkN.y() == n.y() && checkN.z() == n.z();
            pointCheck = checkP.x() == p.x() && checkP.y() == p.y() && checkP.z() == p.z();
            check = check || (normCheck && pointCheck);
        }
        GTEST_EXPECT_TRUE(check);
    }
}

TEST(Rectangle, InitializeWithIndividualVals_CheckVals)
{
    Rectangle ro(1, 2, 3, 6, 7, 8);
    GTEST_ASSERT_EQ(ro.minX(), 1);
    GTEST_ASSERT_EQ(ro.minY(), 2);
    GTEST_ASSERT_EQ(ro.minZ(), 3);
    GTEST_ASSERT_EQ(ro.maxX(), 6);
    GTEST_ASSERT_EQ(ro.maxY(), 7);
    GTEST_ASSERT_EQ(ro.maxZ(), 8);
    GTEST_ASSERT_EQ(ro.volume(), 5 * 5 * 5);
    GTEST_ASSERT_EQ(ro.surfaces().size(), 6);
}

TEST(Rectangle, InitializeWithIndividualVals_CheckSurfaces)
{
    Rectangle ro(1, 2, 3, 6, 7, 8);
    
    vector<Vector> normals
    {
        Vector(1, 0, 0),
        Vector(0, 1, 0),
        Vector(0, 0, 1),
        Vector(1, 0, 0),
        Vector(0, 1, 0),
        Vector(0, 0, 1)
    };
    vector<Point> points
    {
        Point(1, 0, 0),
        Point(0, 2, 0),
        Point(0, 0, 3),
        Point(6, 0, 0),
        Point(0, 7, 0),
        Point(0, 0, 8),
    };

    bool normCheck, pointCheck, check;
    Vector n, checkN;
    Point p, checkP;
    for (int i = 0; i < 6; i++)
    {
        check = false;
        n = normals[i];
        p = points[i];
        for (auto surface : ro.surfaces())
        {
            checkN = surface.normal();
            checkP = surface.point();
            normCheck = checkN.x() == n.x() && checkN.y() == n.y() && checkN.z() == n.z();
            pointCheck = checkP.x() == p.x() && checkP.y() == p.y() && checkP.z() == p.z();
            check = check || (normCheck && pointCheck);
        }
        GTEST_EXPECT_TRUE(check);
    }
}

TEST(Rectangle, PointIntersect_DoesIntersect)
{
    Point p1(1, 1, 1), p2(5, 5, 5), pi(3, 3, 3);
    Rectangle ro(p1, p2);
    EXPECT_TRUE(ro.intersects(pi));
}

TEST(Rectangle, PointIntersect_DoesNotIntersect)
{
    Point p1(1, 1, 1), p2(5, 5, 5), pi(0, 0, 0);
    Rectangle ro(p1, p2);
    EXPECT_FALSE(ro.intersects(pi));
}

TEST(Rectangle, LineIntersect_DoesIntersect_TwoPoints)
{
    Point p1(1, 1, 1), p2(5, 5, 5);
    Point p3(0, 0, 0), p4(3, 3, 3);
    Rectangle ro(p1, p2);
    Line l(p3, p4);
    EXPECT_TRUE(ro.intersects(l));
}

TEST(Rectangle, LineIntersect_DoesIntersect_TwoPoints_Reverse)
{
    Point p1(1, 1, 1), p2(5, 5, 5);
    Point p3(0, 0, 0), p4(3, 3, 3);
    Rectangle ro(p1, p2);
    Line l(p4, p3);
    EXPECT_TRUE(ro.intersects(l));
}

TEST(Rectangle, LineIntersect_DoesNotIntersect_TwoPoints)
{
    Point p1(1, 1, 1), p2(5, 5, 5);
    Point p3(0, 0, 0), p4(-1, -1, -1);
    Rectangle ro(p1, p2);
    Line l(p3, p4);
    EXPECT_FALSE(ro.intersects(l));
}

TEST(Rectangle, LineIntersect_DoesNotIntersect_TwoPoints_Reverse)
{
    Point p1(1, 1, 1), p2(5, 5, 5);
    Point p3(0, 0, 0), p4(-1, -1, -1);
    Rectangle ro(p1, p2);
    Line l(p4, p3);
    EXPECT_FALSE(ro.intersects(l));
}

TEST(Rectangle, RectagleIntersect_FullyContained)
{
    Rectangle ro(Point(1, 1, 1), Point(2, 2, 2));
    Rectangle r(Point(0, 0, 0), Point(5, 5, 5));
    EXPECT_TRUE(ro.intersects(r));
}

TEST(Rectangle, RectagleIntersect_MinPointContained)
{
    Rectangle ro(Point(4, 4, 4), Point(8, 8, 8));
    Rectangle r(Point(0, 0, 0), Point(5, 5, 5));
    EXPECT_TRUE(ro.intersects(r));
}

TEST(Rectangle, RectagleIntersect_MaxPointContained)
{
    Rectangle ro(Point(-4, -4, -4), Point(3, 3, 3));
    Rectangle r(Point(0, 0, 0), Point(5, 5, 5));
    EXPECT_TRUE(ro.intersects(r));
}

TEST(Rectangle, RectagleIntersect_DoesNotIntersect)
{
    Rectangle ro(Point(-4, -4, -4), Point(0, 0, 0));
    Rectangle r(Point(2, 2, 2), Point(5, 5, 5));
    EXPECT_FALSE(ro.intersects(r));
}

#pragma endregion //Rectangle