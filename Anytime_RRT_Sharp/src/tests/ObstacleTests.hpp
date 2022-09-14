#include <gtest/gtest.h>
#include "../Obstacle.hpp"

#pragma region SphereObstacle

TEST(SphereObstacle, DefaultInitialize_CheckVals)
{
    SphereObstacle so;
    GTEST_ASSERT_EQ(so.x(), 0);
    GTEST_ASSERT_EQ(so.y(), 0);
    GTEST_ASSERT_EQ(so.z(), 0);
    GTEST_ASSERT_EQ(so.radius(), 0);
}

TEST(SphereObstacle, InitializeWithPointAndRad_CheckVals)
{
    Point p = { 1, 2, 3 };
    SphereObstacle so(p, 4);
    GTEST_ASSERT_EQ(so.x(), 1);
    GTEST_ASSERT_EQ(so.y(), 2);
    GTEST_ASSERT_EQ(so.z(), 3);
    GTEST_ASSERT_EQ(so.radius(), 4);
}

TEST(SphereObstacle, InitializeWithIndividualVals_CheckVals)
{
    SphereObstacle so(1, 2, 3, 4);
    GTEST_ASSERT_EQ(so.x(), 1);
    GTEST_ASSERT_EQ(so.y(), 2);
    GTEST_ASSERT_EQ(so.z(), 3);
    GTEST_ASSERT_EQ(so.radius(), 4);
}

TEST(SphereObstacle, PointIntersect_DoesIntersect)
{
    SphereObstacle so(1, 1, 1, 5);
    Point p = { 2, 3, 2 };
    EXPECT_TRUE(so.intersects(p));
}

TEST(SphereObstacle, PointIntersect_DoesNotIntersect)
{
    SphereObstacle so(1, 1, 1, 5);
    Point p = { 8, -7, -10 };
    EXPECT_FALSE(so.intersects(p));
}

TEST(SphereObstacle, LineIntersect_DoesIntersect_TwoPoints)
{
    SphereObstacle so(1, 1, 1, 5);
    Point p1(1, 1, 1), p2(2, 2, 8);
    Line l(p1, p2);
    EXPECT_TRUE(so.intersects(l));
}

TEST(SphereObstacle, LineIntersect_DoesIntersect_TwoPoints_Reverse)
{
    SphereObstacle so(1, 1, 1, 5);
    Point p1(1, 1, 1), p2(2, 2, 8);
    Line l(p2, p1);
    EXPECT_TRUE(so.intersects(l));
}

TEST(SphereObstacle, LineIntersect_DoesIntersect_OnePoint)
{
    SphereObstacle so(1, 1, 1, 5);
    Point p1(0, 0, 0), p2(2, 2, 8);
    Line l(p1, p2);
    EXPECT_TRUE(so.intersects(l));
}

TEST(SphereObstacle, LineIntersect_DoesIntersect_OnePoint_Reverse)
{
    SphereObstacle so(1, 1, 1, 5);
    Point p1(0, 0, 0), p2(2, 2, 8);
    Line l(p2, p1);
    EXPECT_TRUE(so.intersects(l));
}

TEST(SphereObstacle, LineIntersect_DoesNotIntersect)
{
    SphereObstacle so(4, 4, 4, 5);
    Point p1(0, 0, 0), p2(-2, -2, -2);
    Line l(p1, p2);
    EXPECT_FALSE(so.intersects(l));
}

TEST(SphereObstacle, LineIntersect_DoesNotIntersect_Reverse)
{
    SphereObstacle so(4, 4, 4, 5);
    Point p1(0, 0, 0), p2(-2, -2, -2);
    Line l(p2, p1);
    EXPECT_FALSE(so.intersects(l));
}

TEST(SphereObstacle, RectagleIntersect_FullyContained)
{
    SphereObstacle so(3, 3, 3, 1);
    Rectangle r = { { 1, 1, 1 }, { 5, 5, 5 } };
    EXPECT_TRUE(so.intersects(r));
}

TEST(SphereObstacle, RectagleIntersect_PointContained)
{
    SphereObstacle so(2, 2, 2, 4);
    Rectangle r = { { 1, 1, 1}, { 5, 5, 5 } };
    EXPECT_TRUE(so.intersects(r));
}

TEST(SphereObstacle, RectagleIntersect_PointNotContainedRadiusContained_Left)
{
    SphereObstacle so(0, 3, 3, 3);
    Rectangle r = { { 1, 1, 1 }, { 5, 5, 5 } };
    EXPECT_TRUE(so.intersects(r));
}

TEST(SphereObstacle, RectagleIntersect_PointNotContainedRadiusContained_Right)
{
    SphereObstacle so(6, 3, 3, 3);
    Rectangle r = { { 1, 1, 1 }, { 5, 5, 5 } };
    EXPECT_TRUE(so.intersects(r));
}

TEST(SphereObstacle, RectagleIntersect_PointNotContainedRadiusContained_Back)
{
    SphereObstacle so(3, 6, 3, 3);
    Rectangle r = { { 1, 1, 1 }, { 5, 5, 5 } };
    EXPECT_TRUE(so.intersects(r));
}

TEST(SphereObstacle, RectagleIntersect_PointNotContainedRadiusContained_Front)
{
    SphereObstacle so(3, 0, 3, 3);
    Rectangle r = { { 1, 1, 1 }, { 5, 5, 5 } };
    EXPECT_TRUE(so.intersects(r));
}

TEST(SphereObstacle, RectagleIntersect_PointNotContainedRadiusContained_Top)
{
    SphereObstacle so(3, 3, 6, 3);
    Rectangle r = { { 1, 1, 1 }, { 5, 5, 5 } };
    EXPECT_TRUE(so.intersects(r));
}

TEST(SphereObstacle, RectagleIntersect_PointNotContainedRadiusContained_Bottom)
{
    SphereObstacle so(3, 3, 0, 3);
    Rectangle r = { { 1, 1, 1 }, { 5, 5, 5 } };
    EXPECT_TRUE(so.intersects(r));
}

TEST(SphereObstacle, RectagleIntersect_DoesNotIntersect)
{
    SphereObstacle so(-3, -3, -3, 3);
    Rectangle r = { { 1, 1, 1 }, { 5, 5, 5 } };
    EXPECT_FALSE(so.intersects(r));
}

#pragma endregion //SphereObstacle

#pragma region RectangleObstacle

TEST(RectangleObstacle, DefaultInitialize_CheckVals)
{
    RectangleObstacle ro;
    GTEST_ASSERT_EQ(ro.minX(), 0);
    GTEST_ASSERT_EQ(ro.minY(), 0);
    GTEST_ASSERT_EQ(ro.minZ(), 0);
    GTEST_ASSERT_EQ(ro.maxX(), 0);
    GTEST_ASSERT_EQ(ro.maxY(), 0);
    GTEST_ASSERT_EQ(ro.maxZ(), 0);
    GTEST_ASSERT_EQ(ro.volume(), 0);
    GTEST_ASSERT_EQ(ro.surfaces().size(), 0);
}

TEST(RectangleObstacle, InitializeWithMinMaxPoints_CheckVals)
{
    Point p1(1, 2, 3), p2(6, 7, 8);
    RectangleObstacle ro(p1, p2);
    GTEST_ASSERT_EQ(ro.minX(), 1);
    GTEST_ASSERT_EQ(ro.minY(), 2);
    GTEST_ASSERT_EQ(ro.minZ(), 3);
    GTEST_ASSERT_EQ(ro.maxX(), 6);
    GTEST_ASSERT_EQ(ro.maxY(), 7);
    GTEST_ASSERT_EQ(ro.maxZ(), 8);
    GTEST_ASSERT_EQ(ro.volume(), 5 * 5 * 5);
    GTEST_ASSERT_EQ(ro.surfaces().size(), 6);
}

TEST(RectangleObstacle, InitializeWithMinMaxPoints_CheckSurfaces)
{
    Point p1(1, 2, 3), p2(6, 7, 8);
    RectangleObstacle ro(p1, p2);

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

TEST(RectangleObstacle, InitializeWithIndividualVals_CheckVals)
{
    RectangleObstacle ro(1, 2, 3, 6, 7, 8);
    GTEST_ASSERT_EQ(ro.minX(), 1);
    GTEST_ASSERT_EQ(ro.minY(), 2);
    GTEST_ASSERT_EQ(ro.minZ(), 3);
    GTEST_ASSERT_EQ(ro.maxX(), 6);
    GTEST_ASSERT_EQ(ro.maxY(), 7);
    GTEST_ASSERT_EQ(ro.maxZ(), 8);
    GTEST_ASSERT_EQ(ro.volume(), 5 * 5 * 5);
    GTEST_ASSERT_EQ(ro.surfaces().size(), 6);
}

TEST(RectangleObstacle, InitializeWithIndividualVals_CheckSurfaces)
{
    RectangleObstacle ro(1, 2, 3, 6, 7, 8);
    
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

TEST(RectangleObstacle, PointIntersect_DoesIntersect)
{
    Point p1(1, 1, 1), p2(5, 5, 5), pi(3, 3, 3);
    RectangleObstacle ro(p1, p2);
    EXPECT_TRUE(ro.intersects(pi));
}

TEST(RectangleObstacle, PointIntersect_DoesNotIntersect)
{
    Point p1(1, 1, 1), p2(5, 5, 5), pi(0, 0, 0);
    RectangleObstacle ro(p1, p2);
    EXPECT_FALSE(ro.intersects(pi));
}

TEST(RectangleObstacle, LineIntersect_DoesIntersect_TwoPoints)
{
    Point p1(1, 1, 1), p2(5, 5, 5);
    Point p3(0, 0, 0), p4(3, 3, 3);
    RectangleObstacle ro(p1, p2);
    Line l(p3, p4);
    EXPECT_TRUE(ro.intersects(l));
}

TEST(RectangleObstacle, LineIntersect_DoesIntersect_TwoPoints_Reverse)
{
    Point p1(1, 1, 1), p2(5, 5, 5);
    Point p3(0, 0, 0), p4(3, 3, 3);
    RectangleObstacle ro(p1, p2);
    Line l(p4, p3);
    EXPECT_TRUE(ro.intersects(l));
}

TEST(RectangleObstacle, LineIntersect_DoesNotIntersect_TwoPoints)
{
    Point p1(1, 1, 1), p2(5, 5, 5);
    Point p3(0, 0, 0), p4(-1, -1, -1);
    RectangleObstacle ro(p1, p2);
    Line l(p3, p4);
    EXPECT_FALSE(ro.intersects(l));
}

TEST(RectangleObstacle, LineIntersect_DoesNotIntersect_TwoPoints_Reverse)
{
    Point p1(1, 1, 1), p2(5, 5, 5);
    Point p3(0, 0, 0), p4(-1, -1, -1);
    RectangleObstacle ro(p1, p2);
    Line l(p4, p3);
    EXPECT_FALSE(ro.intersects(l));
}

TEST(RectangleObstacle, RectagleIntersect_FullyContained)
{
    RectangleObstacle ro(Point(1, 1, 1), Point(2, 2, 2));
    Rectangle r(Point(0, 0, 0), Point(5, 5, 5));
    EXPECT_TRUE(ro.intersects(r));
}

TEST(RectangleObstacle, RectagleIntersect_MinPointContained)
{
    RectangleObstacle ro(Point(4, 4, 4), Point(8, 8, 8));
    Rectangle r(Point(0, 0, 0), Point(5, 5, 5));
    EXPECT_TRUE(ro.intersects(r));
}

TEST(RectangleObstacle, RectagleIntersect_MaxPointContained)
{
    RectangleObstacle ro(Point(-4, -4, -4), Point(3, 3, 3));
    Rectangle r(Point(0, 0, 0), Point(5, 5, 5));
    EXPECT_TRUE(ro.intersects(r));
}

TEST(RectangleObstacle, RectagleIntersect_DoesNotIntersect)
{
    RectangleObstacle ro(Point(-4, -4, -4), Point(0, 0, 0));
    Rectangle r(Point(2, 2, 2), Point(5, 5, 5));
    EXPECT_FALSE(ro.intersects(r));
}

#pragma endregion //RectangleObstacle