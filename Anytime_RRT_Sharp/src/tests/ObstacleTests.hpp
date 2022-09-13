#include <gtest/gtest.h>
#include "../Obstacle.hpp"

TEST(SphereObstacle, DefaultInitialize_CheckVals)
{
    SphereObstacle o;
    GTEST_ASSERT_EQ(o.x(), 0);
    GTEST_ASSERT_EQ(o.y(), 0);
    GTEST_ASSERT_EQ(o.z(), 0);
    GTEST_ASSERT_EQ(o.radius(), 0);
}

TEST(SphereObstacle, InitializeWithPointAndRad_CheckVals)
{
    Point p = { 1, 2, 3 };
    SphereObstacle o(p, 4);
    GTEST_ASSERT_EQ(o.x(), 1);
    GTEST_ASSERT_EQ(o.y(), 2);
    GTEST_ASSERT_EQ(o.z(), 3);
    GTEST_ASSERT_EQ(o.radius(), 4);
}

TEST(SphereObstacle, InitializeWithIndividualVals_CheckVals)
{
    SphereObstacle o(1, 2, 3, 4);
    GTEST_ASSERT_EQ(o.x(), 1);
    GTEST_ASSERT_EQ(o.y(), 2);
    GTEST_ASSERT_EQ(o.z(), 3);
    GTEST_ASSERT_EQ(o.radius(), 4);
}

TEST(SphereObstacle, PointIntersect_DoesIntersect)
{
    SphereObstacle o(1, 1, 1, 5);
    Point p = { 2, 3, 2 };
    EXPECT_TRUE(o.intersects(p));
}

TEST(SphereObstacle, PointIntersect_DoesNotIntersect)
{
    SphereObstacle o(1, 1, 1, 5);
    Point p = { 8, -7, -10 };
    EXPECT_FALSE(o.intersects(p));
}

TEST(SphereObstacle, LineIntersect_DoesIntersect_TwoPoints)
{
    SphereObstacle o(1, 1, 1, 5);
    Point p1(1, 1, 1), p2(2, 2, 8);
    Line l(p1, p2);
    EXPECT_TRUE(o.intersects(l));
}

TEST(SphereObstacle, LineIntersect_DoesIntersect_TwoPoints_Reverse)
{
    SphereObstacle o(1, 1, 1, 5);
    Point p1(1, 1, 1), p2(2, 2, 8);
    Line l(p2, p1);
    EXPECT_TRUE(o.intersects(l));
}

TEST(SphereObstacle, LineIntersect_DoesIntersect_OnePoint)
{
    SphereObstacle o(1, 1, 1, 5);
    Point p1(0, 0, 0), p2(2, 2, 8);
    Line l(p1, p2);
    EXPECT_TRUE(o.intersects(l));
}

TEST(SphereObstacle, LineIntersect_DoesIntersect_OnePoint_Reverse)
{
    SphereObstacle o(1, 1, 1, 5);
    Point p1(0, 0, 0), p2(2, 2, 8);
    Line l(p2, p1);
    EXPECT_TRUE(o.intersects(l));
}

TEST(SphereObstacle, LineIntersect_DoesNotIntersect)
{
    SphereObstacle o(4, 4, 4, 5);
    Point p1(0, 0, 0), p2(-2, -2, -2);
    Line l(p1, p2);
    EXPECT_FALSE(o.intersects(l));
}

TEST(SphereObstacle, LineIntersect_DoesNotIntersect_Reverse)
{
    SphereObstacle o(4, 4, 4, 5);
    Point p1(0, 0, 0), p2(-2, -2, -2);
    Line l(p2, p1);
    EXPECT_FALSE(o.intersects(l));
}

TEST(SphereObstacle, RectagleIntersect_FullyContained)
{
    SphereObstacle o(3, 3, 3, 1);
    Rectangle r = { { 1, 1, 1 }, { 5, 5, 5 } };
    EXPECT_TRUE(o.intersects(r));
}

TEST(SphereObstacle, RectagleIntersect_PointContained)
{
    SphereObstacle o(2, 2, 2, 4);
    Rectangle r = { { 1, 1, 1}, { 5, 5, 5 } };
    EXPECT_TRUE(o.intersects(r));
}

TEST(SphereObstacle, RectagleIntersect_PointNotContainedRadiusContained_Left)
{
    SphereObstacle o(0, 3, 3, 3);
    Rectangle r = { { 1, 1, 1 }, { 5, 5, 5 } };
    EXPECT_TRUE(o.intersects(r));
}

TEST(SphereObstacle, RectagleIntersect_PointNotContainedRadiusContained_Right)
{
    SphereObstacle o(6, 3, 3, 3);
    Rectangle r = { { 1, 1, 1 }, { 5, 5, 5 } };
    EXPECT_TRUE(o.intersects(r));
}

TEST(SphereObstacle, RectagleIntersect_PointNotContainedRadiusContained_Back)
{
    SphereObstacle o(3, 6, 3, 3);
    Rectangle r = { { 1, 1, 1 }, { 5, 5, 5 } };
    EXPECT_TRUE(o.intersects(r));
}

TEST(SphereObstacle, RectagleIntersect_PointNotContainedRadiusContained_Front)
{
    SphereObstacle o(3, 0, 3, 3);
    Rectangle r = { { 1, 1, 1 }, { 5, 5, 5 } };
    EXPECT_TRUE(o.intersects(r));
}

TEST(SphereObstacle, RectagleIntersect_PointNotContainedRadiusContained_Top)
{
    SphereObstacle o(3, 3, 6, 3);
    Rectangle r = { { 1, 1, 1 }, { 5, 5, 5 } };
    EXPECT_TRUE(o.intersects(r));
}

TEST(SphereObstacle, RectagleIntersect_PointNotContainedRadiusContained_Bottom)
{
    SphereObstacle o(3, 3, 0, 3);
    Rectangle r = { { 1, 1, 1 }, { 5, 5, 5 } };
    EXPECT_TRUE(o.intersects(r));
}

TEST(SphereObstacle, RectagleIntersect_DoesNotIntersect)
{
    SphereObstacle o(-3, -3, -3, 3);
    Rectangle r = { { 1, 1, 1 }, { 5, 5, 5 } };
    EXPECT_FALSE(o.intersects(r));
}