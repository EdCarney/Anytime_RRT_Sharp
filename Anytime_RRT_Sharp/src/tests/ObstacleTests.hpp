#include <gtest/gtest.h>
#include "../Obstacle.hpp"

TEST(Obstacle, DefaultInitialize_CheckVals)
{
    Obstacle o;
    GTEST_ASSERT_EQ(o.x(), 0);
    GTEST_ASSERT_EQ(o.y(), 0);
    GTEST_ASSERT_EQ(o.radius(), 0);
}

TEST(Obstacle, InitializeWithPointAndRad_CheckVals)
{
    Point p = { 1, 2 };
    Obstacle o(p, 3);
    GTEST_ASSERT_EQ(o.x(), 1);
    GTEST_ASSERT_EQ(o.y(), 2);
    GTEST_ASSERT_EQ(o.radius(), 3);
}

TEST(Obstacle, InitializeWithIndividualVals_CheckVals)
{
    Obstacle o(1, 2, 3);
    GTEST_ASSERT_EQ(o.x(), 1);
    GTEST_ASSERT_EQ(o.y(), 2);
    GTEST_ASSERT_EQ(o.radius(), 3);
}

TEST(Obstacle, PointIntersect_DoesIntersect)
{
    Obstacle o(1, 1, 5);
    Point p = { 2, 3 };
    EXPECT_TRUE(o.intersects(p));
}

TEST(Obstacle, PointIntersect_DoesNotIntersect)
{
    Obstacle o(1, 1, 5);
    Point p = { 8, -7 };
    EXPECT_FALSE(o.intersects(p));
}

TEST(Obstacle, CircleIntersect_DoesIntersectPoint)
{
    Obstacle o(1, 1, 5);
    Circle c = { 2, 2, 1 };
    EXPECT_TRUE(o.intersects(c));
}

TEST(Obstacle, CircleIntersect_DoesIntersectRadius)
{
    Obstacle o(1, 1, 5);
    Circle c = { 7, 0, 5 };
    EXPECT_TRUE(o.intersects(c));
}

TEST(Obstacle, CircleIntersect_DoesNotIntersect)
{
    Obstacle o(1, 1, 5);
    Circle c = { -5, -5, 1 };
    EXPECT_FALSE(o.intersects(c));
}

TEST(Obstacle, RectagleIntersect_FullyContained)
{
    Obstacle o(3, 3, 1);
    Rectangle r = { { 1, 1 }, { 5, 5 } };
    EXPECT_TRUE(o.intersects(r));
}

TEST(Obstacle, RectagleIntersect_PointContained)
{
    Obstacle o(2, 2, 4);
    Rectangle r = { { 1, 1 }, { 5, 5 } };
    EXPECT_TRUE(o.intersects(r));
}

TEST(Obstacle, RectagleIntersect_PointNotContainedRadiusContained_Left)
{
    Obstacle o(0, 3, 3);
    Rectangle r = { { 1, 1 }, { 5, 5 } };
    EXPECT_TRUE(o.intersects(r));
}

TEST(Obstacle, RectagleIntersect_PointNotContainedRadiusContained_Right)
{
    Obstacle o(6, 3, 3);
    Rectangle r = { { 1, 1 }, { 5, 5 } };
    EXPECT_TRUE(o.intersects(r));
}

TEST(Obstacle, RectagleIntersect_PointNotContainedRadiusContained_Top)
{
    Obstacle o(3, 6, 3);
    Rectangle r = { { 1, 1 }, { 5, 5 } };
    EXPECT_TRUE(o.intersects(r));
}

TEST(Obstacle, RectagleIntersect_PointNotContainedRadiusContained_Bottom)
{
    Obstacle o(3, 0, 3);
    Rectangle r = { { 1, 1 }, { 5, 5 } };
    EXPECT_TRUE(o.intersects(r));
}

TEST(Obstacle, RectagleIntersect_DoesNotIntersect)
{
    Obstacle o(-3, -3, 3);
    Rectangle r = { { 1, 1 }, { 5, 5 } };
    EXPECT_FALSE(o.intersects(r));
}