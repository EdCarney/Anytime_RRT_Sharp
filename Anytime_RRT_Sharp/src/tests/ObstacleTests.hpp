#include <gtest/gtest.h>
#include "../Obstacle.hpp"

TEST(Obstacle, DefaultInitialize_CheckVals)
{
    Obstacle o;
    GTEST_ASSERT_EQ(o.x(), 0);
    GTEST_ASSERT_EQ(o.y(), 0);
    GTEST_ASSERT_EQ(o.z(), 0);
    GTEST_ASSERT_EQ(o.radius(), 0);
}

TEST(Obstacle, InitializeWithPointAndRad_CheckVals)
{
    Point p = { 1, 2, 3 };
    Obstacle o(p, 4);
    GTEST_ASSERT_EQ(o.x(), 1);
    GTEST_ASSERT_EQ(o.y(), 2);
    GTEST_ASSERT_EQ(o.z(), 3);
    GTEST_ASSERT_EQ(o.radius(), 4);
}

TEST(Obstacle, InitializeWithIndividualVals_CheckVals)
{
    Obstacle o(1, 2, 3, 4);
    GTEST_ASSERT_EQ(o.x(), 1);
    GTEST_ASSERT_EQ(o.y(), 2);
    GTEST_ASSERT_EQ(o.z(), 3);
    GTEST_ASSERT_EQ(o.radius(), 4);
}

TEST(Obstacle, PointIntersect_DoesIntersect)
{
    Obstacle o(1, 1, 1, 5);
    Point p = { 2, 3, 2 };
    EXPECT_TRUE(o.intersects(p));
}

TEST(Obstacle, PointIntersect_DoesNotIntersect)
{
    Obstacle o(1, 1, 1, 5);
    Point p = { 8, -7, -10 };
    EXPECT_FALSE(o.intersects(p));
}

TEST(Obstacle, SphereIntersect_DoesIntersectPoint)
{
    Obstacle o(1, 1, 1, 5);
    Sphere s = { 2, 2, 2, 1 };
    EXPECT_TRUE(o.intersects(s));
}

TEST(Obstacle, SphereIntersect_DoesIntersectRadius)
{
    Obstacle o(1, 1, 0, 5);
    Sphere s = { 7, 0, 0, 5 };
    EXPECT_TRUE(o.intersects(s));
}

TEST(Obstacle, SphereIntersect_DoesNotIntersect)
{
    Obstacle o(1, 1, 1, 5);
    Sphere s = { -5, -5, -5, 1 };
    EXPECT_FALSE(o.intersects(s));
}

TEST(Obstacle, RectagleIntersect_FullyContained)
{
    Obstacle o(3, 3, 3, 1);
    Rectangle r = { { 1, 1, 1 }, { 5, 5, 5 } };
    EXPECT_TRUE(o.intersects(r));
}

TEST(Obstacle, RectagleIntersect_PointContained)
{
    Obstacle o(2, 2, 2, 4);
    Rectangle r = { { 1, 1, 1}, { 5, 5, 5 } };
    EXPECT_TRUE(o.intersects(r));
}

TEST(Obstacle, RectagleIntersect_PointNotContainedRadiusContained_Left)
{
    Obstacle o(0, 3, 3, 3);
    Rectangle r = { { 1, 1, 1 }, { 5, 5, 5 } };
    EXPECT_TRUE(o.intersects(r));
}

TEST(Obstacle, RectagleIntersect_PointNotContainedRadiusContained_Right)
{
    Obstacle o(6, 3, 3, 3);
    Rectangle r = { { 1, 1, 1 }, { 5, 5, 5 } };
    EXPECT_TRUE(o.intersects(r));
}

TEST(Obstacle, RectagleIntersect_PointNotContainedRadiusContained_Back)
{
    Obstacle o(3, 6, 3, 3);
    Rectangle r = { { 1, 1, 1 }, { 5, 5, 5 } };
    EXPECT_TRUE(o.intersects(r));
}

TEST(Obstacle, RectagleIntersect_PointNotContainedRadiusContained_Front)
{
    Obstacle o(3, 0, 3, 3);
    Rectangle r = { { 1, 1, 1 }, { 5, 5, 5 } };
    EXPECT_TRUE(o.intersects(r));
}

TEST(Obstacle, RectagleIntersect_PointNotContainedRadiusContained_Top)
{
    Obstacle o(3, 3, 6, 3);
    Rectangle r = { { 1, 1, 1 }, { 5, 5, 5 } };
    EXPECT_TRUE(o.intersects(r));
}

TEST(Obstacle, RectagleIntersect_PointNotContainedRadiusContained_Bottom)
{
    Obstacle o(3, 3, 0, 3);
    Rectangle r = { { 1, 1, 1 }, { 5, 5, 5 } };
    EXPECT_TRUE(o.intersects(r));
}

TEST(Obstacle, RectagleIntersect_DoesNotIntersect)
{
    Obstacle o(-3, -3, -3, 3);
    Rectangle r = { { 1, 1, 1 }, { 5, 5, 5 } };
    EXPECT_FALSE(o.intersects(r));
}