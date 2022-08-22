#include <gtest/gtest.h>
#include "ARRTS.hpp"

TEST(ARRTS_Obstacles, AddOneObstacle_CheckNum)
{
    ArrtsService service;
    service.AddObstacle(1, 1, 1);
    GTEST_ASSERT_EQ(service.GetNumObstacles(), 1);
}

TEST(ARRTS_Obstacles, AddOneObstacle_CheckVal)
{
    ArrtsService service;
    service.AddObstacle(1, 2, 3);
    Obstacle obs = service.GetObstacle(0);
    GTEST_ASSERT_EQ(obs.x, 1);
    GTEST_ASSERT_EQ(obs.y, 2);
    GTEST_ASSERT_EQ(obs.radius, 3);
}

TEST(ARRTS_Obstacles, AddMultipleObstacle_CheckNum)
{
    ArrtsService service;
    const double x[] = { 1, 2, 3 };
    const double y[] = { 1, 2, 3 };
    const double r[] = { 1, 2, 3 };
    service.AddObstacles(x, y, r, 3);
    GTEST_ASSERT_EQ(service.GetNumObstacles(), 3);
}

TEST(ARRTS_Obstacles, AddMultipleObstacle_CheckVals)
{
    ArrtsService service;
    const double x[] = { 1, 2, 3 };
    const double y[] = { 4, 5, 6 };
    const double r[] = { 7, 8, 9 };
    service.AddObstacles(x, y, r, 3);

    Obstacle obs1 = service.GetObstacle(0);
    Obstacle obs2 = service.GetObstacle(1);
    Obstacle obs3 = service.GetObstacle(2);
    
    GTEST_ASSERT_EQ(obs1.x, 1);
    GTEST_ASSERT_EQ(obs2.x, 2);
    GTEST_ASSERT_EQ(obs3.x, 3);

    GTEST_ASSERT_EQ(obs1.y, 4);
    GTEST_ASSERT_EQ(obs2.y, 5);
    GTEST_ASSERT_EQ(obs3.y, 6);

    GTEST_ASSERT_EQ(obs1.radius, 7);
    GTEST_ASSERT_EQ(obs2.radius, 8);
    GTEST_ASSERT_EQ(obs3.radius, 9);
}

TEST(ARRTS_Obstacles, AddSingleThenMultipleObstacle_CheckNum)
{
    ArrtsService service;
    service.AddObstacle(1, 2, 3);
    const double x[] = { 1, 2, 3 };
    const double y[] = { 1, 2, 3 };
    const double r[] = { 1, 2, 3 };
    service.AddObstacles(x, y, r, 3);
    GTEST_ASSERT_EQ(service.GetNumObstacles(), 4);
}

TEST(ARRTS_Obstacles, AddSingleThenMultipleObstacle_CheckVals)
{
    ArrtsService service;
    service.AddObstacle(1, 2, 3);
    const double x[] = { 4, 5, 6 };
    const double y[] = { 7, 8, 9 };
    const double r[] = { 10, 11, 12 };
    service.AddObstacles(x, y, r, 3);
    
    Obstacle obs1 = service.GetObstacle(0);
    Obstacle obs2 = service.GetObstacle(1);
    Obstacle obs3 = service.GetObstacle(2);
    Obstacle obs4 = service.GetObstacle(3);

    GTEST_ASSERT_EQ(obs1.x, 1);
    GTEST_ASSERT_EQ(obs2.x, 4);
    GTEST_ASSERT_EQ(obs3.x, 5);
    GTEST_ASSERT_EQ(obs4.x, 6);

    GTEST_ASSERT_EQ(obs1.y, 2);
    GTEST_ASSERT_EQ(obs2.y, 7);
    GTEST_ASSERT_EQ(obs3.y, 8);
    GTEST_ASSERT_EQ(obs4.y, 9);

    GTEST_ASSERT_EQ(obs1.radius, 3);
    GTEST_ASSERT_EQ(obs2.radius, 10);
    GTEST_ASSERT_EQ(obs3.radius, 11);
    GTEST_ASSERT_EQ(obs4.radius, 12);
}

TEST(ARRTS_Obstacles, AddMultipleThenSingleObstacle_CheckNum)
{
    ArrtsService service;
    const double x[] = { 1, 2, 3 };
    const double y[] = { 1, 2, 3 };
    const double r[] = { 1, 2, 3 };
    service.AddObstacles(x, y, r, 3);
    service.AddObstacle(1, 2, 3);
    GTEST_ASSERT_EQ(service.GetNumObstacles(), 4);
}

TEST(ARRTS_Obstacles, AddMultipleThenSingleObstacle_CheckVals)
{
    ArrtsService service;
    const double x[] = { 4, 5, 6 };
    const double y[] = { 7, 8, 9 };
    const double r[] = { 10, 11, 12 };
    service.AddObstacles(x, y, r, 3);
    service.AddObstacle(1, 2, 3);
    
    Obstacle obs1 = service.GetObstacle(0);
    Obstacle obs2 = service.GetObstacle(1);
    Obstacle obs3 = service.GetObstacle(2);
    Obstacle obs4 = service.GetObstacle(3);

    GTEST_ASSERT_EQ(obs1.x, 4);
    GTEST_ASSERT_EQ(obs2.x, 5);
    GTEST_ASSERT_EQ(obs3.x, 6);
    GTEST_ASSERT_EQ(obs4.x, 1);

    GTEST_ASSERT_EQ(obs1.y, 7);
    GTEST_ASSERT_EQ(obs2.y, 8);
    GTEST_ASSERT_EQ(obs3.y, 9);
    GTEST_ASSERT_EQ(obs4.y, 2);

    GTEST_ASSERT_EQ(obs1.radius, 10);
    GTEST_ASSERT_EQ(obs2.radius, 11);
    GTEST_ASSERT_EQ(obs3.radius, 12);
    GTEST_ASSERT_EQ(obs4.radius, 3);
}

TEST(ARRTS_Obstacles, GetFirstObstacleWhenNone)
{
    ArrtsService service;
    EXPECT_ANY_THROW(service.GetObstacle(0));
}

TEST(ARRTS_Obstacles, GetNegativeObstacleWhenNone)
{
    ArrtsService service;
    EXPECT_ANY_THROW(service.GetObstacle(-1));
}

TEST(ARRTS_Obstacles, GetSecondObstacleWhenOne)
{
    ArrtsService service;
    service.AddObstacle(1, 2, 3);
    EXPECT_ANY_THROW(service.GetObstacle(1));
}

int main(int argc, char* argv[])
{
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}