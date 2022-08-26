#include <gtest/gtest.h>
#include "../ARRTS.hpp"

#pragma region ARRTS_StartState

TEST(ARRTS_StartState, SetAndGet)
{
    ArrtsService service;
    service.SetStartState(5, 6, 7);
    State state = service.GetStartState();
    GTEST_ASSERT_EQ(state.x(), 5);
    GTEST_ASSERT_EQ(state.y(), 6);
    GTEST_ASSERT_EQ(state.theta(), 7);
}

TEST(ARRTS_StartState, SetUpdateAndGet)
{
    ArrtsService service;
    service.SetStartState(5, 6, 7);
    service.SetStartState(8, 9, 10);
    State state = service.GetStartState();
    GTEST_ASSERT_EQ(state.x(), 8);
    GTEST_ASSERT_EQ(state.y(), 9);
    GTEST_ASSERT_EQ(state.theta(), 10);
}

#pragma endregion //ARRTS_StartState

#pragma region ARRTS_GoalState

TEST(ARRTS_GoalState, SetAndGet)
{
    ArrtsService service;
    service.SetGoalState(5, 6, 7);
    State state = service.GetGoalState();
    GTEST_ASSERT_EQ(state.x(), 5);
    GTEST_ASSERT_EQ(state.y(), 6);
    GTEST_ASSERT_EQ(state.theta(), 7);
}

TEST(ARRTS_GoalState, SetUpdateAndGet)
{
    ArrtsService service;
    service.SetGoalState(5, 6, 7);
    service.SetGoalState(8, 9, 10);
    State state = service.GetGoalState();
    GTEST_ASSERT_EQ(state.x(), 8);
    GTEST_ASSERT_EQ(state.y(), 9);
    GTEST_ASSERT_EQ(state.theta(), 10);
}

#pragma endregion //ARRTS_GoalState

#pragma region ARRTS_Obstacles

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
    GTEST_ASSERT_EQ(obs.x(), 1);
    GTEST_ASSERT_EQ(obs.y(), 2);
    GTEST_ASSERT_EQ(obs.radius(), 3);
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
    
    GTEST_ASSERT_EQ(obs1.x(), 1);
    GTEST_ASSERT_EQ(obs2.x(), 2);
    GTEST_ASSERT_EQ(obs3.x(), 3);

    GTEST_ASSERT_EQ(obs1.y(), 4);
    GTEST_ASSERT_EQ(obs2.y(), 5);
    GTEST_ASSERT_EQ(obs3.y(), 6);

    GTEST_ASSERT_EQ(obs1.radius(), 7);
    GTEST_ASSERT_EQ(obs2.radius(), 8);
    GTEST_ASSERT_EQ(obs3.radius(), 9);
}

TEST(ARRTS_Obstacles, AddSingleThenSingleObstacle_CheckNum)
{
    ArrtsService service;
    service.AddObstacle(1, 2, 3);
    service.AddObstacle(1, 2, 3);
    GTEST_ASSERT_EQ(service.GetNumObstacles(), 2);
}

TEST(ARRTS_Obstacles, AddSingleThenSingleObstacle_CheckVals)
{
    ArrtsService service;
    service.AddObstacle(1, 2, 3);
    service.AddObstacle(4, 5, 6);

    Obstacle obs1 = service.GetObstacle(0);
    Obstacle obs2 = service.GetObstacle(1);

    GTEST_ASSERT_EQ(obs1.x(), 1);
    GTEST_ASSERT_EQ(obs2.x(), 4);

    GTEST_ASSERT_EQ(obs1.y(), 2);
    GTEST_ASSERT_EQ(obs2.y(), 5);

    GTEST_ASSERT_EQ(obs1.radius(), 3);
    GTEST_ASSERT_EQ(obs2.radius(), 6);
}

TEST(ARRTS_Obstacles, AddMultipleThenMultipleObstacle_CheckNum)
{
    ArrtsService service;
    const double x1[] = { 1, 2 };
    const double y1[] = { 1, 2 };
    const double r1[] = { 1, 2 };
    const double x2[] = { 1, 2 };
    const double y2[] = { 1, 2 };
    const double r2[] = { 1, 2 };
    service.AddObstacles(x1, y1, r1, 2);
    service.AddObstacles(x2, y2, r2, 2);
    GTEST_ASSERT_EQ(service.GetNumObstacles(), 4);
}

TEST(ARRTS_Obstacles, AddMultipleThenMultipleObstacle_CheckVals)
{
    ArrtsService service;
    const double x1[] = { 1, 2 };
    const double y1[] = { 3, 4 };
    const double r1[] = { 5, 6 };
    const double x2[] = { 7, 8 };
    const double y2[] = { 9, 10 };
    const double r2[] = { 11, 12 };
    service.AddObstacles(x1, y1, r1, 2);
    service.AddObstacles(x2, y2, r2, 2);
    
    Obstacle obs1 = service.GetObstacle(0);
    Obstacle obs2 = service.GetObstacle(1);
    Obstacle obs3 = service.GetObstacle(2);
    Obstacle obs4 = service.GetObstacle(3);

    GTEST_ASSERT_EQ(obs1.x(), 1);
    GTEST_ASSERT_EQ(obs2.x(), 2);
    GTEST_ASSERT_EQ(obs3.x(), 7);
    GTEST_ASSERT_EQ(obs4.x(), 8);

    GTEST_ASSERT_EQ(obs1.y(), 3);
    GTEST_ASSERT_EQ(obs2.y(), 4);
    GTEST_ASSERT_EQ(obs3.y(), 9);
    GTEST_ASSERT_EQ(obs4.y(), 10);

    GTEST_ASSERT_EQ(obs1.radius(), 5);
    GTEST_ASSERT_EQ(obs2.radius(), 6);
    GTEST_ASSERT_EQ(obs3.radius(), 11);
    GTEST_ASSERT_EQ(obs4.radius(), 12);
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

    GTEST_ASSERT_EQ(obs1.x(), 1);
    GTEST_ASSERT_EQ(obs2.x(), 4);
    GTEST_ASSERT_EQ(obs3.x(), 5);
    GTEST_ASSERT_EQ(obs4.x(), 6);

    GTEST_ASSERT_EQ(obs1.y(), 2);
    GTEST_ASSERT_EQ(obs2.y(), 7);
    GTEST_ASSERT_EQ(obs3.y(), 8);
    GTEST_ASSERT_EQ(obs4.y(), 9);

    GTEST_ASSERT_EQ(obs1.radius(), 3);
    GTEST_ASSERT_EQ(obs2.radius(), 10);
    GTEST_ASSERT_EQ(obs3.radius(), 11);
    GTEST_ASSERT_EQ(obs4.radius(), 12);
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

    GTEST_ASSERT_EQ(obs1.x(), 4);
    GTEST_ASSERT_EQ(obs2.x(), 5);
    GTEST_ASSERT_EQ(obs3.x(), 6);
    GTEST_ASSERT_EQ(obs4.x(), 1);

    GTEST_ASSERT_EQ(obs1.y(), 7);
    GTEST_ASSERT_EQ(obs2.y(), 8);
    GTEST_ASSERT_EQ(obs3.y(), 9);
    GTEST_ASSERT_EQ(obs4.y(), 2);

    GTEST_ASSERT_EQ(obs1.radius(), 10);
    GTEST_ASSERT_EQ(obs2.radius(), 11);
    GTEST_ASSERT_EQ(obs3.radius(), 12);
    GTEST_ASSERT_EQ(obs4.radius(), 3);
}

TEST(ARRTS_Obstacles, AddFromFile_CheckNum)
{
    ArrtsService service;
    FILE* file = fopen("./test/obstacles.txt", "r");
    service.AddObstaclesFromFile(file);

    GTEST_ASSERT_EQ(service.GetNumObstacles(), 23);
}

TEST(ARRTS_Obstacles, AddFromFile_CheckVals)
{
    ArrtsService service;
    FILE* file = fopen("./test/obstacles.txt", "r");
    service.AddObstaclesFromFile(file);

    Obstacle obs1 = service.GetObstacle(0);
    Obstacle obs2 = service.GetObstacle(11);
    Obstacle obs3 = service.GetObstacle(22);

    GTEST_ASSERT_EQ(obs1.x(), 80);
    GTEST_ASSERT_EQ(obs2.x(), 35);
    GTEST_ASSERT_EQ(obs3.x(), 60);

    GTEST_ASSERT_EQ(obs1.y(), 40);
    GTEST_ASSERT_EQ(obs2.y(), 80);
    GTEST_ASSERT_EQ(obs3.y(), 100);

    GTEST_ASSERT_EQ(obs1.radius(), 8);
    GTEST_ASSERT_EQ(obs2.radius(), 8);
    GTEST_ASSERT_EQ(obs3.radius(), 8);
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

#pragma endregion //ARRTS_Obstacles