#include <gtest/gtest.h>
#include "../ArrtsParams.hpp"

#pragma region ArrtsParams_StartState

TEST(ArrtsParams_StartState, Initialize_CheckVals)
{
    State start(5, 6, 7, 8);
    State goal(9, 10, 11, -5);
    vector<Obstacle> obstacles;
    double goalRadius = 5.5;
    ArrtsParams params(start, goal, obstacles, goalRadius);

    State state = params.start();
    GTEST_ASSERT_EQ(state.x(), 5);
    GTEST_ASSERT_EQ(state.y(), 6);
    GTEST_ASSERT_EQ(state.z(), 7);
    GTEST_ASSERT_EQ(state.theta(), 8);
}

TEST(ArrtsParams_StartState, InitializeFromDataDirectory_CheckVals)
{
    ArrtsParams params("./test");

    GTEST_ASSERT_EQ(params.start().x(), 5);
    GTEST_ASSERT_EQ(params.start().y(), 0);
    GTEST_ASSERT_EQ(params.start().z(), 100);
    GTEST_ASSERT_EQ(params.start().theta(), 0);
}

#pragma endregion //ArrtsParams_StartState

#pragma region ArrtsParams_GoalState

TEST(ArrtsParams_GoalState, Initialize_CheckVals)
{
    State start(5, 6, 7, 8);
    State goal(9, 10, 11, -5);
    vector<Obstacle> obstacles;
    double goalRadius = 5.5;
    ArrtsParams params(start, goal, obstacles, goalRadius);

    State state = params.goal();
    GTEST_ASSERT_EQ(state.x(), 9);
    GTEST_ASSERT_EQ(state.y(), 10);
    GTEST_ASSERT_EQ(state.z(), 11);
    GTEST_ASSERT_EQ(state.theta(), -5);
}

TEST(ArrtsParams_GoalState, InitializeFromDataDirectory_CheckVals)
{
    ArrtsParams params("./test");

    GTEST_ASSERT_EQ(params.goal().x(), 100);
    GTEST_ASSERT_EQ(params.goal().y(), 60);
    GTEST_ASSERT_EQ(params.goal().z(), 5);
    GTEST_ASSERT_EQ(params.goal().theta(), 0);
    GTEST_ASSERT_EQ(params.goalRadius(), 2.5);
}

#pragma endregion //ArrtsParams_GoalState

#pragma region ArrtsParams_Obstacles

TEST(ArrtsParams_Obstacles, InitializeOneObstacle_CheckNum)
{
    State start(5, 6, 1, 8);
    State goal(9, 10, 11, -5);
    vector<Obstacle> obstacles = { Obstacle(1, 1, 1, 1) };
    double goalRadius = 5.5;
    ArrtsParams params(start, goal, obstacles, goalRadius);

    GTEST_ASSERT_EQ(params.obstacles().size(), 1);
}

TEST(ArrtsParams_Obstacles, InitializeMultipleObstacle_CheckNum)
{
    State start(5, 1, 1, 8);
    State goal(9, 10, 11, -5);
    vector<Obstacle> obstacles = { Obstacle(1, 1, 1, 1), Obstacle(-1, -1, -1, 3), Obstacle(0, 0, 0, 5)};
    double goalRadius = 5.5;
    ArrtsParams params(start, goal, obstacles, goalRadius);

    GTEST_ASSERT_EQ(params.obstacles().size(), 3);
}

TEST(ArrtsParams_Obstacles, InitializeOneObstacle_CheckVals)
{
    State start(5, 6, 7, 8);
    State goal(9, 10, 11, -5);
    vector<Obstacle> obstacles = { Obstacle(1, 2, 3, 4) };
    double goalRadius = 5.5;
    ArrtsParams params(start, goal, obstacles, goalRadius);

    GTEST_ASSERT_EQ(params.obstacles(0).x(), 1);
    GTEST_ASSERT_EQ(params.obstacles(0).y(), 2);
    GTEST_ASSERT_EQ(params.obstacles(0).z(), 3);
    GTEST_ASSERT_EQ(params.obstacles(0).radius(), 4);
}

TEST(ArrtsParams_Obstacles, InitializeMultipleObstacle_CheckVals)
{
    State start(5, 6, 7, 8);
    State goal(9, 10, 11, -5);
    vector<Obstacle> obstacles = { Obstacle(1, 2, 3, 4), Obstacle(-1, -1, -1, 3), Obstacle(0, 0, 0, 5)};
    double goalRadius = 5.5;
    ArrtsParams params(start, goal, obstacles, goalRadius);

    GTEST_ASSERT_EQ(params.obstacles(0).x(), 1);
    GTEST_ASSERT_EQ(params.obstacles(0).y(), 2);
    GTEST_ASSERT_EQ(params.obstacles(0).z(), 3);
    GTEST_ASSERT_EQ(params.obstacles(0).radius(), 4);
    
    GTEST_ASSERT_EQ(params.obstacles(1).x(), -1);
    GTEST_ASSERT_EQ(params.obstacles(1).y(), -1);
    GTEST_ASSERT_EQ(params.obstacles(1).z(), -1);
    GTEST_ASSERT_EQ(params.obstacles(1).radius(), 3);

    GTEST_ASSERT_EQ(params.obstacles(2).x(), 0);
    GTEST_ASSERT_EQ(params.obstacles(2).y(), 0);
    GTEST_ASSERT_EQ(params.obstacles(2).z(), 0);
    GTEST_ASSERT_EQ(params.obstacles(2).radius(), 5);
}

TEST(ArrtsParams_Obstacles, AddFromFile_CheckVals)
{
    ArrtsParams params("./test");

    Obstacle obs1 = params.obstacles(0);
    Obstacle obs2 = params.obstacles(11);
    Obstacle obs3 = params.obstacles(22);

    GTEST_ASSERT_EQ(obs1.x(), 80);
    GTEST_ASSERT_EQ(obs2.x(), 35);
    GTEST_ASSERT_EQ(obs3.x(), 60);

    GTEST_ASSERT_EQ(obs1.y(), 40);
    GTEST_ASSERT_EQ(obs2.y(), 80);
    GTEST_ASSERT_EQ(obs3.y(), 100);

    GTEST_ASSERT_EQ(obs1.z(), 2);
    GTEST_ASSERT_EQ(obs2.z(), 24);
    GTEST_ASSERT_EQ(obs3.z(), 46);

    GTEST_ASSERT_EQ(obs1.radius(), 8);
    GTEST_ASSERT_EQ(obs2.radius(), 8);
    GTEST_ASSERT_EQ(obs3.radius(), 8);
}

#pragma endregion //ArrtsParams_Obstacles