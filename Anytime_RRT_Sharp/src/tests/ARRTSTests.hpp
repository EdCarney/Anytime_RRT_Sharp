#include <gtest/gtest.h>
#include "../ARRTS.hpp"

#pragma region ARRTS_StartState

TEST(ARRTS_StartState, SetAndGet)
{
    ArrtsService service;
    service.setStartState(5, 6, 7);
    State state = service.startState();
    GTEST_ASSERT_EQ(state.x(), 5);
    GTEST_ASSERT_EQ(state.y(), 6);
    GTEST_ASSERT_EQ(state.theta(), 7);
}

TEST(ARRTS_StartState, SetUpdateAndGet)
{
    ArrtsService service;
    service.setStartState(5, 6, 7);
    service.setStartState(8, 9, 10);
    State state = service.startState();
    GTEST_ASSERT_EQ(state.x(), 8);
    GTEST_ASSERT_EQ(state.y(), 9);
    GTEST_ASSERT_EQ(state.theta(), 10);
}

TEST(ARRTS_StartState, ReadStatesFromFileAndGetStart)
{
    ArrtsService service;
    FILE* file = fopen("./test/states.txt", "r");
    service.readStatesFromFile(file);
    GTEST_ASSERT_EQ(service.startState().x(), 5);
    GTEST_ASSERT_EQ(service.startState().y(), 60);
    GTEST_ASSERT_EQ(service.startState().theta(), 0);
}

TEST(ARRTS_StartState, InitializeFromDataDirectoryAndGet)
{
    ArrtsService service;
    service.initializeFromDataDirectory("./test");
    GTEST_ASSERT_EQ(service.startState().x(), 5);
    GTEST_ASSERT_EQ(service.startState().y(), 60);
    GTEST_ASSERT_EQ(service.startState().theta(), 0);
}

#pragma endregion //ARRTS_StartState

#pragma region ARRTS_GoalState

TEST(ARRTS_GoalState, SetAndGet)
{
    ArrtsService service;
    service.setGoalState(5, 6, 7);
    State state = service.goalState();
    GTEST_ASSERT_EQ(state.x(), 5);
    GTEST_ASSERT_EQ(state.y(), 6);
    GTEST_ASSERT_EQ(state.theta(), 7);
}

TEST(ARRTS_GoalState, SetUpdateAndGet)
{
    ArrtsService service;
    service.setGoalState(5, 6, 7);
    service.setGoalState(8, 9, 10);
    State state = service.goalState();
    GTEST_ASSERT_EQ(state.x(), 8);
    GTEST_ASSERT_EQ(state.y(), 9);
    GTEST_ASSERT_EQ(state.theta(), 10);
}

TEST(ARRTS_GoalState, ReadStatesFromFileAndGetGoal)
{
    ArrtsService service;
    FILE* file = fopen("./test/states.txt", "r");
    service.readStatesFromFile(file);
    GTEST_ASSERT_EQ(service.goalState().x(), 100);
    GTEST_ASSERT_EQ(service.goalState().y(), 60);
    GTEST_ASSERT_EQ(service.goalState().theta(), 0);
}

TEST(ARRTS_GoalState, InitializeFromDataDirectoryAndGet)
{
    ArrtsService service;
    service.initializeFromDataDirectory("./test");
    GTEST_ASSERT_EQ(service.goalState().x(), 100);
    GTEST_ASSERT_EQ(service.goalState().y(), 60);
    GTEST_ASSERT_EQ(service.goalState().theta(), 0);
}

#pragma endregion //ARRTS_GoalState

#pragma region ARRTS_Obstacles

TEST(ARRTS_Obstacles, AddOneObstacle_CheckNum)
{
    ArrtsService service;
    service.addObstacle(1, 1, 1);
    GTEST_ASSERT_EQ(service.obstacles().size(), 1);
}

TEST(ARRTS_Obstacles, AddOneObstacle_CheckVal)
{
    ArrtsService service;
    service.addObstacle(1, 2, 3);
    Obstacle obs = service.obstacles(0);
    GTEST_ASSERT_EQ(obs.x(), 1);
    GTEST_ASSERT_EQ(obs.y(), 2);
    GTEST_ASSERT_EQ(obs.radius(), 3);
}

TEST(ARRTS_Obstacles, AddMultipleObstacle_CheckNum)
{
    ArrtsService service;
    vector<double> x = { 1, 2, 3 };
    vector<double> y = { 1, 2, 3 };
    vector<double> r = { 1, 2, 3 };
    service.addObstacles(x, y, r);
    GTEST_ASSERT_EQ(service.obstacles().size(), 3);
}

TEST(ARRTS_Obstacles, AddMultipleObstacle_CheckVals)
{
    ArrtsService service;
    vector<double> x = { 1, 2, 3 };
    vector<double> y = { 4, 5, 6 };
    vector<double> r = { 7, 8, 9 };
    service.addObstacles(x, y, r);

    Obstacle obs1 = service.obstacles(0);
    Obstacle obs2 = service.obstacles(1);
    Obstacle obs3 = service.obstacles(2);
    
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
    service.addObstacle(1, 2, 3);
    service.addObstacle(1, 2, 3);
    GTEST_ASSERT_EQ(service.obstacles().size(), 2);
}

TEST(ARRTS_Obstacles, AddSingleThenSingleObstacle_CheckVals)
{
    ArrtsService service;
    service.addObstacle(1, 2, 3);
    service.addObstacle(4, 5, 6);

    Obstacle obs1 = service.obstacles(0);
    Obstacle obs2 = service.obstacles(1);

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
    vector<double> x1 = { 1, 2 };
    vector<double> y1 = { 1, 2 };
    vector<double> r1 = { 1, 2 };
    vector<double> x2 = { 1, 2 };
    vector<double> y2 = { 1, 2 };
    vector<double> r2 = { 1, 2 };
    service.addObstacles(x1, y1, r1);
    service.addObstacles(x2, y2, r2);
    GTEST_ASSERT_EQ(service.obstacles().size(), 4);
}

TEST(ARRTS_Obstacles, AddMultipleThenMultipleObstacle_CheckVals)
{
    ArrtsService service;
    vector<double> x1 = { 1, 2 };
    vector<double> y1 = { 3, 4 };
    vector<double> r1 = { 5, 6 };
    vector<double> x2 = { 7, 8 };
    vector<double> y2 = { 9, 10 };
    vector<double> r2 = { 11, 12 };
    service.addObstacles(x1, y1, r1);
    service.addObstacles(x2, y2, r2);
    
    Obstacle obs1 = service.obstacles(0);
    Obstacle obs2 = service.obstacles(1);
    Obstacle obs3 = service.obstacles(2);
    Obstacle obs4 = service.obstacles(3);

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
    service.addObstacle(1, 2, 3);
    vector<double> x = { 1, 2, 3 };
    vector<double> y = { 1, 2, 3 };
    vector<double> r = { 1, 2, 3 };
    service.addObstacles(x, y, r);
    GTEST_ASSERT_EQ(service.obstacles().size(), 4);
}

TEST(ARRTS_Obstacles, AddSingleThenMultipleObstacle_CheckVals)
{
    ArrtsService service;
    service.addObstacle(1, 2, 3);
    vector<double> x = { 4, 5, 6 };
    vector<double> y = { 7, 8, 9 };
    vector<double> r = { 10, 11, 12 };
    service.addObstacles(x, y, r);
    
    Obstacle obs1 = service.obstacles(0);
    Obstacle obs2 = service.obstacles(1);
    Obstacle obs3 = service.obstacles(2);
    Obstacle obs4 = service.obstacles(3);

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
    vector<double> x = { 1, 2, 3 };
    vector<double> y = { 1, 2, 3 };
    vector<double> r = { 1, 2, 3 };
    service.addObstacles(x, y, r);
    service.addObstacle(1, 2, 3);
    GTEST_ASSERT_EQ(service.obstacles().size(), 4);
}

TEST(ARRTS_Obstacles, AddMultipleThenSingleObstacle_CheckVals)
{
    ArrtsService service;
    vector<double> x = { 4, 5, 6 };
    vector<double> y = { 7, 8, 9 };
    vector<double> r = { 10, 11, 12 };
    service.addObstacles(x, y, r);
    service.addObstacle(1, 2, 3);
    
    Obstacle obs1 = service.obstacles(0);
    Obstacle obs2 = service.obstacles(1);
    Obstacle obs3 = service.obstacles(2);
    Obstacle obs4 = service.obstacles(3);

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
    service.readObstaclesFromFile(file);

    GTEST_ASSERT_EQ(service.obstacles().size(), 23);
}

TEST(ARRTS_Obstacles, AddFromFile_CheckVals)
{
    ArrtsService service;
    FILE* file = fopen("./test/obstacles.txt", "r");
    service.readObstaclesFromFile(file);

    Obstacle obs1 = service.obstacles(0);
    Obstacle obs2 = service.obstacles(11);
    Obstacle obs3 = service.obstacles(22);

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
    EXPECT_ANY_THROW(service.obstacles(0));
}

TEST(ARRTS_Obstacles, GetNegativeObstacleWhenNone)
{
    ArrtsService service;
    EXPECT_ANY_THROW(service.obstacles(-1));
}

TEST(ARRTS_Obstacles, GetSecondObstacleWhenOne)
{
    ArrtsService service;
    service.addObstacle(1, 2, 3);
    EXPECT_ANY_THROW(service.obstacles(1));
}

TEST(ARRTS_Obstacles, InitializeFromDataDirectoryAndGet)
{
    ArrtsService service;
    service.initializeFromDataDirectory("./test");
    
    Obstacle obs1 = service.obstacles(0);
    Obstacle obs2 = service.obstacles(11);
    Obstacle obs3 = service.obstacles(22);

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

#pragma endregion //ARRTS_Obstacles

#pragma region ARRTS_Limits

TEST(ARRTS_Limits, SetWithPointsAndGet)
{
    ArrtsService service;
    Point p1(1, 2), p2(3, 4);
    service.setLimits(p1, p2);
    GTEST_ASSERT_EQ(service.limits().minPoint().x(), p1.x());
    GTEST_ASSERT_EQ(service.limits().minPoint().y(), p1.y());
    GTEST_ASSERT_EQ(service.limits().maxPoint().x(), p2.x());
    GTEST_ASSERT_EQ(service.limits().maxPoint().y(), p2.y());
}

TEST(ARRTS_Limits, SetWithLimitsAndGet)
{
    ArrtsService service;
    double minX = 2, minY = 3, maxX = 4, maxY = 5;
    service.setLimits(minX, minY, maxX, maxY);
    GTEST_ASSERT_EQ(service.limits().minPoint().x(), minX);
    GTEST_ASSERT_EQ(service.limits().minPoint().y(), minY);
    GTEST_ASSERT_EQ(service.limits().maxPoint().x(), maxX);
    GTEST_ASSERT_EQ(service.limits().maxPoint().y(), maxY);
}

#pragma endregion //ARRTS_Limits

#pragma region ARRTS_Vehicle

TEST(ARRTS_Vehicle, AddMultiplePoint_CheckVals)
{
    ArrtsService service;
    vector<double> x = { 1, 2, 3 };
    vector<double> y = { 4, 5, 6 };
    service.setVehicle(x, y);

    Point n1 = service.vehicle().offsetNodes(0);
    Point n2 = service.vehicle().offsetNodes(1);
    Point n3 = service.vehicle().offsetNodes(2);
    
    GTEST_ASSERT_EQ(n1.x(), 1);
    GTEST_ASSERT_EQ(n2.x(), 2);
    GTEST_ASSERT_EQ(n3.x(), 3);

    GTEST_ASSERT_EQ(n1.y(), 4);
    GTEST_ASSERT_EQ(n2.y(), 5);
    GTEST_ASSERT_EQ(n3.y(), 6);
}

TEST(ARRTS_Vehicle, ReadVehicleFromFileAndGet)
{
    ArrtsService service;
    FILE* file = fopen("./test/robot.txt", "r");
    service.readVehicleFromFile(file);

    Point n1 = service.vehicle().offsetNodes(0);
    Point n2 = service.vehicle().offsetNodes(20);
    Point n3 = service.vehicle().offsetNodes(36);

    GTEST_ASSERT_EQ(service.vehicle().offsetNodes().size(), 37);

    GTEST_ASSERT_EQ(n1.x(), 0);
    GTEST_ASSERT_EQ(n2.x(), -1);
    GTEST_ASSERT_EQ(n3.x(), -0.8);

    GTEST_ASSERT_EQ(n1.y(), 0);
    GTEST_ASSERT_EQ(n2.y(), 0);
    GTEST_ASSERT_EQ(n3.y(), -0.3);
}

TEST(ARRTS_Vehicle, InitializeFromDataDirectoryAndGet)
{
    ArrtsService service;
    service.initializeFromDataDirectory("./test");
    
    Point n1 = service.vehicle().offsetNodes(0);
    Point n2 = service.vehicle().offsetNodes(20);
    Point n3 = service.vehicle().offsetNodes(36);

    GTEST_ASSERT_EQ(service.vehicle().offsetNodes().size(), 37);

    GTEST_ASSERT_EQ(n1.x(), 0);
    GTEST_ASSERT_EQ(n2.x(), -1);
    GTEST_ASSERT_EQ(n3.x(), -0.8);

    GTEST_ASSERT_EQ(n1.y(), 0);
    GTEST_ASSERT_EQ(n2.y(), 0);
    GTEST_ASSERT_EQ(n3.y(), -0.3);
}

#pragma endregion //ARRTS_Vehicle