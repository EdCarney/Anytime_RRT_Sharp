#include <gtest/gtest.h>
#include "ARRTS.hpp"
#include "Obstacle.hpp"
#include "Vehicle.hpp"
#include "workspaceGraph.hpp"

#pragma region ARRTS_StartState

TEST(ARRTS_StartState, SetAndGet)
{
    ArrtsService service;
    service.SetStartState(5, 6, 7);
    State state = service.GetStartState();
    GTEST_ASSERT_EQ(state.GetX(), 5);
    GTEST_ASSERT_EQ(state.GetY(), 6);
    GTEST_ASSERT_EQ(state.GetTheta(), 7);
}

TEST(ARRTS_StartState, SetUpdateAndGet)
{
    ArrtsService service;
    service.SetStartState(5, 6, 7);
    service.SetStartState(8, 9, 10);
    State state = service.GetStartState();
    GTEST_ASSERT_EQ(state.GetX(), 8);
    GTEST_ASSERT_EQ(state.GetY(), 9);
    GTEST_ASSERT_EQ(state.GetTheta(), 10);
}

#pragma endregion //ARRTS_StartState

#pragma region ARRTS_GoalState

TEST(ARRTS_GoalState, SetAndGet)
{
    ArrtsService service;
    service.SetGoalState(5, 6, 7);
    State state = service.GetGoalState();
    GTEST_ASSERT_EQ(state.GetX(), 5);
    GTEST_ASSERT_EQ(state.GetY(), 6);
    GTEST_ASSERT_EQ(state.GetTheta(), 7);
}

TEST(ARRTS_GoalState, SetUpdateAndGet)
{
    ArrtsService service;
    service.SetGoalState(5, 6, 7);
    service.SetGoalState(8, 9, 10);
    State state = service.GetGoalState();
    GTEST_ASSERT_EQ(state.GetX(), 8);
    GTEST_ASSERT_EQ(state.GetY(), 9);
    GTEST_ASSERT_EQ(state.GetTheta(), 10);
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
    GTEST_ASSERT_EQ(obs.GetX(), 1);
    GTEST_ASSERT_EQ(obs.GetY(), 2);
    GTEST_ASSERT_EQ(obs.GetRadius(), 3);
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
    
    GTEST_ASSERT_EQ(obs1.GetX(), 1);
    GTEST_ASSERT_EQ(obs2.GetX(), 2);
    GTEST_ASSERT_EQ(obs3.GetX(), 3);

    GTEST_ASSERT_EQ(obs1.GetY(), 4);
    GTEST_ASSERT_EQ(obs2.GetY(), 5);
    GTEST_ASSERT_EQ(obs3.GetY(), 6);

    GTEST_ASSERT_EQ(obs1.GetRadius(), 7);
    GTEST_ASSERT_EQ(obs2.GetRadius(), 8);
    GTEST_ASSERT_EQ(obs3.GetRadius(), 9);
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

    GTEST_ASSERT_EQ(obs1.GetX(), 1);
    GTEST_ASSERT_EQ(obs2.GetX(), 4);

    GTEST_ASSERT_EQ(obs1.GetY(), 2);
    GTEST_ASSERT_EQ(obs2.GetY(), 5);

    GTEST_ASSERT_EQ(obs1.GetRadius(), 3);
    GTEST_ASSERT_EQ(obs2.GetRadius(), 6);
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

    GTEST_ASSERT_EQ(obs1.GetX(), 1);
    GTEST_ASSERT_EQ(obs2.GetX(), 2);
    GTEST_ASSERT_EQ(obs3.GetX(), 7);
    GTEST_ASSERT_EQ(obs4.GetX(), 8);

    GTEST_ASSERT_EQ(obs1.GetY(), 3);
    GTEST_ASSERT_EQ(obs2.GetY(), 4);
    GTEST_ASSERT_EQ(obs3.GetY(), 9);
    GTEST_ASSERT_EQ(obs4.GetY(), 10);

    GTEST_ASSERT_EQ(obs1.GetRadius(), 5);
    GTEST_ASSERT_EQ(obs2.GetRadius(), 6);
    GTEST_ASSERT_EQ(obs3.GetRadius(), 11);
    GTEST_ASSERT_EQ(obs4.GetRadius(), 12);
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

    GTEST_ASSERT_EQ(obs1.GetX(), 1);
    GTEST_ASSERT_EQ(obs2.GetX(), 4);
    GTEST_ASSERT_EQ(obs3.GetX(), 5);
    GTEST_ASSERT_EQ(obs4.GetX(), 6);

    GTEST_ASSERT_EQ(obs1.GetY(), 2);
    GTEST_ASSERT_EQ(obs2.GetY(), 7);
    GTEST_ASSERT_EQ(obs3.GetY(), 8);
    GTEST_ASSERT_EQ(obs4.GetY(), 9);

    GTEST_ASSERT_EQ(obs1.GetRadius(), 3);
    GTEST_ASSERT_EQ(obs2.GetRadius(), 10);
    GTEST_ASSERT_EQ(obs3.GetRadius(), 11);
    GTEST_ASSERT_EQ(obs4.GetRadius(), 12);
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

    GTEST_ASSERT_EQ(obs1.GetX(), 4);
    GTEST_ASSERT_EQ(obs2.GetX(), 5);
    GTEST_ASSERT_EQ(obs3.GetX(), 6);
    GTEST_ASSERT_EQ(obs4.GetX(), 1);

    GTEST_ASSERT_EQ(obs1.GetY(), 7);
    GTEST_ASSERT_EQ(obs2.GetY(), 8);
    GTEST_ASSERT_EQ(obs3.GetY(), 9);
    GTEST_ASSERT_EQ(obs4.GetY(), 2);

    GTEST_ASSERT_EQ(obs1.GetRadius(), 10);
    GTEST_ASSERT_EQ(obs2.GetRadius(), 11);
    GTEST_ASSERT_EQ(obs3.GetRadius(), 12);
    GTEST_ASSERT_EQ(obs4.GetRadius(), 3);
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

    GTEST_ASSERT_EQ(obs1.GetX(), 80);
    GTEST_ASSERT_EQ(obs2.GetX(), 35);
    GTEST_ASSERT_EQ(obs3.GetX(), 60);

    GTEST_ASSERT_EQ(obs1.GetY(), 40);
    GTEST_ASSERT_EQ(obs2.GetY(), 80);
    GTEST_ASSERT_EQ(obs3.GetY(), 100);

    GTEST_ASSERT_EQ(obs1.GetRadius(), 8);
    GTEST_ASSERT_EQ(obs2.GetRadius(), 8);
    GTEST_ASSERT_EQ(obs3.GetRadius(), 8);
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

#pragma region Vehicle

TEST(Vehicle, AddOnePoint_CheckNum)
{
    Vehicle v;
    v.AddOffsetNode(1, 1);
    GTEST_ASSERT_EQ(v.GetNumNodes(), 1);
}

TEST(Vehicle, AddOnePoint_CheckVal)
{
    Vehicle v;
    v.AddOffsetNode(1, 2);
    Point n = v.GetOffsetNode(0);
    GTEST_ASSERT_EQ(n.GetX(), 1);
    GTEST_ASSERT_EQ(n.GetY(), 2);
}

TEST(Vehicle, AddMultiplePoint_CheckNum)
{
    Vehicle v;
    const double x[] = { 1, 2, 3 };
    const double y[] = { 1, 2, 3 };
    v.AddOffsetNodes(x, y, 3);
    GTEST_ASSERT_EQ(v.GetNumNodes(), 3);
}

TEST(Vehicle, AddMultiplePoint_CheckVals)
{
    Vehicle v;
    const double x[] = { 1, 2, 3 };
    const double y[] = { 4, 5, 6 };
    v.AddOffsetNodes(x, y, 3);

    Point n1 = v.GetOffsetNode(0);
    Point n2 = v.GetOffsetNode(1);
    Point n3 = v.GetOffsetNode(2);
    
    GTEST_ASSERT_EQ(n1.GetX(), 1);
    GTEST_ASSERT_EQ(n2.GetX(), 2);
    GTEST_ASSERT_EQ(n3.GetX(), 3);

    GTEST_ASSERT_EQ(n1.GetY(), 4);
    GTEST_ASSERT_EQ(n2.GetY(), 5);
    GTEST_ASSERT_EQ(n3.GetY(), 6);
}

TEST(Vehicle, AddSingleThenSinglePoint_CheckNum)
{
    Vehicle v;
    v.AddOffsetNode(1, 2);
    v.AddOffsetNode(1, 2);
    GTEST_ASSERT_EQ(v.GetNumNodes(), 2);
}

TEST(Vehicle, AddSingleThenSinglePoint_CheckVals)
{
    Vehicle v;
    v.AddOffsetNode(1, 2);
    v.AddOffsetNode(3, 4);

    Point n1 = v.GetOffsetNode(0);
    Point n2 = v.GetOffsetNode(1);

    GTEST_ASSERT_EQ(n1.GetX(), 1);
    GTEST_ASSERT_EQ(n2.GetX(), 3);

    GTEST_ASSERT_EQ(n1.GetY(), 2);
    GTEST_ASSERT_EQ(n2.GetY(), 4);
}

TEST(Vehicle, AddMultipleThenMultiplePoint_CheckNum)
{
    Vehicle v;
    const double x1[] = { 1, 2 };
    const double y1[] = { 1, 2 };
    const double x2[] = { 1, 2 };
    const double y2[] = { 1, 2 };
    v.AddOffsetNodes(x1, y1, 2);
    v.AddOffsetNodes(x2, y2, 2);
    GTEST_ASSERT_EQ(v.GetNumNodes(), 4);
}

TEST(Vehicle, AddMultipleThenMultiplePoint_CheckVals)
{
    Vehicle v;
    const double x1[] = { 1, 2 };
    const double y1[] = { 3, 4 };
    const double x2[] = { 5, 6 };
    const double y2[] = { 7, 8 };
    v.AddOffsetNodes(x1, y1, 2);
    v.AddOffsetNodes(x2, y2, 2);
    
    Point n1 = v.GetOffsetNode(0);
    Point n2 = v.GetOffsetNode(1);
    Point n3 = v.GetOffsetNode(2);
    Point n4 = v.GetOffsetNode(3);

    GTEST_ASSERT_EQ(n1.GetX(), 1);
    GTEST_ASSERT_EQ(n2.GetX(), 2);
    GTEST_ASSERT_EQ(n3.GetX(), 5);
    GTEST_ASSERT_EQ(n4.GetX(), 6);

    GTEST_ASSERT_EQ(n1.GetY(), 3);
    GTEST_ASSERT_EQ(n2.GetY(), 4);
    GTEST_ASSERT_EQ(n3.GetY(), 7);
    GTEST_ASSERT_EQ(n4.GetY(), 8);
}

TEST(Vehicle, AddSingleThenMultiplePoint_CheckNum)
{
    Vehicle v;
    v.AddOffsetNode(1, 2);
    const double x[] = { 1, 2, 3 };
    const double y[] = { 1, 2, 3 };
    const double r[] = { 1, 2, 3 };
    v.AddOffsetNodes(x, y, 3);
    GTEST_ASSERT_EQ(v.GetNumNodes(), 4);
}

TEST(Vehicle, AddSingleThenMultiplePoint_CheckVals)
{
    Vehicle v;
    v.AddOffsetNode(1, 2);
    const double x[] = { 4, 5, 6 };
    const double y[] = { 7, 8, 9 };
    const double r[] = { 10, 11, 12 };
    v.AddOffsetNodes(x, y, 3);
    
    Point n1 = v.GetOffsetNode(0);
    Point n2 = v.GetOffsetNode(1);
    Point n3 = v.GetOffsetNode(2);
    Point n4 = v.GetOffsetNode(3);

    GTEST_ASSERT_EQ(n1.GetX(), 1);
    GTEST_ASSERT_EQ(n2.GetX(), 4);
    GTEST_ASSERT_EQ(n3.GetX(), 5);
    GTEST_ASSERT_EQ(n4.GetX(), 6);

    GTEST_ASSERT_EQ(n1.GetY(), 2);
    GTEST_ASSERT_EQ(n2.GetY(), 7);
    GTEST_ASSERT_EQ(n3.GetY(), 8);
    GTEST_ASSERT_EQ(n4.GetY(), 9);
}

TEST(Vehicle, AddMultipleThenSinglePoint_CheckNum)
{
    Vehicle v;
    const double x[] = { 1, 2, 3 };
    const double y[] = { 1, 2, 3 };
    v.AddOffsetNodes(x, y, 3);
    v.AddOffsetNode(1, 2);
    GTEST_ASSERT_EQ(v.GetNumNodes(), 4);
}

TEST(Vehicle, AddMultipleThenSinglePoint_CheckVals)
{
    Vehicle v;
    const double x[] = { 4, 5, 6 };
    const double y[] = { 7, 8, 9 };
    v.AddOffsetNodes(x, y, 3);
    v.AddOffsetNode(1, 2);
    
    Point n1 = v.GetOffsetNode(0);
    Point n2 = v.GetOffsetNode(1);
    Point n3 = v.GetOffsetNode(2);
    Point n4 = v.GetOffsetNode(3);

    GTEST_ASSERT_EQ(n1.GetX(), 4);
    GTEST_ASSERT_EQ(n2.GetX(), 5);
    GTEST_ASSERT_EQ(n3.GetX(), 6);
    GTEST_ASSERT_EQ(n4.GetX(), 1);

    GTEST_ASSERT_EQ(n1.GetY(), 7);
    GTEST_ASSERT_EQ(n2.GetY(), 8);
    GTEST_ASSERT_EQ(n3.GetY(), 9);
    GTEST_ASSERT_EQ(n4.GetY(), 2);
}

TEST(Vehicle, AddFromFile_CheckNum)
{
    Vehicle v;
    FILE* file = fopen("./test/robot.txt", "r");
    v.AddOffsetNodesFromFile(file);

    GTEST_ASSERT_EQ(v.GetNumNodes(), 37);
}

TEST(Vehicle, AddFromFile_CheckVals)
{
    Vehicle v;
    FILE* file = fopen("./test/robot.txt", "r");
    v.AddOffsetNodesFromFile(file);

    Point n1 = v.GetOffsetNode(0);
    Point n2 = v.GetOffsetNode(20);
    Point n3 = v.GetOffsetNode(36);

    GTEST_ASSERT_EQ(n1.GetX(), 0);
    GTEST_ASSERT_EQ(n2.GetX(), -1);
    GTEST_ASSERT_EQ(n3.GetX(), -0.8);

    GTEST_ASSERT_EQ(n1.GetY(), 0);
    GTEST_ASSERT_EQ(n2.GetY(), 0);
    GTEST_ASSERT_EQ(n3.GetY(), -0.3);
}

TEST(Vehicle, GetFirstPointWhenNone)
{
    Vehicle v;
    EXPECT_ANY_THROW(v.GetOffsetNode(0));
    EXPECT_ANY_THROW(v.GetNode(0));
}

TEST(Vehicle, GetNegativePointWhenNone)
{
    Vehicle v;
    EXPECT_ANY_THROW(v.GetOffsetNode(-1));
    EXPECT_ANY_THROW(v.GetNode(-1));
}

TEST(Vehicle, GetSecondPointWhenOne)
{
    Vehicle v;
    v.AddOffsetNode(1, 2);
    EXPECT_ANY_THROW(v.GetOffsetNode(1));
    EXPECT_ANY_THROW(v.GetNode(1));
}

TEST(Vehicle, Initialize_CheckVals)
{
    Vehicle v;

    GTEST_ASSERT_EQ(v.GetState().GetX(), 0);
    GTEST_ASSERT_EQ(v.GetState().GetY(), 0);
    GTEST_ASSERT_EQ(v.GetState().GetTheta(), 0);
    GTEST_ASSERT_EQ(v.GetBoundingRadius(), 0);
    GTEST_ASSERT_EQ(v.GetNumNodes(), 0);
    EXPECT_TRUE(v.GetOffsetNodes() == NULL);
}

TEST(Vehicle, UpdateState_CheckVals)
{
    Vehicle v;
    State p = { 1, 2, 3 };
    v.UpdateState(p);

    GTEST_ASSERT_EQ(v.GetState().GetX(), 1);
    GTEST_ASSERT_EQ(v.GetState().GetY(), 2);
    GTEST_ASSERT_EQ(v.GetState().GetTheta(), 3);
    GTEST_ASSERT_EQ(v.GetBoundingRadius(), 0);
    GTEST_ASSERT_EQ(v.GetNumNodes(), 0);
    GTEST_ASSERT_EQ(v.GetNumNodes(), 0);
    EXPECT_TRUE(v.GetOffsetNodes() == NULL);
}

TEST(Vehicle, AddNodesUpdateStateNoRotation_CheckVals)
{
    Vehicle v;
    State p = { 1, 2, 0 };
    const double x[] = { -0.5, 0.5, 0.5, -0.5 };
    const double y[] = { -0.5, -0.5, 0.5, 0.5 };

    v.AddOffsetNodes(x, y, 4);
    v.UpdateState(p);

    double tol = 0.001;

    GTEST_ASSERT_EQ(v.GetState().GetX(), 1);
    GTEST_ASSERT_EQ(v.GetState().GetY(), 2);
    GTEST_ASSERT_EQ(v.GetState().GetTheta(), 0);

    EXPECT_NEAR(v.GetBoundingRadius(), 0.707106, tol);

    EXPECT_NEAR(v.GetNode(0).GetX(), 0.5, tol);
    EXPECT_NEAR(v.GetNode(1).GetX(), 1.5, tol);
    EXPECT_NEAR(v.GetNode(2).GetX(), 1.5, tol);
    EXPECT_NEAR(v.GetNode(3).GetX(), 0.5, tol);

    EXPECT_NEAR(v.GetNode(0).GetY(), 1.5, tol);
    EXPECT_NEAR(v.GetNode(1).GetY(), 1.5, tol);
    EXPECT_NEAR(v.GetNode(2).GetY(), 2.5, tol);
    EXPECT_NEAR(v.GetNode(3).GetY(), 2.5, tol);

    GTEST_ASSERT_EQ(v.GetNumNodes(), 4);
}

TEST(Vehicle, AddNodesUpdateStateWithPosRotation_CheckVals)
{
    Vehicle v;
    const double x[] = { -0.5, 0.5, 0.5, -0.5 };
    const double y[] = { -0.5, -0.5, 0.5, 0.5 };

    v.AddOffsetNodes(x, y, 4);

    State p = { 1, 2, M_PI / 4.0 };
    v.UpdateState(p);

    double tol = 0.001;

    GTEST_ASSERT_EQ(v.GetState().GetX(), 1);
    GTEST_ASSERT_EQ(v.GetState().GetY(), 2);
    GTEST_ASSERT_EQ(v.GetState().GetTheta(), M_PI / 4.0);

    EXPECT_NEAR(v.GetBoundingRadius(), 0.707106, tol);

    EXPECT_NEAR(v.GetNode(0).GetX(), 1.00000, tol);
    EXPECT_NEAR(v.GetNode(1).GetX(), 1.70711, tol);
    EXPECT_NEAR(v.GetNode(2).GetX(), 1.00000, tol);
    EXPECT_NEAR(v.GetNode(3).GetX(), 0.29289, tol);

    EXPECT_NEAR(v.GetNode(0).GetY(), 1.29289, tol);
    EXPECT_NEAR(v.GetNode(1).GetY(), 2.00000, tol);
    EXPECT_NEAR(v.GetNode(2).GetY(), 2.70711, tol);
    EXPECT_NEAR(v.GetNode(3).GetY(), 2.00000, tol);

    GTEST_ASSERT_EQ(v.GetNumNodes(), 4);
}

#pragma endregion //Vehicle

#pragma region Obstacle

TEST(Obstacle, DefaultInitialize_CheckVals)
{
    Obstacle o;
    GTEST_ASSERT_EQ(o.GetX(), 0);
    GTEST_ASSERT_EQ(o.GetY(), 0);
    GTEST_ASSERT_EQ(o.GetRadius(), 0);
}

TEST(Obstacle, InitializeWithPointAndRad_CheckVals)
{
    Point p = { 1, 2 };
    Obstacle o(p, 3);
    GTEST_ASSERT_EQ(o.GetX(), 1);
    GTEST_ASSERT_EQ(o.GetY(), 2);
    GTEST_ASSERT_EQ(o.GetRadius(), 3);
}

TEST(Obstacle, InitializeWithIndividualVals_CheckVals)
{
    Obstacle o(1, 2, 3);
    GTEST_ASSERT_EQ(o.GetX(), 1);
    GTEST_ASSERT_EQ(o.GetY(), 2);
    GTEST_ASSERT_EQ(o.GetRadius(), 3);
}

TEST(Obstacle, PointIntersect_DoesIntersect)
{
    Obstacle o(1, 1, 5);
    Point p = { 2, 3 };
    EXPECT_TRUE(o.Intersects(p));
}

TEST(Obstacle, PointIntersect_DoesNotIntersect)
{
    Obstacle o(1, 1, 5);
    Point p = { 8, -7 };
    EXPECT_FALSE(o.Intersects(p));
}

TEST(Obstacle, CircleIntersect_DoesIntersectPoint)
{
    Obstacle o(1, 1, 5);
    Circle c = { 2, 2, 1 };
    EXPECT_TRUE(o.Intersects(c));
}

TEST(Obstacle, CircleIntersect_DoesIntersectRadius)
{
    Obstacle o(1, 1, 5);
    Circle c = { 7, 0, 5 };
    EXPECT_TRUE(o.Intersects(c));
}

TEST(Obstacle, CircleIntersect_DoesNotIntersect)
{
    Obstacle o(1, 1, 5);
    Circle c = { -5, -5, 1 };
    EXPECT_FALSE(o.Intersects(c));
}

TEST(Obstacle, RectagleIntersect_FullyContained)
{
    Obstacle o(3, 3, 1);
    Rectangle r = { { 1, 1 }, { 5, 5 } };
    EXPECT_TRUE(o.Intersects(r));
}

TEST(Obstacle, RectagleIntersect_PointContained)
{
    Obstacle o(2, 2, 4);
    Rectangle r = { { 1, 1 }, { 5, 5 } };
    EXPECT_TRUE(o.Intersects(r));
}

TEST(Obstacle, RectagleIntersect_PointNotContainedRadiusContained_Left)
{
    Obstacle o(0, 3, 3);
    Rectangle r = { { 1, 1 }, { 5, 5 } };
    EXPECT_TRUE(o.Intersects(r));
}

TEST(Obstacle, RectagleIntersect_PointNotContainedRadiusContained_Right)
{
    Obstacle o(6, 3, 3);
    Rectangle r = { { 1, 1 }, { 5, 5 } };
    EXPECT_TRUE(o.Intersects(r));
}

TEST(Obstacle, RectagleIntersect_PointNotContainedRadiusContained_Top)
{
    Obstacle o(3, 6, 3);
    Rectangle r = { { 1, 1 }, { 5, 5 } };
    EXPECT_TRUE(o.Intersects(r));
}

TEST(Obstacle, RectagleIntersect_PointNotContainedRadiusContained_Bottom)
{
    Obstacle o(3, 0, 3);
    Rectangle r = { { 1, 1 }, { 5, 5 } };
    EXPECT_TRUE(o.Intersects(r));
}

TEST(Obstacle, RectagleIntersect_DoesNotIntersect)
{
    Obstacle o(-3, -3, 3);
    Rectangle r = { { 1, 1 }, { 5, 5 } };
    EXPECT_FALSE(o.Intersects(r));
}

#pragma endregion //Obstacle

int main(int argc, char* argv[])
{
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}