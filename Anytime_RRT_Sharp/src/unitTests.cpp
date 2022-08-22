#include <gtest/gtest.h>
#include "ARRTS.hpp"

#pragma region ARRTS_StartPosition

TEST(ARRTS_StartPosition, SetAndGet)
{
    ArrtsService service;
    service.SetStartPosition(5, 6, 7);
    Position pos = service.GetStartPosition();
    GTEST_ASSERT_EQ(pos.x, 5);
    GTEST_ASSERT_EQ(pos.y, 6);
    GTEST_ASSERT_EQ(pos.theta, 7);
}

TEST(ARRTS_StartPosition, SetUpdateAndGet)
{
    ArrtsService service;
    service.SetStartPosition(5, 6, 7);
    service.SetStartPosition(8, 9, 10);
    Position pos = service.GetStartPosition();
    GTEST_ASSERT_EQ(pos.x, 8);
    GTEST_ASSERT_EQ(pos.y, 9);
    GTEST_ASSERT_EQ(pos.theta, 10);
}

#pragma endregion //ARRTS_StartPosition

#pragma region ARRTS_GoalPosition

TEST(ARRTS_GoalPosition, SetAndGet)
{
    ArrtsService service;
    service.SetGoalPosition(5, 6, 7);
    Position pos = service.GetGoalPosition();
    GTEST_ASSERT_EQ(pos.x, 5);
    GTEST_ASSERT_EQ(pos.y, 6);
    GTEST_ASSERT_EQ(pos.theta, 7);
}

TEST(ARRTS_GoalPosition, SetUpdateAndGet)
{
    ArrtsService service;
    service.SetGoalPosition(5, 6, 7);
    service.SetGoalPosition(8, 9, 10);
    Position pos = service.GetGoalPosition();
    GTEST_ASSERT_EQ(pos.x, 8);
    GTEST_ASSERT_EQ(pos.y, 9);
    GTEST_ASSERT_EQ(pos.theta, 10);
}

#pragma endregion //ARRTS_GoalPosition

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

    GTEST_ASSERT_EQ(obs1.x, 1);
    GTEST_ASSERT_EQ(obs2.x, 4);

    GTEST_ASSERT_EQ(obs1.y, 2);
    GTEST_ASSERT_EQ(obs2.y, 5);

    GTEST_ASSERT_EQ(obs1.radius, 3);
    GTEST_ASSERT_EQ(obs2.radius, 6);
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

    GTEST_ASSERT_EQ(obs1.x, 1);
    GTEST_ASSERT_EQ(obs2.x, 2);
    GTEST_ASSERT_EQ(obs3.x, 7);
    GTEST_ASSERT_EQ(obs4.x, 8);

    GTEST_ASSERT_EQ(obs1.y, 3);
    GTEST_ASSERT_EQ(obs2.y, 4);
    GTEST_ASSERT_EQ(obs3.y, 9);
    GTEST_ASSERT_EQ(obs4.y, 10);

    GTEST_ASSERT_EQ(obs1.radius, 5);
    GTEST_ASSERT_EQ(obs2.radius, 6);
    GTEST_ASSERT_EQ(obs3.radius, 11);
    GTEST_ASSERT_EQ(obs4.radius, 12);
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

#pragma endregion //ARRTS_Obstacles

#pragma region ARRTS_VehiclePoints


TEST(ARRTS_VehiclePoints, AddOnePoint_CheckNum)
{
    ArrtsService service;
    service.AddVehiclePoint(1, 1);
    GTEST_ASSERT_EQ(service.GetNumVehiclePoints(), 1);
}

TEST(ARRTS_VehiclePoints, AddOnePoint_CheckVal)
{
    ArrtsService service;
    service.AddVehiclePoint(1, 2);
    Node n = service.GetVehiclePoint(0);
    GTEST_ASSERT_EQ(n.x, 1);
    GTEST_ASSERT_EQ(n.y, 2);
}

TEST(ARRTS_VehiclePoints, AddMultiplePoint_CheckNum)
{
    ArrtsService service;
    const double x[] = { 1, 2, 3 };
    const double y[] = { 1, 2, 3 };
    service.AddVehiclePoints(x, y, 3);
    GTEST_ASSERT_EQ(service.GetNumVehiclePoints(), 3);
}

TEST(ARRTS_VehiclePoints, AddMultiplePoint_CheckVals)
{
    ArrtsService service;
    const double x[] = { 1, 2, 3 };
    const double y[] = { 4, 5, 6 };
    service.AddVehiclePoints(x, y, 3);

    Node n1 = service.GetVehiclePoint(0);
    Node n2 = service.GetVehiclePoint(1);
    Node n3 = service.GetVehiclePoint(2);
    
    GTEST_ASSERT_EQ(n1.x, 1);
    GTEST_ASSERT_EQ(n2.x, 2);
    GTEST_ASSERT_EQ(n3.x, 3);

    GTEST_ASSERT_EQ(n1.y, 4);
    GTEST_ASSERT_EQ(n2.y, 5);
    GTEST_ASSERT_EQ(n3.y, 6);
}

TEST(ARRTS_VehiclePoints, AddSingleThenSinglePoint_CheckNum)
{
    ArrtsService service;
    service.AddVehiclePoint(1, 2);
    service.AddVehiclePoint(1, 2);
    GTEST_ASSERT_EQ(service.GetNumVehiclePoints(), 2);
}

TEST(ARRTS_VehiclePoints, AddSingleThenSinglePoint_CheckVals)
{
    ArrtsService service;
    service.AddVehiclePoint(1, 2);
    service.AddVehiclePoint(3, 4);

    Node n1 = service.GetVehiclePoint(0);
    Node n2 = service.GetVehiclePoint(1);

    GTEST_ASSERT_EQ(n1.x, 1);
    GTEST_ASSERT_EQ(n2.x, 3);

    GTEST_ASSERT_EQ(n1.y, 2);
    GTEST_ASSERT_EQ(n2.y, 4);
}

TEST(ARRTS_VehiclePoints, AddMultipleThenMultiplePoint_CheckNum)
{
    ArrtsService service;
    const double x1[] = { 1, 2 };
    const double y1[] = { 1, 2 };
    const double x2[] = { 1, 2 };
    const double y2[] = { 1, 2 };
    service.AddVehiclePoints(x1, y1, 2);
    service.AddVehiclePoints(x2, y2, 2);
    GTEST_ASSERT_EQ(service.GetNumVehiclePoints(), 4);
}

TEST(ARRTS_VehiclePoints, AddMultipleThenMultiplePoint_CheckVals)
{
    ArrtsService service;
    const double x1[] = { 1, 2 };
    const double y1[] = { 3, 4 };
    const double x2[] = { 5, 6 };
    const double y2[] = { 7, 8 };
    service.AddVehiclePoints(x1, y1, 2);
    service.AddVehiclePoints(x2, y2, 2);
    
    Node n1 = service.GetVehiclePoint(0);
    Node n2 = service.GetVehiclePoint(1);
    Node n3 = service.GetVehiclePoint(2);
    Node n4 = service.GetVehiclePoint(3);

    GTEST_ASSERT_EQ(n1.x, 1);
    GTEST_ASSERT_EQ(n2.x, 2);
    GTEST_ASSERT_EQ(n3.x, 5);
    GTEST_ASSERT_EQ(n4.x, 6);

    GTEST_ASSERT_EQ(n1.y, 3);
    GTEST_ASSERT_EQ(n2.y, 4);
    GTEST_ASSERT_EQ(n3.y, 7);
    GTEST_ASSERT_EQ(n4.y, 8);
}

TEST(ARRTS_VehiclePoints, AddSingleThenMultiplePoint_CheckNum)
{
    ArrtsService service;
    service.AddVehiclePoint(1, 2);
    const double x[] = { 1, 2, 3 };
    const double y[] = { 1, 2, 3 };
    const double r[] = { 1, 2, 3 };
    service.AddVehiclePoints(x, y, 3);
    GTEST_ASSERT_EQ(service.GetNumVehiclePoints(), 4);
}

TEST(ARRTS_VehiclePoints, AddSingleThenMultiplePoint_CheckVals)
{
    ArrtsService service;
    service.AddVehiclePoint(1, 2);
    const double x[] = { 4, 5, 6 };
    const double y[] = { 7, 8, 9 };
    const double r[] = { 10, 11, 12 };
    service.AddVehiclePoints(x, y, 3);
    
    Node n1 = service.GetVehiclePoint(0);
    Node n2 = service.GetVehiclePoint(1);
    Node n3 = service.GetVehiclePoint(2);
    Node n4 = service.GetVehiclePoint(3);

    GTEST_ASSERT_EQ(n1.x, 1);
    GTEST_ASSERT_EQ(n2.x, 4);
    GTEST_ASSERT_EQ(n3.x, 5);
    GTEST_ASSERT_EQ(n4.x, 6);

    GTEST_ASSERT_EQ(n1.y, 2);
    GTEST_ASSERT_EQ(n2.y, 7);
    GTEST_ASSERT_EQ(n3.y, 8);
    GTEST_ASSERT_EQ(n4.y, 9);
}

TEST(ARRTS_VehiclePoints, AddMultipleThenSinglePoint_CheckNum)
{
    ArrtsService service;
    const double x[] = { 1, 2, 3 };
    const double y[] = { 1, 2, 3 };
    service.AddVehiclePoints(x, y, 3);
    service.AddVehiclePoint(1, 2);
    GTEST_ASSERT_EQ(service.GetNumVehiclePoints(), 4);
}

TEST(ARRTS_VehiclePoints, AddMultipleThenSinglePoint_CheckVals)
{
    ArrtsService service;
    const double x[] = { 4, 5, 6 };
    const double y[] = { 7, 8, 9 };
    service.AddVehiclePoints(x, y, 3);
    service.AddVehiclePoint(1, 2);
    
    Node n1 = service.GetVehiclePoint(0);
    Node n2 = service.GetVehiclePoint(1);
    Node n3 = service.GetVehiclePoint(2);
    Node n4 = service.GetVehiclePoint(3);

    GTEST_ASSERT_EQ(n1.x, 4);
    GTEST_ASSERT_EQ(n2.x, 5);
    GTEST_ASSERT_EQ(n3.x, 6);
    GTEST_ASSERT_EQ(n4.x, 1);

    GTEST_ASSERT_EQ(n1.y, 7);
    GTEST_ASSERT_EQ(n2.y, 8);
    GTEST_ASSERT_EQ(n3.y, 9);
    GTEST_ASSERT_EQ(n4.y, 2);
}

TEST(ARRTS_VehiclePoints, GetFirstPointWhenNone)
{
    ArrtsService service;
    EXPECT_ANY_THROW(service.GetVehiclePoint(0));
}

TEST(ARRTS_VehiclePoints, GetNegativePointWhenNone)
{
    ArrtsService service;
    EXPECT_ANY_THROW(service.GetVehiclePoint(-1));
}

TEST(ARRTS_VehiclePoints, GetSecondPointWhenOne)
{
    ArrtsService service;
    service.AddVehiclePoint(1, 2);
    EXPECT_ANY_THROW(service.GetVehiclePoint(1));
}
#pragma endregion //ARRTS_VehiclePoints

int main(int argc, char* argv[])
{
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}