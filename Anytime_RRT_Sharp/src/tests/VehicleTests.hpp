#include <gtest/gtest.h>
#include "../Vehicle.hpp"

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