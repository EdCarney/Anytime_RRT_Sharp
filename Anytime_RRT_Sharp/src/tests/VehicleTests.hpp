#include <gtest/gtest.h>
#include "../Vehicle.hpp"

TEST(Vehicle, AddOnePoint_CheckNum)
{
    Vehicle v;
    v.addOffsetNode(1, 1);
    GTEST_ASSERT_EQ(v.nodes().size(), 1);
}

TEST(Vehicle, AddOnePoint_CheckVal)
{
    Vehicle v;
    v.addOffsetNode(1, 2);
    Point n = v.offsetNodes(0);
    GTEST_ASSERT_EQ(n.GetX(), 1);
    GTEST_ASSERT_EQ(n.GetY(), 2);
}

TEST(Vehicle, AddMultiplePoint_CheckNum)
{
    Vehicle v;
    const double x[] = { 1, 2, 3 };
    const double y[] = { 1, 2, 3 };
    v.addOffsetNodes(x, y, 3);
    GTEST_ASSERT_EQ(v.nodes().size(), 3);
}

TEST(Vehicle, AddMultiplePoint_CheckVals)
{
    Vehicle v;
    const double x[] = { 1, 2, 3 };
    const double y[] = { 4, 5, 6 };
    v.addOffsetNodes(x, y, 3);

    Point n1 = v.offsetNodes(0);
    Point n2 = v.offsetNodes(1);
    Point n3 = v.offsetNodes(2);
    
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
    v.addOffsetNode(1, 2);
    v.addOffsetNode(1, 2);
    GTEST_ASSERT_EQ(v.nodes().size(), 2);
}

TEST(Vehicle, AddSingleThenSinglePoint_CheckVals)
{
    Vehicle v;
    v.addOffsetNode(1, 2);
    v.addOffsetNode(3, 4);

    Point n1 = v.offsetNodes(0);
    Point n2 = v.offsetNodes(1);

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
    v.addOffsetNodes(x1, y1, 2);
    v.addOffsetNodes(x2, y2, 2);
    GTEST_ASSERT_EQ(v.nodes().size(), 4);
}

TEST(Vehicle, AddMultipleThenMultiplePoint_CheckVals)
{
    Vehicle v;
    const double x1[] = { 1, 2 };
    const double y1[] = { 3, 4 };
    const double x2[] = { 5, 6 };
    const double y2[] = { 7, 8 };
    v.addOffsetNodes(x1, y1, 2);
    v.addOffsetNodes(x2, y2, 2);
    
    Point n1 = v.offsetNodes(0);
    Point n2 = v.offsetNodes(1);
    Point n3 = v.offsetNodes(2);
    Point n4 = v.offsetNodes(3);

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
    v.addOffsetNode(1, 2);
    const double x[] = { 1, 2, 3 };
    const double y[] = { 1, 2, 3 };
    const double r[] = { 1, 2, 3 };
    v.addOffsetNodes(x, y, 3);
    GTEST_ASSERT_EQ(v.nodes().size(), 4);
}

TEST(Vehicle, AddSingleThenMultiplePoint_CheckVals)
{
    Vehicle v;
    v.addOffsetNode(1, 2);
    const double x[] = { 4, 5, 6 };
    const double y[] = { 7, 8, 9 };
    const double r[] = { 10, 11, 12 };
    v.addOffsetNodes(x, y, 3);
    
    Point n1 = v.offsetNodes(0);
    Point n2 = v.offsetNodes(1);
    Point n3 = v.offsetNodes(2);
    Point n4 = v.offsetNodes(3);

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
    v.addOffsetNodes(x, y, 3);
    v.addOffsetNode(1, 2);
    GTEST_ASSERT_EQ(v.nodes().size(), 4);
}

TEST(Vehicle, AddMultipleThenSinglePoint_CheckVals)
{
    Vehicle v;
    const double x[] = { 4, 5, 6 };
    const double y[] = { 7, 8, 9 };
    v.addOffsetNodes(x, y, 3);
    v.addOffsetNode(1, 2);
    
    Point n1 = v.offsetNodes(0);
    Point n2 = v.offsetNodes(1);
    Point n3 = v.offsetNodes(2);
    Point n4 = v.offsetNodes(3);

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
    v.addOffsetNodesFromFile(file);

    GTEST_ASSERT_EQ(v.nodes().size(), 37);
}

TEST(Vehicle, AddFromFile_CheckVals)
{
    Vehicle v;
    FILE* file = fopen("./test/robot.txt", "r");
    v.addOffsetNodesFromFile(file);

    Point n1 = v.offsetNodes(0);
    Point n2 = v.offsetNodes(20);
    Point n3 = v.offsetNodes(36);

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
    EXPECT_ANY_THROW(v.offsetNodes(0));
    EXPECT_ANY_THROW(v.nodes(0));
}

TEST(Vehicle, GetNegativePointWhenNone)
{
    Vehicle v;
    EXPECT_ANY_THROW(v.offsetNodes(-1));
    EXPECT_ANY_THROW(v.nodes(-1));
}

TEST(Vehicle, GetSecondPointWhenOne)
{
    Vehicle v;
    v.addOffsetNode(1, 2);
    EXPECT_ANY_THROW(v.offsetNodes(1));
    EXPECT_ANY_THROW(v.nodes(1));
}

TEST(Vehicle, Initialize_CheckVals)
{
    Vehicle v;

    GTEST_ASSERT_EQ(v.state().GetX(), 0);
    GTEST_ASSERT_EQ(v.state().GetY(), 0);
    GTEST_ASSERT_EQ(v.state().GetTheta(), 0);
    GTEST_ASSERT_EQ(v.boundingRadius(), 0);
    GTEST_ASSERT_EQ(v.nodes().size(), 0);
    EXPECT_TRUE(v.nodes().empty());
    EXPECT_TRUE(v.offsetNodes().empty());
}

TEST(Vehicle, UpdateState_CheckVals)
{
    Vehicle v;
    State p = { 1, 2, 3 };
    v.updateState(p);

    GTEST_ASSERT_EQ(v.state().GetX(), 1);
    GTEST_ASSERT_EQ(v.state().GetY(), 2);
    GTEST_ASSERT_EQ(v.state().GetTheta(), 3);
    GTEST_ASSERT_EQ(v.boundingRadius(), 0);
    GTEST_ASSERT_EQ(v.nodes().size(), 0);
    GTEST_ASSERT_EQ(v.nodes().size(), 0);
    EXPECT_TRUE(v.nodes().empty());
    EXPECT_TRUE(v.offsetNodes().empty());
}

TEST(Vehicle, AddNodesUpdateStateNoRotation_CheckVals)
{
    Vehicle v;
    State p = { 1, 2, 0 };
    const double x[] = { -0.5, 0.5, 0.5, -0.5 };
    const double y[] = { -0.5, -0.5, 0.5, 0.5 };

    v.addOffsetNodes(x, y, 4);
    v.updateState(p);

    double tol = 0.001;

    GTEST_ASSERT_EQ(v.state().GetX(), 1);
    GTEST_ASSERT_EQ(v.state().GetY(), 2);
    GTEST_ASSERT_EQ(v.state().GetTheta(), 0);

    EXPECT_NEAR(v.boundingRadius(), 0.707106, tol);

    EXPECT_NEAR(v.nodes(0).GetX(), 0.5, tol);
    EXPECT_NEAR(v.nodes(1).GetX(), 1.5, tol);
    EXPECT_NEAR(v.nodes(2).GetX(), 1.5, tol);
    EXPECT_NEAR(v.nodes(3).GetX(), 0.5, tol);

    EXPECT_NEAR(v.nodes(0).GetY(), 1.5, tol);
    EXPECT_NEAR(v.nodes(1).GetY(), 1.5, tol);
    EXPECT_NEAR(v.nodes(2).GetY(), 2.5, tol);
    EXPECT_NEAR(v.nodes(3).GetY(), 2.5, tol);

    GTEST_ASSERT_EQ(v.nodes().size(), 4);
}

TEST(Vehicle, AddNodesUpdateStateWithPosRotation_CheckVals)
{
    Vehicle v;
    const double x[] = { -0.5, 0.5, 0.5, -0.5 };
    const double y[] = { -0.5, -0.5, 0.5, 0.5 };

    v.addOffsetNodes(x, y, 4);

    State p = { 1, 2, M_PI / 4.0 };
    v.updateState(p);

    double tol = 0.001;

    GTEST_ASSERT_EQ(v.state().GetX(), 1);
    GTEST_ASSERT_EQ(v.state().GetY(), 2);
    GTEST_ASSERT_EQ(v.state().GetTheta(), M_PI / 4.0);

    EXPECT_NEAR(v.boundingRadius(), 0.707106, tol);

    EXPECT_NEAR(v.nodes(0).GetX(), 1.00000, tol);
    EXPECT_NEAR(v.nodes(1).GetX(), 1.70711, tol);
    EXPECT_NEAR(v.nodes(2).GetX(), 1.00000, tol);
    EXPECT_NEAR(v.nodes(3).GetX(), 0.29289, tol);

    EXPECT_NEAR(v.nodes(0).GetY(), 1.29289, tol);
    EXPECT_NEAR(v.nodes(1).GetY(), 2.00000, tol);
    EXPECT_NEAR(v.nodes(2).GetY(), 2.70711, tol);
    EXPECT_NEAR(v.nodes(3).GetY(), 2.00000, tol);

    GTEST_ASSERT_EQ(v.nodes().size(), 4);
}