#include <gtest/gtest.h>

TEST(Example, Test1)
{
    GTEST_ASSERT_EQ(1, 1);
}

int main(int argc, char* argv[])
{
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}