#include <gtest/gtest.h>

#include "flyappy_autonomy_code/flyappy.hpp"

TEST(MyFeature, Something)
{
    Flyappy flyappy;

    const int a = 1;
    EXPECT_EQ(a, 1);
}

int main(int argc, char** argv)
{
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
