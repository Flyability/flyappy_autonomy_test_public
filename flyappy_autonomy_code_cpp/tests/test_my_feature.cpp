#include <gtest/gtest.h>

#include "flyappy/flyappy.hpp"

TEST(MyFeature, Something)
{
    flyappy::Flyappy flyappy;

    const int a = 1;
    EXPECT_EQ(a, 1);
}
