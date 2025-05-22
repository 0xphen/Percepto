#include <gtest/gtest.h>
#include "percepto/math/MathUtils.h"

using namespace percepto::math;

TEST(MathTest, SolveQuadratic_Variants)
{
  {
    SCOPED_TRACE("Two real roots");
    auto result = solveQuadratic(1.0, -5.0, 6.0);  // t^2 - 5t + 6 = 0 → roots at 2 and 3
    ASSERT_TRUE(result);
    auto [t0, t1] = *result;
    EXPECT_DOUBLE_EQ(t0, 2.0);
    EXPECT_DOUBLE_EQ(t1, 3.0);
  }

  {
    SCOPED_TRACE("Tangent root");
    auto result = solveQuadratic(1.0, -4.0, 4.0);  // t^2 - 4t + 4 = 0 → one root at t = 2
    ASSERT_TRUE(result);
    auto [t0, t1] = *result;
    EXPECT_DOUBLE_EQ(t0, 2.0);
    EXPECT_DOUBLE_EQ(t1, 2.0);
  }

  {
    SCOPED_TRACE("No real roots");
    auto result = solveQuadratic(1.0, 4.0, 10.0);  // t^2 + 4t + 10 = 0 → discriminant < 0
    ASSERT_FALSE(result);
  }
}