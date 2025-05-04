#include <gtest/gtest.h>

#include <percepto/geometry/Vec3.h>
#include "TestHelpers.h"

using percepto::geometry::Vec3;

class Vec3Test : public ::testing::Test
{
 protected:
  const Vec3 a = Vec3(1.0, 2.0, 3.0);
  const Vec3 b = Vec3(4.0, 5.0, 6.0);
};

TEST_F(Vec3Test, UnaryMinus)
{
  Vec3 neg = -a;

  EXPECT_EQ(neg.x, -1);
  EXPECT_EQ(neg.y, -2);
  EXPECT_EQ(neg.z, -3);
}

TEST_F(Vec3Test, ConstIndexAccessTest_ReturnsCorrectComponentForValidIndex)
{
  EXPECT_DOUBLE_EQ(a[0], 1.0);
  EXPECT_DOUBLE_EQ(a[1], 2.0);
  EXPECT_DOUBLE_EQ(a[2], 3.0);
}

TEST_F(Vec3Test, ConstIndexAccessTest_ThrowsOutOfRangeForInvalidIndex)
{
  EXPECT_THROW(a[-1], std::out_of_range);
  EXPECT_THROW(a[3], std::out_of_range);
}

TEST_F(Vec3Test, MutIndexAccessTest_ReturnsCorrectComponentAndMutateForValidIndex)
{
  Vec3 v = a;
  v[0] = 10.0;

  EXPECT_DOUBLE_EQ(v[0], 10.0);
}

TEST_F(Vec3Test, MutIndexAccessTest_ThrowsOutOfRangeForInvalidIndex)
{
  Vec3 v = a;
  EXPECT_THROW(v[-1] = 10.0, std::out_of_range);
}

TEST_F(Vec3Test, SumOperator_ReturnsNewVec3)
{
  Vec3 c = a + b;
  EXPECT_VEC3_EQ(c, Vec3(5, 7, 9));

  // Ensure `operator+` did not modify `a`
  EXPECT_DOUBLE_EQ(a.x, 1.0);
  EXPECT_DOUBLE_EQ(a.y, 2.0);
  EXPECT_DOUBLE_EQ(a.z, 3.0);
}

TEST_F(Vec3Test, SubOperator_ReturnsNewVec3)
{
  Vec3 c = b - a;
  EXPECT_VEC3_EQ(c, Vec3(3, 3, 3));

  // Ensure `operator+` did not modify `a`
  EXPECT_DOUBLE_EQ(a.x, 1.0);
  EXPECT_DOUBLE_EQ(a.y, 2.0);
  EXPECT_DOUBLE_EQ(a.z, 3.0);
}

// class Vec3ConstIndexAccessTest: public ::testing::TestWithParam<std::tuple<Vec3, int, double>>
// {};

// TEST_P(Vec3ConstIndexAccessTest, CorrectlyIndexVec3) {
//   Vec3 input = std::get<0>(GetParam());
//   int index = std::get<1>(GetParam());
//   double expected = std::get<2>(GetParam());

//   double result = input[index];
//   EXPECT_EQ(result, expected);
// }

// INSTANTIATE_TEST_SUITE_P(OperatorIndexCases, Vec3ConstIndexAccessTest,
//                           ::testing::Values(
//                             std::make_tuple(Vec3(4.0, 5.0, 6.0), 1, 5.0)
//                           )
//                           );