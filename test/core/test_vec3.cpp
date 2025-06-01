#include <gtest/gtest.h>

#include <percepto/core/vec3.h>
#include "test_helpers.h"

using percepto::core::Vec3;
using percepto::test::CoreTest;

class Vec3Test : public ::testing::Test
{
 protected:
  const Vec3 a = Vec3(1.0, 2.0, 3.0);
  const Vec3 b = Vec3(4.0, 5.0, 6.0);
  static constexpr double EPS = 1e-6;
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
  // An index < 0 and > 2 should result in an out of range error
  EXPECT_THROW(a[-1], std::out_of_range);
  EXPECT_THROW(a[3], std::out_of_range);
}

TEST_F(Vec3Test, MutIndexAccessTest_ReturnsCorrectComponentAndMutateForValidIndex)
{
  Vec3 v = a;
  EXPECT_DOUBLE_EQ(v[0], 1.0);

  v[0] = 10.0;
  EXPECT_DOUBLE_EQ(v[0], 10.0);
}

TEST_F(Vec3Test, MutIndexAccessTest_ThrowsOutOfRangeForInvalidIndex)
{
  Vec3 v = a;
  // An index < 0 and > 2 should result in an out of range error
  EXPECT_THROW(v[-1] = 10.0, std::out_of_range);
  EXPECT_THROW(v[3] = 10.0, std::out_of_range);
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

TEST_F(Vec3Test, MulOperator_ReturnsNewVec3)
{
  Vec3 b = a * 2;

  EXPECT_DOUBLE_EQ(b.x, 2.0);
  EXPECT_DOUBLE_EQ(b.y, 4.0);
  EXPECT_DOUBLE_EQ(b.z, 6.0);

  // Ensure `operator*` did not modify `a`
  EXPECT_DOUBLE_EQ(a.x, 1.0);
  EXPECT_DOUBLE_EQ(a.y, 2.0);
  EXPECT_DOUBLE_EQ(a.z, 3.0);
}

TEST_F(Vec3Test, DotProduct_ReturnsNewVec3)
{
  double val = a.dot(b);

  EXPECT_DOUBLE_EQ(val, 32.0);

  // Ensure `dot` function did not modify `a` and `b`
  EXPECT_DOUBLE_EQ(a.x, 1.0);
  EXPECT_DOUBLE_EQ(a.y, 2.0);
  EXPECT_DOUBLE_EQ(a.z, 3.0);
  EXPECT_DOUBLE_EQ(b.x, 4.0);
  EXPECT_DOUBLE_EQ(b.y, 5.0);
  EXPECT_DOUBLE_EQ(b.z, 6.0);
}

TEST_F(Vec3Test, Cross_TwoNonParallelVectors_ReturnsCorrectPerpVector)
{
  EXPECT_VEC3_EQ(a.cross(b), Vec3(-3.0, 6.0, -3.0));
}

TEST_F(Vec3Test, Cross_ZeroVectorYieldsZeroResult)
{
  Vec3 zero{0.0, 0.0, 0.0};
  EXPECT_VEC3_EQ(a.cross(zero), zero);
  EXPECT_VEC3_EQ(zero.cross(a), zero);
}

TEST_F(Vec3Test, Cross_AntiCommutative_SwappingOperandsNegates)
{
  Vec3 c = a.cross(b);
  EXPECT_VEC3_EQ(b.cross(a), Vec3(-c.x, -c.y, -c.z));
}

TEST_F(Vec3Test, Cross_ResultPerpendicularToInputs)
{
  Vec3 c = a.cross(b);
  EXPECT_NEAR(c.dot(a), 0.0, EPS);
  EXPECT_NEAR(c.dot(b), 0.0, EPS);
}

TEST_F(Vec3Test, Cross_UnitBasisVectors_ProduceExpectedBasis)
{
  Vec3 i{1.0, 0.0, 0.0}, j{0.0, 1.0, 0.0}, k{0.0, 0.0, 1.0};
  EXPECT_VEC3_EQ(i.cross(j), k);
  EXPECT_VEC3_EQ(j.cross(k), i);
  EXPECT_VEC3_EQ(k.cross(i), j);
}

TEST_F(Vec3Test, Cross_OrthogonalInputs_MagnitudeEqualsProductOfLengths)
{
  Vec3 u{1.0, 0.0, 0.0}, v{0.0, 2.0, 0.0};
  Vec3 uv = u.cross(v);
  EXPECT_DOUBLE_EQ(uv.length(), u.length() * v.length());
}
