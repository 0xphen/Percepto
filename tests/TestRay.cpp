#include <gtest/gtest.h>
#include <percepto/geometry/Ray.h>
#include <iostream>
#include "TestHelpers.h"

using percepto::geometry::Ray, percepto::geometry::Vec3;
using percepto::geometry::test::GeometryTest;

TEST_F(GeometryTest, RayTest_DirectionIsNormalized)
{
  const Vec3& normalizedDirection = ray.direction();

  double len = direction.length();
  EXPECT_NEAR(normalizedDirection.x, 1.0 / len, 1e-9);
  EXPECT_NEAR(normalizedDirection.y, 2.0 / len, 1e-9);
  EXPECT_NEAR(normalizedDirection.z, 3.0 / len, 1e-9);
}

TEST_F(GeometryTest, ComputesPointAlongRay)
{
  Vec3 directionTemp = ray.direction();

  double t = 5;
  Vec3 point = ray.at(t);

  EXPECT_VEC3_EQ(point, directionTemp * t);
}

TEST_F(GeometryTest, RayTest_ThrowsIfDirectionIsZero)
{
  Vec3 zeroDir(0.0, 0.0, 0.0);

  // Directly test the validation function
  EXPECT_THROW({ percepto::geometry::Ray::validateRayDirection(zeroDir); }, std::invalid_argument);

  // Test that Ray constructor also throws for zero-length direction
  EXPECT_THROW({ Ray ray(origin, zeroDir, t_min, t_max); }, std::invalid_argument);
}

TEST_F(GeometryTest, RayTest_AtZeroReturnsOrigin)
{
  EXPECT_VEC3_EQ(ray.at(0.0), origin);
}

TEST_F(GeometryTest, RayTest_AccessorsReturnCorrectValues)
{
  EXPECT_VEC3_EQ(ray.origin(), origin);
  EXPECT_DOUBLE_EQ(ray.tMin(), t_min);
  EXPECT_DOUBLE_EQ(ray.tMax(), t_max);
}