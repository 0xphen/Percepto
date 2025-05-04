#include <gtest/gtest.h>
#include <percepto/geometry/Ray.h>
#include <iostream>
#include "TestHelpers.h"

using percepto::geometry::Ray, percepto::geometry::Vec3;

class RayTest : public ::testing::Test
{
 protected:
  const Vec3 origin = Vec3(0.0, 0.0, 0.0);
  const Vec3 direction = Vec3(1.0, 2.0, 3.0);
  double t_min = 0.1;
  double t_max = 100;

  const Ray ray = Ray(origin, direction, t_min, t_max);
};

TEST_F(RayTest, DirectionIsNormalized)
{
  const Vec3& normalizedDirection = ray.direction();

  double len = direction.length();
  EXPECT_NEAR(normalizedDirection.x, 1.0 / len, 1e-9);
  EXPECT_NEAR(normalizedDirection.y, 2.0 / len, 1e-9);
  EXPECT_NEAR(normalizedDirection.z, 3.0 / len, 1e-9);
}

TEST_F(RayTest, ComputesPointAlongRay)
{
  Vec3 directionTemp = ray.direction();

  double t = 5;
  Vec3 point = ray.at(t);

  EXPECT_VEC3_EQ(point, directionTemp * t);
}

TEST_F(RayTest, ThrowsIfDirectionIsZero)
{
  Vec3 zeroDir(0.0, 0.0, 0.0);

  EXPECT_THROW({ Ray ray(origin, zeroDir, t_min, t_max); }, std::invalid_argument);
}

TEST_F(RayTest, AtZeroReturnsOrigin)
{
  EXPECT_VEC3_EQ(ray.at(0.0), origin);
}

TEST_F(RayTest, AccessorsReturnCorrectValues)
{
  EXPECT_VEC3_EQ(ray.origin(), origin);
  EXPECT_DOUBLE_EQ(ray.tMin(), t_min);
  EXPECT_DOUBLE_EQ(ray.tMax(), t_max);
}