#pragma once

#include <gtest/gtest.h>
#include "percepto/core/Ray.h"
#include "percepto/core/Vec3.h"

using percepto::core::Ray, percepto::core::Vec3;

#define EXPECT_VEC3_EQ(v1, v2)        \
  do                                  \
  {                                   \
    EXPECT_DOUBLE_EQ((v1).x, (v2).x); \
    EXPECT_DOUBLE_EQ((v1).y, (v2).y); \
    EXPECT_DOUBLE_EQ((v1).z, (v2).z); \
  } while (0)

namespace percepto::test
{

class CoreTest : public ::testing::Test
{
 public:
  const Vec3 origin = Vec3(0.0, 0.0, 0.0);
  const Vec3 direction = Vec3(1.0, 2.0, 3.0);
  const double t_min = 0.1;
  const double t_max = 100.0;
  const Ray ray = Ray(origin, direction, t_min, t_max);
};

class GeometryTest : public CoreTest
{
 public:
  const Vec3 sphere_centre = Vec3(5.0, 2.0, 12.0);
  double sphere_radius = 5.0;
};

class MathTest : public GeometryTest
{
};

}  // namespace percepto::test