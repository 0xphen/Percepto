#pragma once

#include <gtest/gtest.h>
#include "percepto/geometry/Ray.h"
#include "percepto/geometry/Vec3.h"

#define EXPECT_VEC3_EQ(v1, v2)        \
  do                                  \
  {                                   \
    EXPECT_DOUBLE_EQ((v1).x, (v2).x); \
    EXPECT_DOUBLE_EQ((v1).y, (v2).y); \
    EXPECT_DOUBLE_EQ((v1).z, (v2).z); \
  } while (0)

namespace percepto::geometry::test
{

class GeometryTest : public ::testing::Test
{
 protected:
  const Vec3 origin = Vec3(0.0, 0.0, 0.0);
  const Vec3 direction = Vec3(1.0, 2.0, 3.0);
  const double t_min = 0.1;
  const double t_max = 100.0;
  const Ray ray = Ray(origin, direction, t_min, t_max);
};

}  // namespace percepto::geometry::test