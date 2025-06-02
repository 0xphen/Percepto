#pragma once

#include <gtest/gtest.h>

#include "percepto/core/ray.h"
#include "percepto/core/vec3.h"
#include "percepto/geometry/triangle.h"

using percepto::core::Ray, percepto::core::Vec3, percepto::geometry::Triangle;

#define EXPECT_VEC3_EQ(v1, v2)        \
  do                                  \
  {                                   \
    EXPECT_DOUBLE_EQ((v1).x, (v2).x); \
    EXPECT_DOUBLE_EQ((v1).y, (v2).y); \
    EXPECT_DOUBLE_EQ((v1).z, (v2).z); \
  } while (0)

namespace percepto::test
{

class CoreTestFixture : public ::testing::Test
{
 public:
  const Vec3 origin = Vec3(0.0, 0.0, 0.0);
  const Vec3 direction = Vec3(1.0, 2.0, 3.0);
  const double t_min = 0.1;
  const double t_max = 100.0;
  const Ray ray = Ray(origin, direction, t_min, t_max);
};

class GeometryTestFixture : public CoreTestFixture
{
 public:
  const Vec3 sphere_centre = Vec3(5.0, 2.0, 12.0);
  double sphere_radius = 5.0;
};

class MathTestFixture : public GeometryTestFixture
{
};

class TriangleTestFixture : public GeometryTestFixture
{
 public:
  const Triangle standard_triangle{Vec3(0.0, 0.0, 0.0), Vec3(1.0, 0.0, 0.0), Vec3(0.0, 1.0, 0.0)};
};

class IntersectionTestFixture : public TriangleTestFixture
{
};

class SceneTestFixture : public TriangleTestFixture
{
};

}  // namespace percepto::test