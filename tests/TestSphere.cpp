#include <gtest/gtest.h>
#include <percepto/core/Ray.h>
#include <percepto/core/Vec3.h>
#include <percepto/geometry/Sphere.h>
#include "TestHelpers.h"

using percepto::core::Ray, percepto::geometry::Sphere, percepto::core::Vec3, percepto::core::Ray;
using percepto::test::CoreTest;

TEST_F(CoreTest, SphereTest_RaySphereIntersection)
{
  Vec3 centre(10.0, 12.0, 7.0);
  double radius = 5.0;
  Sphere sphere(centre, radius);

  Vec3 ray_direction = (centre - origin).normalized();
  Ray ray(origin, ray_direction, t_min, t_max);

  double t_hit;

  double hit = sphere.intercept(ray, t_hit);
  std::cout << "HIT: " << t_hit << std::endl;
}