#include <gtest/gtest.h>
#include "TestHelpers.h"
#include "percepto/core/Ray.h"
#include "percepto/core/Vec3.h"
#include "percepto/geometry/Sphere.h"
#include "percepto/math/MathUtils.h"

using percepto::geometry::Sphere, percepto::core::Vec3;
using namespace percepto::math;
using percepto::test::MathTest;

TEST_F(MathTest, SolveQuadratic_Variants)
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

TEST_F(MathTest, ComputeQuadraticCoefficients_Variants)
{
  {
    SCOPED_TRACE("Tangent root");
    Sphere sphere(sphere_centre, sphere_radius);
    Vec3 ray_direction = origin - sphere_centre;
    Ray ray(origin, ray_direction, t_min, t_max);

    QuadraticCoefficients coef = computeQuadraticCoefficients(ray, sphere);

    EXPECT_DOUBLE_EQ(coef.a, 1);
    EXPECT_NEAR(coef.b, 26.3058928752, 1e-9);
    EXPECT_DOUBLE_EQ(coef.c, 148);
  }

  {
    SCOPED_TRACE("Orthogonal Direction");
    Vec3 sphere_center = Vec3(0, 5, 0);
    Vec3 ray_direction = Vec3(1, 0, 0);  // Orthogonal to (0, -5, 0)
    Ray ray(origin, ray_direction, t_min, t_max);
    Sphere sphere(sphere_center, sphere_radius);

    QuadraticCoefficients coef = computeQuadraticCoefficients(ray, sphere);
    EXPECT_NEAR(coef.b, 0.0, 1e-9);  // Dot product should be zero
  }
}

// sphere_centre = 5, 2, 12
// ray_origin = 0, 0, 0
// sphere_radius = 5
// origin_to_centre =  ray_origin - sphere_centre = -5, -2, -12
// ray_direction = origin_to_centre
// ray_direction_magnitude = sqrt(-5*-5 + -2*-2 + -12*-12) = 13.152946438
// ray_direction_normalized = (-5/13.152946438, -2/13.152946438, -12/13.152946438) = (-0.3801429606,
// -0.1520571843, -0.9123431055) a = ray_direction_normalized.dot(ray_direction_normalized) = ~
// 0.999999 b = 2 * origin_to_centre.dot(ray_direction_normalized) = 26.3058928752 c =
// origin_to_center.dot(origin_to_center) - sphere_radius * sphere_radius = 173 - 4 = 148