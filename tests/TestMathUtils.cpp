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
    SCOPED_TRACE("Happy path: ray origin outside sphere, intersects at two points");

    Sphere sphere(sphere_centre, sphere_radius);
    Vec3 ray_direction = origin - sphere_centre;
    Ray ray(origin, ray_direction, t_min, t_max);

    // This test represents the common "happy path" scenario:
    // - The ray origin is outside the sphere
    // - The ray is directed toward the sphere
    // - The ray intersects the sphere at two points
    //
    // In this case, the quadratic equation a·t² + b·t + c = 0 has two real roots
    // (i.e., discriminant > 0). Here, we're not solving for the roots but verifying
    // that the computed coefficients a, b, and c are correct for this configuration.

    QuadraticCoefficients coef = computeQuadraticCoefficients(ray, sphere);

    EXPECT_DOUBLE_EQ(coef.a, 1);
    EXPECT_NEAR(coef.b, 26.3058928752, 1e-9);
    EXPECT_DOUBLE_EQ(coef.c, 148);
  }

  {
    SCOPED_TRACE("Ray direction orthogonal to (origin - center)");

    Vec3 sphere_center = Vec3(0, 5, 0);
    Vec3 ray_direction = Vec3(1, 0, 0);  // Along X-axis

    // origin = (0, 0, 0) from the fixture
    // Vector from sphere center to ray origin = origin - center = (0, -5, 0)
    // Ray direction = (1, 0, 0)
    //
    // These two vectors are perpendicular:
    // (0, -5, 0) • (1, 0, 0) = 0
    //
    // So, b = 2 * (origin - center) • direction = 0
    // This test confirms that when the ray direction is perpendicular to
    // the vector from sphere center to origin, coefficient `b` is zero.

    Ray ray(origin, ray_direction, t_min, t_max);
    Sphere sphere(sphere_center, sphere_radius);

    QuadraticCoefficients coef = computeQuadraticCoefficients(ray, sphere);

    EXPECT_NEAR(coef.b, 0.0, 1e-9);
  }

  {
    SCOPED_TRACE("Ray origin inside sphere");

    Vec3 sphere_center = Vec3(5, 2, 12);
    Vec3 ray_origin(3, 1, 11);  // Ray origin lies inside the sphere

    Vec3 ray_direction = ray_origin - sphere_center;
    Ray ray(ray_origin, ray_direction, t_min, t_max);

    Sphere sphere(sphere_center, sphere_radius);

    QuadraticCoefficients coef = computeQuadraticCoefficients(ray, sphere);

    // When the ray origin is inside the sphere,
    // the squared distance ||origin - center||² is less than radius²,
    // so the c coefficient is always negative.
    ASSERT_LT(coef.c,
              0);  // c = ||origin - center||² - r² = 6 - 25 = -19 → always negative inside sphere
  }
}