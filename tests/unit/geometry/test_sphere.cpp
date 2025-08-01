#include <gtest/gtest.h>

#include <percepto/common/types.h>
#include <percepto/core/ray.h>
#include <percepto/core/vec3.h>
#include <percepto/geometry/sphere.h>
#include "test_helpers.h"

using percepto::core::Ray, percepto::geometry::Sphere, percepto::core::Vec3, percepto::core::Ray,
    percepto::common::HitRecord;
using percepto::test::GeometryTestFixture;

TEST_F(GeometryTestFixture, SphereTest_RaySphereIntersection)
{
  {
    SCOPED_TRACE("Ray-Sphere Intersection: Ray from origin directly toward sphere center");

    Sphere sphere(sphere_centre, sphere_radius);

    // Construct ray pointing directly at the center of the sphere
    Vec3 ray_direction = (sphere_centre - origin).normalized();
    Ray ray(origin, ray_direction, t_min, t_max);

    HitRecord hit_record;

    // Perform ray-sphere intersection
    bool hit = sphere.intersect(ray, hit_record);
    ASSERT_TRUE(hit) << "Expected ray to intersect the sphere, but no hit was detected.";

    // Validate the intersection distance (t_hit)
    // Explanation:
    // - Sphere center = (5, 2, 12)
    // - Ray origin = (0, 0, 0)
    // - Ray direction = normalized(center - origin)
    // - Using the ray-sphere equation: ||O + tD - C||^2 = r^2
    // - The solution yields t_hit ≈ 8.152946438562545
    EXPECT_VEC3_EQ(hit_record.point, ray.at(hit_record.t));
    EXPECT_NEAR(hit_record.t, 8.152946438562545, 1e-9)
        << "Unexpected t_hit value. This indicates the intersection distance is incorrect.";

    // Validate that the computed hit point lies exactly on the sphere's surface
    double distance_to_center = (hit_record.point - sphere_centre).length();
    EXPECT_NEAR(distance_to_center, sphere_radius, 1e-6)
        << "Hit point does not lie on the sphere's surface — distance to center != radius.";
  }

  {
    SCOPED_TRACE("Ray angled off to side should miss sphere");
    const Vec3 direction = Vec3(1.0, 1.0, 0.0).normalized();

    Ray ray(origin, direction, 0.1, 100.0);
    Sphere sphere(sphere_centre, sphere_radius);

    HitRecord hit_record;
    bool hit = sphere.intersect(ray, hit_record);

    ASSERT_FALSE(hit) << "Expected ray to miss sphere, but it hit.";
  }

  {
    SCOPED_TRACE("Ray should graze sphere tangentially with a single intersection point");

    const Vec3 center = Vec3(5.0, 0.0, 0.0);
    const double radius = 1.0;

    // Ray origin is exactly 1 unit above the sphere center on the y-axis,
    // meaning the ray is offset by exactly the sphere's radius — it will just graze the sphere.
    const Vec3 origin = Vec3(0.0, 1.0, 0.0);

    // Ray direction is parallel to the x-axis (straight along x)
    const Vec3 direction = Vec3(1.0, 0.0, 0.0).normalized();

    Sphere sphere(center, radius);
    Ray ray(origin, direction, 0.1, 100.0);

    HitRecord hit_record;
    bool hit = sphere.intersect(ray, hit_record);

    ASSERT_TRUE(hit) << "Expected a tangent hit, but got no intersection.";

    // Check: the hit point lies exactly on the sphere's surface
    double d = (hit_record.point - center).length();
    EXPECT_NEAR(d, radius, 1e-6)
        << "Tangent hit point is not exactly on the surface of the sphere.";

    // Verify discriminant == 0 for a tangent hit
    // In ray-sphere intersection, we solve a quadratic equation:
    //    a*t^2 + b*t + c = 0
    // The discriminant (b² - 4ac) tells us:
    // - If < 0 → no hit
    // - If = 0 → tangent hit (1 solution)
    // - If > 0 → 2 hits (entry and exit)
    //
    // This case is geometrically tangent, so discriminant should be 0.
    QuadraticCoefficients coef = computeQuadraticCoefficients(ray, sphere);
    double disc = coef.b * coef.b - 4 * coef.a * coef.c;

    EXPECT_DOUBLE_EQ(disc, 0.0) << "Expected discriminant to be zero for a tangent hit, but got "
                                << disc;
  }
}