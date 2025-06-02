#include <gtest/gtest.h>
#include <cmath>
#include <random>

#include "percepto/core/ray.h"
#include "percepto/core/vec3.h"
#include "percepto/math/intersection/moller_trumbore.h"
#include "test_helpers.h"

using percepto::math::intersection::moller_trumbore, percepto::core::Ray, percepto::core::Vec3,
    percepto::test::IntersectionTestFixture;

TEST_F(IntersectionTestFixture, InteriorHit)
{
  const double expected_u = 0.25;
  const double expected_v = 0.25;

  {
    SCOPED_TRACE("UnrotatedTriangle_FrontFaceInteriorHit");

    // Ray starts at (0.25, 0.25, +1.0) and points straight down (0,0,-1)
    Ray ray(Vec3(expected_u, expected_v, 1.0), Vec3(0.0, 0.0, -1.0), 0.0, 100.0);

    auto hit = moller_trumbore(standard_triangle.v0(), standard_triangle.v1(),
                               standard_triangle.v2(), ray);
    ASSERT_TRUE(hit);

    // The ray travels 1 unit (from z=+1 to z=0), so t==1.0
    EXPECT_DOUBLE_EQ(hit->t, 1.0);
    EXPECT_DOUBLE_EQ(hit->u, expected_u);
    EXPECT_DOUBLE_EQ(hit->v, expected_v);
  }

  {
    /// Rotate triangle by 30° about the X‐axis
    SCOPED_TRACE("RotatedTriangle_FrontFaceInteriorHit");

    const double cos30 = std::sqrt(3.0) / 2.0;  // ≈ 0.8660254
    const double sin30 = 0.5;

    // Original (unrotated) triangle vertices:
    Vec3 v0 = standard_triangle.v0();        // (0,0,0)
    Vec3 v1 = standard_triangle.v1();        // (1,0,0)
    Vec3 v2_unrot = standard_triangle.v2();  // (0,1,0)

    // Rotate v2_unrot by 30° about the X‐axis:
    Vec3 v2(v2_unrot.x, v2_unrot.y * cos30 - v2_unrot.z * sin30,
            v2_unrot.y * sin30 + v2_unrot.z * cos30);

    // Compute the (unit) normal of the rotated triangle:
    Vec3 normal = (v1 - v0).cross(v2 - v0);

    // Take the interior point (expected_u, expected_v, 0) and rotate it exactly the same way:
    Vec3 targetPt(expected_u,
                  expected_v * cos30 - 0.0 * sin30,  
                  expected_v * sin30 + 0.0 * cos30 
    );

    // Place the ray 1 unit above that rotated plane along the normal:
    Vec3 rayOrig = targetPt + normal * 1.0;
    Vec3 rayDir = normal * -1.0;

    Ray ray(rayOrig, rayDir, 0.0, 100.0);
    auto hit = moller_trumbore(v0, v1, v2, ray);

    ASSERT_TRUE(hit);
    EXPECT_NEAR(hit->t, 1.0, 1e-6);
    EXPECT_NEAR(hit->u, expected_u, 1e-6);
    EXPECT_NEAR(hit->v, expected_v, 1e-6);
  }
}

TEST(MollerTrumbore_Intersection, BackFaceCulled_NoHit)
{
  Vec3 v0(0.0, 0.0, 0.0);
  Vec3 v1(1.0, 0.0, 0.0);
  Vec3 v2(0.0, 1.0, 0.0);

  // Ray from (0.25, 0.25, +1.0) pointing up (0,0,+1). That would hit the back side of the triangle.
  Ray ray(Vec3(0.25, 0.25, 1.0), Vec3(0.0, 0.0, 1.0), 0.0, 100.0);

  auto hit = moller_trumbore(v0, v1, v2, ray);
  EXPECT_FALSE(hit);
}

TEST(MollerTrumbore_Intersection, ParallelRay_NoHit)
{
  Vec3 v0(0.0, 0.0, 0.0);
  Vec3 v1(1.0, 0.0, 0.0);
  Vec3 v2(0.0, 1.0, 0.0);

  Ray ray(Vec3(0.5, 0.5, 1.0), Vec3(1.0, 0.0, 0.0), 0.0, 100.0);

  auto hit = moller_trumbore(v0, v1, v2, ray);
  EXPECT_FALSE(hit);
}

TEST(MollerTrumbore_Intersection, EdgeHit_AB)
{
  Vec3 v0(0.0, 0.0, 0.0);
  Vec3 v1(1.0, 0.0, 0.0);
  Vec3 v2(0.0, 1.0, 0.0);

  // Ray aimed at the midpoint of edge AB: (0.5, 0.0, 0.0), coming from above
  Ray ray(Vec3(0.5, 0.0, 1.0), Vec3(0.0, 0.0, -1.0), 0.0, 100.0);

  auto hit = moller_trumbore(v0, v1, v2, ray);
  ASSERT_TRUE(hit);

  // t = 1.0 (from z=1 down to z=0)
  EXPECT_DOUBLE_EQ(hit->t, 1.0);

  // Intersection is exactly on AB → u=0.5, v=0.0
  EXPECT_DOUBLE_EQ(hit->u, 0.5);
  EXPECT_DOUBLE_EQ(hit->v, 0.0);
}

TEST(MollerTrumbore_Intersection, EdgeHit_AC)
{
  Vec3 v0(0.0, 0.0, 0.0);
  Vec3 v1(1.0, 0.0, 0.0);
  Vec3 v2(0.0, 1.0, 0.0);

  // Ray aimed at the midpoint of edge AC: (0.0, 0.5, 0.0), from above
  Ray ray(Vec3(0.0, 0.5, 1.0), Vec3(0.0, 0.0, -1.0), 0.0, 100.0);

  auto hit = moller_trumbore(v0, v1, v2, ray);
  ASSERT_TRUE(hit);

  EXPECT_DOUBLE_EQ(hit->t, 1.0);
  EXPECT_DOUBLE_EQ(hit->u, 0.0);
  EXPECT_DOUBLE_EQ(hit->v, 0.5);
}

TEST(MollerTrumbore_Intersection, VertexHit_A)
{
  Vec3 v0(0.0, 0.0, 0.0);
  Vec3 v1(1.0, 0.0, 0.0);
  Vec3 v2(0.0, 1.0, 0.0);

  // Ray aimed exactly at vertex A: (0.0, 0.0, 0.0)
  Ray ray(Vec3(0.0, 0.0, 1.0), Vec3(0.0, 0.0, -1.0), 0.0, 100.0);

  auto hit = moller_trumbore(v0, v1, v2, ray);
  ASSERT_TRUE(hit);

  EXPECT_DOUBLE_EQ(hit->t, 1.0);
  EXPECT_DOUBLE_EQ(hit->u, 0.0);
  EXPECT_DOUBLE_EQ(hit->v, 0.0);
}

TEST(MollerTrumbore_Intersection, OutsideTriangle_NoHit)
{
  Vec3 v0(0.0, 0.0, 0.0);
  Vec3 v1(1.0, 0.0, 0.0);
  Vec3 v2(0.0, 1.0, 0.0);

  // Ray aimed at (1.0,1.0,0.0) which is outside the triangle
  Ray ray(Vec3(1.0, 1.0, 1.0), Vec3(0.0, 0.0, -1.0), 0.0, 100.0);

  auto hit = moller_trumbore(v0, v1, v2, ray);
  EXPECT_FALSE(hit);
}

TEST(MollerTrumbore_Intersection, DegenerateTriangle_NoHit)
{
  // Two vertices coincide, making area = 0
  Vec3 v0(0.0, 0.0, 0.0);
  Vec3 v1(0.0, 0.0, 0.0);  // same as v0
  Vec3 v2(1.0, 0.0, 0.0);  // colinear → no valid triangle

  // Ray aimed at where the “triangle” would have been, but triangle is degenerate
  Ray ray(Vec3(0.5, 0.0, 1.0), Vec3(0.0, 0.0, -1.0), 0.0, 100.0);

  auto hit = moller_trumbore(v0, v1, v2, ray);
  EXPECT_FALSE(hit);
}