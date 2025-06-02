#include <gtest/gtest.h>
#include <cmath>
#include <random>

#include "percepto/core/ray.h"
#include "percepto/core/vec3.h"
#include "percepto/math/intersection/moller_trumbore.h"
#include "test_helpers.h"

using percepto::math::intersection::moller_trumbore, percepto::core::Ray, percepto::core::Vec3,
    percepto::test::IntersectionTestFixture;

TEST_F(IntersectionTestFixture, TriangleHits)
{
  const double expected_u = 0.25;
  const double expected_v = 0.25;
  Vec3 v0 = unit_right_triangle.v0();
  Vec3 v1 = unit_right_triangle.v1();
  Vec3 v2 = unit_right_triangle.v2();

  {
    SCOPED_TRACE("UnrotatedTriangle_FrontFaceHit");

    // Ray starts at (0.25, 0.25, +1.0) and points straight down (0,0,-1)
    Ray ray(Vec3(expected_u, expected_v, 1.0), Vec3(0.0, 0.0, -1.0), 0.0, 100.0);

    auto hit = moller_trumbore(v0, v1, v2, ray);
    ASSERT_TRUE(hit);

    // The ray travels 1 unit (from z=+1 to z=0), so t == 1.0
    EXPECT_DOUBLE_EQ(hit->t, 1.0);
    EXPECT_DOUBLE_EQ(hit->u, expected_u);
    EXPECT_DOUBLE_EQ(hit->v, expected_v);
  }

  {
    SCOPED_TRACE("RotatedTriangle_FrontFaceHit");

    // Rotate the unit-right triangle by 30° about the X axis
    const double cos30 = std::sqrt(3.0) / 2.0;  // ≈ 0.8660254
    const double sin30 = 0.5;

    Vec3 v2_unrot = v2;

    // Rotate v2_unrot = (0,1,0) about X
    Vec3 v2(v2_unrot.x, v2_unrot.y * cos30 - v2_unrot.z * sin30,
            v2_unrot.y * sin30 + v2_unrot.z * cos30);  // → (0, cos30, sin30)

    // Compute the (unit) normal of the rotated triangle
    Vec3 normal = (v1 - v0).cross(v2 - v0);

    // Rotate the original interior point (0.25, 0.25, 0) the same way
    Vec3 targetPt(expected_u,
                  expected_v * cos30,  // 0.25⋅cos30
                  expected_v * sin30   // 0.25⋅sin30
    );

    // Place the ray origin 1 unit above that rotated plane along the normal
    Vec3 rayOrig = targetPt + normal * 1.0;
    Vec3 rayDir = normal * -1.0;  // shoot back toward the triangle

    Ray ray(rayOrig, rayDir, 0.0, 100.0);
    auto hit = moller_trumbore(v0, v1, v2, ray);

    ASSERT_TRUE(hit);
    EXPECT_NEAR(hit->t, expected_u / expected_u + (expected_v * 0.0), 1e-6);
    // t ≈ 1.0
    EXPECT_NEAR(hit->u, expected_u, 1e-6);
    EXPECT_NEAR(hit->v, expected_v, 1e-6);
  }

  {
    SCOPED_TRACE("NearParallelRay_SlightlyShiftedHit");

    // Ray origin directly above (0.2, 0.2, 0), at z=1
    Vec3 rayOrig(0.2, 0.2, 1.0);

    // A direction almost parallel to the XY plane but pointing downward:
    //   (0, 1e-8, -1). That slight Y tilt moves the hit from (0.2, 0.2) to (0.2, 0.20000001).
    Vec3 rayDir(0.0, 1e-8, -1.0);
    Ray ray(rayOrig, rayDir, 0.0, 100.0);

    auto hit = moller_trumbore(v0, v1, v2, ray);
    ASSERT_TRUE(hit);

    // Plane intersection still occurs at t ≈ 1.0
    EXPECT_NEAR(hit->t, 1.0, 1e-6);
    // Original X = 0.2 → u ≈ 0.2
    EXPECT_NEAR(hit->u, 0.2, 1e-6);
    // Original Y shifted to 0.20000001 → v ≈ 0.20000001
    EXPECT_NEAR(hit->v, 0.20000001, 1e-6);
  }

  {
    SCOPED_TRACE("RayOriginOnPlane_ImmediateHit");

    // Ray origin lies exactly at (0.25, 0.25, 0) inside the triangle.
    // Direction straight down (0,0,-1) → immediate intersection at t=0.
    Vec3 rayOrig(0.25, 0.25, 0.0);
    Vec3 rayDir(0.0, 0.0, -1.0);

    Ray ray(rayOrig, rayDir, 0.0, 100.0);
    auto hit = moller_trumbore(v0, v1, v2, ray);
    ASSERT_TRUE(hit);
    // Since the origin is on the triangle plane, t == 0
    EXPECT_DOUBLE_EQ(hit->t, 0.0);

    // Barycentric (u,v) = (0.25, 0.25)
    EXPECT_NEAR(hit->u, expected_u, 1e-6);
    EXPECT_NEAR(hit->v, expected_v, 1e-6);
  }

  {
    SCOPED_TRACE("Vertex_Hit");
    // Ray aimed exactly at vertex A: (0.0, 0.0, 0.0)
    Ray ray(Vec3(0.0, 0.0, 1.0), Vec3(0.0, 0.0, -1.0), 0.0, 100.0);

    auto hit = moller_trumbore(v0, v1, v2, ray);
    ASSERT_TRUE(hit);

    EXPECT_DOUBLE_EQ(hit->t, 1.0);
    EXPECT_DOUBLE_EQ(hit->u, 0.0);
    EXPECT_DOUBLE_EQ(hit->v, 0.0);
  }

  {
    SCOPED_TRACE("LargeCoordinates_InteriorHit");
    // Shift a unit‐triangle up to z = 1e8:
    Vec3 v0(1e8, 1e8, 1e8);
    Vec3 v1(1e8 + 1, 1e8, 1e8);
    Vec3 v2(1e8, 1e8 + 1, 1e8);

    // Ray origin at z = 1e8 + 5.0, directly above (1e8 + 0.2, 1e8 + 0.3, 1e8).
    Vec3 rayOrig(1e8 + 0.2, 1e8 + 0.3, 1e8 + 5.0);
    Vec3 rayDir(0.0, 0.0, -1.0);  // unit‐length in Z

    Ray ray(rayOrig, rayDir, 0.0, 1e9);
    auto hit = moller_trumbore(v0, v1, v2, ray);
    ASSERT_TRUE(hit);

    // Now t should be exactly 5.0 (drop from z=1e8+5 to z=1e8)
    EXPECT_NEAR(hit->t, 5.0, 1e-6);

    // And at that moment (x,y) = (1e8 + 0.2, 1e8 + 0.3),
    // so in local triangle space (subtract v0.x/v0.y) → (0.2, 0.3).
    EXPECT_NEAR(hit->u, 0.2, 1e-6);
    EXPECT_NEAR(hit->v, 0.3, 1e-6);
  }

  // {
  //   SCOPED_TRACE("TinyCoordinates_InteriorHit");
  //   // Very small triangle in z=0: (0,0,0), (1e-8,0,0), (0,1e-8,0)
  //   Vec3 v0(0.0, 0.0, 0.0);
  //   Vec3 v1(1e-8, 0.0, 0.0);
  //   Vec3 v2(0.0, 1e-8, 0.0);

  //   // Ray origin at (1e-9, 1e-9, 1e-8), pointing straight down.
  //   Vec3 rayOrig(1e-9, 1e-9, 1e-8);
  //   Vec3 rayDir(0.0, 0.0, -1.0);

  //   Ray ray(rayOrig, rayDir, 0.0, 1e-4);
  //   auto hit = moller_trumbore(v0, v1, v2, ray);
  //   ASSERT_TRUE(hit);

  //   // t = 1e-8 to reach z=0
  //   EXPECT_NEAR(hit->t, 1e-8, 1e-14);

  //   // Intersection point is (1e-9, 1e-9, 0). Barycentrics on the tiny triangle:
  //   //   u = (x / 1e-8) = 0.1, v = (y / 1e-8) = 0.1
  //   EXPECT_NEAR(hit->u, 0.1, 1e-6);
  //   EXPECT_NEAR(hit->v, 0.1, 1e-6);
  // }
}

TEST_F(IntersectionTestFixture, EdgeHits)
{
  Vec3 v0 = unit_right_triangle.v0();
  Vec3 v1 = unit_right_triangle.v1();
  Vec3 v2 = unit_right_triangle.v2();

  {
    SCOPED_TRACE("UnrotatedTriangle_Edge_AB");
    // Midpoint of edge AB is (0.5, 0.0, 0.0); ray comes straight down from z=1.
    Ray ray(Vec3(0.5, 0.0, 1.0), Vec3(0.0, 0.0, -1.0), 0.0, 100.0);
    auto hit = moller_trumbore(v0, v1, v2, ray);
    ASSERT_TRUE(hit);

    // It travels 1 unit down to z=0 → t == 1.0
    EXPECT_DOUBLE_EQ(hit->t, 1.0);
    // On AB means (u, v) = (0.5, 0.0)
    EXPECT_DOUBLE_EQ(hit->u, 0.5);
    EXPECT_DOUBLE_EQ(hit->v, 0.0);
  }

  {
    SCOPED_TRACE("UnrotatedTriangle_Edge_AC");
    // Midpoint of edge AC is (0.0, 0.5, 0.0); ray comes straight down from z=1.
    Ray ray(Vec3(0.0, 0.5, 1.0), Vec3(0.0, 0.0, -1.0), 0.0, 100.0);
    auto hit = moller_trumbore(v0, v1, v2, ray);
    ASSERT_TRUE(hit);

    EXPECT_DOUBLE_EQ(hit->t, 1.0);
    // On AC means (u, v) = (0.0, 0.5)
    EXPECT_DOUBLE_EQ(hit->u, 0.0);
    EXPECT_DOUBLE_EQ(hit->v, 0.5);
  }

  {
    SCOPED_TRACE("UnrotatedTriangle_Edge_BC");
    // Midpoint of edge BC is (0.5, 0.5, 0); ray straight down from z=1
    Ray ray(Vec3(0.5, 0.5, 1.0), Vec3(0.0, 0.0, -1.0), 0.0, 100.0);

    auto hit = moller_trumbore(v0, v1, v2, ray);
    ASSERT_TRUE(hit);

    EXPECT_DOUBLE_EQ(hit->t, 1.0);
    // On BC means u+v=1 → u=0.5, v=0.5
    EXPECT_DOUBLE_EQ(hit->u, 0.5);
    EXPECT_DOUBLE_EQ(hit->v, 0.5);
  }
}

TEST_F(IntersectionTestFixture, NoHit)

{
  Vec3 v0 = unit_right_triangle.v0();
  Vec3 v1 = unit_right_triangle.v1();
  Vec3 v2 = unit_right_triangle.v2();

  {
    SCOPED_TRACE("Simple_NoHit");
    Ray ray(Vec3(0.5, 0.5, 1.0), Vec3(1.0, 0.0, 0.0), 0.0, 100.0);

    auto hit = moller_trumbore(v0, v1, v2, ray);
    EXPECT_FALSE(hit);
  }

  {
    SCOPED_TRACE("RayOriginOnEdge_InPlane_NoHit");
    // Ray origin lies exactly on edge AB at (0.5, 0, 0). Ray direction is (0,1,0),
    // which stays in the plane. It never “enters” the interior from above/below.
    Vec3 rayOrig(0.5, 0.0, 0.0);
    Vec3 rayDir(0.0, 1.0, 0.0);

    Ray ray(rayOrig, rayDir, 0.0, 100.0);
    auto hit = moller_trumbore(v0, v1, v2, ray);
    EXPECT_FALSE(hit);
  }

  {
    SCOPED_TRACE("VertexAtV0_RayAway_NoHit");
    // Ray origin = v0. Direction = (1,1,1) → points out of the triangle plane.
    Vec3 rayOrig(0.0, 0.0, 0.0);
    Vec3 rayDir(1.0, 1.0, 1.0);

    Ray ray(rayOrig, rayDir, 0.0, 100.0);
    auto hit = moller_trumbore(v0, v1, v2, ray);

    // Because it points away from the face, we expect no hit:
    EXPECT_FALSE(hit);
  }

  {
    SCOPED_TRACE("BackFace_WhenCullingEnabled_NoHit");
    // Ray from (0.25, 0.25, +1.0) pointing up (0,0,+1). That would hit the back side of the
    // triangle.
    Vec3 rayOrig(0.25, 0.25, 1.0);
    Vec3 rayDir(0.0, 0.0, 1.0);

    Ray ray(rayOrig, rayDir, 0.0, 100.0);
    auto hit = moller_trumbore(v0, v1, v2, ray);
    EXPECT_FALSE(hit);
  }

  {
    SCOPED_TRACE("TwoVerticesCoincide_NoHit");
    Vec3 v0(0.0, 0.0, 0.0);
    Vec3 v1(0.0, 0.0, 0.0);
    Vec3 v2(1.0, 0.0, 0.0);

    Vec3 rayOrig(0.5, 0.0, 1.0);
    Vec3 rayDir(0.0, 0.0, -1.0);

    Ray ray(rayOrig, rayDir, 0.0, 100.0);
    auto hit = moller_trumbore(v0, v1, v2, ray);
    EXPECT_FALSE(hit);
  }

  {
    SCOPED_TRACE("SkinnyTriangle_NoIntersectionOrHandledGracefully");
    // Points almost collinear: (0,0,0), (1e-8,0,0), (2e-8,0,0)
    Vec3 v0(0.0, 0.0, 0.0);
    Vec3 v1(1e-8, 0.0, 0.0);
    Vec3 v2(2e-8, 0.0, 0.0);

    // Ray aimed at where the triangle would be, from above:
    Vec3 rayOrig(1e-9, 1e-9, 1.0);
    Vec3 rayDir(0.0, 0.0, -1.0);

    Ray ray(rayOrig, rayDir, 0.0, 100.0);
    auto hit = moller_trumbore(v0, v1, v2, ray);

    EXPECT_FALSE(hit);
  }
}

TEST_F(IntersectionTestFixture, RandomFuzz_Validation)
{
  Vec3 v0 = unit_right_triangle.v0();
  Vec3 v1 = unit_right_triangle.v1();
  Vec3 v2 = unit_right_triangle.v2();

  std::mt19937_64 rng(42);
  std::uniform_real_distribution<double> dist(-10.0, 10.0);

  for (int i = 0; i < 5000; i++)
  {
    // Random triangle vertices, reject degenerate if area ≈ 0
    Vec3 v0(dist(rng), dist(rng), dist(rng));
    Vec3 v1(dist(rng), dist(rng), dist(rng));
    Vec3 v2(dist(rng), dist(rng), dist(rng));
    Vec3 e1 = v1 - v0, e2 = v2 - v0;

    if (e1.cross(e2).length() < 1e-8)
    {
      --i;
      continue;
    }

    // Random ray origin & direction
    Vec3 orig(dist(rng), dist(rng), dist(rng));
    Vec3 dir(dist(rng), dist(rng), dist(rng));
    if (dir.length() < 1e-8)
    {
      --i;
      continue;
    }
    dir = dir.normalized();
    Ray ray(orig, dir, 0.0, 100.0);

    auto hit = moller_trumbore(v0, v1, v2, ray);
    if (!hit)
    {
      continue;  // misses are fine
    }

    // Reconstruct intersection in two ways:
    Vec3 P1 = orig + dir * hit->t;
    Vec3 P2 = v0 * (1.0 - hit->u - hit->v) + v1 * (hit->u) + v2 * (hit->v);
    EXPECT_NEAR((P1 - P2).length(), 0.0, 1e-6);

    // Barycentrics must lie in [0,1] and u + v <= 1
    EXPECT_GE(hit->u, -1e-6);
    EXPECT_GE(hit->v, -1e-6);
    EXPECT_LE(hit->u + hit->v, 1.0 + 1e-6);
  }
}