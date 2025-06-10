#include <gtest/gtest.h>
#include <cmath>
#include <random>

#include "percepto/core/ray.h"
#include "percepto/core/vec3.h"
#include "percepto/math/intersection/moller_trumbore.h"
#include "test_helpers.h"

using percepto::core::Ray;
using percepto::core::Vec3;
using percepto::math::intersection::moller_trumbore;
using percepto::test::IntersectionTestFixture;

// -----------------------------------------------------
// TEST BLOCK: Interior and Vertex Hits (non‐rotated and rotated)
// -----------------------------------------------------

TEST_F(IntersectionTestFixture, UnrotatedTriangle_InteriorHit)
{
  // “Classic” interior hit on unit right triangle in z=0.
  Vec3 v0 = unit_right_triangle.v0();  // (0,0,0)
  Vec3 v1 = unit_right_triangle.v1();  // (1,0,0)
  Vec3 v2 = unit_right_triangle.v2();  // (0,1,0)

  // Ray starts at (0.25, 0.25, +1), points straight down (0,0,-1).
  Ray ray(Vec3(0.25, 0.25, 1.0), Vec3(0.0, 0.0, -1.0), 0.0, 100.0);
  auto hit = moller_trumbore(v0, v1, v2, ray);
  ASSERT_TRUE(hit);

  // Travels 1 unit down → t == 1.0, barycentrics u = 0.25, v = 0.25
  EXPECT_DOUBLE_EQ(hit->t, 1.0);
  EXPECT_DOUBLE_EQ(hit->u, 0.25);
  EXPECT_DOUBLE_EQ(hit->v, 0.25);
}

TEST_F(IntersectionTestFixture, RotatedTriangle_InteriorHit)
{
  // Rotate the same unit triangle by 30° about X, then ray‐cast “into” it.
  Vec3 v0 = unit_right_triangle.v0();        // (0,0,0)
  Vec3 v1 = unit_right_triangle.v1();        // (1,0,0)
  Vec3 v2_unrot = unit_right_triangle.v2();  // (0,1,0)

  const double cos30 = std::sqrt(3.0) / 2.0;  // ≈ 0.8660254
  const double sin30 = 0.5;

  // Rotate v2 about X: (0,1,0) → (0, cos30, sin30)
  Vec3 v2(v2_unrot.x, v2_unrot.y * cos30 - v2_unrot.z * sin30,
          v2_unrot.y * sin30 + v2_unrot.z * cos30);

  // Compute un‐normalized normal of rotated triangle (it already has unit length)
  Vec3 normal = (v1 - v0).cross(v2 - v0);

  // Rotate the interior point (0.25, 0.25, 0) the same way:
  // → (0.25, 0.25⋅cos30, 0.25⋅sin30)
  Vec3 targetPt(0.25, 0.25 * cos30, 0.25 * sin30);

  // Place ray origin 1 unit “above” along the triangle’s normal.
  Vec3 rayOrig = targetPt + normal * 1.0;
  Vec3 rayDir = normal * -1.0;  // shoot straight back toward the plane

  Ray ray(rayOrig, rayDir, 0.0, 100.0);
  auto hit = moller_trumbore(v0, v1, v2, ray);
  ASSERT_TRUE(hit);

  // Now t ≈ 1.0 (because we moved exactly 1 unit along –normal).
  EXPECT_NEAR(hit->t, 1.0, 1e-6);
  // Barycentrics in the original triangle were (0.25, 0.25).
  EXPECT_NEAR(hit->u, 0.25, 1e-6);
  EXPECT_NEAR(hit->v, 0.25, 1e-6);
}

TEST_F(IntersectionTestFixture, RotatedTriangle_EdgeHit_AB)
{
  // Same rotation as above, but now aim at rotated midpoint of edge AB.
  Vec3 v0 = unit_right_triangle.v0();        // (0,0,0)
  Vec3 v1 = unit_right_triangle.v1();        // (1,0,0)
  Vec3 v2_unrot = unit_right_triangle.v2();  // (0,1,0)

  const double cos30 = std::sqrt(3.0) / 2.0;
  const double sin30 = 0.5;

  Vec3 v2(v2_unrot.x, v2_unrot.y * cos30 - v2_unrot.z * sin30,
          v2_unrot.y * sin30 + v2_unrot.z * cos30);

  Vec3 normal = (v1 - v0).cross(v2 - v0);

  // Midpoint of AB in unrotated: (0.5, 0, 0) → rotation about X: y,z unchanged (still 0).
  // So rotated midpoint = (0.5, 0, 0).
  Vec3 edgeMid(0.5, 0.0, 0.0);

  // Move that midpoint “up” 1 unit along the normal:
  Vec3 rayOrig = edgeMid + normal * 1.0;
  Vec3 rayDir = normal * -1.0;

  Ray ray(rayOrig, rayDir, 0.0, 100.0);
  auto hit = moller_trumbore(v0, v1, v2, ray);
  ASSERT_TRUE(hit);

  EXPECT_NEAR(hit->t, 1.0, 1e-6);
  // On AB means (u, v) = (0.5, 0)
  EXPECT_NEAR(hit->u, 0.5, 1e-6);
  EXPECT_NEAR(hit->v, 0.0, 1e-6);
}

TEST_F(IntersectionTestFixture, RotatedTriangle_VertexHit_v0)
{
  // Rotate triangle as before, then aim at rotated vertex v0 = (0,0,0).
  // The normal is computed exactly as in the previous rotated tests.
  Vec3 v0 = unit_right_triangle.v0();
  Vec3 v1 = unit_right_triangle.v1();
  Vec3 v2_unrot = unit_right_triangle.v2();

  const double cos30 = std::sqrt(3.0) / 2.0;
  const double sin30 = 0.5;
  Vec3 v2(v2_unrot.x, v2_unrot.y * cos30 - v2_unrot.z * sin30,
          v2_unrot.y * sin30 + v2_unrot.z * cos30);
  Vec3 normal = (v1 - v0).cross(v2 - v0);

  // v0 stays at (0,0,0) after rotation, so put the ray 1 unit “above” along normal.
  Vec3 rayOrig = v0 + normal * 1.0;
  Vec3 rayDir = normal * -1.0;

  Ray ray(rayOrig, rayDir, 0.0, 100.0);
  auto hit = moller_trumbore(v0, v1, v2, ray);
  ASSERT_TRUE(hit);

  EXPECT_NEAR(hit->t, 1.0, 1e-6);
  EXPECT_NEAR(hit->u, 0.0, 1e-6);
  EXPECT_NEAR(hit->v, 0.0, 1e-6);
}

// -----------------------------------------------------
// TEST BLOCK: Near‐parallel “grazing” and “origin‐on‐plane” hits
// -----------------------------------------------------

TEST_F(IntersectionTestFixture, NearParallelRay_SlightlyShiftedHit)
{
  Vec3 v0 = unit_right_triangle.v0();
  Vec3 v1 = unit_right_triangle.v1();
  Vec3 v2 = unit_right_triangle.v2();

  // Ray origin at (0.2, 0.2, 1), direction almost in‐plane: (0, 1e-8, -1).
  Vec3 rayOrig(0.2, 0.2, 1.0);
  Vec3 rayDir(0.0, 1e-8, -1.0);  // not normalized on purpose—Möller–Trumbore handles it

  Ray ray(rayOrig, rayDir, 0.0, 100.0);
  auto hit = moller_trumbore(v0, v1, v2, ray);
  ASSERT_TRUE(hit);

  // Intersection at t≈1: (0.2, 0.2 + 1e-8, 0)
  EXPECT_NEAR(hit->t, 1.0, 1e-6);
  EXPECT_NEAR(hit->u, 0.2, 1e-6);
  EXPECT_NEAR(hit->v, 0.20000001, 1e-6);
}

TEST_F(IntersectionTestFixture, RayOriginOnPlane_ImmediateHit)
{
  Vec3 v0 = unit_right_triangle.v0();
  Vec3 v1 = unit_right_triangle.v1();
  Vec3 v2 = unit_right_triangle.v2();

  // Ray origin exactly on interior of triangle at (0.25,0.25,0).
  // Direction (0,0,-1) → points “into” triangle’s back half‐space,
  // but since the origin is already on plane, we record t=0.
  Vec3 rayOrig(0.25, 0.25, 0.0);
  Vec3 rayDir(0.0, 0.0, -1.0);

  Ray ray(rayOrig, rayDir, 0.0, 100.0);
  auto hit = moller_trumbore(v0, v1, v2, ray);
  ASSERT_TRUE(hit);

  EXPECT_DOUBLE_EQ(hit->t, 0.0);
  EXPECT_NEAR(hit->u, 0.25, 1e-6);
  EXPECT_NEAR(hit->v, 0.25, 1e-6);
}

// -----------------------------------------------------
// TEST BLOCK: Simple Vertex and Back‐face misses
// -----------------------------------------------------

TEST_F(IntersectionTestFixture, VertexOnPlane_RayAway_NoHit)
{
  Vec3 v0 = unit_right_triangle.v0();
  Vec3 v1 = unit_right_triangle.v1();
  Vec3 v2 = unit_right_triangle.v2();

  // Ray origin at v0 = (0,0,0), direction (1,1,1) → pointing “up”
  // relative to triangle normal (0,0,1), so it never enters.
  Vec3 rayOrig(0.0, 0.0, 0.0);
  Vec3 rayDir(1.0, 1.0, 1.0);

  Ray ray(rayOrig, rayDir, 0.0, 100.0);
  auto hit = moller_trumbore(v0, v1, v2, ray);
  EXPECT_FALSE(hit);
}

TEST_F(IntersectionTestFixture, BackFace_WhenCullingEnabled_NoHit)
{
  Vec3 v0 = unit_right_triangle.v0();
  Vec3 v1 = unit_right_triangle.v1();
  Vec3 v2 = unit_right_triangle.v2();

  // Ray from (0.25,0.25,1), direction (0,0,1) → shoots “away” from front face.
  Vec3 rayOrig(0.25, 0.25, 1.0);
  Vec3 rayDir(0.0, 0.0, 1.0);

  Ray ray(rayOrig, rayDir, 0.0, 100.0);
  auto hit = moller_trumbore(v0, v1, v2, ray);
  EXPECT_FALSE(hit);
}

// -----------------------------------------------------
// TEST BLOCK: Edge Hits on Unrotated Triangle
// -----------------------------------------------------

TEST_F(IntersectionTestFixture, UnrotatedTriangle_EdgeHits)
{
  Vec3 v0 = unit_right_triangle.v0();
  Vec3 v1 = unit_right_triangle.v1();
  Vec3 v2 = unit_right_triangle.v2();

  {
    SCOPED_TRACE("Edge_AB_Hit");
    // Midpoint of AB = (0.5,0,0); ray straight down from z=1
    Ray ray(Vec3(0.5, 0.0, 1.0), Vec3(0.0, 0.0, -1.0), 0.0, 100.0);
    auto hit = moller_trumbore(v0, v1, v2, ray);
    ASSERT_TRUE(hit);
    EXPECT_DOUBLE_EQ(hit->t, 1.0);
    EXPECT_DOUBLE_EQ(hit->u, 0.5);
    EXPECT_DOUBLE_EQ(hit->v, 0.0);
  }
  {
    SCOPED_TRACE("Edge_AC_Hit");
    // Midpoint of AC = (0,0.5,0); ray straight down
    Ray ray(Vec3(0.0, 0.5, 1.0), Vec3(0.0, 0.0, -1.0), 0.0, 100.0);
    auto hit = moller_trumbore(v0, v1, v2, ray);
    ASSERT_TRUE(hit);
    EXPECT_DOUBLE_EQ(hit->t, 1.0);
    EXPECT_DOUBLE_EQ(hit->u, 0.0);
    EXPECT_DOUBLE_EQ(hit->v, 0.5);
  }
  {
    SCOPED_TRACE("Edge_BC_Hit");
    // Midpoint of BC = (0.5,0.5,0); ray straight down
    Ray ray(Vec3(0.5, 0.5, 1.0), Vec3(0.0, 0.0, -1.0), 0.0, 100.0);
    auto hit = moller_trumbore(v0, v1, v2, ray);
    ASSERT_TRUE(hit);
    EXPECT_DOUBLE_EQ(hit->t, 1.0);
    EXPECT_DOUBLE_EQ(hit->u, 0.5);
    EXPECT_DOUBLE_EQ(hit->v, 0.5);
  }
}

// -----------------------------------------------------
// TEST BLOCK: No‐Hit Cases (degenerate, parallel, “skinny”)
// -----------------------------------------------------

TEST_F(IntersectionTestFixture, NoHit_SimpleMiss)
{
  Vec3 v0 = unit_right_triangle.v0();
  Vec3 v1 = unit_right_triangle.v1();
  Vec3 v2 = unit_right_triangle.v2();

  // Ray parallel to XY plane: origin (0.5,0.5,1), direction (1,0,0)
  Ray ray(Vec3(0.5, 0.5, 1.0), Vec3(1.0, 0.0, 0.0), 0.0, 100.0);
  auto hit = moller_trumbore(v0, v1, v2, ray);
  EXPECT_FALSE(hit);
}

TEST_F(IntersectionTestFixture, NoHit_RayOriginOnEdgeInPlane)
{
  Vec3 v0 = unit_right_triangle.v0();
  Vec3 v1 = unit_right_triangle.v1();
  Vec3 v2 = unit_right_triangle.v2();

  // Origin on edge AB = (0.5,0,0), direction in-plane = (0,1,0).
  Ray ray(Vec3(0.5, 0.0, 0.0), Vec3(0.0, 1.0, 0.0), 0.0, 100.0);
  auto hit = moller_trumbore(v0, v1, v2, ray);
  EXPECT_FALSE(hit);
}

TEST_F(IntersectionTestFixture, NoHit_TwoVerticesCoincide)
{
  // Degenerate: v0 == v1.
  Vec3 v0(0.0, 0.0, 0.0);
  Vec3 v1(0.0, 0.0, 0.0);
  Vec3 v2(1.0, 0.0, 0.0);

  Ray ray(Vec3(0.5, 0.0, 1.0), Vec3(0.0, 0.0, -1.0), 0.0, 100.0);
  auto hit = moller_trumbore(v0, v1, v2, ray);
  EXPECT_FALSE(hit);
}

TEST_F(IntersectionTestFixture, NoHit_SkinnyTriangle)
{
  // Almost straight‐line points: (0,0,0), (1e-8,0,0), (2e-8,0,0).
  Vec3 v0(0.0, 0.0, 0.0);
  Vec3 v1(1e-8, 0.0, 0.0);
  Vec3 v2(2e-8, 0.0, 0.0);

  // Ray aimed “down” at where that line would be:
  Ray ray(Vec3(1e-9, 1e-9, 1.0), Vec3(0.0, 0.0, -1.0), 0.0, 100.0);
  auto hit = moller_trumbore(v0, v1, v2, ray);
  EXPECT_FALSE(hit);
}

// -----------------------------------------------------
// TEST BLOCK: Large‐coordinate stability
// -----------------------------------------------------

TEST_F(IntersectionTestFixture, LargeCoordinates_InteriorHit)
{
  // Shift a unit‐triangle up to z = 1e8
  Vec3 v0(1e8, 1e8, 1e8);
  Vec3 v1(1e8 + 1, 1e8, 1e8);
  Vec3 v2(1e8, 1e8 + 1, 1e8);

  // Ray origin at (1e8+0.2, 1e8+0.3, 1e8+5), direction (0,0,-1)
  Vec3 rayOrig(1e8 + 0.2, 1e8 + 0.3, 1e8 + 5.0);
  Vec3 rayDir(0.0, 0.0, -1.0);  // unit length

  Ray ray(rayOrig, rayDir, 0.0, 1e9);
  auto hit = moller_trumbore(v0, v1, v2, ray);
  ASSERT_TRUE(hit);

  // Must drop from z=1e8+5 ↓ to z=1e8 → t = 5.0 exactly
  EXPECT_NEAR(hit->t, 5.0, 1e-6);

  // At intersection, (x,y) = (1e8+0.2, 1e8+0.3). In local coords that is (0.2, 0.3).
  EXPECT_NEAR(hit->u, 0.2, 1e-6);
  EXPECT_NEAR(hit->v, 0.3, 1e-6);
}

// -----------------------------------------------------
// TEST BLOCK: Tiny‐coordinate stability
// -----------------------------------------------------

// TEST_F(IntersectionTestFixture, TinyCoordinates_InteriorHit)
// {
//   // Very small triangle in z=0: (0,0,0), (1e-8,0,0), (0,1e-8,0)
//   Vec3 v0(0.0, 0.0, 0.0);
//   Vec3 v1(1e-8, 0.0, 0.0);
//   Vec3 v2(0.0, 1e-8, 0.0);

//   // Ray origin at (1e-9,1e-9,1e-8), direction straight down
//   Vec3 rayOrig(1e-9, 1e-9, 1e-8);
//   Vec3 rayDir(0.0, 0.0, -1.0);

//   Ray ray(rayOrig, rayDir, 0.0, 1e-4);

//   // We need to lower the “parallel/degenerate” epsilon so that det ≈ 1e-16 is accepted:
//   auto hit = moller_trumbore(v0, v1, v2, ray);
//   ASSERT_TRUE(hit);

//   // Ray travels from  z=1e-8  down to  z=0  →  t = 1e-8
//   EXPECT_NEAR(hit->t, 1e-8, 1e-14);

//   // Intersection at (1e-9,1e-9,0).  Barycentrics = (x/edge, y/edge) = (0.1, 0.1)
//   EXPECT_NEAR(hit->u, 0.1, 1e-6);
//   EXPECT_NEAR(hit->v, 0.1, 1e-6);
// }

// -----------------------------------------------------
// TEST BLOCK: Random‐fuzz validation
// -----------------------------------------------------

TEST_F(IntersectionTestFixture, RandomFuzz_Validation)
{
  std::mt19937_64 rng(42);
  std::uniform_real_distribution<double> dist(-10.0, 10.0);

  for (int i = 0; i < 5000; ++i)
  {
    // Generate three random vertices & reject if area too small:
    Vec3 v0(dist(rng), dist(rng), dist(rng));
    Vec3 v1(dist(rng), dist(rng), dist(rng));
    Vec3 v2(dist(rng), dist(rng), dist(rng));
    Vec3 e1 = v1 - v0, e2 = v2 - v0;
    // Use a smaller determinant‐epsilon so we accept triangles ~1e-8 in size:
    if (e1.cross(e2).length() < 1e-12)
    {
      --i;  // discard and retry
      continue;
    }

    // Generate a random ray; if direction too small, retry
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
      continue;  // no‐hit is fine; just skip the reconstruction check
    }

    // Reconstruct intersection in two ways:
    Vec3 P1 = orig + dir * hit->t;
    Vec3 P2 = v0 * (1.0 - hit->u - hit->v) + v1 * hit->u + v2 * hit->v;

    EXPECT_NEAR((P1 - P2).length(), 0.0, 1e-6);
    EXPECT_GE(hit->u, -1e-6);
    EXPECT_GE(hit->v, -1e-6);
    EXPECT_LE(hit->u + hit->v, 1.0 + 1e-6);
  }
}
