#include <gtest/gtest.h>
#include <random>  

#include "percepto/core/ray.h"
#include "percepto/core/vec3.h"
#include "percepto/math/intersection/moller_trumbore.h"
#include "test_helpers.h"

using percepto::math::intersection::moller_trumbore, percepto::core::Ray, percepto::core::Vec3;

TEST(MollerTrumbore_Intersection, FrontFaceInteriorHit) {
    Vec3 v0(0.0, 0.0, 0.0);
    Vec3 v1(1.0, 0.0, 0.0);
    Vec3 v2(0.0, 1.0, 0.0);

    // Ray starts at (0.25, 0.25, +1.0) and points straight down (0,0,-1)
    Ray ray(Vec3(0.25, 0.25, 1.0), Vec3(0.0, 0.0, -1.0), 0.0, 100.0);

    auto hit = moller_trumbore(v0, v1, v2, ray);
    ASSERT_TRUE(hit); 

    // The ray travels 1 unit (from z=+1 to z=0), so t==1.0
    EXPECT_DOUBLE_EQ(hit->t, 1.0);
    EXPECT_DOUBLE_EQ(hit->u, 0.25);
    EXPECT_DOUBLE_EQ(hit->v, 0.25);
}

TEST(MollerTrumbore_Intersection, BackFaceCulled_NoHit) {
    Vec3 v0(0.0, 0.0, 0.0);
    Vec3 v1(1.0, 0.0, 0.0);
    Vec3 v2(0.0, 1.0, 0.0);

    // Ray from (0.25, 0.25, +1.0) pointing up (0,0,+1). That would hit the back side of the triangle.
    Ray ray(Vec3(0.25, 0.25, 1.0), Vec3(0.0, 0.0, 1.0), 0.0, 100.0);

    auto hit = moller_trumbore(v0, v1, v2, ray);
    EXPECT_FALSE(hit);
}

TEST(MollerTrumbore_Intersection, ParallelRay_NoHit) {
    Vec3 v0(0.0, 0.0, 0.0);
    Vec3 v1(1.0, 0.0, 0.0);
    Vec3 v2(0.0, 1.0, 0.0);

    Ray ray(Vec3(0.5, 0.5, 1.0), Vec3(1.0, 0.0, 0.0), 0.0, 100.0);

    auto hit = moller_trumbore(v0, v1, v2, ray);
    EXPECT_FALSE(hit);
}

TEST(MollerTrumbore_Intersection, EdgeHit_AB) {
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

TEST(MollerTrumbore_Intersection, EdgeHit_AC) {
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

TEST(MollerTrumbore_Intersection, VertexHit_A) {
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

TEST(MollerTrumbore_Intersection, OutsideTriangle_NoHit) {
    Vec3 v0(0.0, 0.0, 0.0);
    Vec3 v1(1.0, 0.0, 0.0);
    Vec3 v2(0.0, 1.0, 0.0);

    // Ray aimed at (1.0,1.0,0.0) which is outside the triangle
    Ray ray(Vec3(1.0, 1.0, 1.0), Vec3(0.0, 0.0, -1.0), 0.0, 100.0);

    auto hit = moller_trumbore(v0, v1, v2, ray);
    EXPECT_FALSE(hit);
}

TEST(MollerTrumbore_Intersection, DegenerateTriangle_NoHit) {
    // Two vertices coincide, making area = 0
    Vec3 v0(0.0, 0.0, 0.0);
    Vec3 v1(0.0, 0.0, 0.0);  // same as v0
    Vec3 v2(1.0, 0.0, 0.0);  // colinear → no valid triangle

    // Ray aimed at where the “triangle” would have been, but triangle is degenerate
    Ray ray(Vec3(0.5, 0.0, 1.0), Vec3(0.0, 0.0, -1.0), 0.0, 100.0);

    auto hit = moller_trumbore(v0, v1, v2, ray);
    EXPECT_FALSE(hit);
}

// TEST(MollerTrumbore_Intersection, RandomFuzzRays_Validation) {
//     Vec3 A(0.0, 0.0, 0.0), 
//          B(1.0, 0.0, 0.0), 
//          C(0.0, 1.0, 0.0);

//     std::mt19937_64 rng(42);
//     std::uniform_real_distribution<double> dist(-1.0, 2.0);

//     for (int i = 0; i < 5000; ++i) {
//         // Random origin in a 3×3×3 box centered on triangle
//         Vec3 orig{ dist(rng), dist(rng), dist(rng) };
//         Vec3 dir{ dist(rng), dist(rng), dist(rng) };

//         if (dir.length() < 1e-8) { --i; continue; }
//         dir = dir.normalized();
//         Ray ray(orig, dir, 0.0, 100.0);

//         auto hit = moller_trumbore(A, B, C, ray);
//         if (!hit) 
//             continue;  // no hit is fine

//         // Reconstruct intersection point two ways:
//         Vec3 P1 = orig + dir * hit->t;
//         Vec3 P2 = A * (1.0 - hit->u - hit->v)
//                 + B * hit->u
//                 + C * hit->v;

//         // They must closely match
//         EXPECT_NEAR((P1 - P2).length(), 0.0, 1e-6);

//         // Barycentric coords should be in [0,1], u+v ≤ 1
//         EXPECT_GE(hit->u, -1e-6);
//         EXPECT_GE(hit->v, -1e-6);
//         EXPECT_LE(hit->u + hit->v, 1.0 + 1e-6);
//     }
// }
