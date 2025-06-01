#pragma once

#include <optional>

#include "percepto/core/ray.h"
#include "percepto/core/vec3.h"

using percepto::core::Ray, percepto::core::Vec3;

namespace percepto::math::intersection
{
struct Hit
{
  double t, u, v;
};

inline constexpr double EPSILON = 1e-6;

/**
 * Performs the Möller–Trumbore ray–triangle intersection test.
 *
 * Given triangle vertices v0, v1, v2 and a ray (origin O, direction D),
 * computes whether the ray hits the triangle. If it does, returns a Hit
 * containing:
 *   - t: distance along the ray to the intersection point
 *   - u, v: barycentric coordinates within the triangle
 *
 * The algorithm:
 * 1. Compute two edges of the triangle: edgeAB = v1 - v0, edgeAC = v2 - v0.
 * 2. Translate the ray origin into the triangle’s local frame: originToA = O - v0.
 * 3. Build a perpendicular vector p = D × edgeAC and use it to
 *    test for parallelism via det = edgeAB · p. Early out if det is near zero.
 * 4. Compute u = (originToA · p) / det and reject if u lies outside [0,1].
 * 5. Build q = originToA × edgeAB, then compute v = (D · q) / det and reject
 *    if v < 0 or u + v > 1 (outside triangle).
 * 6. Finally, compute t = (edgeAC · q) / det; reject if t is negative or too close.
 *
 * This implementation only stores vertex positions—no precomputed normals or plane
 * equations—making it memory-efficient and fast.
 */
std::optional<Hit> moller_trumbore(const Vec3& v0, const Vec3& v1, const Vec3& v2, const Ray& ray)
{
  Vec3 edgeAB = v1 - v0;  // A→B
  Vec3 edgeAC = v2 - v0;  // A→C

  Vec3 originToA = ray.origin() - v0;

  // Calculate p = D × edgeAC, which is perpendicular to edgeAB
  Vec3 p = ray.direction().cross(edgeAC);
  double det = edgeAB.dot(p);

  // If det is near zero, the ray is parallel to the triangle plane
  if (det < EPSILON) return std::nullopt;

  double invDet = 1.0 / det;

  double u = originToA.dot(p) * invDet;
  if (u < 0.0 || u > 1.0) return std::nullopt;  // Intersection outside the triangle

  Vec3 q = originToA.cross(edgeAB);

  double v = ray.direction().dot(q) * invDet;
  if (v < 0.0 || u + v > 1.0) return std::nullopt;  // Intersection outside the triangle

  double t = edgeAC.dot(q) * invDet;
  if (t < EPSILON) return std::nullopt;  // Intersection is behind the ray origin

  return Hit{t, u, v};
}
}  // namespace percepto::math::intersection
