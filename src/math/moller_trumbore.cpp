#include <optional>

#include "percepto/core/ray.h"
#include "percepto/core/types.h"
#include "percepto/core/vec3.h"
#include "percepto/math/intersection/moller_trumbore.h"

using namespace percepto::core;

namespace percepto::math::intersection
{

/**
 * @brief   Compute ray–triangle intersection using the Möller–Trumbore algorithm (back-face culling
 * enabled).
 *
 * Given three triangle vertices `v0, v1, v2` (assumed CCW for front face) and a `ray`, this
 * function returns a `Hit` containing `(t, u, v)` if the ray intersects the triangle’s front face.
 * If the ray is nearly parallel to the triangle, strikes its back face, or the triangle is
 * degenerate, the function returns `std::nullopt`.
 *
 * Algorithm details:
 * 1. Compute edges: `edge1 = v1 – v0`, `edge2 = v2 – v0`.
 * 2. Form `p = ray.direction() × edge2`.
 * 3. Compute determinant `det = edge1 · p`.
 *    - If `|det| < EPSILON`, ray is parallel or triangle is degenerate → no hit.
 *    - If `det < EPSILON`, ray is hitting the back face (or nearly parallel from the other side) →
 * no hit.
 * 4. Compute `invDet = 1/det`.
 * 5. Compute barycentric `u = ( (ray.origin() – v0) · p ) * invDet`. If `u < 0 || u > 1`, no hit.
 * 6. Compute `q = (ray.origin() – v0) × edge1`.
 * 7. Compute barycentric `v = ( ray.direction() · q ) * invDet`. If `v < 0 || (u + v) > 1`, no hit.
 * 8. Compute `t = ( edge2 · q ) * invDet`. If `t < EPSILON` (behind origin or too close), no hit.
 * 9. Otherwise, return `Hit{t, u, v}`.
 *
 * @param[in]  v0, v1, v2   The triangle’s three vertices (wound CCW on the front face).
 * @param[in]  ray         The ray to test (origin, direction).
 * @return `std::optional<Hit>` containing `(t, u, v)` if a valid front-face intersection occurs;
 *         `std::nullopt` otherwise.
 */
std::optional<TriangleHitResult> moller_trumbore(const Vec3& v0, const Vec3& v1, const Vec3& v2,
                                                 const Ray& ray)
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

  return TriangleHitResult{t, u, v};
}
}  // namespace percepto::math::intersection
