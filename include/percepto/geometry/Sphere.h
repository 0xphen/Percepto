#pragma once

#include <cmath>
#include "percepto/core/Intersectable.h"
#include "percepto/core/Ray.h"
#include "percepto/core/Vec3.h"
#include "percepto/math/MathUtils.h"

using percepto::core::Vec3, percepto::core::Ray, percepto::core::Intersectable,
    percepto::math::solveQuadratic;

namespace percepto::geometry
{
class Sphere : public Intersectable<Sphere>
{
 public:
  Sphere(const Vec3& centre, double radius) : centre_(centre), radius_(radius) {}

  /**
   * @brief Checks whether a given ray intersects this sphere and returns the closest valid hit
   * distance.
   *
   * This function implements the standard ray-sphere intersection test using the analytic solution
   * of the quadratic equation derived from substituting the ray equation into the implicit equation
   * of a sphere.
   *
   * A ray is defined as:
   *   R(t) = o + t * d
   * where:
   *   - o is the ray origin
   *   - d is the ray direction (usually normalized)
   *   - t is the distance along the ray
   *
   * A sphere is defined by the implicit equation:
   *   ||p - c||² = r²
   * where:
   *   - p is any point on the sphere
   *   - c is the center of the sphere
   *   - r is the sphere radius
   *
   * Substituting R(t) into the sphere's equation leads to a quadratic in t:
   *   at² + bt + c = 0
   * where:
   *   - a = d • d
   *   - b = 2 * (o - c) • d
   *   - c = (o - c) • (o - c) - r²
   *
   * The discriminant (b² - 4ac) determines the number of real intersection points:
   *   - < 0: No intersection
   *   - = 0: One tangent intersection
   *   - > 0: Two intersections (enter/exit points)
   *
   * The roots are computed using a helper function `solveQuadratic()`, which returns the
   * real solutions (if any) in increasing order.
   *
   * The function returns the smallest positive t (i.e., closest visible intersection),
   * or false if no valid intersection occurs in front of the ray origin.
   *
   * @param ray     The input ray to test against the sphere.
   * @param t_hit   Output parameter. If the ray intersects, this will contain the distance (t) to
   *                the closest intersection point.
   * @return true if the ray intersects the sphere (in front of the ray origin); false otherwise.
   */

  [[nodiscard]]
  bool intercept(const Ray& ray, double& t_hit) const
  {
    Vec3 ray_direction = ray.direction();
    Vec3 origin_to_center = ray.origin() - this->centre_;

    double a = ray_direction.dot(ray_direction);  // Normally 1 if direction is normalized

    double b = 2.0 * origin_to_center.dot(ray_direction);

    double c = origin_to_center.dot(origin_to_center) - this->radius_ * this->radius_;

    double t0, t1;
    auto result = solveQuadratic(a, b, c);
    if (!result) return false;

    t0 = result->first;
    t1 = result->second;

    if (t0 < 0.0)
    {
      t0 = t1;
      if (t0 < 0.0) return false;  // both intersections are behind the ray
    }

    t_hit = t0;
    return true;
  }

 private:
  Vec3 centre_;
  double radius_;
};
}  // namespace percepto::geometry