#include "percepto/math/MathUtils.h"
#include "percepto/core/Ray.h"
#include "percepto/core/Vec3.h"
#include "percepto/geometry/Sphere.h"

using percepto::core::Vec3, percepto::core::Ray, percepto::geometry::Sphere;

namespace percepto::math
{
QuadraticCoefficients computeQuadraticCoefficients(const Ray& ray, const Sphere& sphere)
{
  Vec3 ray_direction = ray.direction();
  Vec3 origin_to_center =
      ray.origin() - sphere.centre();  // Vector from sphere center to ray origin

  double a = ray_direction.dot(ray_direction);  //  a = d • d. Normally 1 if direction is normalized
  double b = 2.0 * origin_to_center.dot(ray_direction);  // b = 2(o - c) • d
  double c = origin_to_center.dot(origin_to_center) -
             sphere.radius() * sphere.radius();  // c = ||o - c||² - r²

  return {a, b, c};
}

std::optional<std::pair<double, double>> solveQuadratic(double a, double b, double c)
{
  double disc = b * b - 4 * a * c;
  if (disc < 0.0) return std::nullopt;

  double sqrt_disc = std::sqrt(disc);
  double t0 = (-b - sqrt_disc) / (2 * a);
  double t1 = (-b + sqrt_disc) / (2 * a);

  if (t0 > t1) std::swap(t0, t1);

  return std::make_pair(t0, t1);
}
}  // namespace percepto::math