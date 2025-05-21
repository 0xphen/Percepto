#pragma once

#include "cmath"
#include "optional"
#include "utility"

namespace percepto::math
{
/**
 * @brief Solves a quadratic equation of the form a·t² + b·t + c = 0 and returns the real roots, if
 * any.
 *
 * This function computes the real roots of a quadratic equation using the discriminant method.
 * If the equation has real roots, they are returned in a std::pair<double, double> ordered such
 * that the first element is the smaller root and the second is the larger root.
 *
 * @param a Coefficient of t²
 * @param b Coefficient of t
 * @param c Constant term
 * @return std::optional<std::pair<double, double>> containing the roots if they exist,
 *         or std::nullopt if the discriminant is negative (no real solutions).
 */
inline std::optional<std::pair<double, double>> solveQuadratic(double a, double b, double c)
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