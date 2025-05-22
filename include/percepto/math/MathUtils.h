#pragma once

#include <cmath>
#include <optional>
#include <utility>

#include "percepto/core/Ray.h"
#include "percepto/core/Vec3.h"

namespace percepto::geometry
{
// Forward declaration to avoid circular dependency.
// The full definition of Sphere is not needed in this header.
class Sphere;
}  // namespace percepto::geometry

namespace percepto::math
{

/**
 * @brief Struct representing the coefficients of a quadratic equation:
 *        a·t² + b·t + c = 0
 */
struct QuadraticCoefficients
{
  double a, b, c;
};

/**
 * @brief Computes the quadratic coefficients for the ray-sphere intersection equation.
 *
 * This is derived from substituting the ray equation R(t) = o + t·d into the
 * sphere's equation ‖p - c‖² = r², where:
 *   - o is the ray origin,
 *   - d is the ray direction,
 *   - c is the sphere center,
 *   - r is the sphere radius.
 *
 * @param ray     The ray to test.
 * @param sphere  The sphere to test against.
 * @return QuadraticCoefficients with a, b, c values for the intersection test.
 */
QuadraticCoefficients computeQuadraticCoefficients(const percepto::core::Ray& ray,
                                                   const percepto::geometry::Sphere& sphere);

/**
 * @brief Solves the quadratic equation a·t² + b·t + c = 0 for real roots.
 *
 * @param a Coefficient of t²
 * @param b Coefficient of t
 * @param c Constant term
 * @return An optional pair containing the two real roots (sorted ascending),
 *         or std::nullopt if the discriminant is negative (no real roots).
 */
std::optional<std::pair<double, double>> solveQuadratic(double a, double b, double c);

}  // namespace percepto::math
