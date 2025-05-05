#pragma once

#include <iostream>
#include <limits>
#include <optional>
#include "Ray.h"
#include "Vec3.h"

namespace percepto::geometry
{

//===----------------------------------------------------------------------===//
//
//  RayBuilder.h
//
//  This class provides a safe and chainable interface to construct Ray objects.
//  It allows setting the origin, direction, and t-range bounds explicitly.
//
//  RayBuilder encapsulates validation (e.g., direction must be non-zero)
//  and supports both strict `build()` and safe `tryBuild()` construction paths.
//
//===----------------------------------------------------------------------===//

class RayBuilder
{
 private:
  Vec3 origin_{0, 0, 0};                                    // Default origin at world space zero
  Vec3 direction_{1.0, 0, 0};                               // Default direction along X-axis
  double t_min_ = 0.0;                                      // Start of ray interval
  double t_max_ = std::numeric_limits<double>::infinity();  // Unbounded max by default

 public:
  RayBuilder& setOrigin(const Vec3& origin)
  {
    origin_ = origin;
    return *this;
  }

  RayBuilder& setDirection(const Vec3& direction)
  {
    direction_ = direction;
    return *this;
  }

  RayBuilder& setRange(double t_min, double t_max)
  {
    if (t_max <= t_min)
    {
      throw std::invalid_argument("t_max must be greater than t_min.");
    }
    t_min_ = t_min;
    t_max_ = t_max;
    return *this;
  }

  Ray build() const { return Ray(origin_, direction_, t_min_, t_max_); }

  // Attempts to build the ray and capture errors as std::nullopt instead of throwing.
  // Useful for cases where invalid rays may occur (e.g., random sampling or sensor noise).
  std::optional<Ray> tryBuild() const
  {
    try
    {
      return build();
    }
    catch (const std::invalid_argument& e)
    {
      std::cerr << "[RayBuilder] Failed to build ray: " << e.what() << std::endl;
      return std::nullopt;
    }
  }

  const Vec3& origin() const { return origin_; }
  const Vec3& direction() const { return direction_; }
  double tMin() const { return t_min_; }
  double tMax() const { return t_max_; }
};

}  // namespace percepto::geometry
