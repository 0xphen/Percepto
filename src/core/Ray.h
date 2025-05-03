#pragma once

#include <stdexcept>
#include "Vec3.h"

namespace percepto {

//===----------------------------------------------------------------------===//
//
//  Ray.h
//
//  This header defines the `Ray` class.
//  A Ray consists of an origin point and a normalized direction vector.
//  The class provides efficient methods to compute positions along the ray.
//
//  This class is implemented entirely in the header with inline member
//  functions to enable compiler optimizations and reduce function call
//  overhead.
//
//  Usage example:
//      Ray r(origin, direction);
//      Vec3 hit_point = r.at(distance);
//
//===----------------------------------------------------------------------===//
class Ray {
public:
    static constexpr double kMinDirectionLengthSquared = 1e-12;

    Ray(const Vec3& origin, const Vec3& direction, double t_min, double t_max)
        : origin_(origin), t_min_(t_min), t_max_(t_max) {
          // Ensure the direction vector is not zero-length (or nearly zero)
          // to prevent undefined ray behavior (e.g., invalid normalization).
          if (direction.length_squared() < kMinDirectionLengthSquared) {
            throw std::invalid_argument("Ray direction vector must not be zero.");
          }

          direction_ = direction.normalized();
        }

    const Vec3& origin() const { return origin_; }
    const Vec3& direction() const { return direction_; }
    double tMin() const { return t_min_; }
    double tMax() const { return t_max_; }


    [[nodiscard]] Vec3 at(double t) const {
        return origin_ + t * direction_;
    }

  private:
    Vec3 origin_;
    Vec3 direction_;
    double t_min_;
    double t_max_;
  };
}
