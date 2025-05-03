#pragma once

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
    Ray(const Vec3& origin, const Vec3& direction)
        : origin_(origin), direction_(direction.normalized()) {}

    const Vec3& origin() const { return origin_; }
    const Vec3& direction() const { return direction_; }

    [[nodiscard]] Vec3 at(double t) const {
        return origin_ + t * direction_;
    }

private:
    Vec3 origin_;
    Vec3 direction_;
};

}
