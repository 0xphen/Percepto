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

std::optional<Hit> moller_trumbore(const Vec3& a, const Vec3& b, const Vec3&, const Vec3& c,
                                   const Ray& ray)
{
}
}  // namespace percepto::math::intersection
