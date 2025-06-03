#pragma once

#include <optional>
#include "percepto/core/ray.h"
#include "percepto/core/vec3.h"
#include "percepto/types.h"

using percepto::core::Ray, percepto::core::Vec3, percepto::TriangleHitResult;

namespace percepto::math::intersection
{
std::optional<TriangleHitResult> moller_trumbore(const Vec3& v0, const Vec3& v1, const Vec3& v2,
                                                 const Ray& ray);

}  // namespace percepto::math::intersection
