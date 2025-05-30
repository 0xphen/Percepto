#pragma once

#include "percepto/core/intersectable.h"
#include "percepto/core/ray.h"
#include "percepto/core/vec3.h"
#include "percepto/math/intersection/moller_trumbore.h"

using percepto::core::Vec3, percepto::core::Ray;

namespace percepto::geometry
{
class Triangle : public percepto::core::<Triangle>
{
 public:
  bool intersect(const Ray& ray, double& t_hit) const {}
};
}  // namespace percepto::geometry