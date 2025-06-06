#pragma once

#include "percepto/core/ray.h"
#include "percepto/types.h"

namespace percepto::core
{

/**
 * @brief Intersectable (CRTP Interface)
 *
 * A compile-time polymorphic base class for geometric objects that can be
 * intersected by a ray. This interface uses the Curiously Recurring Template
 * Pattern (CRTP) to allow derived classes (e.g., Sphere, Triangle, Plane) to
 * implement their own `intercept` method without incurring runtime overhead.
 *
 * CRTP enables compile-time polymorphism, which allows:
 * - Elimination of virtual tables (vtables)
 * - Full compile-time dispatch
 * - Aggressive inlining and compiler optimization
 */

template <typename Derived>
class Intersectable
{
 public:
  // Dispatches the call to the derived class’s implementation of `intercept`
  // using CRTP. Returns true if the ray hits the object, and writes the hit
  // record to `hit_record`.
  [[nodiscard]]
  bool intersect(const percepto::core::Ray& ray, percepto::HitRecord& hit_record) const
  {
    return static_cast<const Derived*>(this)->intercept(ray, hit_record);
  }
};

}  // namespace percepto::core
