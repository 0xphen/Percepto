#pragma once

#include "percepto/core/intersectable.h"
#include "percepto/core/ray.h"
#include "percepto/core/types.h"
#include "percepto/core/vec3.h"
#include "percepto/math/intersection/moller_trumbore.h"

using percepto::core::Vec3, percepto::core::Ray, percepto::math::intersection::moller_trumbore,
    percepto::core::HitRecord;

namespace percepto::geometry
{
class Triangle : public percepto::core::Intersectable<Triangle>
{
 public:
  /**
   * @brief Check if a ray hits this triangle and return the hit distance.
   *
   * Calls the Möller–Trumbore routine (with back-face culling) on vertices v0_, v1_, v2_.
   *
   * @param[in]  ray    The ray to test.
   * @param[out] hit_record  If true is returned, holds the `HitRecord`
   * @return true if the ray intersects the front face (t > 0), false otherwise.
   */
  bool intersect(const Ray& ray, HitRecord& hit_record) const
  {
    auto hit_data = moller_trumbore(v0_, v1_, v2_, ray);
    if (!hit_data.has_value()) return false;

    const auto& [t, u, v] = hit_data.value();
    hit_record.t = t;
    hit_record.point = ray.at(t);

    return true;
  }

  const Vec3& v0() const { return v0_; }
  const Vec3& v1() const { return v1_; }
  const Vec3& v2() const { return v2_; }

 private:
  Vec3 v0_;  // Triangle vertice A
  Vec3 v1_;  // Triangle vertice B
  Vec3 v2_;  // Triangle vertice C
};
}  // namespace percepto::geometry