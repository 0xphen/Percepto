#pragma once

#include <variant>
#include <vector>

#include "percepto/common/types.h"
#include "percepto/core/ray.h"
#include "percepto/geometry/sphere.h"
#include "percepto/geometry/triangle.h"

using percepto::geometry::Sphere, percepto::geometry::Triangle, percepto::core::Ray,
    percepto::common::HitRecord;

namespace percepto::core
{
class Scene
{
 public:
  using Object = std::variant<Sphere, Triangle>;

  void add_object(const Object& object);
  bool intersect(const Ray& ray, HitRecord& hit_record);
  int size() const;
  const std::vector<Object>& objects() const noexcept { return scene_; }

 private:
  std::vector<Object> scene_;
};
}  // namespace percepto::core