#include <limits>
#include <variant>

#include "percepto/core/ray.h"
#include "percepto/core/scene.h"
#include "percepto/core/vec3.h"
#include "percepto/types.h"

using percepto::core::Scene, percepto::core::Ray, percepto::core::Vec3;

namespace percepto::core
{
void Scene::add_object(const Object& object)
{
  scene_.push_back(object);
}

bool Scene::intersect(const Ray& ray, HitRecord& hit_record)
{
  double closest_hit = std::numeric_limits<double>::infinity();
  bool hit_object = false;

  for (const auto& object : scene_)
  {
    std::visit(
        [&](const auto& obj)
        {
          HitRecord temp_hit_record;
          if (obj.intersect(ray, temp_hit_record) && temp_hit_record.t < closest_hit)
          {
            closest_hit = temp_hit_record.t;
            hit_record = temp_hit_record;
            hit_object = true;
          }
        },
        object);
  }

  return hit_object;
}

int Scene::size() const
{
  return static_cast<int>(scene_.size());
}
}  // namespace percepto::core
