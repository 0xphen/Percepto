#pragma once

#include <memory>
#include <vector>

#include "percepto/geometry/sphere.h"

namespace percepto::geometry
{

class Scene
{
 private:
  std::vector<percepto::geometry::Sphere> objects;
};
}  // namespace percepto::geometry