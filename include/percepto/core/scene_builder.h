#pragma once

#include "percepto/core/scene.h"
#include "percepto/types.h"

namespace percepto::core
{
/// Helper class for constructing (parsing + populating) a Scene.
class SceneBuilder
{
  /// Build a Scene using the given file format.
  ///
  /// @param format  Parser type (CSV, OBJ, GLTF, JSON)
  /// @return        Fully populated Scene
  percepto::core::Scene build_scene(percepto::core::SceneFormat format);
};

}  // namespace percepto::core
