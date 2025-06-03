#pragma once

#include "percepto/core/vec3.h"

namespace percepto
{
inline constexpr double EPSILON = 1e-6;

// Stores raw intersection data returned by the Möller–Trumbore algorithm for triangles.
struct TriangleHitResult
{
  double t;  // Ray parameter at intersection point.
  double u;  // Barycentric coordinate u.
  double v;  // Barycentric coordinate v.
};

// Describes detailed intersection information used throughout the ray tracing pipeline.
struct HitRecord
{
  double t = 0.0;               // Distance from ray origin to intersection point.
  percepto::core::Vec3 point;   // World-space position of the hit point.
  percepto::core::Vec3 normal;  // Surface normal at the intersection.
  bool front_face = true;       // True if ray hits front face; false if hitting from inside.
};

/// Used by the SceneBuilder to determine which parser to invoke when
/// loading scene geometry and objects.
enum class SceneFormat
{
  CSV,   ///< Custom CSV listing of objects
  OBJ,   ///< Wavefront OBJ model
  GLTF,  ///< glTF 2.0 format
  JSON   ///< JSON‐based scene description
};

}  // namespace percepto