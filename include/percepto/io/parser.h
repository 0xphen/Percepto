#pragma once

#include <memory>
#include <string>

namespace percepto::core
{
class Scene;  // forward‐declaration; full definition is in scene.h
}

namespace percepto::io
{

/// Read a CSV of triangles (each non-blank, non-‘#’ line has nine floats: x0,y0,z0, x1,y1,z1,
/// x2,y2,z2).
/// @param filename  Path to the CSV file.
/// @return           A Scene (wrapped in unique_ptr)
/// @throws std::runtime_error on I/O failure, parse error, or incorrect field count.
std::unique_ptr<percepto::core::Scene> parse_csv(const std::string& filename);

/// Read a Wavefront OBJ file (with one or more meshes) and return a Scene.
percepto::core::Scene parse_obj(const std::string& filename);
}  // namespace percepto::io
