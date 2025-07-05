#pragma once

#include <string>
#include <vector>

namespace percepto::core
{
struct LiDARConfig
{
  int azimuth_steps;                     // Number of discrete azimuth steps per 360°
  std::vector<double> elevation_angles;  // Elevation angles (radians) for each laser channel.
};

struct RayTracerConfig
{
  double ray_t_min;
  double ray_t_max;
};

class ConfigLoader
{
 public:
  static LiDARConfig loadLiDARConfig();
  static RayTracerConfig loadRayTracerConfig(const std::string& filepath);

 private:
  // Helper to get the full path to config.toml from where executable runs
  static std::string get_config_filepath();
};
}  // namespace percepto::core