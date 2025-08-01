#include <cassert>
#include <cmath>
#include <stdexcept>
#include <system_error>

#include "percepto/common/config_loader.h"
#include "percepto/core/ray.h"
#include "percepto/core/vec3.h"
#include "percepto/io/logger.h"
#include "percepto/lidar/emitter.h"

using percepto::lidar::LidarEmitter;

namespace percepto::lidar
{
LidarEmitter::LidarEmitter(percepto::common::LiDARConfig lidar_cfg)
    : elevation_angles_(std::move(lidar_cfg.elevation_angles))
{
  if (elevation_angles_.empty()) throw std::invalid_argument("elevation_angles cannot be empty");

  cos_elev_.reserve(elevation_angles_.size());
  sin_elev_.reserve(elevation_angles_.size());

  for (const double& angle : elevation_angles_)
  {
    cos_elev_.emplace_back(std::cos(angle));
    sin_elev_.emplace_back(std::sin(angle));
  }

  // Precompute evenly spaced azimuth angles over a full 360° (2π radians)
  // for all scan steps. These angles are reused across all scan revolutions.
  azimuth_angles_.reserve(lidar_cfg.azimuth_steps);
  for (int i = 0; i < lidar_cfg.azimuth_steps; i++)
  {
    azimuth_angles_.emplace_back(TWO_PI * double(i) / double(lidar_cfg.azimuth_steps));
  }
}

percepto::core::Ray LidarEmitter::get_ray(const int i, const int j)
{
  if (i < 0 || i >= azimuth_angles_.size())
  {
    throw std::out_of_range("Azimuth index 'i' out of bounds.");
  }

  if (j < 0 || j >= elevation_angles_.size())
  {
    throw std::out_of_range("Elevation index 'j' out of bounds.");
  }

  // Pick precomputed cosφ, sinφ for this channel
  double current_azimuth_angle = azimuth_angles_[i];
  double cos_el = cos_elev_[j];
  double sin_el = sin_elev_[j];

  // Spherical→Cartesian:
  //   x = cosφ·cosθ, y = cosφ·sinθ, z = sinφ
  percepto::core::Vec3 dir{cos_el * std::cos(current_azimuth_angle),
                           cos_el * std::sin(current_azimuth_angle), sin_el};

  return percepto::core::Ray{default_origin, dir};
}

}  // namespace percepto::lidar