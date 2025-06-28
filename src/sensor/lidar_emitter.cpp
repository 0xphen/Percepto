#include <cassert>
#include <cmath>
#include <system_error>

#include "percepto/core/ray.h"
#include "percepto/core/vec3.h"
#include "percepto/io/logger.h"
#include "percepto/sensor/lidar_emitter.h"

using percepto::sensor::LidarEmitter;

namespace percepto::sensor
{
LidarEmitter::LidarEmitter(int azimuth_steps, std::vector<double> elevation_angles)
    : azimuth_steps_(azimuth_steps), elevation_angles_(std::move(elevation_angles))
{
  if (azimuth_steps_ <= 0) throw std::invalid_argument("azimuth_steps must be > 0");
  if (elevation_angles_.empty()) throw std::invalid_argument("elevation_angles cannot be empty");

  cos_elev_.reserve(elevation_angles_.size());
  sin_elev_.reserve(elevation_angles_.size());

  for (const double& angle : elevation_angles_)
  {
    cos_elev_.push_back(std::cos(angle));
    sin_elev_.push_back(std::sin(angle));
  }

  // Precompute evenly spaced azimuth angles over a full 360° (2π radians)
  // for all scan steps. These angles are reused across all scan revolutions.
  azimuth_angles_.reserve(azimuth_steps);
  for (int i = 0; i < azimuth_steps; i++)
  {
    azimuth_angles_.push_back(2.0 * M_PI * double(i) / double(azimuth_steps));
  }
}

percepto::core::Ray LidarEmitter::next()
{
  double current_azimuth_angle = azimuth_angles_[current_azimuth_];

  // Pick precomputed cosφ, sinφ for this channel
  double cos_el = cos_elev_[current_channel_];
  double sin_el = sin_elev_[current_channel_];

  // Spherical→Cartesian:
  //   x = cosφ·cosθ, y = cosφ·sinθ, z = sinφ
  percepto::core::Vec3 dir{float(cos_el * std::cos(current_azimuth_angle)),
                           float(cos_el * std::sin(current_azimuth_angle)), float(sin_el)};

  auto ray = percepto::core::Ray{default_origin, dir, 0.0, 100.0};

  // Advance channel first (inner loop)
  if (++current_channel_ == int(elevation_angles_.size()))
  {
    current_channel_ = 0;
    // Outer loop: advance azimuth once per channel‐cycle
    if (++current_azimuth_ == azimuth_steps_)
    {
      current_azimuth_ = 0;
    }
  }

  return ray;
}
}  // namespace percepto::sensor