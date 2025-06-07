#include <cmath>

#include "percepto/core/ray.h"
#include "percepto/core/vec3.h"
#include "percepto/sensor/lidar_emitter.h"

using percepto::sensor::LidarEmitter;

namespace percepto::sensor
{
LidarEmitter::LidarEmitter(int azimuth_steps, std::vector<double> elevation_angles)
    : azimuth_steps_(azimuth_steps),
      elevation_angles_(std::move(elevation_angles)),
      cos_elev_(),
      sin_elev_()
{
  assert(azimuth_steps_ > 0 && !elevation_angles_.empty());

  cos_elev_.reserve(elevation_angles_.size());
  sin_elev_.reserve(elevation_angles_.size());

  for (double angle : elevation_angles_)
  {
    cos_elev_.push_back(std::cos(angle));
    sin_elev_.push_back(std::sin(angle));
  }
}

percepto::core::Ray LidarEmitter::next()
{
  // Compute azimuth angle in [0,2π) as fraction of full rotation
  double azimuth = two_pi * double(current_azimuth_) / double(azimuth_steps_);

  // Pick precomputed cosφ, sinφ for this channel
  double cos_el = cos_elev_[current_channel_];
  double sin_el = sin_elev_[current_channel_];

  // Spherical→Cartesian:
  //   x = cosφ·cosθ, y = cosφ·sinθ, z = sinφ
  percepto::core::Vec3 dir{float(cos_el * std::cos(azimuth)), float(cos_el * std::sin(azimuth)),
                           float(sin_el)};

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