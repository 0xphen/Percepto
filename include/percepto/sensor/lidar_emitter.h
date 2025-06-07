#pragma once

#include <cassert>
#include <cmath>
#include <vector>

#include "percepto/core/ray.h"
#include "percepto/core/vec3.h"

namespace percepto::sensor
{

/**
 * @brief Emits LiDAR rays by sweeping fixed elevation angles
 *        through one 360° revolution in discrete azimuth steps.
 */
class LidarEmitter
{
 public:
  /**
   * @param azimuth_steps     Number of discrete azimuth steps per 360°.
   * @param elevation_angles  Elevation angles (radians) for each laser channel.
   */
  LidarEmitter(int azimuth_steps, std::vector<double> elevation_angles);

  /// Start a new scan (reset both azimuth and channel indices).
  void reset()
  {
    current_azimuth_ = 0;
    current_channel_ = 0;
  }

  /// Emit the next ray; wraps around after one full revolution.
  percepto::core::Ray next();

 private:
  int azimuth_steps_;
  int current_azimuth_ = 0;  ///< [0, azimuth_steps_)
  int current_channel_ = 0;  ///< [0, channel_count)
  std::vector<double> elevation_angles_;
  std::vector<double> cos_elev_, sin_elev_;
  static constexpr double two_pi = 2.0 * M_PI;
  inline static const percepto::core::Vec3 default_origin{0.0, 0.0, 0.0};
};

}  // namespace percepto::sensor
