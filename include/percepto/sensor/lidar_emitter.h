#pragma once

#include <vector>

#include "percepto/core/config_loader.h"
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
   * @param lidar_cfg     Lidar config.
   */
  LidarEmitter(percepto::core::LiDARConfig lidar_cfg);

  /// Start a new scan (reset both azimuth and channel indices).
  void reset()
  {
    current_azimuth_ = 0;
    current_channel_ = 0;
  }

  /// Returns the number of azimuth steps this emitter was configured with.
  int azimuth_steps() const { return azimuth_steps_; }

  const std::vector<double>& elevation_angles() const { return elevation_angles_; }

  /// Returns the precomputed cosines of each elevation angle.
  const std::vector<double>& elevation_cosines() const { return cos_elev_; }

  /// Returns the precomputed sines of each elevation angle.
  const std::vector<double>& elevation_sines() const { return sin_elev_; }

  /// Emit the next ray; wraps around after one full revolution.
  percepto::core::Ray next();

  const std::vector<double>& azimuth_angles() const { return azimuth_angles_; }

 private:
  int azimuth_steps_;
  int current_azimuth_ = 0;  ///< [0, azimuth_steps_)
  int current_channel_ = 0;  ///< [0, channel_count)
  std::vector<double> elevation_angles_;
  std::vector<double> cos_elev_, sin_elev_;
  std::vector<double> azimuth_angles_;
  static constexpr double two_pi = 2.0 * M_PI;
  inline static const percepto::core::Vec3 default_origin{0.0, 0.0, 0.0};
};

}  // namespace percepto::sensor
