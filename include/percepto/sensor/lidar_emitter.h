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

  /// Returns the number of azimuth steps this emitter was configured with.
  int azimuth_steps() const { return azimuth_angles_.size(); }

  const std::vector<double>& elevation_angles() const { return elevation_angles_; }

  /// Returns the precomputed cosines of each elevation angle.
  const std::vector<double>& elevation_cosines() const { return cos_elev_; }

  /// Returns the precomputed sines of each elevation angle.
  const std::vector<double>& elevation_sines() const { return sin_elev_; }

  /// Emit the next ray; wraps around after one full revolution.
  percepto::core::Ray next();

  /**
   * @brief Generates a LiDAR ray for the given azimuth and elevation indices.
   *
   * @param i Azimuth index (0 to azimuth_steps - 1).
   * @param j Elevation index (0 to elevation_steps - 1).
   * @return The generated `percepto::core::Ray` from the sensor's origin.
   */
  percepto::core::Ray get_ray(const int i, const int j);

  const std::vector<double>& azimuth_angles() const { return azimuth_angles_; }

 private:
  std::vector<double> elevation_angles_;
  std::vector<double> cos_elev_, sin_elev_;
  std::vector<double> azimuth_angles_;
  static constexpr double TWO_PI = 2.0 * M_PI;
  inline static const percepto::core::Vec3 default_origin{0.0, 0.0, 0.0};
};

}  // namespace percepto::sensor
