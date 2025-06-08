#pragma once

#include <vector>

#include "percepto/core/vec3.h"

namespace percepto::sensor
{
struct FrameScan
{
  // N = number of azimuth steps; M = number of channels
  int azimuth_steps;
  int channel_count;

  // distance measurements [i][j]
  std::vector<std::vector<float>> ranges;

  // 3D points computed from ranges + directions
  std::vector<std::vector<percepto::core::Vec3>> points;

  // the actual azimuth angles used
  std::vector<double> azimuth_angles;

  // intensity per return
  std::vector<std::vector<float>> intensities;

  // timestamp of the scan (e.g. start time)
  double timestamp;

  // Count of valid intersections
  int hits;

  FrameScan(int N, int M)
      : azimuth_steps(N),
        channel_count(M),
        ranges(N, std::vector<float>(M, 0.0f)),
        points(N, std::vector<percepto::core::Vec3>(M)),
        azimuth_angles(N, 0.0),
        intensities(N, std::vector<float>(M, 0.0f)),
        hits(0),
        timestamp(0.0)
  {
  }
};
}  // namespace percepto::sensor
