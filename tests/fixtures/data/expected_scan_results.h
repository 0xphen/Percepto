#pragma once

#include <array>
#include <cmath>
#include <vector>
#include "percepto/core/vec3.h"

namespace tests::data
{

// From triangles.csv:
inline const std::vector<std::array<percepto::core::Vec3, 3>> triangle_vertices{
    {{{1, 1, 1}, {4, 2, 3}, {2, 4, 4}}},
    {{{0, 0, 0}, {1, 0, 0}, {0, 1, 0}}},
    {{{0, 0, 0}, {0, 1, 1}, {0, 2, 0}}}};

// Expected distances for each (triangle, channel, azimuth):
//   order: tri0[ch0,az0], tri0[ch0,az1], …, tri2[ch2,az2]
inline const std::vector<float>& expected_scan_distances()
{
  static const std::vector<float> v = {
      NAN,     4.1727f, NAN,  // tri0, ch0
      NAN,     NAN,     NAN,  // tri0, ch1
      NAN,     NAN,     NAN,  // tri0, ch2

      1.0000f, NAN,     NAN,  // tri1, ch0
      1.0000f, NAN,     NAN,  // tri1, ch1
      1.0000f, NAN,     NAN,  // tri1, ch2

      NAN,     1.4142f, NAN,  // tri2, ch0
      NAN,     2.0000f, NAN,  // tri2, ch1
      NAN,     NAN,     NAN   // tri2, ch2
  };
  return v;
}

inline constexpr int AZIMUTH_STEPS = 1;

inline std::array<float, 3> ELEVATION_ANGLES{{30.0, 45.0, 60.0}};

}  // namespace tests::data
