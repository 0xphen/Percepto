#pragma once
#include <cmath>
#include <vector>

/// Returns a flat vector of expected t‐values for the triangles defined in
/// "test/data/triangles.csv", sampled over 3 elevation channels and 3 azimuth steps.
/// The ordering is: triangle 0 (ch0-az0, ch0-az1, ch0-az2, ch1-az0, ..., ch2-az2),
/// followed by triangle 1, then triangle 2.
inline const std::vector<float>& getExpectedDistances()
{
  static const std::vector<float> data = {
      // tri0:
      NAN,
      4.1727f,
      NAN,
      NAN,
      NAN,
      NAN,
      NAN,
      NAN,
      NAN,

      // tri1:
      1.0000f,
      NAN,
      NAN,
      1.0000f,
      NAN,
      NAN,
      1.0000f,
      NAN,
      NAN,

      // tri2:
      NAN,
      1.4142f,
      NAN,
      NAN,
      2.0000f,
      NAN,
      NAN,
      NAN,
      NAN,
  };
  return data;
}
