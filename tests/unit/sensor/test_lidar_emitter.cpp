#include <gtest/gtest.h>
#include <algorithm>
#include <cmath>
#include <vector>

#include "percepto/core/config_loader.h"
#include "percepto/core/ray.h"
#include "percepto/core/vec3.h"
#include "percepto/sensor/lidar_emitter.h"
#include "test_helpers.h"

using percepto::sensor::LidarEmitter, percepto::core::LiDARConfig, percepto::core::Vec3,
    percepto::core::Ray;

percepto::core::Ray calculate_expected_ray(int i, int j, int azimuth_steps,
                                           const std::vector<double>& elevation_angles)
{
  // Replicate the calculation from LidarEmitter::get_ray
  double current_azimuth_angle =
      2.0 * M_PI * static_cast<double>(i) / static_cast<double>(azimuth_steps);
  double el_angle = elevation_angles[j];

  double cos_el = std::cos(el_angle);
  double sin_el = std::sin(el_angle);

  percepto::core::Vec3 dir(cos_el * std::cos(current_azimuth_angle),
                           cos_el * std::sin(current_azimuth_angle), sin_el);

  return percepto::core::Ray{percepto::core::Vec3(0, 0, 0),
                             dir};  // Assuming (0,0,0) is default_origin
}

TEST(LidarEmitterTest, Construction_StoresParametersAndTrigTables)
{
  std::vector<double> elevations_angles{-0.1, 0.67};
  LidarEmitter emitter(LiDARConfig{360, elevations_angles});

  // Stored parameters
  EXPECT_EQ(emitter.azimuth_steps(), 360);
  EXPECT_EQ(emitter.elevation_angles().size(), elevations_angles.size());

  // Raw angles match
  for (size_t i = 0; i < elevations_angles.size(); ++i)
  {
    EXPECT_DOUBLE_EQ(emitter.elevation_angles()[i], elevations_angles[i]);
  }

  // Precomputed trig values match
  const auto& cosines = emitter.elevation_cosines();
  const auto& sines = emitter.elevation_sines();
  for (size_t i = 0; i < elevations_angles.size(); ++i)
  {
    EXPECT_DOUBLE_EQ(cosines[i], std::cos(elevations_angles[i]));
    EXPECT_DOUBLE_EQ(sines[i], std::sin(elevations_angles[i]));
  }
}

TEST(LidarEmitterTest, ThrowsOnInvalidConstructorArgs)
{
  EXPECT_THROW(LidarEmitter emitter(LiDARConfig{0, {}}), std::invalid_argument);
}

TEST(LidarEmitterTest, GetRay_ValidIndices)
{
  std::vector<double> elevation_angles{-0.2, 0.2, 0.5};
  LidarEmitter e(LiDARConfig{1, elevation_angles});

  // Case 1: First ray (0, 0)
  Ray actual_ray_0_0 = e.get_ray(0, 0);
  Ray expected_ray_0_0 = calculate_expected_ray(0, 0, 1, elevation_angles);

  EXPECT_VEC3_EQ(actual_ray_0_0.origin(), expected_ray_0_0.origin());
  EXPECT_VEC3_EQ(actual_ray_0_0.direction(), expected_ray_0_0.direction());

  // Case 2: Middle ray (0,1)
  Ray actual_ray_1_0 = e.get_ray(0, 1);
  Ray expected_ray_1_0 = calculate_expected_ray(0, 1, 1, elevation_angles);
  EXPECT_VEC3_EQ(actual_ray_1_0.direction(), expected_ray_1_0.direction());

  // Case 3: Last ray (0,2)
  Ray actual_ray_3_1 = e.get_ray(0, 2);
  Ray expected_ray_3_1 = calculate_expected_ray(0, 2, 1, elevation_angles);
  EXPECT_VEC3_EQ(actual_ray_3_1.direction(), expected_ray_3_1.direction());
}

TEST(LidarEmitterTest, GetRay_AssertsOnInvalidAzimuthIndex)
{
  LidarEmitter e(LiDARConfig{4, {-0.2, +0.2}});
  EXPECT_THROW(e.get_ray(-100, 0), std::out_of_range);
}

TEST(LidarEmitterTest, GetRay_AssertsOnInvalidElevationIndex)
{
  LidarEmitter e(LiDARConfig{4, {-0.2, +0.2}});
  EXPECT_THROW(e.get_ray(1, 5), std::out_of_range);
}

TEST(LidarEmitterTest, DirectionsAreUnitLength)
{
  int azimuth_steps = 4;
  auto elevation_angles = {-0.2, 0.0, 0.2};
  LidarEmitter e(LiDARConfig{azimuth_steps, elevation_angles});

  for (int i = 0; i < azimuth_steps; ++i)
  {
    for (int j = 0; j < elevation_angles.size(); ++j)
    {
      auto d = e.get_ray(i, j).direction();
      float len = std::sqrt(d.x * d.x + d.y * d.y + d.z * d.z);
      EXPECT_NEAR(len, 1.0f, 1e-6f);
    }
  }
}
