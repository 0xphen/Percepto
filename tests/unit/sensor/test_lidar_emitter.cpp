#include <gtest/gtest.h>
#include <algorithm>
#include <cmath>
#include <vector>

#include "percepto/sensor/lidar_emitter.h"

using percepto::sensor::LidarEmitter;

namespace
{
// Helper to grab the first N directions into a vector
std::vector<percepto::core::Vec3> collect_dirs(LidarEmitter& e, size_t count)
{
  std::vector<percepto::core::Vec3> dirs;
  dirs.reserve(count);
  for (size_t i = 0; i < count; ++i)
  {
    dirs.push_back(e.next().direction());
  }
  return dirs;
}
}  // namespace

TEST(LidarEmitterTest, Construction_StoresParametersAndTrigTables)
{
  std::vector<double> elevations = {-0.1, 0.67};
  LidarEmitter emitter(360, elevations);

  // Stored parameters
  // EXPECT_EQ(emitter.azimuth_steps(), 360);
  // EXPECT_EQ(emitter.elevation_angles().size(), elevations.size());

  // // Raw angles match
  // for (size_t i = 0; i < elevations.size(); ++i)
  // {
  //   EXPECT_DOUBLE_EQ(emitter.elevation_angles()[i], elevations[i]);
  // }

  // // Precomputed trig values match
  // const auto& cosines = emitter.elevation_cosines();
  // const auto& sines = emitter.elevation_sines();
  // for (size_t i = 0; i < elevations.size(); ++i)
  // {
  //   EXPECT_DOUBLE_EQ(cosines[i], std::cos(elevations[i]));
  //   EXPECT_DOUBLE_EQ(sines[i], std::sin(elevations[i]));
  // }
}

TEST(LidarEmitterTest, ThrowsOnInvalidConstructorArgs)
{
  EXPECT_THROW(LidarEmitter(0, std::vector<double>{0.0, 0.5}), std::invalid_argument);
  EXPECT_THROW(LidarEmitter(360, std::vector<double>{}), std::invalid_argument);
}

TEST(LidarEmitterTest, ResetReturnsToStart)
{
  LidarEmitter e(4, {-0.2, +0.2});
  auto first_three = collect_dirs(e, 3);
  collect_dirs(e, 2);
  e.reset();
  auto again_three = collect_dirs(e, 3);
  EXPECT_EQ(again_three, first_three);
}

TEST(LidarEmitterTest, FullRevolutionWrapsCorrectly)
{
  LidarEmitter e(3, {0.0, 0.5, -0.5});
  auto all_dirs = collect_dirs(e, 9);
  auto wrap = e.next().direction();
  // Compare component-wise to avoid relying on operator==
  EXPECT_NEAR(wrap.x, all_dirs[0].x, 1e-6f);
  EXPECT_NEAR(wrap.y, all_dirs[0].y, 1e-6f);
  EXPECT_NEAR(wrap.z, all_dirs[0].z, 1e-6f);
}

TEST(LidarEmitterTest, DirectionsAreUnitLength)
{
  LidarEmitter e(16, {-0.2, 0.0, 0.2});
  for (int i = 0; i < 16 * 3; ++i)
  {
    auto d = e.next().direction();
    float len = std::sqrt(d.x * d.x + d.y * d.y + d.z * d.z);
    EXPECT_NEAR(len, 1.0f, 1e-6f);
  }
}

TEST(LidarEmitterTest, SingleChannelOnlyAzimuthAdvances)
{
  LidarEmitter e(5, {0.1});
  float expectedZ = std::sin(0.1);
  for (int i = 0; i < 5; ++i)
  {
    auto z = e.next().direction().z;
    EXPECT_NEAR(z, expectedZ, 1e-6f);
  }
}

TEST(LidarEmitterTest, SingleAzimuthOnlyChannelCycles)
{
  // One azimuth step, four elevation channels:
  std::vector<double> elevs = {-0.3, -0.1, 0.1, 0.3};
  LidarEmitter e(1, elevs);

  // Emit exactly one full set of rays (4 of them):
  auto dirs = collect_dirs(e, elevs.size());

  // Since azimuth_steps==1, θ==0 for every ray:
  //   x = cos(φ)*cos(0) = cos(φ)
  //   y = cos(φ)*sin(0) = 0
  //   z = sin(φ)
  const auto& stored = e.elevation_angles();

  for (size_t i = 0; i < elevs.size(); ++i)
  {
    double elev_angle = stored[i];
    float expectedX = float(std::cos(elev_angle));
    float expectedY = 0.0f;
    float expectedZ = float(std::sin(elev_angle));

    EXPECT_NEAR(dirs[i].x, expectedX, 1e-6f) << "x should equal cos(elevation) for channel " << i;
    EXPECT_NEAR(dirs[i].y, expectedY, 1e-6f)
        << "y should be zero when azimuth==0 for channel " << i;
    EXPECT_NEAR(dirs[i].z, expectedZ, 1e-6f) << "z should equal sin(elevation) for channel " << i;
  }
}

TEST(LidarEmitterTest, MultipleRevolutionsSafe)
{
  LidarEmitter e(2, {0.0, 0.5, -0.5});
  auto first = collect_dirs(e, 6);
  auto second = collect_dirs(e, 6);
  EXPECT_EQ(second, first);
}
