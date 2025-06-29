#include <gtest/gtest.h>
#include <algorithm>
#include <iomanip>
#include <iostream>
#include <limits>
#include <memory>
#include <vector>

#include "percepto/core/scene.h"
#include "percepto/core/vec3.h"
#include "percepto/math/intersection/moller_trumbore.h"
#include "percepto/sensor/lidar_emitter.h"
#include "percepto/sensor/lidar_simulator.h"
#include "test_helpers.h"

using percepto::core::Scene;
using percepto::core::Vec3, percepto::core::Ray, percepto::geometry::Triangle;
using percepto::sensor::LidarEmitter;
using percepto::sensor::LidarSimulator;

static constexpr float inf = std::numeric_limits<float>::infinity();
static const percepto::core::Vec3 invalid_point = percepto::core::Vec3(inf, inf, inf);
constexpr double DEG2RAD = M_PI / 180.0;

TEST(LidarSimulatorTest, SmokeTest)
{
  auto azimuth_steps = 4;
  auto emitter_ptr = std::make_unique<LidarEmitter>(azimuth_steps, std::vector<double>{-1.6, 2.0});
  auto scene_ptr = std::make_unique<Scene>();  // empty scene: no hits

  LidarSimulator sim{std::move(emitter_ptr), std::move(scene_ptr)};

  auto all_equal = [](const auto& vec, const auto& value)
  { return std::all_of(vec.begin(), vec.end(), [&](const auto& el) { return el == value; }); };

  auto frames = sim.run_scan(1);
  auto frame = frames[0];

  // Expected azimuth angles for 4 azimuth steps evenly spaced over 360° (2π radians)
  auto expected_azimuth_angles = std::vector<double>{0.0, 1.5708, 3.1416, 4.7124};
  ASSERT_EQ(frame.hits, 0);
  ASSERT_EQ(frame.azimuth_steps, 4);
  ASSERT_EQ(frame.channel_count, 2);

  Vec3 default_vec{0, 0, 0};
  for (size_t i = 0; i < azimuth_steps; i++)
  {
    ASSERT_TRUE(all_equal(frame.points[i], default_vec));
    ASSERT_TRUE(all_equal(frame.ranges[i], 0.0f));
    ASSERT_NEAR(expected_azimuth_angles[i], frame.azimuth_angles[i], 1e-4);
  }
}

TEST(LidarSimulatorTest, SingleHitAndMultipleRevolutions)
{
  // ––– Setup: 1‐channel LiDAR at elevation 38.9° (rad), 8 azimuth steps (45° each) –––
  auto emitter_ptr = std::make_unique<LidarEmitter>(8, std::vector<double>{38.9 * DEG2RAD});

  // ––– Scene: one triangle –––
  auto scene_ptr = std::make_unique<Scene>();
  Vec3 v0{1, 1, 1}, v1{4, 2, 3}, v2{2, 4, 4};
  scene_ptr->add_object(Triangle{v0, v1, v2});

  LidarSimulator sim(std::move(emitter_ptr), std::move(scene_ptr));

  // ––– B. Single‐hit: one revolution –––
  {
    auto frames = sim.run_scan(1);
    auto frame = frames[0];

    EXPECT_EQ(frame.hits, 1);

    constexpr size_t ELEV = 0;
    constexpr size_t AZ = 1;  // 45° step
    constexpr float EXP_R = 4.1727f;

    float r = frame.ranges[AZ][ELEV];
    EXPECT_NEAR(EXP_R, r, 1e-5f) << "range at elev=" << ELEV << " az=" << AZ;

    Vec3 dir = Vec3(7, 7, 8).normalized();
    Vec3 expect_pt = Ray{Vec3{0, 0, 0}, dir, 0, 100}.at(r);
    Vec3 got_pt = frame.points[AZ][ELEV];

    Vec3 d = got_pt - expect_pt;
    float geo_err = std::sqrt(d.x * d.x + d.y * d.y + d.z * d.z);
    EXPECT_LT(geo_err, 5e-3f) << "point error = " << geo_err;
  }

  // ––– C. Multiple‐revolutions: two back‐to‐back sweeps must match –––
  {
    int revs = 2;
    auto frames = sim.run_scan(revs);

    EXPECT_EQ(frames.size(), 2);

    constexpr size_t ELEV = 0;
    constexpr size_t AZ_STEPS = 8;

    for (size_t i = 0; i < revs; i++)
    {
      auto frame = frames[i];
      EXPECT_EQ(frame.hits, 1);

      // First revolution is in indices [0..7], second in [8..15]
      for (size_t az = 0; az < AZ_STEPS; az++)
      {
        float r0 = frame.ranges[az][ELEV];
        Vec3 p0 = frame.points[az][ELEV];
        // TODO: Add Assertion
      }
    }
  }
}