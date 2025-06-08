#include <gtest/gtest.h>
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
  auto emitter_ptr = std::make_unique<LidarEmitter>(4, std::vector<double>{-1.6, 2.0});
  auto scene_ptr = std::make_unique<Scene>();  // empty scene: no hits

  LidarSimulator sim{std::move(emitter_ptr), std::move(scene_ptr)};

  EXPECT_NO_THROW({
    auto frame = sim.run_scan(1);
    ASSERT_EQ(frame.ranges.size(), 4u);
    ASSERT_EQ(frame.points.size(), 4u);
    EXPECT_EQ(frame.hits, 0);
    ASSERT_FALSE(frame.azimuth_angles.empty());
    ASSERT_FALSE(frame.points.empty());

    for (size_t i = 0; i < frame.points.size(); ++i)
    {
      const auto& row = frame.points[i];
      EXPECT_EQ(row.size(), 2);

      // Then compare element‐wise:
      for (size_t j = 0; j < 2; ++j)
      {
        EXPECT_VEC3_EQ(invalid_point, row[j]);
      }
    }

    for (size_t i = 0; i < frame.ranges.size(); ++i)
    {
      const auto& row = frame.ranges[i];
      ASSERT_EQ(row.size(), 2u);
      for (float d : row)
      {
        EXPECT_EQ(d, inf);
      }
    }
  });
}

TEST(LidarSimulatorTest, SingleHit)
{
  // Single‐channel LiDAR at elevation 38.9° (converted to radians),
  // 8 azimuth steps (45° per step).
  auto emitter_ptr = std::make_unique<LidarEmitter>(8, std::vector<double>{38.9 * DEG2RAD});

  // Scene with one triangle
  auto scene_ptr = std::make_unique<Scene>();
  Vec3 v0{1, 1, 1}, v1{4, 2, 3}, v2{2, 4, 4};
  scene_ptr->add_object(Triangle{v0, v1, v2});

  LidarSimulator sim(std::move(emitter_ptr), std::move(scene_ptr));
  auto frame = sim.run_scan(2);

  EXPECT_EQ(frame.hits, 2);

  // The single hit should be on channel 0 at azimuth index 1 (45°).
  constexpr size_t ELEV_CHANNEL = 0;
  constexpr size_t AZIMUTH_IDX = 1;
  constexpr double EXPECTED_RANGE = 4.1727;

  // Note: frame.ranges is indexed [azimuth][elevation].
  float measured = frame.ranges[AZIMUTH_IDX][ELEV_CHANNEL];
  EXPECT_NEAR(measured, EXPECTED_RANGE, 1e-5f) << "Channel " << ELEV_CHANNEL << ", azimuth step "
                                               << AZIMUTH_IDX << " produced range " << measured;

  // Recompute the expected hit point by shooting a ray in the known direction
  // (7,7,8) normalized, then advancing it by the measured range.
  // We precomputed (7,7,8) because at elevation 38.9° & azimuth 45° the beam direction
  // aligns exactly with that vector, and normalization gives us a unit‐length ray.
  Vec3 dir = Vec3(7, 7, 8).normalized();
  Vec3 pre_computed_point = Ray{Vec3(0, 0, 0), dir, 0, 100}.at(measured);

  // frame.points is likewise [elevation][azimuth].
  Vec3 actual_point = frame.points[AZIMUTH_IDX][ELEV_CHANNEL];
  Vec3 d = actual_point - pre_computed_point;
  float geo_err = std::sqrt(d.x * d.x + d.y * d.y + d.z * d.z);

  // here we allow up to 5 mm total 3D error
  EXPECT_LT(geo_err, 5e-3f) << "3D error = " << geo_err;
}
