#include <gtest/gtest.h>
#include <limits>
#include <memory>
#include <vector>

#include "percepto/core/scene.h"
#include "percepto/core/vec3.h"
#include "percepto/sensor/lidar_emitter.h"
#include "percepto/sensor/lidar_simulator.h"
#include "test_helpers.h"

using percepto::core::Scene;
using percepto::sensor::LidarEmitter;
using percepto::sensor::LidarSimulator;
// using percepto::test::EXPECT_VEC3_EQ;

static constexpr float inf = std::numeric_limits<float>::infinity();
static const percepto::core::Vec3 invalid_point = percepto::core::Vec3(inf, inf, inf);

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

TEST(LidarSimulatorTest, SingleHit) {}