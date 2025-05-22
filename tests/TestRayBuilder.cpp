#include <gtest/gtest.h>
#include <percepto/core/RayBuilder.h>
#include <percepto/core/Vec3.h>
#include "TestHelpers.h"

using percepto::core::RayBuilder, percepto::core::Vec3, percepto::core::Ray;
using percepto::test::CoreTest;

TEST_F(CoreTest, RayBuilder_BuildsRayWithCorrectParams)
{
  RayBuilder builder =
      RayBuilder().setOrigin(origin).setDirection(direction).setRange(t_min, t_max);

  // Validate builder stores the raw parameters
  EXPECT_VEC3_EQ(builder.origin(), origin);
  EXPECT_VEC3_EQ(builder.direction(), direction);
  EXPECT_EQ(builder.tMin(), t_min);
  EXPECT_EQ(builder.tMax(), t_max);

  // Try to build the ray
  std::optional<Ray> maybeRay = builder.tryBuild();
  ASSERT_TRUE(maybeRay.has_value());  // Fail fast if building fails

  const Ray& ray = maybeRay.value();

  // Check that built ray contains correct values
  EXPECT_VEC3_EQ(ray.origin(), origin);
  EXPECT_EQ(ray.tMin(), t_min);
  EXPECT_EQ(ray.tMax(), t_max);

  // Direction must be normalized
  const Vec3 expectedDirection = direction.normalized();
  const Vec3& actualDirection = ray.direction();

  EXPECT_VEC3_EQ(actualDirection, expectedDirection);

  // Extra check using EXPECT_NEAR to validate each component
  EXPECT_NEAR(actualDirection.x, expectedDirection.x, 1e-9);
  EXPECT_NEAR(actualDirection.y, expectedDirection.y, 1e-9);
  EXPECT_NEAR(actualDirection.z, expectedDirection.z, 1e-9);
}

TEST_F(CoreTest, RayBuilderTest_TryBuildFailsIfZeroDirection)
{
  RayBuilder rayBuilder =
      RayBuilder().setOrigin(origin).setDirection(Vec3(0.0, 0.0, 0.0)).setRange(t_min, t_max);

  std::optional<Ray> optionalRay = rayBuilder.tryBuild();
  ASSERT_FALSE(optionalRay.has_value());
}