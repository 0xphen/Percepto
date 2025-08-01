#include <gtest/gtest.h>
#include <string>
#include <vector>

#include "percepto/common/types.h"
#include "percepto/core/ray.h"
#include "percepto/core/scene.h"
#include "percepto/core/vec3.h"
#include "percepto/geometry/triangle.h"
#include "test_helpers.h"

using namespace percepto::core;
using percepto::test::SceneTestFixture;

TEST_F(SceneTestFixture, Intersect_Single_Triangle_Hit)
{
  Scene scene;
  scene.add_object(unit_right_triangle);

  EXPECT_EQ(scene.size(), 1);

  Ray ray(Vec3(0.25, 0.25, 1.0), Vec3(0.0, 0.0, -1.0), 0.0, 100.0);

  HitRecord hit_record;
  ASSERT_TRUE(scene.intersect(ray, hit_record));
  EXPECT_NEAR(hit_record.t, 1.0, 1e-6);
  EXPECT_VEC3_EQ(ray.at(hit_record.t), Vec3(0.25, 0.25, 0.0));
}

TEST_F(SceneTestFixture, Intersect_EmptyScene_ReturnsFalse)
{
  Scene scene;
  Ray ray(Vec3(0, 0, 0), Vec3(1, 0, 0), 0.0, 100.0);
  HitRecord hit_record;

  EXPECT_EQ(scene.size(), 0);
  EXPECT_FALSE(scene.intersect(ray, hit_record));
}
TEST_F(SceneTestFixture, Intersect_MultipleObjects_ReturnsClosest)
{
  auto runIntersectionTest =
      [=](const std::vector<Triangle>& objects, const std::string& trace_info)
  {
    SCOPED_TRACE(trace_info);

    Scene scene;
    for (const auto& obj : objects)
    {
      scene.add_object(obj);
    }

    Vec3 rayOrig(0.2, 0.3, 1.0);
    Vec3 rayDir(0.0, 0.0, -1.0);
    Ray ray(rayOrig, rayDir, t_min, t_max);

    HitRecord hit_record;
    ASSERT_TRUE(scene.intersect(ray, hit_record));
    EXPECT_NEAR(hit_record.t, 1.0, 1e-6);
  };

  runIntersectionTest({unit_right_triangle, unit_right_triangle_zm1},
                      "Order: near (t≈1) then far (t≈2)");

  // Reversed insertion order; still should pick the closest.
  runIntersectionTest({unit_right_triangle_zm1, unit_right_triangle},
                      "Order: far (t≈2) then near (t≈1)");
}

TEST_F(SceneTestFixture, Intersect_RayMissesAll_ReturnsFalse)
{
  Scene scene;
  scene.add_object(unit_right_triangle);

  Ray ray(Vec3(2, 2, 0), Vec3(3, 4, 5), t_min, t_max);
  HitRecord hit_record;
  EXPECT_FALSE(scene.intersect(ray, hit_record));
}

TEST_F(SceneTestFixture, Intersect_Degenerate_Object_ReturnsFalse)
{
  Vec3 v0(0.0, 0.0, 0.0);
  Vec3 v1(1e-8, 0.0, 0.0);
  Vec3 v2(2e-8, 0.0, 0.0);

  Scene scene;
  scene.add_object(Triangle(v0, v1, v2));  // Skinny triangle - almost 0 area

  Ray ray(Vec3(1e-9, 1e-9, 1.0), Vec3(0.0, 0.0, -1.0), 0.0, 100.0);

  HitRecord hit_record;
  EXPECT_FALSE(scene.intersect(ray, hit_record));
}
