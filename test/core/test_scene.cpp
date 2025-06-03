#include <gtest/gtest.h>

#include "percepto/core/ray.h"
#include "percepto/core/scene.h"
#include "percepto/core/types.h"
#include "percepto/core/vec3.h"
#include "percepto/geometry/triangle.h"
#include "test_helpers.h"

using namespace percepto::core;
using percepto::test::SceneTestFixture;

TEST_F(SceneTestFixture, Single_Object_Hit)
{
  {
    SCOPED_TRACE("Hits Single Triangle");
    Triangle triangle(unit_right_triangle.v0(), unit_right_triangle.v1(), unit_right_triangle.v2());

    Scene scene;
    scene.add_object(triangle);

    ASSERT_TRUE(scene.size() == 1);

    Ray ray(Vec3(0.25, 0.25, 1.0), Vec3(0.0, 0.0, -1.0), 0.0, 100.0);
    HitRecord hit_record;

    ASSERT_TRUE(scene.intersect(ray, hit_record));
  }
}