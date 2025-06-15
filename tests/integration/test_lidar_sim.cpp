// #include <memory>
// #include <string>
// #include <variant>
// #include <vector>

// #include "fixtures/data/expected_scan_results.h"
// #include "percepto/core/ray.h"
// #include "percepto/core/scene.h"
// #include "percepto/core/vec3.h"
// #include "percepto/geometry/triangle.h"
// #include "percepto/io/csv_parser.h"
// #include "percepto/sensor/lidar_emitter.h"
// #include "percepto/sensor/lidar_simulator.h"
// #include "test_helpers.h"

// using percepto::io::CsvParser;
// using namespace percepto::core;
// using namespace tests::data;

// inline static constexpr const char* file_path = "tests/fixtures/data/triangles.csv";

// TEST(LidarSimIntegration, EndToEndCsvScenario)
// {
//   CsvParser csv_parser;
//   std::unique_ptr<Scene> scene_ptr = csv_parser.load_scene_from_csv(file_path);

//   EXPECT_EQ(int(scene_ptr->size()), 3);
//   auto objects = scene_ptr->objects();
//   for (size_t i = 0; i < scene_ptr->size(); ++i)
//   {
//     auto& object = objects[i];
//     ASSERT_TRUE(std::holds_alternative<Triangle>(object)) << "Object is not a Triangle";
//     Triangle triangle = std::get<Triangle>(object);
//     AssertTriangleMatches(triangle, triangle_vertices[i]);
//   }

//   int azimuth_steps = 1;
//   percepto::sensor::LidarEmitter emitter(AZIMUTH_STEPS, ELEVATION_ANGLES);
// }
