#include <memory>
#include <string>
#include <vector>

#include "data/expected_distances.h"
#include "percepto/core/ray.h"
#include "percepto/core/scene.h"
#include "percepto/core/vec3.h"
#include "percepto/geometry/triangle.h"
#include "percepto/io/csv_parser.h"
#include "percepto/sensor/lidar_emitter.h"
#include "percepto/sensor/lidar_simulator.h"
#include "test_helpers.h"

using percepto::io::CsvParser;
using namespace percepto::core;

inline static constexpr const char* file_path = "tests/data/triangles.csv";

// TEST(LidarSimulator_IntegrationTest, EndToEndCsvScenario)
// {
//   CsvParser csv_parser;
//   std::unique_ptr<Scene> scene_ptr = csv_parser.load_scene_from_csv(file_path);

//   std::cout << "SEE: " << scene_ptr->size() << std::endl;
//   EXPECT_EQ(int(scene_ptr->size()), 3);
// }

TEST(CsvParserTest, ReportsMeaningfulErrorOnBadPath)
{
  CsvParser parser;
  try
  {
    auto scene_ptr = parser.load_scene_from_csv(file_path);
    // If we get here, no exception was thrown—probably not what we expect.
    FAIL() << "Expected std::runtime_error when loading CSV from \"" << file_path << "\"";
  }
  catch (const std::runtime_error& e)
  {
    // Print the error message so you can inspect it in test output:
    std::cerr << "Caught exception: " << e.what() << std::endl;

    // Optionally, assert that the message mentions the path or “not found”:
    std::string msg = e.what();
    EXPECT_NE(msg.find(file_path), std::string::npos)
        << "Error message should mention the file path";
    // If you know the exact text, you can tighten this:
    // EXPECT_TRUE(msg.find("does not exist") != std::string::npos);
  }
  catch (...)
  {
    FAIL() << "Expected std::runtime_error, but caught a different exception";
  }
}
