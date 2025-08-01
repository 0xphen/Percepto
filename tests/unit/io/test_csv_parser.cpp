#include <gtest/gtest.h>
#include <array>
#include <fstream>
#include <string>
#include <variant>
#include <vector>

#include "percepto/core/vec3.h"
#include "percepto/geometry/triangle.h"
#include "percepto/io/csv_parser.h"
#include "test_helpers.h"

using percepto::core::Vec3;
using percepto::geometry::Triangle;
using percepto::test::CsvParserTestFixture;

// ––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––
//  Test: loading three triangles from a well‐formed CSV
// ––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––
TEST_F(CsvParserTestFixture, LoadsTriangle_FromCsv)
{
  std::ofstream out(fs.existing_file);
  ASSERT_TRUE(out.is_open()) << "Failed to open temp file " << fs.existing_file;

  out << "x0,y0,z0,x1,y1,z1,x2,y2,z2\n";  // the 9-column header:
  out << "0.0,0.1,0.2, 1.0,1.1,1.2, 2.0,2.1,2.2\n";
  out << "3.0,3.1,3.2, 4.0,4.1,4.2, 5.0,5.1,5.2\n";
  out << "6.0,6.1,6.2, 7.0,7.1,7.2, 8.0,8.1,8.2\n";
  out.close();

  const std::vector<std::array<Vec3, 3>> triangle_vertices = {
      {Vec3{0.0, 0.1, 0.2}, Vec3{1.0, 1.1, 1.2}, Vec3{2.0, 2.1, 2.2}},
      {Vec3{3.0, 3.1, 3.2}, Vec3{4.0, 4.1, 4.2}, Vec3{5.0, 5.1, 5.2}},
      {Vec3{6.0, 6.1, 6.2}, Vec3{7.0, 7.1, 7.2}, Vec3{8.0, 8.1, 8.2}},
  };

  percepto::io::CsvParser parser;
  auto scene = parser.load_scene_from_csv(fs.existing_file.string());

  ASSERT_EQ(scene->size(), 3u) << "Expected 3 triangles in the scene";
  for (size_t i = 0; i < scene->objects().size(); ++i)
  {
    const auto& obj = scene->objects()[i];
    ASSERT_TRUE(std::holds_alternative<Triangle>(obj)) << "Object #" << i << " is not a Triangle";
  }

  for (size_t i = 0; i < 3; ++i)
  {
    const Triangle& t = std::get<Triangle>(scene->objects()[i]);
    AssertTriangleMatches(t, triangle_vertices[i]);
  }
}

// ––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––
//  Test: too many or too few columns → throws
// ––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––
struct ColumnCountTestCase
{
  const char* name;
  const char* data_line;
};

TEST_F(CsvParserTestFixture, Throws_OnTooManyOrTooFewColumns)
{
  const std::vector<ColumnCountTestCase> cases = {
      {"Too few columns", "0.0,0.0,0.0, 1.0,0.0,0.0, 0.0"},              // 7 doubles
      {"Too many columns", "0.0,0.0,0.0, 1.0,0.0,0.0, 0.0,0.8,9.0,1.2"}  // 10 doubles
  };

  for (auto const& c : cases)
  {
    SCOPED_TRACE(c.name);

    std::ofstream out(fs.existing_file);
    ASSERT_TRUE(out.is_open()) << "Cannot open temp file";
    out << "x0,y0,z0,x1,y1,z1,x2,y2,z2\n";
    out << c.data_line << "\n";

    percepto::io::CsvParser parser;
    EXPECT_THROW(parser.load_scene_from_csv(fs.existing_file.string()), std::runtime_error);
  }
}

// ––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––
//  Test: missing file → throws
// ––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––
TEST_F(CsvParserTestFixture, Throws_OnNonexistentFile)
{
  percepto::io::CsvParser parser;
  // non_existent_file was never created by FileTestFixture
  EXPECT_THROW(parser.load_scene_from_csv(fs.non_existent_file.string()), std::runtime_error);
}

// ––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––
//  Test: unreadable file (permissions removed) → throws
// ––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––
TEST_F(CsvParserTestFixture, Throws_OnUnreadableFile)
{
  // FileTestFixture’s constructor made `fs.unreadable_file` non‐readable.
  percepto::io::CsvParser parser;
  EXPECT_THROW(parser.load_scene_from_csv(fs.unreadable_file.string()), std::runtime_error);
}

// ––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––
//  Test: various formatting of doubles (spaces, +/–, scientific)
// ––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––
// TEST_F(CsvParserTestFixture, ParsesVariousDoubleFormats)
// {
//   std::ofstream out(fs.existing_file);
//   ASSERT_TRUE(out.is_open());

//   out << "x0,y0,z0,x1,y1,z1,x2,y2,z2\n";
//   out << " 0.0 , -1.5 , +2.0 ,  3.0,  4.0,  5.0,  6e-1 ,  7E1,  8.0 \n";
//   out << " 9.9, 10.10, 11.11, 12.12, 13.13, 14.14, 15.15, 16.16, 17.17\n";
//   out.close();

//   percepto::io::CsvParser parser;
//   auto scene = parser.load_scene_from_csv(fs.existing_file.string());
//   ASSERT_EQ(scene->size(), 2u) << "Expected 2 triangles";

//   // 3a) Verify Triangle #0 (from the first data line):
//   {
//     const auto& obj0 = scene->objects()[0];
//     ASSERT_TRUE(std::holds_alternative<Triangle>(obj0)) << "First object should be a Triangle";
//     const Triangle& t0 = std::get<Triangle>(obj0);

//     // Vertex 0 = (0.0, -1.5, +2.0)
//     EXPECT_FLOAT_EQ(t0.v0().x, 0.0f);
//     EXPECT_FLOAT_EQ(t0.v0().y, -1.5f);
//     EXPECT_FLOAT_EQ(t0.v0().z, 2.0f);

//     // Vertex 1 = (3.0, 4.0, 5.0)
//     EXPECT_FLOAT_EQ(t0.v1().x, 3.0f);
//     EXPECT_FLOAT_EQ(t0.v1().y, 4.0f);
//     EXPECT_FLOAT_EQ(t0.v1().z, 5.0f);

//     // Vertex 2 = (6e-1, 7E1, 8.0) → (0.6, 70.0, 8.0)
//     EXPECT_FLOAT_EQ(t0.v2().x, 0.6f);   // 6e-1 == 0.6
//     EXPECT_FLOAT_EQ(t0.v2().y, 70.0f);  // 7E1 == 70.0
//     EXPECT_FLOAT_EQ(t0.v2().z, 8.0f);
//   }

//   // 3b) Verify Triangle #1 (from the second data line):
//   {
//     const auto& obj1 = scene->objects()[1];
//     ASSERT_TRUE(std::holds_alternative<Triangle>(obj1)) << "Second object should be a Triangle";
//     const Triangle& t1 = std::get<Triangle>(obj1);

//     // Vertex 0 = ( 9.9, 10.10, 11.11 )
//     EXPECT_FLOAT_EQ(t1.v0().x, 9.9f);
//     EXPECT_FLOAT_EQ(t1.v0().y, 10.10f);
//     EXPECT_FLOAT_EQ(t1.v0().z, 11.11f);

//     // Vertex 1 = (12.12, 13.13, 14.14)
//     EXPECT_FLOAT_EQ(t1.v1().x, 12.12f);
//     EXPECT_FLOAT_EQ(t1.v1().y, 13.13f);
//     EXPECT_FLOAT_EQ(t1.v1().z, 14.14f);

//     // Vertex 2 = (15.15, 16.16, 17.17)
//     EXPECT_FLOAT_EQ(t1.v2().x, 15.15f);
//     EXPECT_FLOAT_EQ(t1.v2().y, 16.16f);
//     EXPECT_FLOAT_EQ(t1.v2().z, 17.17f);
//   }
// }

// ––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––
//  Test: header‐only CSV → empty scene, no throw
// ––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––
TEST_F(CsvParserTestFixture, EmptyCsvReturnsEmptyScene)
{
  std::ofstream out(fs.existing_file);
  ASSERT_TRUE(out.is_open());
  out << "x0,y0,z0,x1,y1,z1,x2,y2,z2\n";  // header only
  out.close();

  percepto::io::CsvParser parser;

  // Should not throw when there are zero data rows
  EXPECT_NO_THROW({
    auto scene = parser.load_scene_from_csv(fs.existing_file.string());
    EXPECT_EQ(scene->size(), 0u);
    EXPECT_TRUE(scene->objects().empty());
  });
}
