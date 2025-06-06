#pragma once

#include <gtest/gtest.h>
#include <filesystem>
#include <fstream>
#include <system_error>
#include <vector>

#include "percepto/core/ray.h"
#include "percepto/core/vec3.h"
#include "percepto/geometry/triangle.h"

using percepto::core::Ray, percepto::core::Vec3, percepto::geometry::Triangle;

#define EXPECT_VEC3_EQ(v1, v2)        \
  do                                  \
  {                                   \
    EXPECT_DOUBLE_EQ((v1).x, (v2).x); \
    EXPECT_DOUBLE_EQ((v1).y, (v2).y); \
    EXPECT_DOUBLE_EQ((v1).z, (v2).z); \
  } while (0)

namespace percepto::test
{

class CoreTestFixture : public ::testing::Test
{
 public:
  const Vec3 origin = Vec3(0.0, 0.0, 0.0);
  const Vec3 direction = Vec3(1.0, 2.0, 3.0);
  const double t_min = 0.1;
  const double t_max = 100.0;
  const Ray ray = Ray(origin, direction, t_min, t_max);
};

class GeometryTestFixture : public CoreTestFixture
{
 public:
  const Vec3 sphere_centre = Vec3(5.0, 2.0, 12.0);
  double sphere_radius = 5.0;
};

class MathTestFixture : public GeometryTestFixture
{
};

class TriangleTestFixture : public GeometryTestFixture
{
 public:
  const Triangle unit_right_triangle{Vec3(0.0, 0.0, 0.0), Vec3(1.0, 0.0, 0.0), Vec3(0.0, 1.0, 0.0)};

  const Triangle unit_right_triangle_zm1{Vec3(0.0, 0.0, -1.0), Vec3(1.0, 0.0, -1.0),
                                         Vec3(0.0, 1.0, -1.0)};

  const Triangle tilted_triangle{Vec3(0.0, 0.0, 0.0), Vec3(1.0, 2.0, 0.5), Vec3(2.0, 1.0, 1.0)};
};

class IntersectionTestFixture : public TriangleTestFixture
{
};

class SceneTestFixture : public TriangleTestFixture
{
};

class FileTestFixture
{
 public:
  std::filesystem::path existing_file;
  std::filesystem::path non_existent_file;
  std::filesystem::path directory_path;
  std::filesystem::path unreadable_file;
  std::ofstream out;

  static void write_to_csv(std::ofstream& out)
  {
    out << "x0,y0,z0,x1,y1,z1,x2,y2,z2\n";  // the 9-column header:
    out << "0.0,0.1,0.2, 1.0,1.1,1.2, 2.0,2.1,2.2\n";
    out << "3.0,3.1,3.2, 4.0,4.1,4.2, 5.0,5.1,5.2\n";
    out << "6.0,6.1,6.2, 7.0,7.1,7.2, 8.0,8.1,8.2\n";
  }

  FileTestFixture()
  {
    existing_file = make_temp_file_name("csv_exists");
    out.open(existing_file);

    non_existent_file = existing_file;
    non_existent_file += ".does_not_exist";
    delete_if_exists(non_existent_file);  // make sure it really isn’t there

    directory_path = make_temp_file_name("csv_dir");
    std::filesystem::create_directory(directory_path);

    unreadable_file = make_temp_file_name("csv_unreadable");
    {
      std::ofstream out(unreadable_file);
      EXPECT_TRUE(out.is_open()) << "Failed to create unreadable file " << unreadable_file;
      out << "dummy\n";
    }

    // Attempt to remove read permissions:
    std::error_code ec;
    auto perms = std::filesystem::status(unreadable_file, ec).permissions();
    if (!ec)
    {
      // clear owner, group, others read bits
      perms &= ~std::filesystem::perms::owner_read;
      perms &= ~std::filesystem::perms::group_read;
      perms &= ~std::filesystem::perms::others_read;
      std::filesystem::permissions(unreadable_file, perms, ec);
      // If this fails (e.g. on Windows or if permissions are not supported), tests will skip the
      // unreadable case
    }
  }

  ~FileTestFixture()
  {
    delete_if_exists(existing_file);
    delete_if_exists(non_existent_file);
    delete_if_exists(unreadable_file);
    std::error_code ec;
    std::filesystem::remove_all(directory_path, ec);
  }

  static std::filesystem::path make_temp_file_name(const std::string& prefix = "testcsv")
  {
    auto tmp_dir =
        std::filesystem::temp_directory_path();  // e.g. "/tmp" on Linux, or "%TEMP%" on Windows
    auto unique = std::to_string(std::hash<std::string>{}(
        prefix +
        std::to_string(std::chrono::high_resolution_clock::now().time_since_epoch().count())));
    return tmp_dir / (prefix + "_" + unique + ".csv");
  }

  static void delete_if_exists(const std::filesystem::path& p)
  {
    std::error_code ec;
    std::filesystem::remove(p, ec);
    // we intentionally ignore errors here—test cleanup should not throw
  }
};

class CsvParserTestFixture : public ::testing::Test
{
 protected:
  FileTestFixture fs;
};

}  // namespace percepto::test