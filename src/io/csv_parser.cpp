#include "csv.hpp"

#include <filesystem>
#include <fstream>
#include <memory>
#include <string>
#include <system_error>

#include "percepto/core/scene.h"
#include "percepto/core/vec3.h"
#include "percepto/geometry/triangle.h"
#include "percepto/io/csv_parser.h"

using namespace csv;

namespace percepto::io
{
namespace fs = std::filesystem;

/**
 * @brief Ensures that a given path refers to an existing, regular file that can be opened for
 * reading.
 *
 * This function checks:
 *   1. Whether the file exists.
 *   2. Whether it is a regular file (not a directory, symlink to non-file, etc.).
 *   3. Whether it can be opened for reading.
 *
 * On failure, it throws a std::runtime_error with a message indicating the underlying problem.
 *
 * @param filename The path to the file to validate.
 * @throws std::runtime_error If:
 *   - There is an error checking existence (e.g., permission denied).
 *   - The file does not exist.
 *   - There is an error determining file type.
 *   - The path is not a regular file.
 *   - The file cannot be opened for reading.
 */
void percepto::io::CsvParser::ensure_file_readable(const std::string& filename)
{
  fs::path path{filename};
  std::error_code ec;

  if (!fs::exists(path, ec))
  {
    if (ec)
    {
      throw std::runtime_error("Error checking file existence: " + ec.message());
    }
    throw std::runtime_error("File not found: " + filename);
  }

  if (!fs::is_regular_file(path, ec))
  {
    if (ec)
    {
      throw std::runtime_error("Error determining file type: " + ec.message());
    }
    throw std::runtime_error("Not a regular file: " + filename);
  }

  std::ifstream f(filename);
  if (!f.is_open())
  {
    throw std::runtime_error("Cannot open file for reading: " + filename);
  }
}

percepto::geometry::Triangle percepto::io::CsvParser::parse_triangle_from_csv_row(
    const csv::CSVRow& row, size_t row_num)
{
  constexpr size_t expected_fields = 9;
  if (row.size() != expected_fields)
  {
    throw std::runtime_error("Error parsing row " + std::to_string(row_num) + ": expected " +
                             std::to_string(expected_fields) + " fields, but found " +
                             std::to_string(row.size()));
  }

  double coords[9];
  for (size_t i = 0; i < expected_fields; ++i)
  {
    try
    {
      coords[i] = row[i].get<double>();
    }
    catch (const std::exception& e)
    {
      throw std::runtime_error("Error parsing row " + std::to_string(row_num) + ", field " +
                               std::to_string(i + 1) + ": cannot convert to double (" + e.what() +
                               ")");
    }
  }

  Vec3 v0(coords[0], coords[1], coords[2]);
  Vec3 v1(coords[3], coords[4], coords[5]);
  Vec3 v2(coords[6], coords[7], coords[8]);
  return Triangle(v0, v1, v2);
}

/**
 * @brief Load a Scene by parsing each CSV row as a triangle (9 doubles per row).
 *
 * Parsing stops the moment a row fails to parse (throws a std::runtime_error).
 * Subsequent rows are not examined.
 *
 * @param filename Path to a CSV file where each row is "x0,y0,z0,x1,y1,z1,x2,y2,z2".
 * @return std::unique_ptr<percepto::core::Scene> owning all triangles parsed up to the failure.
 * @throws std::runtime_error as soon as any row is malformed or cannot be read.
 */
std::unique_ptr<percepto::core::Scene> percepto::io::CsvParser::load_scene_from_csv(
    const std::string& filename)
{
  this->ensure_file_readable(filename);

  auto scene_ptr = std::make_unique<percepto::core::Scene>();

  CSVFormat format;
  format.variable_columns(VariableColumnPolicy::THROW);

  CSVReader reader(filename, format);
  size_t row_num = 0;
  for (CSVRow& row : reader)
  {
    ++row_num;

    percepto::geometry::Triangle triangle = this->parse_triangle_from_csv_row(row, row_num);
    scene_ptr->add_object(triangle);
  }

  return scene_ptr;
}
}  // namespace percepto::io