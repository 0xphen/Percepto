#pragma once

#include <memory>
#include <string>

#include "csv.hpp"
#include "percepto/core/scene.h"
#include "percepto/geometry/triangle.h"

//  forward‐declaration
namespace csv
{
class CSVRow;
}

namespace percepto::io
{

class CsvParser
{
 public:
  /**
   * @brief Parse a CSV file where each row is exactly 9 doubles (three 3D vertices),
   *        build a Scene of Triangles, and return it as a unique_ptr.
   *
   * @param filename Path to the CSV file ("x0,y0,z0,x1,y1,z1,x2,y2,z2" per row).
   * @return std::unique_ptr<percepto::core::Scene> owning all successfully parsed triangles.
   * @throws std::runtime_error If:
   *   - The file does not exist or is not readable.
   *   - One or more rows fails to parse (collects all errors and rethrows at the end).
   */
  static std::unique_ptr<percepto::core::Scene> load_scene_from_csv(const std::string& filename);

 private:
  // Ensure file exists, is a regular file, and is readable.
  static void ensure_file_readable(const std::string& filename);

  //  Parse exactly 9 fields from CSVRow → one Triangle. Throws on error.
  //    row_num is used in error messages.
  static percepto::geometry::Triangle parse_triangle_from_csv_row(const csv::CSVRow& row,
                                                                  size_t row_num);
};

}  // namespace percepto::io
