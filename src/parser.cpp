#include <charconv>
#include <filesystem>
#include <fstream>
#include <memory>
#include <system_error>
#include <vector>

#include "percepto/core/scene.h"
#include "percepto/core/vec3.h"
#include "percepto/io/parser.h"

namespace percepto::io
{
namespace fs = std::filesystem;

std::unique_ptr<percepto::core::Scene> parse_csv(const std::string& filename) {}

}  // namespace percepto::io