#include <filesystem>
#include <format>
#include <iostream>
#include <toml++/toml.hpp>

#include "percepto/common/config_loader.h"
#include "percepto/io/logger.h"

using percepto::common::ConfigLoader, percepto::common::LiDARConfig;

constexpr const char* DEFAULT_CONFIG = "config.toml";

namespace percepto::common
{
// Helper function to find the config file, useful if executable isn't run from root
std::string ConfigLoader::get_config_filepath()
{
  std::filesystem::path exe_path;

  std::filesystem::path current_path = std::filesystem::current_path();
  std::filesystem::path config_candidate_path;

  config_candidate_path = current_path / DEFAULT_CONFIG;
  if (std::filesystem::exists(config_candidate_path))
  {
    return config_candidate_path.string();
  }

  config_candidate_path = current_path.parent_path() / DEFAULT_CONFIG;
  if (std::filesystem::exists(config_candidate_path))
  {
    return config_candidate_path.string();
  }

  config_candidate_path = current_path.parent_path().parent_path() / DEFAULT_CONFIG;
  if (std::filesystem::exists(config_candidate_path))
  {
    return config_candidate_path.string();
  }

  auto logger = get_percepto_logger();
  logger->error(
      "Config file '{}' not found at expected locations relative to CWD. "
      "Tried: {}, {}, {}",
      DEFAULT_CONFIG, (current_path / DEFAULT_CONFIG).string(),
      (current_path.parent_path() / DEFAULT_CONFIG).string(),
      (current_path.parent_path().parent_path() / DEFAULT_CONFIG).string());

  throw std::runtime_error(std::string("Config file '") + DEFAULT_CONFIG + "' not found.");
}

LiDARConfig ConfigLoader::loadLiDARConfig()
{
  LiDARConfig config_data;
  std::string filepath = get_config_filepath();

  auto logger = get_percepto_logger();

  toml::table tbl;
  try
  {
    tbl = toml::parse_file(filepath);
  }
  catch (const toml::parse_error& err)
  {
    logger->error("Error parsing file {}: {}", filepath, err.description());
  }

  config_data.azimuth_steps = tbl["LIDAR_SENSOR"]["azimuth_steps"].value_or(36);

  auto elevation_angles = tbl["LIDAR_SENSOR"]["elevation_angles"];
  if (toml::array* arr = elevation_angles.as_array())
  {
    for (const auto& v : *arr)
    {
      config_data.elevation_angles.push_back(*v.value<double>());
    }
  }
  else
  {
    config_data.elevation_angles = {};  // defualt to empty array
  }

  return config_data;
}
}  // namespace percepto::common