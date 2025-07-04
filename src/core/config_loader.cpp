#include <filesystem>
#include <iostream>
#include <toml++/toml.hpp>

#include "percepto/core/config_loader.h"
#include "percepto/io/logger.h"

using percepto::core::ConfigLoader, percepto::core::LiDARConfig;

namespace percepto::core
{
// Helper function to find the config file, useful if executable isn't run from root
std::string ConfigLoader::get_config_filepath(const std::string& base_filename)
{
  // This assumes the config.toml is either in the current working directory
  // or in the parent directory (project root) if run from build/benches/
  std::string path = base_filename;
  if (std::filesystem::exists(path))
  {
    return path;
  }
  path = "../" + base_filename;
  if (std::filesystem::exists(path))
  {
    return path;
  }
  path = "../../" + base_filename;
  if (std::filesystem::exists(path))
  {
    return path;
  }
  return base_filename;  // Return original path if not found, let load function error
}

LiDARConfig ConfigLoader::loadLiDARConfig(const std::string& filepath_base)
{
  LiDARConfig config_data;
  std::string filepath = get_config_filepath(filepath_base);

  auto logger = get_percepto_logger();
  try
  {
    auto config = toml::parse_file(filepath);
    const auto& lidar_sensor = *config["LIDAR_SENSOR"].as_table();

    // Read the number of azimuth steps; default to 36 (i.e., 10° increments for a full circle)
    config_data.azimuth_steps = lidar_sensor["azimuth_steps"].value_or(36);

    if (auto elevation_array_node = lidar_sensor["elevation_angles"].as_array())
    {
      for (const auto& v : *elevation_array_node)
      {
        if (auto angle = v.value<int>())
        {
          config_data.elevation_angles.push_back(*angle);
        }
      }
    }
    else
    {
      // 'elevation_angles' key is missing or not an array, use default empty vector
      logger->error(
          "Warning: 'elevation_angles' in LIDAR_SENSOR is missing or not an array. Using empty "
          "vector.");
      config_data.elevation_angles = {};  // Default to an empty vector
    }
  }
  catch (const toml::parse_error& e)
  {
    logger->error("Error parsing TOML file {}: {}", filepath, e.description());
    throw;  // Re-throw to indicate failure
  }
  catch (const std::exception& e)
  {
    logger->error("Error reading LiDAR config from {}: {}", filepath, e.what());
    throw;
  }
  return config_data;
}
}  // namespace percepto::core