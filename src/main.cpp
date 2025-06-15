#include <CLI/CLI.hpp>
#include <cstdlib>
#include <filesystem>
#include <iostream>
#include <memory>
#include <system_error>
#include <toml++/toml.hpp>
#include <tuple>
#include <vector>

#include "percepto/core/scene.h"
#include "percepto/geometry/triangle.h"
#include "percepto/io/csv_parser.h"
#include "percepto/io/logger.h"
#include "percepto/sensor/lidar_emitter.h"
#include "percepto/sensor/lidar_simulator.h"

using namespace std;

/**
 * @brief Loads LiDAR scan configuration from 'config.toml'.
 * @return Tuple containing (revs, azimuth_steps, elevation_angles).
 *
 * Reads the [scan] table and extracts:
 *   - revs: number of revolutions (default: 1)
 *   - azimuth_steps: number of azimuth steps (default: 36)
 *   - elevation_angles: vector of elevation angles (default: {30, 15, 25, -15, 30})
 */
std::tuple<int, int, std::vector<double>> load_config()
{
  auto config = toml::parse_file("config.toml");
  const auto& scan = *config["scan"].as_table();

  // Read number of revolutions; default to 1 if not specified
  int revs = scan["revs"].value_or(1);

  // Read the number of azimuth steps; default to 36 (i.e., 10° increments for a full circle)
  int azimuth_steps = scan["azimuth_steps"].value_or(36);

  // Read the elevation angles array; if not present, use a default set
  std::vector<double> elevation_angles;
  if (auto arr = scan["elevation_angles"].as_array())
  {
    for (const auto& v : *arr)
    {
      if (auto angle = v.value<int>())
      {
        elevation_angles.push_back(*angle);
      }
    }
  }
  else
  {
    elevation_angles = {30, 15, 25, -15, 30};  // Default angles if not specified in config
  }

  return std::make_tuple(revs, azimuth_steps, elevation_angles);
}

/**
 * @brief Entry point for the Percepto LiDAR simulation application.
 * Loads scene geometry, reads configuration, and performs LiDAR scan simulation.
 */

int main(int argc, char** argv)
{
  // ----------------------------------------
  // 🛠️  CLI Setup
  // ----------------------------------------
  CLI::App app{
      "Percepto LiDAR Ray Tracing Simulator.\n"
      "Simulates realistic LiDAR scans from triangle mesh scenes (.csv/.obj)."};

  std::string filepath;
  app.add_option("-f,--filepath", filepath,
                 "Path to the input geometry file (.csv or .obj) to convert into triangle mesh")
      ->required();

  try
  {
    app.parse(argc, argv);
  }
  catch (const CLI::ParseError& e)
  {
    return app.exit(e);
  }

  // ----------------------------------------
  // 🪵 Logger Initialization
  // ----------------------------------------
  auto logger = get_percepto_logger();

  // ----------------------------------------
  // 🧱 Scene Loading
  // ----------------------------------------
  std::unique_ptr<percepto::core::Scene> scene_ptr;
  try
  {
    percepto::io::CsvParser parser;
    scene_ptr = parser.load_scene_from_csv(filepath);
    logger->info("Parsed {} objects from '{}'", scene_ptr->size(), filepath);
  }
  catch (const std::exception& e)
  {
    logger->error("Failed to load scene: {}", e.what());
    return EXIT_FAILURE;
  }

  // ----------------------------------------
  // ⚙️ Configuration Loading
  // ----------------------------------------
  auto [revs, azimuth_steps, elevation_angles] = load_config();
  logger->info("Loaded config: revs = {}, azimuth_steps = {}", revs, azimuth_steps);
  logger->info("Elevation angles: [{}]", fmt::join(elevation_angles, ", "));

  // ----------------------------------------
  // 📡 LiDAR Setup & Simulation
  // ----------------------------------------
  auto emitter = std::make_unique<percepto::sensor::LidarEmitter>(azimuth_steps, elevation_angles);
  percepto::sensor::LidarSimulator simulator(std::move(emitter), std::move(scene_ptr));

  auto frame = simulator.run_scan(revs);
  logger->info("Scan complete: {} hits detected", frame.hits);

  return EXIT_SUCCESS;
}
