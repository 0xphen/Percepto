#include <CLI/CLI.hpp>
#include <cstdlib>
#include <filesystem>
#include <iostream>
#include <memory>
#include <system_error>
#include <toml++/toml.hpp>
#include <tuple>
#include <vector>

#include "percepto/core/config_loader.h"
#include "percepto/core/scene.h"
#include "percepto/geometry/triangle.h"
#include "percepto/io/csv_parser.h"
#include "percepto/io/logger.h"
#include "percepto/sensor/lidar_emitter.h"
#include "percepto/sensor/lidar_simulator.h"

using namespace std;

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
  percepto::core::LiDARConfig lidar_cfg;
  try
  {
    lidar_cfg = percepto::core::ConfigLoader::loadLiDARConfig();
  }
  catch (const std::exception& e)
  {
    std::cerr << "Failed to load configuration: " << e.what() << std::endl;
    return 1;
  }

  logger->info("Loaded config: azimuth_steps = {}", lidar_cfg.azimuth_steps);
  logger->info("Elevation angles: [{}]", fmt::join(lidar_cfg.elevation_angles, ", "));

  // ----------------------------------------
  // 📡 LiDAR Setup & Simulation
  // ----------------------------------------
  auto emitter = std::make_unique<percepto::sensor::LidarEmitter>(std::move(lidar_cfg));
  percepto::sensor::LidarSimulator simulator(std::move(emitter), std::move(scene_ptr));

  auto scans = simulator.run_scan();
  logger->info("Scan complete");

  return EXIT_SUCCESS;
}
