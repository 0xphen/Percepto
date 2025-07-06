#include <spdlog/spdlog.h>
#include <chrono>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <memory>
#include <string>
#include <toml++/toml.hpp>
#include <vector>

#include "percepto/core/config_loader.h"
#include "percepto/io/csv_parser.h"
#include "percepto/io/logger.h"
#include "percepto/sensor/lidar_emitter.h"
#include "percepto/sensor/lidar_simulator.h"

using namespace std::chrono;
namespace fs = std::filesystem;

fs::path resolve_scene_path(const std::string& arg, std::string file_name)
{
  try
  {
    fs::path exe_path = fs::canonical(fs::path(arg));
    fs::path exe_dir = exe_path.parent_path();

    return exe_dir / "../../scenes" / file_name;
  }
  catch (const std::exception& e)
  {
    std::cerr << "[PANIC] Failed to resolve path from arg: " << arg << "\n";
    std::cerr << "Reason: " << e.what() << "\n";
    std::exit(EXIT_FAILURE);
  }
}

int main(int argc, char** argv)
{
  get_percepto_logger()->set_level(spdlog::level::off);

  if (argc != 2)
  {
    std::cerr << "Usage: " << argv[0] << " [dense|sparse]" << std::endl;
    return 1;
  }

  std::string scene_type = argv[1];
  std::string file_name;

  if (scene_type == "dense")
  {
    file_name = "dense_scene.csv";
    std::cout << "--- Running benchmark for DENSE scene ---" << std::endl;
  }
  else if (scene_type == "sparse")
  {
    file_name = "sparse_scene.csv";
    std::cout << "--- Running benchmark for SPARSE scene ---" << std::endl;
  }
  else
  {
    std::cerr << "Error: Invalid scene type. Use 'dense' or 'sparse'." << std::endl;
    return 1;
  }

  percepto::core::LiDARConfig lidar_cfg;
  resolve_scene_path(argv[0], file_name).string();
  try
  {
    lidar_cfg = percepto::core::ConfigLoader::loadLiDARConfig();
  }
  catch (const std::exception& e)
  {
    std::cerr << "Failed to load configuration: " << e.what() << std::endl;
    return 1;
  }

  auto emitter_ptr = std::make_unique<percepto::sensor::LidarEmitter>(std::move(lidar_cfg));

  percepto::io::CsvParser parser;
  std::cout << "Loading scene from: " << file_name << std::endl;
  auto scene_ptr = parser.load_scene_from_csv(resolve_scene_path(argv[0], file_name).string());
  std::cout << "Loaded " << scene_ptr->size() << " triangles." << std::endl;

  int total_rays = emitter_ptr->azimuth_steps() * emitter_ptr->elevation_angles().size();

  percepto::sensor::LidarSimulator sim(std::move(emitter_ptr), std::move(scene_ptr));

  std::cout << "LiDARScanner initialized with " << sim.emitter().azimuth_steps()
            << " azimuth steps and " << sim.emitter().elevation_angles().size()
            << " elevation channels." << std::endl;

  auto start = high_resolution_clock::now();
  auto frames = sim.run_scan();
  auto end = high_resolution_clock::now();

  std::chrono::duration<double> duration = end - start;
  double scan_seconds = duration.count();
  double rays_per_second = total_rays / scan_seconds;

  std::cout << "\n--- Percepto Scan Benchmark Results (" << scene_type << " scene) ---"
            << std::endl;
  std::cout << "  Scene Triangles: " << sim.scene().size() << std::endl;
  std::cout << "  Total Rays Cast: " << total_rays << std::endl;
  std::cout << "  Hits Detected:   " << frames[0].hits << std::endl;
  std::cout << "  Total Runtime:   " << scan_seconds * 1000.0 << " ms (" << scan_seconds << " s)"
            << std::endl;
  std::cout << "  Rays per Second: " << rays_per_second << std::endl;
  std::cout << "--------------------------------------------------------" << std::endl;

  return 0;
}