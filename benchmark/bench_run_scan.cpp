#include <spdlog/spdlog.h>
#include <chrono>
#include <cstdlib>
#include <filesystem>
#include <iomanip>
#include <iostream>
#include <memory>
#include <string>
#include <vector>

#include "percepto/io/csv_parser.h"
#include "percepto/io/logger.h"
#include "percepto/sensor/lidar_emitter.h"
#include "percepto/sensor/lidar_simulator.h"

using namespace std::chrono;
namespace fs = std::filesystem;

constexpr int AZIMUTH_STEPS = 3600;
// 32 laser channels
const std::vector<double> ELEVATION_ANGLES = {
    0.1863,  0.1629,  0.1398,  0.1166,  0.0934,  0.0702,  0.0470,  0.0237,
    0.0005,  -0.0227, -0.0459, -0.0692, -0.0924, -0.1156, -0.1388, -0.1620,
    -0.1852, -0.2084, -0.2316, -0.2548, -0.2780, -0.3012, -0.3244, -0.3476,
    -0.3708, -0.3940, -0.4172, -0.4404, -0.4636, -0.4868, -0.5100, -0.5332};

fs::path resolve_scene_path(const std::string& arg)
{
  try
  {
    fs::path exe_path = fs::canonical(fs::path(arg));
    fs::path exe_dir = exe_path.parent_path();

    return exe_dir / "../../benchmark/triangles.csv";
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

  percepto::io::CsvParser parser;
  auto scene_ptr = parser.load_scene_from_csv(resolve_scene_path(argv[0]).string());

  auto emitter_ptr =
      std::make_unique<percepto::sensor::LidarEmitter>(AZIMUTH_STEPS, ELEVATION_ANGLES);

  int total_rays_cast = emitter_ptr->azimuth_steps() * emitter_ptr->elevation_angles().size();

  percepto::sensor::LidarSimulator sim(std::move(emitter_ptr), std::move(scene_ptr));

  auto start = high_resolution_clock::now();
  sim.run_scan();
  auto end = high_resolution_clock::now();

  auto duration = duration_cast<milliseconds>(end - start);

  std::cout << "Azimuth steps: " << AZIMUTH_STEPS << std::endl;
  std::cout << "Rays per step " << ELEVATION_ANGLES.size() << std::endl;
  std::cout << "Total rays cast: " << total_rays_cast << std::endl;
  std::cout << "Total triangles in scene: " << sim.scene().objects().size() << std::endl;

  std::cout << "Scan took: " << duration.count() << " ms (" << duration.count() / 1000.0 << " s)"
            << std::endl;

  std::cout << std::fixed << std::setprecision(2);
  std::cout << "Rays per second: " << total_rays_cast / (duration.count() / 1000.0) << std::endl;
  return 0;
}