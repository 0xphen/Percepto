#include <limits>
#include <vector>

#include "percepto/core/ray.h"
#include "percepto/core/vec3.h"
#include "percepto/io/logger.h"
#include "percepto/sensor/lidar_simulator.h"
#include "percepto/types.h"

namespace percepto::sensor
{
using namespace percepto::sensor;

FrameScan LidarSimulator::run_scan(int revs)
{
  auto logger = get_percepto_logger();

  auto& le = emitter();
  auto& sc = scene();

  int N = le.azimuth_steps();
  int M = int(le.elevation_angles().size());
  int rows = N * revs;

  // Build a FrameScan sized for `rows` × `M`:
  FrameScan scan(rows, M);

  // Initialize with “no hit” sentinels:
  float inf = std::numeric_limits<float>::infinity();
  scan.ranges.assign(rows, std::vector<float>(M, inf));
  Vec3 invalid{inf, inf, inf};
  scan.points.assign(rows, std::vector<Vec3>(M, invalid));
  scan.azimuth_angles.assign(rows, 0.0);

  le.reset();

  logger->info("N={}, M={}", N, M);

  for (int rev = 0; rev < revs; ++rev)
  {
    logger->info("Revolution {}/{} complete", rev + 1, revs);
    for (int i = 0; i < N; ++i)
    {
      int row = rev * N + i;
      scan.azimuth_angles[row] = 2.0 * M_PI * double(i) / double(N);

      for (int j = 0; j < M; ++j)
      {
        auto ray = le.next();

        HitRecord rec;
        bool hit = sc.intersect(ray, rec);

        if (hit)
        {
          scan.hits++;
          scan.ranges[row][j] = rec.t;
          scan.points[row][j] = rec.point;

          logger->info("Hit @ azimuth={:.2f}°, channel={} (elev={:.2f}°) → distance={:.3f} m",
                       scan.azimuth_angles[row], j, le.elevation_angles()[j], rec.t);
        }
      }
    }
  }

  logger->info("Simulation complete");

  return scan;
}

}  // namespace percepto::sensor