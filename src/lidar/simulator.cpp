#include <limits>
#include <vector>

#include "percepto/common/types.h"
#include "percepto/core/ray.h"
#include "percepto/core/vec3.h"
#include "percepto/io/logger.h"
#include "percepto/lidar/simulator.h"

namespace percepto::lidar
{
using namespace percepto::lidar;

std::vector<common::FrameScan> LidarSimulator::run_scan(int revs)
{
  auto logger = get_percepto_logger();

  auto& le = emitter();
  auto& sc = scene();

  int N = le.azimuth_steps();
  int M = int(le.elevation_angles().size());

  float inf = std::numeric_limits<float>::infinity();
  Vec3 invalid{inf, inf, inf};

  std::vector<common::FrameScan> scans;
  scans.reserve(revs);

  for (int rev = 0; rev < revs; ++rev)
  {
    common::FrameScan scan(N, M);
    scan.azimuth_angles = le.azimuth_angles();
    scan.elevation_angles = le.elevation_angles();

    logger->info("Revolution {}/{} complete", rev + 1, revs);
    for (int i = 0; i < N; ++i)
    {
      for (int j = 0; j < M; ++j)
      {
        auto ray = le.get_ray(i, j);

        HitRecord rec;
        bool hit = sc.intersect(ray, rec);

        if (hit)
        {
          scan.hits++;
          scan.ranges[i][j] = rec.t;
          scan.points[i][j] = rec.point;

          logger->info("Hit @ azimuth={:.2f}°, channel={} (elev={:.2f}°) → distance={:.3f} m",
                       scan.azimuth_angles[i], j, le.elevation_angles()[j], rec.t);
        }
      }
    }

    scans.push_back(std::move(scan));
  }

  logger->info("Simulation complete");

  return scans;
}

}  // namespace percepto::lidar