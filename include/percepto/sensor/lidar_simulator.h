#pragma once

#include <memory>
#include <vector>

#include "percepto/core/scene.h"
#include "percepto/sensor/frame_scan.h"
#include "percepto/sensor/lidar_emitter.h"

namespace percepto::sensor
{
class LidarSimulator
{
 public:
  // Take ownership of both emitter and scene:
  LidarSimulator(std::unique_ptr<LidarEmitter> emitter,
                 std::unique_ptr<percepto::core::Scene> scene)
      : lidar_emitter_(std::move(emitter)), scene_(std::move(scene))
  {
  }

  // Accessors:
  LidarEmitter& emitter() { return *lidar_emitter_; }
  percepto::core::Scene& scene() { return *scene_; }

  std::vector<FrameScan> run_scan(int revs = 1);

 private:
  std::unique_ptr<percepto::sensor::LidarEmitter> lidar_emitter_;
  std::unique_ptr<percepto::core::Scene> scene_;
};

}  // namespace percepto::sensor
