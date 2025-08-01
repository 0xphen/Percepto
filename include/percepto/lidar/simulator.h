#pragma once

#include <memory>
#include <vector>

#include "percepto/common/frame_scan.h"
#include "percepto/core/scene.h"
#include "percepto/lidar/emitter.h"

namespace percepto::lidar
{
class LidarSimulator
{
 public:
  LidarSimulator(std::unique_ptr<LidarEmitter> emitter,
                 std::unique_ptr<percepto::core::Scene> scene)
      : lidar_emitter_(std::move(emitter)), scene_(std::move(scene))
  {
  }

  LidarEmitter& emitter() { return *lidar_emitter_; }
  percepto::core::Scene& scene() { return *scene_; }

  std::vector<percepto::common::FrameScan> run_scan(int revs = 1);

 private:
  std::unique_ptr<percepto::lidar::LidarEmitter> lidar_emitter_;
  std::unique_ptr<percepto::core::Scene> scene_;
};

}  // namespace percepto::lidar
